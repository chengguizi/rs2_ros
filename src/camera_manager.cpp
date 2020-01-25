#include "camera_manager.hpp"
#include "exposure_ctl.hpp"

#include <memory>

#include <numeric>

#include <cassert>

#include <rs2_ros/CameraStats.h>

#include <sensor_msgs/image_encodings.h>


std::vector<std::string> CameraParam::camera_list;

std::vector<std::string>  CameraParam::loadCameras(const ros::NodeHandle& nh)
{
    // // std::cout << nh_local.getNamespace() << std::endl;
    nh.getParam("devices", camera_list);

    return camera_list;
}

void CameraParam::loadParam(const std::string& topic_ns)
{
    this->topic_ns = topic_ns;
    ros::NodeHandle nh_local("~/" + topic_ns);

    // Obtain Device Specific Params
    nh_local.getParam("type",type);
    nh_local.getParam("sn",camera_sn);

    nh_local.getParam("hardware_synchronisation_mode", do_hardware_sync);
    nh_local.getParam("stereo", do_publish_stereo);
    nh_local.getParam("depth", do_publish_depth);
    nh_local.getParam("color", do_publish_color);
    nh_local.getParam("poseimu", do_publish_poseimu);
    nh_local.getParam("loop_closure", do_loop_closure);
    nh_local.getParam("auto_exposure_mode", auto_exposure_mode); // internal, or custom

    // Obtain Type Specific Params
    ros::NodeHandle nh_type("~/" + type);
    nh_type.getParam("width",width);
    nh_type.getParam("height",height);
    nh_type.getParam("frame_rate",hz);

    if (type == "d400")
    {
        nh_type.getParam("laser_power",laser_power);
        nh_type.getParam("alternate_laser_emitter", do_alternate_laser_emitter);
    }

    if (type == "t265")
    {
        nh_type.getParam("pose_coordinate", pose_coordinate);
    }
    
}

void CameraParam::loadExposureControlParam(const std::string& type)
{
    ros::NodeHandle nh_type("~/" + type);

    nh_type.getParam("initial_exposure",initial_exposure);
    nh_type.getParam("initial_gain",initial_gain);
    nh_type.getParam("exposure_max",exposure_max);
    nh_type.getParam("gain_max",gain_max);
    nh_type.getParam("exposure_change_rate",exposure_change_rate);
    nh_type.getParam("exposure_target_mean",exposure_target_mean);
    nh_type.getParam("exposure_dead_region",exposure_dead_region);
    nh_type.getParam("exposure_a_max",exposure_a_max);
    nh_type.getParam("exposure_a_min",exposure_a_min);
    nh_type.getParam("exposure_c",exposure_c);

    ros::NodeHandle nh_local("~/" + topic_ns);
    nh_local.setParam("exposure",initial_exposure);
    nh_local.setParam("gain",initial_gain);

}

CameraManager::CameraManager(const std::string& topic_ns) : initialised(false)
{
    std::cout << std::endl << "==========================================" << std::endl;
    std::cout << "Creating CameraManager for " << topic_ns << std::endl;
    //// Obtain all params associated with the topic_ns
    param.loadParam(topic_ns);

    ros::NodeHandle nh_local("~/" + topic_ns);

    //// Creating Device Driver
    sys = new StereoDriver(param.camera_sn);

    if (!sys->isInitialised())
    {
        delete sys;
        sys = nullptr;
        return;
    }

    //// Configure Laser Emitter
    if (param.type == "d400")
    {
        if (param.laser_power > 0){
            std::cout << "Enable Laser Emitter, with power = " << param.laser_power << std::endl;
            sys->setOption(RS2_OPTION_EMITTER_ENABLED,1);
            sys->setOption(RS2_OPTION_LASER_POWER, param.laser_power);

            if (param.do_alternate_laser_emitter){
                std::cout << "Do Alternating Laser Emitter" << std::endl;
                sys->setOption(RS2_OPTION_EMITTER_ON_OFF, 1);
            }
            
        }else{
            std::cout << "Disable Laser Emitter" << std::endl;
            sys->setOption(RS2_OPTION_EMITTER_ENABLED,0);
        }

        setSyncMode();
    }

    //// Configure Loop Closure
    if (param.type == "t265")
    {
        if (param.do_loop_closure){
            sys->setOption(RS2_OPTION_ENABLE_MAPPING, 1);
            sys->setOption(RS2_OPTION_ENABLE_RELOCALIZATION, 1);
            sys->setOption(RS2_OPTION_ENABLE_POSE_JUMPING, 1);
        }else{
            
            sys->setOption(RS2_OPTION_ENABLE_MAPPING, 0);
            // sys->setOption(RS2_OPTION_ENABLE_RELOCALIZATION, 0);
            // sys->setOption(RS2_OPTION_ENABLE_POSE_JUMPING, 0);    
        }
        std::cout << "Loop Closure is " << sys->getOption(RS2_OPTION_ENABLE_MAPPING) << std::endl;
    }
        

    
    
    //// Configure Auto Exposure
    if (param.auto_exposure_mode == "internal"){
        std::cout << "Enable Internal Auto Exposure" << std::endl;
        sys->enableAE(param.auto_exposure_mean_intensity_setpoint);
    }else if (param.auto_exposure_mode == "custom"){
        std::cout << "Enable Custom Auto Exposure" << std::endl;
        sys->disableAE();

        param.loadExposureControlParam(param.type);
        param.exposure_range = sys->getOptionRange(RS2_OPTION_EXPOSURE);
        param.gain_range = sys->getOptionRange(RS2_OPTION_GAIN);

        sys->setOption(RS2_OPTION_EXPOSURE, param.initial_exposure);
        sys->setOption(RS2_OPTION_GAIN, param.initial_gain);
        
        ExposureControl::Param expo_ctl_param = {param.exposure_max, (int)param.exposure_range.min, (int)param.exposure_range.step,
            param.gain_max, (int)param.gain_range.min, (int)param.gain_range.step,
            param.exposure_change_rate, param.exposure_target_mean, param.exposure_dead_region,
            param.exposure_a_min, param.exposure_a_max, param.exposure_c
        };

        expo_ctl = new ExposureControl(expo_ctl_param);
    }else if (param.auto_exposure_mode == "manual")
    {
        param.loadExposureControlParam(param.type);
        sys->disableAE();
        std::cout << "Set Fixed Exposure & Gain: " << param.initial_exposure << ", " <<  param.initial_gain << std::endl;
        sys->setOption(RS2_OPTION_EXPOSURE, param.initial_exposure);
        sys->setOption(RS2_OPTION_GAIN, param.initial_gain);
    }else{
        throw std::runtime_error("Unknown auto_exposure_mode for " + topic_ns);
    }

    //// Advertise Publishers
    //// Register Callbacks
    if (param.do_publish_stereo)
    {
        sys->enableStereoStream(param.width, param.height, param.hz);
        sys->registerCallback(std::bind(&CameraManager::setStereoFrame, this, std::placeholders::_1));

        pub_stereo = new StereoCameraPublisher(nh_local);
        pub_stats = nh_local.advertise<rs2_ros::CameraStats>("camera_stats",10);

    }

    if (param.do_publish_poseimu)
    {
        sys->enablePoseMotionStream();
        if (param.type == "t265")
        {
            sys->registerCallback(std::bind(&CameraManager::callbackSyncedIMU_t265, this, std::placeholders::_1));
            sys->registerCallback(std::bind(&CameraManager::callbackPose_t265, this, std::placeholders::_1));
            pub_pose = new PosePublisher(nh_local);
            
            double* R = nullptr;
            if (param.pose_coordinate == "VO") // from right-up-back to right-down-front
            {
                ROS_WARN("T265 is using VO right-down-front coordinates");
                R = new double[9]{1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0};
            }
            else if (param.pose_coordinate == "ROS")
            {
                ROS_WARN("T265 is using ROS NWU coordinates");
                R = new double[9]{0.0, -1.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0}; // from right-up-back to NWU
                // 0 -1 0 
                // 0 0 1
                // -1 0 0
            }
                

            if (R)
            {
                pub_pose->doStaticTransform(R);
                delete [] R;
            }
                
        }
        else if (param.type == "d400")
            sys->registerCallback(std::bind(&CameraManager::callbackSyncedIMU_d400,this, std::placeholders::_1));
        else{
            throw std::runtime_error("Unknown IMU-Camera Transformation");
        }

        pub_imu = new IMUPublisher(nh_local);
    }

    if (param.do_publish_depth)
    {
        sys->enableDepthStream(param.width, param.height, param.hz);
        sys->registerCallback(std::bind(&CameraManager::setFrame<StereoDriver::DepthDataType, MtxDepthFrame, sensor_msgs::CameraInfo>, this, std::placeholders::_1, &this->mtxDepthFrame, &this->cameraInfo_depth));

        pub_depth = new ImagePublisher(nh_local, "depth/image_rect_raw");
    }

    if (param.do_publish_color)
    {
        sys->enableColorStream(param.width, param.height, param.hz);
        sys->registerCallback(std::bind(&CameraManager::setFrame<StereoDriver::ColorDataType, MtxColorFrame, sensor_msgs::CameraInfo>, this, std::placeholders::_1, &this->mtxColorFrame, &this->cameraInfo_color));

        pub_color = new ImagePublisher(nh_local, "color/image_raw");
    }

    
    // mark for the first frame
    mtxStereoFrame.frame.left = mtxStereoFrame.frame.right = nullptr;
    mtxDepthFrame.frame.data = nullptr;
    mtxColorFrame.frame.data = nullptr;

    // bind intrinsics function
    mtxDepthFrame.get_intrinsics = std::bind(&StereoDriver::get_depth_intrinsics,sys);
    mtxColorFrame.get_intrinsics = std::bind(&StereoDriver::get_color_intrinsics,sys);
    // mtxColorFrame.get_intrinsics = sys->get_color_intrinsics; ERROR

    initialised = true;
}

CameraManager::~CameraManager()
{ 
    if(sys != nullptr)
        delete sys;
}

void CameraManager::getCameraInfo(const rs2_intrinsics& intrinsics, sensor_msgs::CameraInfo& camerainfo, float baseline)
{

    // const rs2_intrinsics intrinsics = sys->get_stereo_intrinsics();
    // float baseline = sys->get_baseline();

    camerainfo.width = intrinsics.width;
    camerainfo.height = intrinsics.height;
    //camerainfo.header.frame_id = _optical_frame_id[stream_index];
    camerainfo.K.at(0) = intrinsics.fx; // have scale here?
    camerainfo.K.at(2) = intrinsics.ppx;
    camerainfo.K.at(4) = intrinsics.fy;
    camerainfo.K.at(5) = intrinsics.ppy;
    camerainfo.K.at(8) = 1;

    camerainfo.P.at(0) = intrinsics.fx;
    camerainfo.P.at(1) = 0;
    camerainfo.P.at(2) = intrinsics.ppx;

    // camerainfo.P.at(3) = 0; // Tx, -fx * B
    // This is the translation term Tx for right camera, assuming left cam is the origin
    camerainfo.P.at(3) = - intrinsics.fx * baseline;

    camerainfo.P.at(4) = 0;
    camerainfo.P.at(5) = intrinsics.fy;
    camerainfo.P.at(6) = intrinsics.ppy;
    camerainfo.P.at(7) = 0; // Ty
    camerainfo.P.at(8) = 0;
    camerainfo.P.at(9) = 0;
    camerainfo.P.at(10) = 1;
    camerainfo.P.at(11) = 0;

    camerainfo.distortion_model = "plumb_bob";

    // set R (rotation matrix) values to identity matrix
    camerainfo.R.at(0) = 1.0;
    camerainfo.R.at(1) = 0.0;
    camerainfo.R.at(2) = 0.0;
    camerainfo.R.at(3) = 0.0;
    camerainfo.R.at(4) = 1.0;
    camerainfo.R.at(5) = 0.0;
    camerainfo.R.at(6) = 0.0;
    camerainfo.R.at(7) = 0.0;
    camerainfo.R.at(8) = 1.0;

    for (int i = 0; i < 5; i++)
    {
        camerainfo.D.push_back(intrinsics.coeffs[i]);
    }

    // cameraInfo_left = camerainfo;
    // cameraInfo_right = camerainfo;

}

void CameraManager::setSyncMode()
{

    // float frames_queue_size = sys->getOption(RS2_OPTION_FRAMES_QUEUE_SIZE);
    // std::cout << "frames_queue_size = " <<  frames_queue_size << std::endl;

    // sys->setOption(RS2_OPTION_FRAMES_QUEUE_SIZE, 32);

    if (param.hz != 30 && param.hz != 30)
    {
        // Doesn't work for framerate 6Hz
        std::runtime_error("HW Sync only works with higher frame rates");
    }
    //// Configure Hardware Synchronisation
    std::cout << param.topic_ns << ": Setting Hardware Sync Mode to " << param.do_hardware_sync  << std::endl;
    if (param.do_hardware_sync == "none"){
        sys->setOption(RS2_OPTION_INTER_CAM_SYNC_MODE, 0);
    }else if (param.do_hardware_sync == "master"){
        // sys->setOption(RS2_OPTION_FRAMES_QUEUE_SIZE,1);
        // sys->setOption(RS2_OPTION_OUTPUT_TRIGGER_ENABLED, 1);
        sys->setOption(RS2_OPTION_INTER_CAM_SYNC_MODE, 1);
    }else if (param.do_hardware_sync == "slave"){
        // sys->setOption(RS2_OPTION_OUTPUT_TRIGGER_ENABLED, 0);
        sys->setOption(RS2_OPTION_INTER_CAM_SYNC_MODE, 2);
    }else
        throw std::runtime_error("Unknown Hardware Sync Mode: " + param.do_hardware_sync);
}

template <class T, class M, class C>
void CameraManager::setFrame(const T& frame, M* mtxFrame, C* cameraInfo)
{
    // std::cerr <<  param.topic_ns << "Frame " << frame.seq_left << std::endl;
    if (mtxFrame->frame.data == nullptr)
    {
        ROS_INFO_STREAM("First " << frame.name << " Frame Received for " << param.topic_ns);
        //// Get Some Intrinsics and Extrinsics
        getCameraInfo(mtxFrame->get_intrinsics(), *cameraInfo);
    }
        

    if (mtxFrame->inProcess.try_lock()){

        if (mtxFrame->frame.data != nullptr){
            delete [] mtxFrame->frame.data;
        }

        mtxFrame->frame = frame;
        mtxFrame->inProcess.unlock();
        mtxFrame->cv.notify_one();
    }else
    {
        std::cerr << param.topic_ns << ": Missing "<< frame.name << " Frame " << mtxFrame->frame.seq << std::endl;
    }
}

// template void CameraManager::setFrame<StereoDriver::StereoDataType, CameraManager::MtxStereoFrame>(const StereoDriver::StereoDataType& frame, CameraManager::MtxStereoFrame* mtxFrame);

void CameraManager::setStereoFrame(const StereoDriver::StereoDataType& frame)
{
    // std::cerr <<  param.topic_ns << "Frame " << frame.seq_left << std::endl;
    if (mtxStereoFrame.frame.left == nullptr)
    {
        ROS_INFO_STREAM("First Frame Received for " << param.topic_ns);

        //// Get Some Intrinsics and Extrinsics
        rs2_intrinsics left, right;
        sys->get_stereo_intrinsics(left, right);
        getCameraInfo(left,cameraInfo_left);
        getCameraInfo(right,cameraInfo_right,sys->get_baseline());
    }
        

    if (mtxStereoFrame.inProcess.try_lock()){

        if (mtxStereoFrame.frame.left != nullptr){
            delete [] mtxStereoFrame.frame.left;
            delete [] mtxStereoFrame.frame.right;
        }

        mtxStereoFrame.frame = frame;
        mtxStereoFrame.inProcess.unlock();
        mtxStereoFrame.cv.notify_one();
    }else
    {
        std::cerr << param.topic_ns << ": Missing Frame " << mtxStereoFrame.frame.seq_left << std::endl;
    }
    
}

void CameraManager::callbackSyncedIMU_t265(const StereoDriver::SyncedIMUDataType& data)
{
    float gyro[3] = {-data.gx, -data.gy, data.gz}; // change of coordinates, https://github.com/IntelRealSense/librealsense/blob/master/doc/t265.md
    float accel[3] = {-data.ax, -data.ay, data.az}; // change of coordinates
    pub_imu->publish(gyro, accel, ros::Time(data.timestamp), data.seq);
}

void CameraManager::callbackPose_t265(const StereoDriver::PoseDataType& data)
{
    float position[3] = {data.pose.translation.x, data.pose.translation.y, data.pose.translation.z};
    float orientation[4] = {data.pose.rotation.w, data.pose.rotation.x, data.pose.rotation.y, data.pose.rotation.z};

    pub_pose->publish(position, orientation, "map", ros::Time(data.timestamp), data.seq);
}

void CameraManager::callbackSyncedIMU_d400(const StereoDriver::SyncedIMUDataType& data)
{
    float gyro[3] = {data.gx, data.gy, data.gz}; // change of coordinates, https://github.com/IntelRealSense/librealsense/blob/master/doc/t265.md
    float accel[3] = {data.ax, data.ay, data.az}; // change of coordinates
    pub_imu->publish(gyro, accel, ros::Time(data.timestamp), data.seq);
}

void CameraManager::processFrame()
{
    std::vector<std::thread> process_list;
    if (param.do_publish_stereo)
        process_list.push_back(std::thread(&CameraManager::processStereoFrame, this));
    if (param.do_publish_depth)
        process_list.push_back(std::thread(&CameraManager::processDepthFrame, this));
    if (param.do_publish_color)
        process_list.push_back(std::thread(&CameraManager::processColorFrame, this));

    for (auto& process : process_list)
        process.join();
    
}

void CameraManager::processStereoFrame()
{
    std::cout << param.topic_ns << "Stereo process starts..." << std::endl;

    auto& frame = mtxStereoFrame.frame;

    while(ros::ok()){
        std::unique_lock<std::mutex> lk(mtxStereoFrame.inProcess); // this call also locks the thread, with blocking behaviour
        auto ret = mtxStereoFrame.cv.wait_for(lk,std::chrono::seconds(2)); // with ~0.03ms delay, lock reacquired

        if (ret == std::cv_status::timeout ){
            std::cerr << param.topic_ns << ": Wait timeout for new Stereo frame arrival..." << std::endl;
            continue;
        }

        assert(frame.is_published == false);
        frame.is_published = true;

        // ROS_INFO_STREAM_THROTTLE(1, param.topic_ns << " " << frame.seq_left);

        ros::Time timestamp;
        timestamp.fromSec(frame.time_left);

        int cv_encoding = CV_8UC1;
        std::string encoding;

        if (frame.bpp == 1)
        {
            cv_encoding = CV_8UC1;
            encoding = sensor_msgs::image_encodings::TYPE_8UC1;
        }  
        else if (frame.bpp == 2)
        {
            cv_encoding = CV_16UC1;
            encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        }
        
        auto left = cv::Mat(cv::Size(frame.width, frame.height), cv_encoding, frame.left, cv::Mat::AUTO_STEP);
        auto right = cv::Mat(cv::Size(frame.width, frame.height), cv_encoding, frame.right, cv::Mat::AUTO_STEP);

        pub_stereo->publish(left, right, encoding, cameraInfo_left, cameraInfo_right, timestamp, frame.seq_left);

        // workaround for T265
        if (frame.gain == -1)
            ROS_WARN_THROTTLE(30,"T265 does not support RS2_FRAME_METADATA_GAIN_LEVEL yet, so no gain metadata available");

        rs2_ros::CameraStats stats_msg;
        stats_msg.header.stamp = timestamp;
        stats_msg.exposure = frame.exposure;
        stats_msg.gain = frame.gain;
        stats_msg.meanLux = 0;

        if (param.auto_exposure_mode == "custom" )
        {
            expo_ctl->calcHistogram(left,frame.exposure, frame.gain);
            stats_msg.meanLux = expo_ctl->EstimateMeanLuminance();
            int exposure_next = frame.exposure;
            int gain_next = frame.gain;

            if (frame.seq_left % 2)
            {
                expo_ctl->updateExposureGain(stats_msg.meanLux, frame.exposure, frame.gain, exposure_next, gain_next, false);
                exposure_next = std::round(exposure_next/param.exposure_range.step)*param.exposure_range.step;
                gain_next = std::round(gain_next/param.gain_range.step)*param.gain_range.step;
            }
            // std::cout << "update exposure=" << exposure_next << ", gain=" << gain_next << std::endl;

            if (exposure_next != frame.exposure) sys->setOption(RS2_OPTION_EXPOSURE, exposure_next);
            if (gain_next != frame.gain) sys->setOption(RS2_OPTION_GAIN, gain_next);
        }else if (param.auto_exposure_mode == "manual")
        {
            ros::NodeHandle nh_local("~/" + param.topic_ns);
            int param_exposure, param_gain;

            nh_local.getParam("exposure",param_exposure);
            nh_local.getParam("gain",param_gain);

            if (frame.gain == -1)
                frame.gain = param_gain;

            if (param_exposure != frame.exposure || param_gain != frame.gain)
            {
                ROS_INFO_STREAM_THROTTLE(0.5, "New settings: exposure=" << param_exposure << " , gain=" << param_gain );
                sys->setOption(RS2_OPTION_EXPOSURE, param_exposure);
                sys->setOption(RS2_OPTION_GAIN, param_gain);
               
            }
        }
        
        pub_stats.publish(stats_msg);
        
        // std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    std::cout << param.topic_ns << "Stereo process ends..." << std::endl;
}

void CameraManager::processDepthFrame()
{
    std::cout << param.topic_ns << "Depth process starts..." << std::endl;

    auto& frame = mtxDepthFrame.frame;

    while(ros::ok()){
        std::unique_lock<std::mutex> lk(mtxDepthFrame.inProcess); // this call also locks the thread, with blocking behaviour
        auto ret = mtxDepthFrame.cv.wait_for(lk,std::chrono::seconds(2)); // with ~0.03ms delay, lock reacquired

        if (ret == std::cv_status::timeout ){
            std::cerr << param.topic_ns << ": Wait timeout for new Depth frame arrival..." << std::endl;
            continue;
        }

        ros::Time timestamp;
        timestamp.fromSec(frame.time);

        assert(frame.bpp == 2); // 16-bit depth

        auto depth_cv = cv::Mat(cv::Size(frame.width, frame.height), CV_16UC1, frame.data, cv::Mat::AUTO_STEP);

        // "mono16" and "16UC1" gives different result??
        pub_depth->publish(depth_cv, sensor_msgs::image_encodings::TYPE_16UC1 , cameraInfo_depth, timestamp, frame.seq);

    }
    std::cout << param.topic_ns << "Depth process ends..." << std::endl;
}

void CameraManager::processColorFrame()
{
    std::cout << param.topic_ns << "Color process starts..." << std::endl;

    auto& frame = mtxColorFrame.frame;

    while(ros::ok()){
        std::unique_lock<std::mutex> lk(mtxColorFrame.inProcess); // this call also locks the thread, with blocking behaviour
        auto ret = mtxColorFrame.cv.wait_for(lk,std::chrono::seconds(2)); // with ~0.03ms delay, lock reacquired

        if (ret == std::cv_status::timeout ){
            std::cerr << param.topic_ns << ": Wait timeout for new Color frame arrival..." << std::endl;
            continue;
        }

        auto color_cv = cv::Mat(cv::Size(frame.width, frame.height), CV_8UC3, frame.data, cv::Mat::AUTO_STEP);

        ros::Time timestamp;
        timestamp.fromSec(frame.time);

        pub_color->publish(color_cv, sensor_msgs::image_encodings::RGB8, cameraInfo_color, timestamp, frame.seq);
    }
    std::cout << param.topic_ns << "Color process ends..." << std::endl;
}

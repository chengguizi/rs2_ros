#include "camera_manager.hpp"
#include "exposure_ctl.hpp"

#include <memory>
#include <rs2_ros/CameraStats.h>


std::vector<std::string> CameraParam::camera_list;

std::vector<std::string>  CameraParam::loadCameras()
{
    ros::NodeHandle nh_local("~");
    // std::cout << nh_local.getNamespace() << std::endl;
    nh_local.getParam("devices", camera_list);

    return camera_list;
}

void CameraParam::loadParam(const std::string& topic_ns)
{
    this->topic_ns = topic_ns;
    ros::NodeHandle nh_local("~/" + topic_ns);

    // Obtain Device Specific Params
    nh_local.getParam("type",type);
    nh_local.getParam("sn",camera_sn);

    nh_local.getParam("stereo", do_publish_stereo);
    nh_local.getParam("depth", do_publish_depth);
    nh_local.getParam("poseimu", do_publish_poseimu);
    nh_local.getParam("auto_exposure_mode", auto_exposure_mode); // internal, or custom

    // Obtain Type Specific Params
    ros::NodeHandle nh_type("~/" + type);
    nh_type.getParam("width",width);
    nh_type.getParam("height",height);
    nh_type.getParam("frame_rate",hz);
    nh_type.getParam("laser_power",laser_power);

}

void CameraParam::loadExposureControlParam(const std::string& type)
{
    ros::NodeHandle nh_type("~/" + type);

    nh_type.getParam("initial_exposure",initial_exposure);
    nh_type.getParam("initial_gain",initial_gain);
    nh_type.getParam("exposure_max",exposure_max);
    nh_type.getParam("exposure_change_rate",exposure_change_rate);
    nh_type.getParam("exposure_target_mean",exposure_target_mean);
    nh_type.getParam("exposure_dead_region",exposure_dead_region);
    nh_type.getParam("exposure_a_max",exposure_a_max);
    nh_type.getParam("exposure_a_min",exposure_a_min);
    nh_type.getParam("exposure_c",exposure_c);

}

CameraManager::CameraManager(const std::string& topic_ns)
{
    std::cout << "Creating CameraManager for " << topic_ns << std::endl;
    //// Obtain all params associated with the topic_ns
    param.loadParam(topic_ns);

    ros::NodeHandle nh_local("~/" + topic_ns);

    //// Creating Device Driver
    sys = new StereoDriver(param.camera_sn);
    
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
        
        ExposureControl::Param expo_ctl_param = {(int)param.exposure_range.max, (int)param.exposure_range.min, (int)param.exposure_range.step,
            (int)param.gain_range.max, (int)param.gain_range.min, (int)param.gain_range.step,
            param.exposure_change_rate, param.exposure_target_mean, param.exposure_dead_region,
            param.exposure_a_min, param.exposure_a_max, param.exposure_c
        };

        expo_ctl = new ExposureControl(expo_ctl_param);
    }else if (param.auto_exposure_mode == "manual")
    {
        param.loadExposureControlParam(param.type);
        std::cout << "Set Fixed Exposure & Gain: " << param.initial_exposure << ", " <<  param.initial_gain << std::endl;
        sys->disableAE();
        sys->setOption(RS2_OPTION_EXPOSURE, param.initial_exposure);
        sys->setOption(RS2_OPTION_GAIN, param.initial_gain);
    }else{
        std::cerr << "Unknown auto_exposure_mode for " << topic_ns << std::endl;
        exit(-1);
    }

    //// Advertise Publishers
    //// Register Callbacks
    if (param.do_publish_stereo)
    {
        sys->enableStereoStream(param.width, param.height, param.hz);
        sys->registerCallback(std::bind(&CameraManager::setStereoFrame, this, std::placeholders::_1));

        pub = new StereoCameraPublisher(topic_ns);
        pub_stats = nh_local.advertise<rs2_ros::CameraStats>("camera_stats",10);
    }

    if (param.do_publish_poseimu)
    {
        sys->enablePoseMotionStream();
        if (param.type == "t265")
            sys->registerCallback(std::bind(&CameraManager::callbackSyncedIMU_t265,this, std::placeholders::_1));
        else if (param.type == "d400")
            sys->registerCallback(std::bind(&CameraManager::callbackSyncedIMU_d400,this, std::placeholders::_1));
        else{
            std::cerr << "Unknown IMU-Camera Transformation" << std::endl;
            exit(-1);
        }

        pub_imu = new IMUPublisher(topic_ns);
    }

    //// Get Some Intrinsics and Extrinsics
    getCameraInfo();

    frame.left = nullptr;
}

CameraManager::~CameraManager()
{ 
    if(sys != nullptr)
        delete sys;
}

void CameraManager::getCameraInfo()
{
    sensor_msgs::CameraInfo camerainfo;

    rs2_intrinsics intrinsics = sys->get_intrinsics();
    float baseline = sys->get_baseline();

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
    camerainfo.P.at(3) = 0; // Tx, -fx * B
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

    cameraInfo_left = camerainfo;
    cameraInfo_right = camerainfo;

    // This is the translation term Tx for right camera, assuming left cam is the origin
    cameraInfo_right.P.at(3) = - intrinsics.fx * baseline;
}

void CameraManager::setStereoFrame(const StereoDriver::StereoDataType& frame)
{
    if (this->frame.left == nullptr) 
        std::cout << "First Frame Received for " << param.topic_ns << std::endl;
        
    if (inProcess.try_lock()){

        if (this->frame.left != nullptr){
            delete [] this->frame.left;
            delete [] this->frame.right;
        }

        this->frame = frame;
        inProcess.unlock();
        cv.notify_one();
    }else
    {
        std::cerr << "Missing Frame " << frame.seq_left << std::endl;
    }
    
}

void CameraManager::callbackSyncedIMU_t265(const StereoDriver::SyncedIMUDataType& data)
{
    float gyro[3] = {-data.gx, -data.gy, data.gz}; // change of coordinates, https://github.com/IntelRealSense/librealsense/blob/master/doc/t265.md
    float accel[3] = {-data.ax, -data.ay, data.az}; // change of coordinates
    pub_imu->publish(gyro, accel, ros::Time(data.timestamp), data.seq);
}

void CameraManager::callbackSyncedIMU_d400(const StereoDriver::SyncedIMUDataType& data)
{
    float gyro[3] = {data.gx, data.gy, data.gz}; // change of coordinates, https://github.com/IntelRealSense/librealsense/blob/master/doc/t265.md
    float accel[3] = {data.ax, data.ay, data.az}; // change of coordinates
    pub_imu->publish(gyro, accel, ros::Time(data.timestamp), data.seq);
}

void CameraManager::processFrame()
{
    std::cout << param.topic_ns << " process starts..." << std::endl;

    while(ros::ok()){
        std::unique_lock<std::mutex> lk(inProcess); // this call also locks the thread, with blocking behaviour
        auto ret = cv.wait_for(lk,std::chrono::seconds(5)); // with ~0.03ms delay, lock reacquired
        
        if (ret == std::cv_status::timeout ){
            std::cerr << param.topic_ns << ": Wait timeout for new frame arrival. Exiting" << std::endl;
            return;
        }
        auto left = cv::Mat(cv::Size(frame.width, frame.height), CV_8UC1, frame.left, cv::Mat::AUTO_STEP);
        auto right = cv::Mat(cv::Size(frame.width, frame.height), CV_8UC1, frame.right, cv::Mat::AUTO_STEP);

        ros::Time timestamp;
        timestamp.fromSec(frame.time_left);

        pub->publish(left, right, cameraInfo_left, cameraInfo_right, timestamp, frame.seq_left);
        
        // std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    std::cout << param.topic_ns << " process ends..." << std::endl;
}


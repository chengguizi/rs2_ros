// ROS Node for Realsense D415 Streams
// Cheng Huimin, June 2018
//
// The dedicated Realsense interface to publish ROS topics, from the infrared stereo streams

#include <iostream>
#include <iomanip>
#include <deque>
#include <numeric>

#include <csignal>

#include <thread>
#include <chrono>

#include <mutex>
#include <condition_variable>

#include <rs2_interface/stereo_interface.hpp>

#include <opencv2/opencv.hpp>   // Include OpenCV API

#include <ros/ros.h>

#include <rs2_ros/CameraStats.h>

#include "ros_publisher.hpp"
#include "exposure_ctl.hpp"

#include <fstream>
#include <sstream>

struct stereo_frame_t{
    cv::Mat left;
    cv::Mat right;
    uint64_t t = 0; // epoch time (system time)
    // uint64_t t_base = 0;
    uint64_t t_callback;
    uint64_t seq;
    std::mutex inProcess;
    std::condition_variable cv;
}stereo_frame;

std::ostringstream streamout;

// from inner process loop to triggering this callback function takes around 0.2-0.4ms, tested
void stereoImageCallback(StereoDriver::StereoDataType data) // irleft and irright are in the heap, must be deleted after use
{
    // std::cout << "Frame: " << seqleft << std::endl;
    
    if (data.time_left != data.time_right){
        ROS_WARN_STREAM( "ImageCallback(): stereo time sync inconsistent!" );
        return;
    }
            
        
    if (data.seq_left != data.seq_right){
        ROS_WARN_STREAM( "ImageCallback(): stereo frame sequence sync inconsistent!" );
        return;
    }
    
    if ( stereo_frame.inProcess.try_lock())
    { 
        stereo_frame.seq = data.seq_left;
        // stereo_frame.t = tleft;
        stereo_frame.t_callback = ros::Time::now().toNSec(); //std::chrono::system_clock::now().time_since_epoch().count();

        if (stereo_frame.t == 0)
        {
            ROS_WARN("RealSense: First frame successfully captured!");
        }
        else
        {
            //ROS_INFO_STREAM("ImageCallback(): deleting " << seqleft);
            delete[] stereo_frame.left.data; // will cause memory leak if this is not freed
            delete[] stereo_frame.right.data;
            //ROS_INFO_STREAM("ImageCallback(): deleted " << seqleft);
        }
        
        stereo_frame.left = cv::Mat(cv::Size(data.width, data.height), CV_8UC1, data.left, cv::Mat::AUTO_STEP);    
        stereo_frame.right = cv::Mat(cv::Size(data.width, data.height), CV_8UC1, data.right, cv::Mat::AUTO_STEP);
        stereo_frame.t = data.time_left * 1e9;

        stereo_frame.inProcess.unlock();
        stereo_frame.cv.notify_one();

    }else{
        ROS_WARN_STREAM( "Missed Frame(" << data.seq_left << ")" );
        return;
    }

    // std::cout << (uint64_t) (data.time_left * 1e9) << ",  " << data.mid_shutter_time_estimate << std::endl;

    streamout << (stereo_frame.t_callback - data.mid_shutter_time_estimate)/1.0e6 << " " << std::fixed <<data.mid_shutter_time_estimate/1.0e6 << std::endl;
}

void gyroCallback(StereoDriver::GyroDataType data){
    std::cout << data.seq <<" gyro = " << (uint64_t)(data.timestamp * 1e9) << std::endl;
}

void accelCallback(StereoDriver::AccelDataType data){
    std::cout << data.seq << " acce = " << (uint64_t)(data.timestamp * 1e9) << std::endl;
}

void getCameraInfo(rs2_intrinsics intrinsics, float baseline, sensor_msgs::CameraInfo& left, sensor_msgs::CameraInfo& right)
{
    sensor_msgs::CameraInfo camerainfo;


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

    left = camerainfo;
    right = camerainfo;

    // This is the translation term Tx for right camera, assuming left cam is the origin
    right.P.at(3) = - intrinsics.fx * baseline;

}


// void signalHandler(int signum)
// {
//     std::cout << strsignal(signum) << " Signal is received! Terminating RealSense Node..." << std::endl;
//     ros::shutdown();
//     delete sys;
//     exit(signum);
// }

void scaleFrames(cv::Mat &leftImg, cv::Mat &rightImg, int min, int max)
{
    static double a=1, b=0;
    const double update_rate = 0.1;
    const double max_alpha = 5;

    // calculate current frame alpha and beta
    double a_now, b_now;
    ROS_ASSERT(max >= min);
    a_now = std::min( max_alpha*2, 256.0 / (double)(max-min+1));
    a = std::min (update_rate*a_now + (1 - update_rate)*a, max_alpha);

    b_now = -a*min;
    b = update_rate*b_now + (1 - update_rate)*b;


    leftImg.convertTo(leftImg,-1,a,b);
    rightImg.convertTo(rightImg,-1,a,b);
}

int main(int argc, char * argv[]) try
{

    // ros initialisation
    ros::init(argc, argv, "rs2_ros");
    ros::NodeHandle nh;
    ros::NodeHandle local_nh("~");

    ros::Publisher _camstats_pub = local_nh.advertise<rs2_ros::CameraStats>("camera_stats",10);

    int w,h,hz;
    int exposure,gain,laser_power;
    bool auto_exposure_internal,auto_exposure_custom;
    int mean_intensity_setpoint;
    bool _visualisation_on;
    bool brighten_dark_image;

    double exposure_change_rate;
    int target_mean;
    int dead_region;

    // Exposure control param
    double a_min, a_max, c;

    std::string sensor_name;

    local_nh.param<std::string>("sensor_name", sensor_name, "RealSense");
    local_nh.param("width", w,1280);
    local_nh.param("height",h,720);
    local_nh.param("frame_rate",hz,30);
    local_nh.param("exposure",exposure,20000);
    local_nh.param("auto_exposure_internal",auto_exposure_internal,false);
    local_nh.param("auto_exposure_custom",auto_exposure_custom,false);
    local_nh.param("mean_intensity_setpoint",mean_intensity_setpoint,1536);
    local_nh.param("gain",gain,40);
    local_nh.param("laser_power",laser_power,150);

    local_nh.param("exposure_change_rate",exposure_change_rate,1.0);
    local_nh.param("target_mean",target_mean,110);
    local_nh.param("dead_region",dead_region,5);

    local_nh.param("a_min",a_min,0.15);
    local_nh.param("a_max",a_max,0.85);
    local_nh.param("c",c,0.5);

    
    local_nh.param("brighten_dark_image",brighten_dark_image,false);
    local_nh.param("visualisation_on",_visualisation_on,false);



    StereoDriver* sys = new StereoDriver(sensor_name, laser_power);

    // while (ros::ok()){
    //     std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    //     ROS_INFO("Waiting");
    // }

    if (laser_power == 0)
        sys->setOption(RS2_OPTION_EMITTER_ENABLED,0);

    // for more options, please refer rs_option.h
    if (auto_exposure_internal)
        sys->enableAE( static_cast<uint32_t>(mean_intensity_setpoint) );
        
    else
    {
        sys->disableAE();
        sys->setOption(RS2_OPTION_EXPOSURE,exposure); // in usec
        sys->setOption(RS2_OPTION_GAIN,gain);

        ROS_ASSERT(sys->getOption(RS2_OPTION_EXPOSURE) ==  exposure);
        ROS_ASSERT(sys->getOption(RS2_OPTION_GAIN) ==  gain);
    }
    // ROS_ASSERT (sys->getOption(RS2_OPTION_ENABLE_AUTO_EXPOSURE) == auto_exposure_internal);

    ROS_INFO_STREAM("Realsense: Initial shutter speed= 1/" << 1/(exposure*1e-6) << ", gain= " << gain );

    // const auto window_name_l = "Display Image Left";
    // const auto window_name_r = "Display Image Right";
    //cv::namedWindow(window_name_l, cv::WINDOW_AUTOSIZE);
    //cv::namedWindow(window_name_r, cv::WINDOW_AUTOSIZE);

    //////////////////////////////////
    //// Empirical Latency Test
    //////////////////////////////////
    
    {
        ROS_INFO("Start Empirical Latency Test");
        const int N_test = 100;
        auto t1 = ros::Time::now();
        for (int i=0; i<N_test; i++){
            auto exposure = sys->getOption(RS2_OPTION_EXPOSURE);
            (void) exposure;
        }
        auto t2 = ros::Time::now();
        double latency = (t2-t1).toSec()/N_test/2;
        ROS_INFO_STREAM("One-way Latency = " << latency*1000 << "ms");
    }
    

    /////////// Test End /////////////
    

    sys->registerCallback(stereoImageCallback);
    // sys->registerCallback(gyroCallback);
    // sys->registerCallback(accelCallback);
    

    StereoCameraPublisher pub(local_nh); // start with private scope
    IMUPublisher pub_imu(local_nh);

    const std::string device_name = sys->getDeviceName();
    if (device_name.find("T265") != std::string::npos)
    {
        // for Realsense T265
        auto lambda = [&pub_imu](StereoDriver::SyncedIMUDataType data){
            float gyro[3] = {-data.gx, -data.gy, data.gz}; // change of coordinates, https://github.com/IntelRealSense/librealsense/blob/master/doc/t265.md
            float accel[3] = {-data.ax, -data.ay, data.az}; // change of coordinates
            pub_imu.publish(gyro, accel, ros::Time(data.timestamp), data.seq);
        };
        sys->registerCallback(lambda);
    }else if(device_name.find("D435I") != std::string::npos){
        // for Realsense D435i
        auto lambda = [&pub_imu](StereoDriver::SyncedIMUDataType data){
            float gyro[3] = {data.gx, data.gy, data.gz}; // change of coordinates, https://github.com/IntelRealSense/librealsense/blob/master/doc/t265.md
            float accel[3] = {data.ax, data.ay, data.az}; // change of coordinates
            pub_imu.publish(gyro, accel, ros::Time(data.timestamp), data.seq);
        };
        sys->registerCallback(lambda);
    }

    //signal(SIGINT, signalHandler);

    //// Start RealSense Pipe
    sys->enablePoseMotionStream();

    // If available, enable global time
    sys->setOption(RS2_OPTION_GLOBAL_TIME_ENABLED,1);
    std::cout << "GLOBAL TIME ENABLED = " << sys->getOption(RS2_OPTION_GLOBAL_TIME_ENABLED) << std::endl;

    sys->startStereoPipe(w, h, hz, RS2_FORMAT_Y8);

    sensor_msgs::CameraInfo _cameraInfo_left, _cameraInfo_right;
    getCameraInfo( sys->get_intrinsics(), sys->get_baseline(), _cameraInfo_left, _cameraInfo_right);


    ExposureControl exposureCtl(0.15,0.8,0.6);
    const auto exposure_range =  sys->getOptionRange(RS2_OPTION_EXPOSURE);
    const auto gain_range = sys->getOptionRange(RS2_OPTION_GAIN);
    int max_exposure; // ~100Hz
    local_nh.param("max_exposure",max_exposure,10000);
    const int min_exposure = exposure_range.min; // == 20 us or 1/50000
    const int step_expo = exposure_range.step; // == 20
    // std::cout << "step_expo=" << step_expo << std::endl;
    const int max_gain = gain_range.max;
    const int min_gain = gain_range.min;
    const int step_gain = gain_range.step; // == 1

    
    struct SettingFilter{
        int expo;
        int gain;
    };

    SettingFilter settingFilter = {exposure,gain};

    ros::AsyncSpinner spinner(2);
	spinner.start();

    uint frame_idx = 0;

    bool error_exit = false;

    std::ofstream fout;
    fout.open("rs2_driver_jitter.txt");

    while (ros::ok())
    {
        static uint64_t total_count = 0;
        static double min_duration = 1, max_duration = 0 , moving_avg_duration = 0.0;
        static ros::Time last_stamp = ros::Time(0);

        std::unique_lock<std::mutex> lk(stereo_frame.inProcess);
        auto ret = stereo_frame.cv.wait_for(lk,std::chrono::seconds(1)); // with ~0.03ms delay

        if (ret == std::cv_status::timeout){
            ROS_ERROR("Realsense: Wait timeout for new frame arrival. Exiting");
            error_exit = true;
            break;
        }
        
        if (frame_idx != stereo_frame.seq)
        {   
            // Update the window with new data
            //cv::imshow(window_name_l, stereo_frame.left);
            //cv::imshow(window_name_r, stereo_frame.right);

            ros::Time sensor_timestamp; 
            sensor_timestamp.fromNSec(stereo_frame.t);

            ////// Report Jitter /////////////

            if (!last_stamp.isZero()){
                
                double delta_t = (sensor_timestamp - last_stamp).toSec();

                if (delta_t > max_duration)
                    max_duration = delta_t;
                if (delta_t < min_duration)
                    min_duration = delta_t;
                if (moving_avg_duration == 0.0)
                    moving_avg_duration = delta_t;
                else
                    moving_avg_duration = 0.98*moving_avg_duration + 0.02*delta_t;
                
                ROS_INFO_STREAM_THROTTLE(30,"min_duration=" << std::fixed << std::setprecision(2) << min_duration*1000 << "ms, " 
                    << "moving_avg=" << moving_avg_duration*1000 << "ms, max_duration=" << max_duration*1000 << "ms");
                
                double jitter = delta_t - moving_avg_duration;
                
                if ( std::abs(jitter/moving_avg_duration) > 0.3 && total_count > 100){// 30% deviation
                    ROS_ERROR_STREAM("Jitter Detected: avg_gap=" << moving_avg_duration << ", but now=" << delta_t);
                } 
                
            }
            
            last_stamp = sensor_timestamp;
            //////////////////////////////////

            

            exposureCtl.calcHistogram(stereo_frame.left,exposure,gain);

            if (!auto_exposure_custom && auto_exposure_custom && brighten_dark_image)
            {
                int min, max;
                exposureCtl.getIntensityRange(min,max);
                // ROS_INFO_STREAM_THROTTLE(1,"min=" << min  << ",max=" << max);
                scaleFrames(stereo_frame.left, stereo_frame.right, min, max);
            }

            pub.publish(stereo_frame.left, stereo_frame.right, _cameraInfo_left, _cameraInfo_right, sensor_timestamp, stereo_frame.seq);

            int meanLux = exposureCtl.EstimateMeanLuminance();
            // if (_visualisation_on)
                // exposureCtl.showHistogram(exposure, gain);

            // publish statistics
            rs2_ros::CameraStats stats_msg;
            stats_msg.header.stamp = sensor_timestamp;
            stats_msg.exposure = exposure;
            stats_msg.gain = gain;
            stats_msg.meanLux = meanLux;
            _camstats_pub.publish(stats_msg);

            if (!auto_exposure_internal && auto_exposure_custom && stereo_frame.seq%2) // only process half of the frames, give some delays
            {
                int exposure_target = exposure;
                int gain_target = gain;

                if (meanLux < target_mean - dead_region) // image too dark
                {
                    int margin = (target_mean - dead_region) - meanLux;
                    // Consider Exposure first
                    if (exposure < max_exposure)
                    {
                        double calc_exposure = (margin/256.0 * exposure_change_rate + 1)*exposure;
                        exposure_target = std::min(max_exposure, (int)calc_exposure );
                    }
                    else if(gain  <  max_gain)
                    {
                        gain_target = std::min(max_gain, gain + 2*margin);
                    }
                }else if (meanLux > target_mean + dead_region) // image too bight
                {
                    int margin = meanLux - (target_mean + dead_region);
                    // Consider Gain first
                    if (gain > 160 /*good default*/)
                    {
                        gain_target= std::max(160, gain - 2*margin);
                    }
                    else if(exposure > 8000 /*good default*/)
                    {
                        double calc_exposure = (-margin/256.0 * exposure_change_rate + 1)*exposure;
                        exposure_target = std::max(8000, (int)calc_exposure );
                    }
                    else if (gain > min_gain)
                    {
                        gain_target = std::max(min_gain, gain - 2*margin);
                    }
                    else if (exposure > min_exposure)
                    {
                        double calc_exposure = (-margin/256.0 * exposure_change_rate + 1)*exposure;
                        exposure_target = std::max(min_exposure, (int)calc_exposure);
                        
                    }
                }

                // detect big jump
                const double exposure_jump = 0.5;
                const double gain_jump = 0.5;
                if ( std::abs(exposure_target - exposure)/ (double)exposure > exposure_jump 
                        || std::abs(gain_target - gain) / (double)gain >  gain_jump )
                {
                    const double speed = 0.5; // 0 to 1
                    settingFilter.expo = exposure_target*speed + settingFilter.expo*(1-speed);
                    settingFilter.gain = gain_target*speed + settingFilter.gain*(1-speed);

                    
                }else
                {
                    settingFilter.expo = exposure_target;
                    settingFilter.gain = gain_target;
                }

                //rounding
                settingFilter.expo = std::round(settingFilter.expo/step_expo)*step_expo;
                settingFilter.gain = std::round(settingFilter.gain/step_gain)*step_gain;
                

                if (settingFilter.expo != exposure)
                {
                    exposure = settingFilter.expo;
                    sys->setOption(RS2_OPTION_EXPOSURE,exposure);
                }

                if (settingFilter.gain != gain)
                {
                    gain = settingFilter.gain;
                    sys->setOption(RS2_OPTION_GAIN,gain);
                }

                // std::cout << "exposure: " << exposure << ", gain= " << gain << std::endl;

            }

            //cvWaitKey(1); // ~15ms
            frame_idx = stereo_frame.seq;

            total_count++;
        }
        lk.unlock();
        // ros::spinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::cout << "Shutting Down Signal Received..." << std::endl;
    delete sys;

    std::cout << "Writing to file..." << std::endl;
    fout << "jitter-in-millisecond-reference-to-uvc-clock" << std::endl << streamout.str();
    fout.close();

    if (error_exit)
        exit(-1);

    std::cout << "main() exits" << std::endl;

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    ROS_ERROR_STREAM( "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() );
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    ROS_ERROR_STREAM( e.what() );
    return EXIT_FAILURE;
}

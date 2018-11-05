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

#include <rs2_interface/irstereo_interface.hpp>

#include <opencv2/opencv.hpp>   // Include OpenCV API

#include <ros/ros.h>

#include <rs2_ros/CameraStats.h>

#include "ros_publisher.hpp"
#include "exposure_ctl.hpp"

#include <fstream>
#include <sstream>

struct irframe_t{
    cv::Mat left;
    cv::Mat right;
    uint64_t t; // epoch time (system time)
    uint64_t t_base;
    uint64_t t_callback;
    uint64_t seq;
    std::mutex inProcess;
    std::condition_variable cv;
}irframe;

std::ostringstream streamout;

// from inner process loop to triggering this callback function takes around 0.2-0.4ms, tested
void stereoImageCallback(uint64_t t_sensor , void* irleft, void* irright, const int w, const int h, \
    double tleft, double tright, uint64_t seqleft, uint64_t seqright) // irleft and irright are in the heap, must be deleted after use
{
    std::cout << "Frame: " << seqleft << std::endl;
    if ( irframe.inProcess.try_lock())
    {
        irframe.t_callback = std::chrono::system_clock::now().time_since_epoch().count();

        if (tleft != tright)
            ROS_WARN_STREAM( "ImageCallback(): stereo time sync inconsistent!" );
        irframe.t = tleft;
        if (seqleft != seqright)
            ROS_WARN_STREAM( "ImageCallback(): stereo frame sequence sync inconsistent!" );
        irframe.seq = seqleft;

        if (seqleft == 0)
        {
            irframe.t_base = t_sensor;
            ROS_WARN("RealSense: First frame successfully captured!");
        }
        else
        {
            //ROS_INFO_STREAM("ImageCallback(): deleting " << seqleft);
            delete[] irframe.left.data; // will cause memory leak if this is not freed
            delete[] irframe.right.data;
            //ROS_INFO_STREAM("ImageCallback(): deleted " << seqleft);
        }
        
        irframe.left = cv::Mat(cv::Size(w, h), CV_8UC1, irleft, cv::Mat::AUTO_STEP);    
        irframe.right = cv::Mat(cv::Size(w, h), CV_8UC1, irright, cv::Mat::AUTO_STEP);
        irframe.t = t_sensor;

        irframe.inProcess.unlock();

        irframe.cv.notify_one();

    }else
    {
        ROS_WARN_STREAM( "Missed Frame(" << seqleft << ")" );
    }

    streamout << (irframe.t_callback - t_sensor)/1.0e6 << " " << std::fixed <<t_sensor/1.0e6 << std::endl;
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

int main(int argc, char * argv[]) try
{

    // ros initialisation
    ros::init(argc, argv, "rs2_ros");
    ros::NodeHandle nh;
    ros::NodeHandle local_nh("~");

    ros::Publisher _camstats_pub = local_nh.advertise<rs2_ros::CameraStats>("camera_stats",10);

    int w,h,hz;
    int exposure,gain,laser_power;
    bool auto_exposure;
    bool _visualisation_on;
    local_nh.param("width", w,1280);
    local_nh.param("height",h,720);
    local_nh.param("frame_rate",hz,30);
    local_nh.param("exposure",exposure,20000);
    local_nh.param("auto_exposure",auto_exposure,false);
    local_nh.param("gain",gain,40);
    local_nh.param("laser_power",laser_power,150);

    local_nh.param("visualisation_on",_visualisation_on,false);

    IrStereoDriver* sys = new IrStereoDriver("RealSense D415",laser_power);

    // for more options, please refer rs_option.h
    if (auto_exposure)
        sys->setOption(RS2_OPTION_ENABLE_AUTO_EXPOSURE,1);
    else
    {
        sys->setOption(RS2_OPTION_ENABLE_AUTO_EXPOSURE,0);
        sys->setOption(RS2_OPTION_EXPOSURE,exposure); // in usec
        sys->setOption(RS2_OPTION_GAIN,gain);

        ROS_ASSERT(sys->getOption(RS2_OPTION_EXPOSURE) ==  exposure);
        ROS_ASSERT(sys->getOption(RS2_OPTION_GAIN) ==  gain);
    }

    ROS_ASSERT (sys->getOption(RS2_OPTION_ENABLE_AUTO_EXPOSURE) == auto_exposure);

    std::cout << "shutter speed= 1/" << 1/(exposure*1e-6) << ", gain= " << gain << std::endl;

    const auto window_name_l = "Display Image Left";
    const auto window_name_r = "Display Image Right";
    //cv::namedWindow(window_name_l, cv::WINDOW_AUTOSIZE);
    //cv::namedWindow(window_name_r, cv::WINDOW_AUTOSIZE);

    sys->registerCallback(stereoImageCallback);


    StereoCameraPublisher pub(local_nh); // start with private scope

    //signal(SIGINT, signalHandler);

    sys->startPipe(w,h,hz);

    sensor_msgs::CameraInfo _cameraInfo_left, _cameraInfo_right;
    getCameraInfo( sys->get_intrinsics(), sys->get_baseline(), _cameraInfo_left, _cameraInfo_right);


    ExposureControl exposureCtl;
    const auto exposure_range =  sys->getOptionRange(RS2_OPTION_EXPOSURE);
    const auto gain_range = sys->getOptionRange(RS2_OPTION_GAIN);
    const int max_exposure = 10000; // ~100Hz
    const int min_exposure = exposure_range.min; // == 20 us or 1/50000
    const int step_expo = exposure_range.step; // == 20
    std::cout << "step_expo=" << step_expo << std::endl;
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

        std::unique_lock<std::mutex> lk(irframe.inProcess);
        auto ret = irframe.cv.wait_for(lk,std::chrono::seconds(1)); // with ~0.03ms delay

        if (ret == std::cv_status::timeout){
            error_exit = true;
            break;
        }
        
        if (frame_idx != irframe.seq)
        {   
            // Update the window with new data
            //cv::imshow(window_name_l, irframe.left);
            //cv::imshow(window_name_r, irframe.right);

            ros::Time sensor_timestamp; 
            sensor_timestamp.fromNSec(irframe.t);

            

            pub.publish(irframe.left, irframe.right, _cameraInfo_left, _cameraInfo_right, sensor_timestamp, irframe.seq);

            exposureCtl.calcHistogram(irframe.left,exposure,gain);
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

            const static int target_mean = 110;
            const static int dead_region = 10;

            if (irframe.seq%2) // only process half of the frames, give some delays
            {
                int exposure_target = exposure;
                int gain_target = gain;

                if (meanLux < target_mean - dead_region) // image too dark
                {
                    int margin = target_mean - meanLux;
                    // Consider Exposure first
                    if (exposure < max_exposure)
                    {
                        exposure_target = std::min(max_exposure, exposure + 50*margin);
                    }
                    else if(gain  <  max_gain)
                    {
                        gain_target = std::min(max_gain, gain + margin);
                    }
                }else if (meanLux > target_mean + dead_region) // image too bight
                {
                    int margin = meanLux - target_mean;
                    // Consider Gain first
                    if (gain > 160 /*good default*/)
                    {
                        gain_target= std::max(160, gain - margin);
                    }
                    else if(exposure > 8000 /*good default*/)
                    {
                        exposure_target = std::max(8000, exposure - 50*margin);
                    }
                    else if (gain > min_gain)
                    {
                        gain_target = std::max(min_gain, gain - margin);
                    }
                    else if (exposure > min_exposure)
                    {
                        exposure_target = std::max(min_exposure, exposure - 50*margin);
                        
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

                std::cout << "exposure: " << exposure << ", gain= " << gain << std::endl;

            }

            //cvWaitKey(1); // ~15ms
            frame_idx = irframe.seq;
        }
        lk.unlock();
        // ros::spinOnce();
        //std::this_thread::sleep_for(std::chrono::nanoseconds(100));

    }

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
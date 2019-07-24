// ROS Node for Realsense D415 Streams
// Cheng Huimin, June 2018
//
// The dedicated Realsense interface to publish ROS topics, from the infrared stereo streams

#include <iostream>
#include <iomanip>


#include <thread>
#include <chrono>

#include <mutex>

#include "rs2_interface/stereo_interface.hpp"

#include <opencv2/opencv.hpp>   // Include OpenCV API

struct stereo_frame_t{
    cv::Mat left;
    cv::Mat right;
    uint64_t t;
    uint64_t t_base;
    uint64_t seq;
    std::mutex inProcess;
}stereo_frame;

// from inner process loop to triggering this callback function takes around 0.2-0.4ms, tested
void stereoImageCallback(StereoDriver::StereoDataType data) // irleft and irright are in the heap, must be deleted after use
{
    if ( stereo_frame.inProcess.try_lock())
    {
        if (data.time_left != data.time_right)
            std::cerr << "ImageCallback(): stereo time sync inconsistent!" << std::endl;
        stereo_frame.t = data.time_left;
        if (data.seq_left != data.seq_right)
            std::cerr << "ImageCallback(): stereo frame sequence sync inconsistent!" << std::endl;
        stereo_frame.seq = data.seq_left;

        if (data.seq_left == 1)
        {
            stereo_frame.t_base = data.mid_shutter_time_estimate;
        }
        else
        {
           delete[] stereo_frame.left.data; // will cause memory leak if this is not freed
           delete[] stereo_frame.right.data;
        }
        
        stereo_frame.left = cv::Mat(cv::Size(data.width, data.height), CV_8UC1, data.left, cv::Mat::AUTO_STEP);    
        stereo_frame.right = cv::Mat(cv::Size(data.width, data.height), CV_8UC1, data.right, cv::Mat::AUTO_STEP);
        // stereo_frame.t = data.mid_shutter_time_estimate - stereo_frame.t_base;

        std::cout << stereo_frame.t << std::endl;
        stereo_frame.inProcess.unlock();

    }else
    {
        std::cout<< "Missed Frame(" << stereo_frame.seq << ")" << std::endl;
    }
}

int main() try
{
    const std::string target_device_name = "Intel RealSense T265";
    auto device_list = StereoDriver::getDeviceList();
    std::string sn;
    for (auto& device : device_list)
    {
        // std::cout << device.first << std::endl;
        if (device.first == target_device_name)
        {
            sn = device.second;
            break;
        }     
    }
    if (sn.empty())
    {
        std::cerr << target_device_name << " is not found, quitting." << std::endl;
        exit(-1);
    }

    StereoDriver* sys = new StereoDriver(sn);

    sys->setOption(RS2_OPTION_ENABLE_AUTO_EXPOSURE,1);
    // for more options, please refer rs_option.h
    // sys->setOption(RS2_OPTION_EXPOSURE,20000); // in usec
    // sys->setOption(RS2_OPTION_GAIN,16);

    const auto window_name_l = "Display Image Left";
    const auto window_name_r = "Display Image Right";
    cv::namedWindow(window_name_l, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(window_name_r, cv::WINDOW_AUTOSIZE);

    sys->registerCallback(stereoImageCallback);

    sys->enablePoseMotionStream();
    sys->enableStereoStream();
    sys->startPipe();

    uint frame_idx = 0;
    while (cv::waitKey(1) < 0)
    {
        if (frame_idx < stereo_frame.seq)
        {   
            stereo_frame.inProcess.lock();

            // Update the window with new data
            cv::imshow(window_name_l, stereo_frame.left);
            cv::imshow(window_name_r, stereo_frame.right);
            frame_idx = stereo_frame.seq;
            stereo_frame.inProcess.unlock();
        }
    }

    //std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    sys->stopPipe();

    std::cout << "main() exits" << std::endl;

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
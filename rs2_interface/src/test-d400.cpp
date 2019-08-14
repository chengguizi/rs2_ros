// ROS Node for Realsense D415 Streams
// Cheng Huimin, June 2018
//
// The dedicated Realsense interface to publish ROS topics, from the infrared stereo streams

#include <iostream>
#include <iomanip>


#include <thread>
#include <chrono>

#include <mutex>

#include <cassert>

#include "rs2_interface/stereo_interface.hpp"

#include <opencv2/opencv.hpp>   // Include OpenCV API

struct irframe_t{
    cv::Mat left;
    cv::Mat right;
    uint64_t t;
    uint64_t t_base;
    uint64_t seq;
    std::mutex inProcess;
}irframe;

// from inner process loop to triggering this callback function takes around 0.2-0.4ms, tested
void stereoImageCallback(StereoDriver::StereoDataType data) // irleft and irright are in the heap, must be deleted after use
{
    if ( irframe.inProcess.try_lock())
    {
        if (data.time_left != data.time_right)
            std::cerr << "ImageCallback(): stereo time sync inconsistent!" << std::endl;
        
        if (data.seq_left != data.seq_right)
            std::cerr << "ImageCallback(): stereo frame sequence sync inconsistent!" << std::endl;
        

        if (irframe.t == 0)
        {
            irframe.t_base = data.mid_shutter_time_estimate;
        }
        else
        {
           delete[] irframe.left.data; // will cause memory leak if this is not freed
           delete[] irframe.right.data;
        }

        irframe.t = data.time_left;
        irframe.seq = data.seq_left;
        
        irframe.left = cv::Mat(cv::Size(data.width, data.height), CV_8UC1, data.left, cv::Mat::AUTO_STEP);    
        irframe.right = cv::Mat(cv::Size(data.width, data.height), CV_8UC1, data.right, cv::Mat::AUTO_STEP);
        // irframe.t = data.mid_shutter_time_estimate - irframe.t_base;

        std::cout << irframe.t << std::endl;
        irframe.inProcess.unlock();

    }else
    {
        std::cout<< "Missed Frame(" << irframe.seq << ")" << std::endl;
    }
}

int main() try
{
    const std::string target_device_name = "RealSense D4";
    auto device_list = StereoDriver::getDeviceList();
    std::string sn;
    std::cout << "Listing Plugged-in Devices... " << std::endl;

    StereoDriver* sys; 

    for (auto& device : device_list)
    {
        std::cout << device.first << std::endl;
        if (device.first.find(target_device_name) != std::string::npos)
        {
            sys = new StereoDriver(device.second);

            if (!sys->isInitialised())
            {
                delete sys;
                sys = nullptr;
                continue;
            }

            sn = device.second;
            break;
        }     
    }

    if (sn.empty())
    {
        std::cerr << target_device_name << " is not found, quitting." << std::endl;
        exit(-1);
    }

    

    // for more options, please refer rs_option.h
    sys->setOption(RS2_OPTION_EXPOSURE,15000); // in usec
    sys->setOption(RS2_OPTION_GAIN,16);
    sys->enableAE(1800);
    // sys->setOption(RS2_OPTION_LASER_POWER,0);

    const auto window_name_l = "Display Image Left";
    const auto window_name_r = "Display Image Right";
    cv::namedWindow(window_name_l, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(window_name_r, cv::WINDOW_AUTOSIZE);

    

    uint64_t frame_idx = 0;
    irframe.t = 0;
    irframe.seq = 0;

    sys->registerCallback(stereoImageCallback);

    sys->enableStereoStream();

    sys->startPipe();

    while ( true )
    {
        if (frame_idx < irframe.seq)
        {   
            irframe.inProcess.lock();

            // Update the window with new data
            cv::imshow(window_name_l, irframe.left);
            cv::imshow(window_name_r, irframe.right);
            frame_idx = irframe.seq;
            irframe.inProcess.unlock();
        }

        int ret = cv::waitKey(1);
        if (ret != -1 && ret != 255)
            break;
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
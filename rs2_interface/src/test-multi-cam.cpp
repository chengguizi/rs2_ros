// ROS Node for Realsense D415 Streams
// Cheng Huimin, June 2018
//
// The dedicated Realsense interface to publish ROS topics, from the infrared stereo streams

#include <iostream>
#include <iomanip>


#include <thread>
#include <chrono>

#include <mutex>

#include <memory>

#include "rs2_interface/stereo_interface.hpp"

#include <opencv2/opencv.hpp>   // Include OpenCV API

struct frame_t{
    cv::Mat left;
    cv::Mat right;
    uint64_t t = 0;
    // uint64_t t_base;
    uint64_t seq;
    std::mutex* inProcess;
}irframe,fisheyeframe;

// from inner process loop to triggering this callback function takes around 0.2-0.4ms, tested
void stereoImageCallback(StereoDriver::StereoDataType data, frame_t& frame) // irleft and irright are in the heap, must be deleted after use
{
    if (frame.inProcess == nullptr)
    {
        std::cout << "initialise pointer" << std::endl;
        frame.inProcess = new std::mutex();
    }
    if ( frame.inProcess->try_lock())
    {
        if (data.time_left != data.time_right)
            std::cerr << "ImageCallback(): stereo time sync inconsistent!" << std::endl;
        frame.t = data.time_left;
        if (data.seq_left != data.seq_right)
            std::cerr << "ImageCallback(): stereo frame sequence sync inconsistent!" << std::endl;
        frame.seq = data.seq_left;

        if (data.seq_left == 1)
        {
            std::cout << "First Frame Received" << std::endl;
        }
        else
        {
           delete[] frame.left.data; // will cause memory leak if this is not freed
           delete[] frame.right.data;
        }
        
        frame.left = cv::Mat(cv::Size(data.width, data.height), CV_8UC1, data.left, cv::Mat::AUTO_STEP);    
        frame.right = cv::Mat(cv::Size(data.width, data.height), CV_8UC1, data.right, cv::Mat::AUTO_STEP);
        frame.t = data.time_left;

        std::cout << frame.t << std::endl;
        frame.inProcess->unlock();

    }else
    {
        std::cout<< "Missed Frame(" << frame.seq << ")" << std::endl;
    }
}

int main() try
{
    using namespace std::placeholders;

    auto sys_sn = StereoDriver::getDeviceList("RealSense D4");
    auto sys2_sn = StereoDriver::getDeviceList("RealSense T2");
    StereoDriver* sys = new StereoDriver(sys_sn.begin()->second);
    StereoDriver* sys2 = new StereoDriver(sys2_sn.begin()->second);

    // for more options, please refer rs_option.h
    // sys->setOption(RS2_OPTION_EXPOSURE,10000); // in usec
    // sys->setOption(RS2_OPTION_GAIN,16);

    sys->enableAE(2200);
    sys2->enableAE(2200);

    const auto window_name_ir_l = "irframe Image Left";
    const auto window_name_ir_r = "irframe Image Right";

    const auto window_name_fisheye_l = "fisheye Image Left";
    const auto window_name_fisheye_r = "fisheye Image Right";

    cv::namedWindow(window_name_ir_l, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(window_name_ir_r, cv::WINDOW_AUTOSIZE);

    cv::namedWindow(window_name_fisheye_l, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(window_name_fisheye_r, cv::WINDOW_AUTOSIZE);

    // sys->registerCallback(stereoImageCallback);
    sys->registerCallback(std::bind(stereoImageCallback, _1, std::ref(irframe)));
    sys2->registerCallback(std::bind(stereoImageCallback, _1, std::ref(fisheyeframe)));

    sys->enableStereoStream(1280, 720, 30);
    sys->startPipe();
    sys2->enableStereoStream(848, 800, 30, RS2_FORMAT_Y8);
    sys2->startPipe();

    uint irframe_idx = 0;
    uint fisheyeframe_idx = 0;
    while (cv::waitKey(1) < 0)
    {
        if (irframe_idx < irframe.seq)
        {   
            irframe.inProcess->lock();

            // Update the window with new data
            cv::imshow(window_name_ir_l, irframe.left);
            cv::imshow(window_name_ir_r, irframe.right);
            irframe_idx = irframe.seq;
            irframe.inProcess->unlock();
        }

        if (fisheyeframe_idx < fisheyeframe.seq)
        {   
            fisheyeframe.inProcess->lock();

            // Update the window with new data
            cv::imshow(window_name_fisheye_l, fisheyeframe.left);
            cv::imshow(window_name_fisheye_r, fisheyeframe.right);
            fisheyeframe_idx = fisheyeframe.seq;
            fisheyeframe.inProcess->unlock();
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
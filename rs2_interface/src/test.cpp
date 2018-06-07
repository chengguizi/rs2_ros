// ROS Node for Realsense D415 Streams
// Cheng Huimin, June 2018
//
// The dedicated Realsense interface to publish ROS topics, from the infrared stereo streams

#include <iostream>
#include <iomanip>


#include <thread>
#include <chrono>

#include <mutex>

#include "rs2_interface/irstereo_interface.hpp"

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
void stereoImageCallback(unsigned long long dev_time, void* irleft, void* irright, const int w, const int h, \
    double tleft, double tright, unsigned long long seqleft, unsigned long long seqright) // irleft and irright are in the heap, must be deleted after use
{
    if ( irframe.inProcess.try_lock())
    {
        if (tleft != tright)
            std::cerr << "ImageCallback(): stereo time sync inconsistent!" << std::endl;
        irframe.t = tleft;
        if (seqleft != seqright)
            std::cerr << "ImageCallback(): stereo frame sequence sync inconsistent!" << std::endl;
        irframe.seq = seqleft;

        if (seqleft == 1)
        {
            irframe.t_base = dev_time;
        }
        else
        {
           delete[] irframe.left.data; // will cause memory leak if this is not freed
           delete[] irframe.right.data;
        }
        
        irframe.left = cv::Mat(cv::Size(w, h), CV_8UC1, irleft, cv::Mat::AUTO_STEP);    
        irframe.right = cv::Mat(cv::Size(w, h), CV_8UC1, irright, cv::Mat::AUTO_STEP);
        irframe.t = dev_time - irframe.t_base;

        std::cout << irframe.t << std::endl;
        irframe.inProcess.unlock();

    }else
    {
        std::cout<< "Missed Frame(" << irframe.seq << ")" << std::endl;
    }
}

int main(int argc, char * argv[]) try
{
    IrStereoDriver* sys = new IrStereoDriver();

    // for more options, please refer rs_option.h
    sys->setOption(RS2_OPTION_EXPOSURE,20000); // in usec
    sys->setOption(RS2_OPTION_GAIN,40);

    const auto window_name_l = "Display Image Left";
    const auto window_name_r = "Display Image Right";
    cv::namedWindow(window_name_l, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(window_name_r, cv::WINDOW_AUTOSIZE);

    sys->registerCallback(stereoImageCallback);


    sys->startPipe();

    uint frame_idx = 0;
    while (cv::waitKey(1) < 0)
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
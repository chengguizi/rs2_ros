// ROS Node for Realsense D415 Streams
// Cheng Huimin, June 2018
//
// The dedicated Realsense interface to publish ROS topics, from the infrared stereo streams

#include <iostream>
#include <iomanip>

#include <csignal>

#include <thread>
#include <chrono>

#include <mutex>
#include <condition_variable>

#include <rs2_interface/irstereo_interface.hpp>

#include <opencv2/opencv.hpp>   // Include OpenCV API

#include <ros/ros.h>

#include "ros_publisher.h"

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

// from inner process loop to triggering this callback function takes around 0.2-0.4ms, tested
void stereoImageCallback(uint64_t t_sensor , void* irleft, void* irright, const int w, const int h, \
    double tleft, double tright, uint64_t seqleft, uint64_t seqright) // irleft and irright are in the heap, must be deleted after use
{
    if ( irframe.inProcess.try_lock())
    {
        irframe.t_callback = std::chrono::system_clock::now().time_since_epoch().count();

        if (tleft != tright)
            std::cerr << "ImageCallback(): stereo time sync inconsistent!" << std::endl;
        irframe.t = tleft;
        if (seqleft != seqright)
            std::cerr << "ImageCallback(): stereo frame sequence sync inconsistent!" << std::endl;
        irframe.seq = seqleft;

        if (seqleft == 1)
        {
            irframe.t_base = t_sensor;
            ROS_INFO_STREAM("ImageCallback(): First frame successfully captured!");
        }
        else
        {
           delete[] irframe.left.data; // will cause memory leak if this is not freed
           delete[] irframe.right.data;
        }
        
        irframe.left = cv::Mat(cv::Size(w, h), CV_8UC1, irleft, cv::Mat::AUTO_STEP);    
        irframe.right = cv::Mat(cv::Size(w, h), CV_8UC1, irright, cv::Mat::AUTO_STEP);
        irframe.t = t_sensor;

        irframe.inProcess.unlock();

        irframe.cv.notify_one();

    }else
    {
        std::cout<< "Missed Frame(" << irframe.seq << ")" << std::endl;
    }
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
    IrStereoDriver* sys = new IrStereoDriver();

    // for more options, please refer rs_option.h
    sys->setOption(RS2_OPTION_EXPOSURE,20000); // in usec
    sys->setOption(RS2_OPTION_GAIN,40);

    const auto window_name_l = "Display Image Left";
    const auto window_name_r = "Display Image Right";
    //cv::namedWindow(window_name_l, cv::WINDOW_AUTOSIZE);
    //cv::namedWindow(window_name_r, cv::WINDOW_AUTOSIZE);

    sys->registerCallback(stereoImageCallback);

    // ros initialisation
    ros::init(argc, argv, "rs2_camera");
    ros::NodeHandle nh("~");
    StereoOdometerPublisher pub(nh); // start with private scope

    //signal(SIGINT, signalHandler);

    ROS_INFO_STREAM("hello world from ROS!");


    sys->startPipe();

    uint frame_idx = 0;
    while (ros::ok())
    {

        std::unique_lock<std::mutex> lk(irframe.inProcess);
        irframe.cv.wait_for(lk,std::chrono::seconds(1)); // with ~0.03ms delay
        
        if (frame_idx != irframe.seq)
        {   
            // Update the window with new data
            //cv::imshow(window_name_l, irframe.left);
            //cv::imshow(window_name_r, irframe.right);

            ros::Time sensor_timestamp; 
            sensor_timestamp.fromNSec(irframe.t);
            
            pub.publish(irframe.left, irframe.right, sensor_timestamp, irframe.seq);

            //cvWaitKey(1); // ~15ms
            frame_idx = irframe.seq;
        }
        lk.unlock();
        ros::spinOnce();
        //std::this_thread::sleep_for(std::chrono::nanoseconds(100));

    }

    delete sys;
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
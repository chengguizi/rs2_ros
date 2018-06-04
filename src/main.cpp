// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API

#include <iostream>

// std::string get_sensor_name(const rs2::sensor& sensor)
//     {
//         // Sensors support additional information, such as a human readable name
//         if (sensor.supports(RS2_CAMERA_INFO_NAME))
//             return sensor.get_info(RS2_CAMERA_INFO_NAME);
//         else
//             return "Unknown Sensor";
// }

int main(int argc, char * argv[]) try
{

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    //Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_INFRARED,1, 640, 480, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_INFRARED,2, 640, 480, RS2_FORMAT_Y8, 30);

    // Start streaming with default recommended configuration
    rs2::pipeline_profile selection = pipe.start(cfg);
    // Obtain the active device
    rs2::device selected_device = selection.get_device();

    std::vector<rs2::sensor> sensors = selected_device.query_sensors();

        // std::cout << "Device consists of " << sensors.size() << " sensors:\n" << std::endl;
        // int index = 0;
        // // We can now iterate the sensors and print their names
        // for (rs2::sensor sensor : sensors)
        // {
        //     std::cout << "  " << index++ << " : " << get_sensor_name(sensor) << std::endl;
        // }

    auto depth_sensor = selected_device.first<rs2::depth_sensor>();
    depth_sensor.set_option(RS2_OPTION_EXPOSURE,15000); // in usec
    depth_sensor.set_option(RS2_OPTION_GAIN,30);

    using namespace cv;
    const auto window_name_l = "Display Image Left";
    namedWindow(window_name_l, WINDOW_AUTOSIZE);
    const auto window_name_r = "Display Image Right";
    namedWindow(window_name_r, WINDOW_AUTOSIZE);

    while (waitKey(1) < 0)
    {
        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera

        rs2::video_frame irleft = data.get_infrared_frame(1);
        rs2::video_frame irright = data.get_infrared_frame(2);
        // Query frame size (width and height)
        const int w = irleft.get_width();
        const int h = irleft.get_height();

        // Create OpenCV matrix of size (w,h) from 
        Mat image_left(Size(w, h), CV_8UC1, (void*)irleft.get_data(), Mat::AUTO_STEP);
        Mat image_right(Size(w, h), CV_8UC1, (void*)irright.get_data(), Mat::AUTO_STEP);

        // Update the window with new data
        //cvGetWindowHandle(window_name_l);
        imshow(window_name_l, image_left);
        //cvGetWindowHandle(window_name_r);
        imshow(window_name_r, image_right);
    }

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

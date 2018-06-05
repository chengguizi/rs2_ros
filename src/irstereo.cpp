// ROS Node for Realsense D415 Streams
// Cheng Huimin, June 2018
//
// Class of the physical sensor for callback operations

#include <librealsense2/rs.hpp>
#include <iostream>
#include <cstring>
//#include <stdio.h> // printf
//#include <stdlib.h> // linux

#include "irstereo.h"


////////////////////////////////
// helper functions
////////////////////////////////

std::string get_device_name(const rs2::device& dev)
{
    // Each device provides some information on itself, such as name:
    std::string name = "Unknown Device";
    if (dev.supports(RS2_CAMERA_INFO_NAME))
        name = dev.get_info(RS2_CAMERA_INFO_NAME);

    // and the serial number of the device:
    std::string sn = "########";
    if (dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER))
        sn = std::string("#") + dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

    return name + " " + sn;
}

std::string get_sensor_name(const rs2::sensor& sensor)
{
    // Sensors support additional information, such as a human readable name
    if (sensor.supports(RS2_CAMERA_INFO_NAME))
        return sensor.get_info(RS2_CAMERA_INFO_NAME);
    else
        return "Unknown Sensor";
}


////////////////////////////////
// class member functions
////////////////////////////////

IrStereoDriver::IrStereoDriver() : _isStreaming(false)
{
    _pipe = new rs2::pipeline();
    init();
}

IrStereoDriver::~IrStereoDriver()
{
    stopPipe();
    delete _dev;
    delete _pipe;
}

void IrStereoDriver::init()
{
    // First, create a rs2::context.
    // The context represents the current platform with respect to connected devices
    rs2::context ctx;

    // Using the context we can get all connected devices in a device list
    rs2::device_list devices = ctx.query_devices();

    rs2::device selected_device;
    if (devices.size() == 0)
    {
        std::cerr << "No device connected, please connect a RealSense device" << std::endl;

        //To help with the boilerplate code of waiting for a device to connect
        //The SDK provides the rs2::device_hub class
        rs2::device_hub device_hub(ctx);

        //Using the device_hub we can block the program until a device connects
        selected_device = device_hub.wait_for_device();
    }
    else
    {
        std::cout << "Found the following devices:" << std::endl;

        // device_list is a "lazy" container of devices which allows
        //The device list provides 2 ways of iterating it
        //The first way is using an iterator (in this case hidden in the Range-based for loop)
        int index = 0;
        for (rs2::device device : devices)
        {
            std::cout << "  " << index++ << " : " << get_device_name(device) << std::endl;
        }

        uint32_t selected_device_index = 0;

        std::cout << std::endl << "Selecting the first device..." << std::endl;

        // Update the selected device
        selected_device = devices[selected_device_index];
    }

    std::cout << "Device initialised successfully!" << std::endl;
    _dev = new auto(selected_device);


    //// select the IR stereo sensors from the first detected realsense device
    std::vector<rs2::sensor> sensors = _dev->query_sensors();

    std::cout << "Device consists of " << sensors.size() << " sensors:\n" << std::endl;
    int index = 0;
    // We can now iterate the sensors and print their names
    for (rs2::sensor sensor : sensors)
    {
        std::cout << "  " << index++ << " : " << get_sensor_name(sensor) << std::endl;
    }

    auto stereo_sensor = _dev->first<rs2::depth_stereo_sensor>();
    _stereo = new auto(stereo_sensor);
    std::cout << "Stereo Sensor initialised successfully!" << std::endl;
    std::cout << std::endl << "=======================================" << std::endl << std::endl;
}

void IrStereoDriver::startPipe()
{
    //Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_INFRARED,1, 1280, 720, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_INFRARED,2, 1280, 720, RS2_FORMAT_Y8, 30);

    // Start streaming with default recommended configuration
    rs2::pipeline_profile selection = _pipe->start(cfg);
    std::cout << "Pipeline Streaming Starts..." << std::endl;
    _profile = new auto(selection);

    _isStreaming = true;
    _thread = std::thread(&IrStereoDriver::process,this);
}

void IrStereoDriver::stopPipe()
{   
    if (!_isStreaming)
        return;

    _isStreaming = false;
    if(_thread.joinable())
        _thread.join();

    _pipe->stop();
    std::cout << "Pipeline Streaming Stopped..." << std::endl;

    delete _profile;
}


void IrStereoDriver::process()
{

    rs2::frameset dummyset = _pipe->wait_for_frames(500); // Wait for next set of frames from the camera
    rs2::video_frame dummy = dummyset.get_infrared_frame(1);
    // Query frame size (width and height)
    int w = dummy.get_width();
    int h = dummy.get_height();

    std::cout << "Process Starts..." << std::endl;

    while(_isStreaming)
    {
        rs2::frameset dataset = _pipe->wait_for_frames(500); // Wait for next set of frames from the camera
        
        rs2::video_frame frame_left = dataset.get_infrared_frame(1);
        rs2::video_frame frame_right = dataset.get_infrared_frame(2);
        
        void* irleft = new char[w*h];
        memcpy(irleft,frame_left.get_data(),w*h);

        void* irright = new char[w*h];
        memcpy(irright,frame_right.get_data(),w*h);

        for (callbackType* cb : _cblist)
        {
            (*cb)(irleft,irright,w,h);
        }
    }
    std::cout << "Process Ended..." << std::endl;
}

void IrStereoDriver::setOption(rs2_option option, float value)
{
    _stereo->set_option(option,value); 
}

void IrStereoDriver::registerCallback(callbackType &cb)
{
    _cblist.push_back(&cb);
}
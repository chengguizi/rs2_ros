// ROS Node for Realsense D415 Streams
// Cheng Huimin, June 2018
//
// Class of the physical sensor for callback operations

#include <librealsense2/rs.hpp>
#include <iostream>
#include <cstring>
#include <string>
//#include <stdio.h> // printf
//#include <stdlib.h> // linux

#include "rs2_interface/irstereo_interface.hpp"


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

IrStereoDriver::IrStereoDriver(std::string dev_name_str, int laser_power) : _isStreaming(false), _dev_name_str(dev_name_str), _laser_power(laser_power)
{
    _pipe = new rs2::pipeline();
    init();
}

IrStereoDriver::~IrStereoDriver()
{
    stopPipe();
    delete _dev;
    delete _pipe;
    std::cout << "Stereo driver stopped..." << std::endl;
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
        // rs2::device_hub device_hub(ctx);

        //Using the device_hub we can block the program until a device connects
        // selected_device = device_hub.wait_for_device();

        exit(-1);
    }
    else
    {
        std::cout << "Found the following devices:" << std::endl;

        // device_list is a "lazy" container of devices which allows
        //The device list provides 2 ways of iterating it
        //The first way is using an iterator (in this case hidden in the Range-based for loop)
        int index = 0;
        bool rs2_found = false;
        for (rs2::device device : devices)
        {
            std::string name = device.get_info(RS2_CAMERA_INFO_NAME); // get_device_name(device);
            std::cout << "  " << index << " : " << name << std::endl;

            if (!rs2_found && name.find(_dev_name_str) != std::string::npos)
            {
                std::cout << "found RealSense Camera: " << _dev_name_str << std::endl;
                rs2_found =  true;
            }

            if(!rs2_found)
                index++;
        }

        if (!rs2_found) // no realsense device present
        {
            std::cerr << "No RealSense camera was found, exiting..." << std::endl;
            exit(1);
        }

        // Update the selected device
        selected_device = devices[index];
        std::cout << "Device Serial Number: #" << selected_device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
    }

    std::cout << "=======================================" << std::endl;
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

    // obtain laser options
    if (_stereo->supports(RS2_OPTION_LASER_POWER))
    {
        //auto range = _stereo->get_option_range(RS2_OPTION_LASER_POWER);
        _stereo->set_option(RS2_OPTION_LASER_POWER,_laser_power);
        ;
        std::cout << "set laser power = " << _stereo->get_option(RS2_OPTION_LASER_POWER) << std::endl;
    }else
        std::cerr << "Stereo Sensor does not support laser power options" << std::endl;

    std::cout << "Stereo Sensor initialised successfully!" << std::endl;
    std::cout << "=======================================" << std::endl << std::endl;
}

void IrStereoDriver::startPipe(int width, int height, int hz)
{
    //Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_INFRARED,1, width, height, RS2_FORMAT_Y8, hz); // left
    cfg.enable_stream(RS2_STREAM_INFRARED,2, width, height, RS2_FORMAT_Y8, hz); // right

    // Start streaming with default recommended configuration
    rs2::pipeline_profile selection = _pipe->start(cfg);
    _profile = new auto(selection);

    auto stream_profile_left = _profile->get_stream(RS2_STREAM_INFRARED,1);
    auto stream_profile_right = _profile->get_stream(RS2_STREAM_INFRARED,2);
    auto video_profile_left = stream_profile_left.as<rs2::video_stream_profile>();
    
    _intrinsics =  video_profile_left.get_intrinsics();
    auto _hz = video_profile_left.fps();

    std::cout << std::endl << "Realsense Hardware Intrinsics" << std::endl
                << "fx= " << _intrinsics.fx// 1.88mm focal length?
                << ", fy= " << _intrinsics.fy << std::endl
                << "width= " << _intrinsics.width // 1.4um pixel size
                << ", height= " << _intrinsics.height << std::endl
                << "ppx= " << _intrinsics.ppx // principal point
                << ", ppy= " << _intrinsics.ppy << std::endl
                << "model= " << rs2_distortion_to_string(_intrinsics.model) << std::endl;

    _extrinsics_left_to_right = stream_profile_left.get_extrinsics_to(stream_profile_right); // baseline 55mm?

    _baseline = _stereo->get_option(RS2_OPTION_STEREO_BASELINE) / 1000.0 ; // convert from mm to meter

    std::cout << std::endl << "Realsense Hardware Extrinsics (left to right)" << std::endl
                << "translation: ";
    
    for (const auto& element : _extrinsics_left_to_right.translation ) 
                std::cout  << element << ", ";

    std::cout << std::endl << "rotation: ";
    for (const auto& element : _extrinsics_left_to_right.rotation ) 
                std::cout  << element << ", ";
    std::cout << std::endl;

    std::cout << "baseline: " << _baseline << std::endl;

    if (width != _intrinsics.width || height != _intrinsics.height || _hz != hz)
    {
        std::cerr << "ERROR: intrinsics dimensions mismatch requested resolutions" << std::endl;
        std::cerr << "width: " << width << " --> " << _intrinsics.width << std::endl;
        std::cerr << "height: " << height << " --> " << _intrinsics.height << std::endl;
        std::cerr << "frame rate: " << hz << " --> " << _hz << std::endl;
        exit(1);
    }

    _isStreaming = true;
    _thread = std::thread(&IrStereoDriver::process,this);

    std::cout << "Pipeline Streaming Starts with resolution of [" << width <<"*" << height << "]@" << hz << "Hz" << std::endl;
    std::cout << "=======================================" << std::endl << std::endl;
}

rs2_intrinsics IrStereoDriver::get_intrinsics() const
{
    return _intrinsics;
}

rs2_extrinsics IrStereoDriver::get_extrinsics_left_to_right() const
{
    return _extrinsics_left_to_right;
}

float IrStereoDriver::get_baseline() const
{
    return _baseline;
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

    // switch (dummy.get_frame_timestamp_domain())
    // {
    //     case RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME:
    //         std::cout<< "frame_timestamp_domain = RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME" << std::endl;
    //     break;
    //     case RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK:
    //         std::cout<< "frame_timestamp_domain = RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK" << std::endl;
    //     break;
    //     default:
    //         std::cout<< "frame_timestamp_domain = (error)" << std::endl;
    //     break;
    // }

    std::cout << "Process Starts..." << std::endl;

    while(_isStreaming)
    {
        rs2::frameset dataset = _pipe->wait_for_frames(10000); // Wait for next set of frames from the camera
        
        uint64_t now = std::chrono::system_clock::now().time_since_epoch().count();

        int num_frames = dataset.size();
        if (num_frames != 2)
            std::cerr << "frameset contains " << num_frames << "frames, should be 2."<< std::endl;

        rs2::video_frame frame_left = dataset.get_infrared_frame(1);
        double time_left = frame_left.get_timestamp()/1000;
        uint64_t seq_left = frame_left.get_frame_number();

        int w = frame_left.get_width();
        int h = frame_left.get_height();

        // Refer to https://github.com/IntelRealSense/librealsense/issues/2188
        //metadata in usec
        
        auto meta_sensortime = frame_left.get_frame_metadata(RS2_FRAME_METADATA_SENSOR_TIMESTAMP); // middle of shutter
        auto meta_frametime = frame_left.get_frame_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP); // start UVC frame tx

        uint64_t meta_backendtime = frame_left.get_frame_metadata(RS2_FRAME_METADATA_BACKEND_TIMESTAMP); // usb controller to kernel , in millisecond?
        // uint64_t meta_toa = frame_left.get_frame_metadata(RS2_FRAME_METADATA_TIME_OF_ARRIVAL); // kernel to user space

        const uint64_t delay_sensor_to_frame = meta_frametime - meta_sensortime; // 1e7; // 10us delay, RS2_FRAME_METADATA_FRAME_TIMESTAMP - RS2_FRAME_METADATA_SENSOR_TIMESTAMP
        // std::cout << std::fixed << "delay_sensor_to_frame"<< delay_sensor_to_frame << "us" << std::endl;
        // std::cout << "meta_backendtime" << meta_backendtime << std::endl;
        // uint64_t delay_uvc_to_frontend = meta_toa - meta_backendtime ; // ~16000us delay meta_toa - meta_backendtime
        uint64_t sensor_time = meta_backendtime * 1e6 - delay_sensor_to_frame* 1e3; // in nanosecond, epoch time

        rs2::video_frame frame_right = dataset.get_infrared_frame(2);
        double time_right = frame_right.get_timestamp()/1000;
        uint64_t seq_right = frame_right.get_frame_number();

        //metadata
        // auto meta_toa2 = frame_right.get_frame_metadata(RS2_FRAME_METADATA_TIME_OF_ARRIVAL);
        // auto meta_sensortime2 = frame_right.get_frame_metadata(RS2_FRAME_METADATA_SENSOR_TIMESTAMP);
        // auto meta_frametime2 = frame_right.get_frame_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP);
        // auto meta_backendtime2 = frame_right.get_frame_metadata(RS2_FRAME_METADATA_BACKEND_TIMESTAMP);

        // uint64_t delay_uvc_to_frontend2 = meta_toa2 - meta_backendtime2;
        // std::cout << delay_uvc_to_frontend << "  " << delay_uvc_to_frontend2 << std::endl;

        void* irleft = new char[w*h];
        memcpy(irleft,frame_left.get_data(),w*h);

        void* irright = new char[w*h];
        memcpy(irright,frame_right.get_data(),w*h);

        for (callbackType* cb : _cblist)
        {
            (*cb)(sensor_time, irleft,irright,w,h,time_left,time_right,seq_left,seq_right);
            // time_left and time_right is time since boot of the realsense hardware
        }

    }
    std::cout << "Process Ended..." << std::endl;
}

void IrStereoDriver::setOption(rs2_option option, float value)
{
    _stereo->set_option(option,value); 
}

float IrStereoDriver::getOption(rs2_option option)
{
    return _stereo->get_option(option); 
}

rs2::option_range IrStereoDriver::getOptionRange(rs2_option option)
{
    return _stereo->get_option_range(option); 
}

void IrStereoDriver::registerCallback(callbackType &cb)
{
    _cblist.push_back(&cb);
}
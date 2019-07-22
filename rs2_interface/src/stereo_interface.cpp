// ROS Node for Realsense D415 Streams
// Cheng Huimin, June 2018
//
// Class of the physical sensor for callback operations

#include <librealsense2/rs.hpp>
#include <librealsense2/h/rs_advanced_mode_command.h> // for controlling the AE setpoint
#include <librealsense2/rs_advanced_mode.hpp>
#include <iostream>
#include <iomanip>
#include <cstring>
#include <string>
// #include <stdio.h> // printf
// #include <stdlib.h> // linux
#include <unistd.h>

#include <algorithm>
#include <cassert>

#include "rs2_interface/stereo_interface.hpp"


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

StereoDriver::StereoDriver(std::string dev_name_str, int laser_power) : _dev_name_str(dev_name_str), _laser_power(laser_power) //, _isStreaming(false)
{
    // High CPU usage issue https://github.com/IntelRealSense/librealsense/issues/2037
    init();
    _pipe = new rs2::pipeline(_ctx); // Jun 2019: This line has to be called after init(), otherwise T265 will not be detected

    // _cblist_stereo.clear();
}

StereoDriver::~StereoDriver()
{
    stopStereoPipe();
    delete _dev;
    delete _pipe;
    std::cout << "Stereo driver stopped..." << std::endl;
}

void StereoDriver::init()
{
    // First, create a rs2::context.
    // The context represents the current platform with respect to connected devices
    rs2::device selected_device;

    // Using the context we can get all connected devices in a device list
    _ctx.query_devices();
    auto devices = _ctx.query_devices();
    
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
    
    // We can now iterate the sensors and print their names
    int index = 0;
    int found_ir_stereo = -1;
    int found_fisheye_stereo = -1;
    for (rs2::sensor sensor : sensors)
    {
        auto sensor_name = get_sensor_name(sensor);
        std::cout << "  " << index << " : " << sensor_name << std::endl;

        // determine stereo sensor type
        if (sensor_name == "Stereo Module")
            found_ir_stereo = index;
        else if(sensor_name == "Tracking Module")
            found_fisheye_stereo = index;
        index++;
    }

    if (!found_ir_stereo && !found_fisheye_stereo)
    {
        std::cerr << "No Stereo Sensor Found, Quitting" << std::endl;
        exit(-1);
    }

    // auto stereo_sensor;
    if (found_ir_stereo >= 0){ // D400
        _stereo = new auto(sensors[found_ir_stereo]); // new auto is needed, otherwise segmentation fault
        _stereo->set_option(RS2_OPTION_LASER_POWER,_laser_power);
        std::cout << "set laser power = " << _stereo->get_option(RS2_OPTION_LASER_POWER) << std::endl;
    }
    else if (found_fisheye_stereo >= 0)
        _stereo = new auto(sensors[found_fisheye_stereo]); // T265

    std::cout << "Stereo Sensor initialised successfully!" << std::endl;
    std::cout << "=======================================" << std::endl << std::endl;
}

void StereoDriver::enablePoseMotionStream(){

    std::vector<rs2::sensor> sensors = _dev->query_sensors();

    for (auto& sensor : sensors){
        auto available_streams = sensor.get_stream_profiles();

        for (auto& stream : available_streams){

            // std::cout << stream.stream_name() << std::endl;
            switch (stream.stream_type())
            {
            case RS2_STREAM_POSE:
                _cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
                std::cout << "Enabled Pose Stream" << std::endl;
                break;
            case RS2_STREAM_GYRO:
                _cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
                std::cout << "Enabled Gyro Stream" << std::endl;
                break;
            case RS2_STREAM_ACCEL:
                _cfg.enable_stream(RS2_STREAM_ACCEL,RS2_FORMAT_MOTION_XYZ32F);
                std::cout << "Enabled Accel Stream" << std::endl;
            default:
                break;
            }
        }
    }
    
}

std::vector<std::string> tokenize_floats(std::string input, char separator){
    std::vector<std::string> tokens;
    std::stringstream ss(input);
    std::string token;

    while (std::getline(ss, token, separator)) {
        tokens.push_back(token);
    }

    return tokens;
}

void print(const rs2_extrinsics& extrinsics)
{
    std::stringstream ss;
     ss << " Rotation Matrix:\n";

    // Align displayed data along decimal point
    for (auto i = 0 ; i < 3 ; ++i)
    {
        for (auto j = 0 ; j < 3 ; ++j)
        {
            std::ostringstream oss;
            oss << extrinsics.rotation[j*3 +i];
            auto tokens = tokenize_floats(oss.str().c_str(),'.');
            ss << std::right << std::setw(4) << tokens[0];
            if (tokens.size()>1)
                ss << "." << std::left <<std::setw(12) << tokens[1];
        }
        ss << std::endl;
    }

    ss << "\n Translation Vector: ";
    for (auto i = 0u ; i < sizeof(extrinsics.translation)/sizeof(extrinsics.translation[0]) ; ++i)
        ss << std::setprecision(15) << extrinsics.translation[i] << "  ";

    std::cout << ss.str() << std::endl << std::endl;
}

void StereoDriver::startStereoPipe(int width, int height, int hz, rs2_format stream_format)
{

    _cfg.enable_device(_dev->get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)); // This is needed for multi cam setup

    auto available_streams = _stereo->get_stream_profiles();

    auto find_result = std::find_if(available_streams.begin(), available_streams.end(), [&](rs2::stream_profile& s) {return s.stream_type() == RS2_STREAM_INFRARED || s.stream_type() == RS2_STREAM_FISHEYE;});

    assert(find_result != available_streams.end());

    auto stream_type = find_result->stream_type();

    _cfg.enable_stream(stream_type,1, width, height, stream_format, hz); // left
    _cfg.enable_stream(stream_type,2, width, height, stream_format, hz); // right
    std::cout << "Enabled Both " << rs2_stream_to_string(stream_type) << " Stream" << std::endl;
    std::cout << "Stereo Stream Format: " << rs2_format_to_string(stream_format) << std::endl;

    std::cout  << "Starting Pipe with " << width << "x" << height << " " << stream_format << " @ " << hz << "Hz" << std::endl;
    // Start streaming with default recommended configuration

    rs2::pipeline_profile selection = _pipe->start(_cfg, std::bind(&StereoDriver::frameCallback,this, std::placeholders::_1));
    _profile = new auto(selection);

    auto stream_profile_left = _profile->get_stream(stream_type,1);
    auto stream_profile_right = _profile->get_stream(stream_type,2);
    auto video_profile_left = stream_profile_left.as<rs2::video_stream_profile>();
    

    std::cout << std::endl << "Realsense Hardware Intrinsics:" << std::endl;

    _intrinsics =  video_profile_left.get_intrinsics();
    auto _hz = video_profile_left.fps();

    
    std::cout   << "fx= " << _intrinsics.fx// 1.88mm focal length?
                << ", fy= " << _intrinsics.fy << std::endl
                << "width= " << _intrinsics.width // 1.4um pixel size
                << ", height= " << _intrinsics.height << std::endl
                << "ppx= " << _intrinsics.ppx // principal point
                << ", ppy= " << _intrinsics.ppy << std::endl
                << "model= " << rs2_distortion_to_string(_intrinsics.model) << std::endl;

    _extrinsics_left_to_right = stream_profile_left.get_extrinsics_to(stream_profile_right); // baseline 55mm?

    print(_extrinsics_left_to_right);

    _baseline = -_extrinsics_left_to_right.translation[0]; // x-axis

    // _baseline = _stereo->get_option(RS2_OPTION_STEREO_BASELINE) / 1000.0 ; // convert from mm to meter

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

    // _isStreaming = true;
    // _thread = std::thread(&StereoDriver::process,this);

    std::cout << "Pipeline Streaming Starts with resolution of [" << width <<"*" << height << "]@" << hz << "Hz" << std::endl;
    std::cout << "=======================================" << std::endl << std::endl;
}

rs2_intrinsics StereoDriver::get_intrinsics() const
{
    return _intrinsics;
}

rs2_extrinsics StereoDriver::get_extrinsics_left_to_right() const
{
    return _extrinsics_left_to_right;
}

float StereoDriver::get_baseline() const
{
    return _baseline;
}

void StereoDriver::stopStereoPipe()
{   
    // if (!_isStreaming)
    //     return;

    // _isStreaming = false;
    // if(_thread.joinable())
    //     _thread.join();

    _pipe->stop();
    std::cout << "Pipeline Streaming Stopped..." << std::endl;

    delete _profile;
}

// The call back either returns as frameset (synchronised) or a single frame (unsynchronised)
void StereoDriver::frameCallback(const rs2::frame& frame)
{
    static int num_stereo_frames = 0, num_pose_frames = 0, num_gyro_frames = 0, num_accel_frames = 0;

    std::lock_guard<std::mutex> lock(callback_stereo_mutex);
    
    // std::cout << "frameCallback()" << std::endl;
    // With callbacks, all synchronized stream will arrive in a single frameset
    if(rs2::frameset dataset = frame.as<rs2::frameset>())
    {
        num_stereo_frames++;
        int num_frames = dataset.size();
        if (num_frames != 2)
            std::cerr << "frameset contains " << num_frames << "frames, should be 2."<< std::endl;

        const rs2_stream stream_type = dataset.get_profile().stream_type();

        rs2::video_frame frame_left = (stream_type == RS2_STREAM_INFRARED) ? dataset.get_infrared_frame(1) : dataset.get_fisheye_frame(1);
        rs2::video_frame frame_right = (stream_type == RS2_STREAM_INFRARED) ? dataset.get_infrared_frame(2) : dataset.get_fisheye_frame(2);
        
        double time_left = frame_left.get_timestamp()/1000;
        uint64_t seq_left = frame_left.get_frame_number();


        if (num_stereo_frames == 1)
        {
            auto meta_timedomain = frame_left.get_frame_timestamp_domain();
            std::cout << "stereo left time = " << (uint64_t) (time_left * 1e9) << ", with time domain "<< meta_timedomain << std::endl;
        }

        int w = frame_left.get_width();
        int h = frame_left.get_height();

        // Refer to https://github.com/IntelRealSense/librealsense/issues/2188
        //metadata in usec

        // auto meta_exposure = frame_left.get_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE);

        uint64_t mid_shutter_time_estimate = 0;

        //// NO LONGER NEEDED DUE TO FIRMWARE UPDATE FOR GLOBAL TIME IMPLEMENTATION FROM LIBREALSENSE

        // if (frame_left.supports_frame_metadata(RS2_FRAME_METADATA_SENSOR_TIMESTAMP) 
        //     && frame_left.supports_frame_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP))
        // {
        //     auto meta_sensortime = frame_left.get_frame_metadata(RS2_FRAME_METADATA_SENSOR_TIMESTAMP); // middle of shutter
        //     auto meta_frametime = frame_left.get_frame_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP); // start UVC frame tx
        //     // uint64_t meta_toa = frame_left.get_frame_metadata(RS2_FRAME_METADATA_TIME_OF_ARRIVAL); // kernel to user space
        //     uint64_t meta_backendtime = frame_left.get_frame_metadata(RS2_FRAME_METADATA_BACKEND_TIMESTAMP); // usb controller to kernel , in millisecond?

        //     const uint64_t delay_sensor_to_frame = meta_frametime - meta_sensortime;
        //     // 1e7; // 10us delay, RS2_FRAME_METADATA_FRAME_TIMESTAMP - RS2_FRAME_METADATA_SENSOR_TIMESTAMP
        //     // std::cout << std::fixed << "delay_sensor_to_frame"<< delay_sensor_to_frame << "us" << std::endl;
        //     // std::cout << "meta_backendtime" << meta_backendtime << std::endl;
        //     // uint64_t delay_uvc_to_frontend = meta_toa - meta_backendtime ; // ~16000us delay meta_toa - meta_backendtime

        //     mid_shutter_time_estimate = meta_backendtime * 1e6 - delay_sensor_to_frame* 1e3; // in nanosecond, epoch time
        // }else
        // {
        //     mid_shutter_time_estimate = frame_left.get_frame_metadata(RS2_FRAME_METADATA_TIME_OF_ARRIVAL) * 1e6;
        //     // std::cout << mid_shutter_time_estimate << std::endl;
        // }
        
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

        StereoDataType data = {mid_shutter_time_estimate, irleft, irright, w, h, time_left, time_right, seq_left, seq_right};
        for (callbackStereo& cb : _cblist_stereo){
            (cb)(data);
        }
        

    }else if (rs2::pose_frame pose = frame.as<rs2::pose_frame>()){
        num_pose_frames++;

        // None of these supported in pose_frame
        // std::cout << pose.supports_frame_metadata(RS2_FRAME_METADATA_SENSOR_TIMESTAMP) << std::endl;
        // std::cout << pose.supports_frame_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP) << std::endl;
        // std::cout << pose.supports_frame_metadata(RS2_FRAME_METADATA_BACKEND_TIMESTAMP) << std::endl;
        // std::cout << pose.supports_frame_metadata(RS2_FRAME_METADATA_TIME_OF_ARRIVAL) << std::endl;

        double meta_timestamp = pose.get_timestamp() / 1000.0; // in seconds
        uint64_t meta_seq = pose.get_frame_number();
        rs2_pose data_pose = pose.get_pose_data();

        for (auto& cb : _cblist_pose)
        {
            PoseDataType data = {meta_timestamp, meta_seq, data_pose};
            cb(data);
        }
        
        if (num_pose_frames == 1)
        {
            auto meta_timedomain = pose.get_frame_timestamp_domain();
            std::cout << "pose time = " << (uint64_t) (meta_timestamp * 1e9) << ", with time domain "<< meta_timedomain << std::endl;
        }
        
    }else if (rs2::motion_frame motion = frame.as<rs2::motion_frame>()){
        auto stream_type = motion.get_profile().stream_type();
        double meta_timestamp = motion.get_timestamp() / 1000.0; // in seconds
        uint64_t meta_seq = motion.get_frame_number();

        rs2_vector data_motion = motion.get_motion_data();

        if ( stream_type == RS2_STREAM_GYRO){
            num_gyro_frames++;
            GyroDataType data = {meta_timestamp, meta_seq, data_motion.x, data_motion.y, data_motion.z};
            imuBuffer.pushGyro(data);
            for (auto& cb : _cblist_gyro){
                cb(data);
            }

            if (num_gyro_frames == 1)
            {
                auto meta_timedomain = motion.get_frame_timestamp_domain();
                std::cout << "gyro time = " << (uint64_t) ( meta_timestamp * 1e9) << ", with time domain "<< meta_timedomain << std::endl;
            }
        
            
        }else if (stream_type == RS2_STREAM_ACCEL){
            num_accel_frames++;
            AccelDataType data = {meta_timestamp, meta_seq, data_motion.x, data_motion.y, data_motion.z};
            imuBuffer.update(data, std::bind(&StereoDriver::imuCallback, this, std::placeholders::_1));
            for (auto& cb : _cblist_accel){
                cb(data);
            }

            if (num_accel_frames == 1)
            {
                auto meta_timedomain = motion.get_frame_timestamp_domain();
                std::cout << "accel time = " << (uint64_t) ( meta_timestamp * 1e9) << ", with time domain "<< meta_timedomain << std::endl;
            }
            
        }


    }else{
        std::cerr << "Unknown frame type in callback!" << std::endl;
    }
}

void StereoDriver::imuCallback(const SyncedIMUDataType& data)
{
    for (auto& cb : _cblist_imu){
        cb(data);
    }
}

void StereoDriver::setOption(rs2_option option, float value)
{
    if (_stereo->supports(option))
        _stereo->set_option(option,value); 
    else
        std::cerr << "The sensor does not support option " << rs2_option_to_string(option) << std::endl;
}

float StereoDriver::getOption(rs2_option option)
{
    if (_stereo->supports(option))
        return _stereo->get_option(option);
    else
    {
        std::cerr << "The sensor does not support option " << rs2_option_to_string(option) << std::endl;
        return 0;
    }
}

void StereoDriver::enableAE(uint32_t meanIntensitySetPoint)
{
    setOption(RS2_OPTION_ENABLE_AUTO_EXPOSURE,1);

    if (!_dev->supports(RS2_CAMERA_INFO_ADVANCED_MODE)){
        std::cout << "enableAE(): Advance Mode for Exposure not Available..." << std::endl;
        return;
    }

    auto adv_mode = rs400::advanced_mode( (*_dev) );
    STAEControl ae_ctl_t;
    ae_ctl_t.meanIntensitySetPoint = meanIntensitySetPoint;
    adv_mode.set_ae_control(ae_ctl_t);
    std::cout << "AE Setpoint set to " << meanIntensitySetPoint << std::endl;
}

rs2::option_range StereoDriver::getOptionRange(rs2_option option)
{
    return _stereo->get_option_range(option); 
}

void StereoDriver::registerCallback(callbackStereo cb)
{
    _cblist_stereo.push_back(cb);
}

void StereoDriver::registerCallback(callbackGyro cb)
{
    _cblist_gyro.push_back(cb);
}

void StereoDriver::registerCallback(callbackAccel cb)
{
    _cblist_accel.push_back(cb);
}

void StereoDriver::registerCallback(callbackIMU cb)
{
    _cblist_imu.push_back(cb);
}
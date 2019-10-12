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

#include <chrono>

#include "rs2_interface/stereo_interface.hpp"


rs2::context StereoDriver::_ctx = rs2::context(); // to remove linking errors, for static members

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

std::map<std::string, std::string> StereoDriver::getDeviceList(std::string target_device_name)
{
    std::cout << "Getting device list for target: " << target_device_name << std::endl;
    auto devices =  _ctx.query_devices();

    std::map<std::string, std::string> ret;
    for (rs2::device device : devices)
    {
        const std::string name = device.get_info(RS2_CAMERA_INFO_NAME);

        if (target_device_name.empty() || name.find(target_device_name) != std::string::npos)
            ret.insert( std::pair<std::string, std::string> (name, device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)));
    }

    return ret;
}

// std::vector<std::string> StereoDriver::getSNfromName(std::string target_device_name)
// {
//     auto device_list = StereoDriver::getDeviceList();
//     std::vector<std::string> sn;
//     std::cout << "Listing Plugged-in Devices... " << std::endl;
//     for (auto& device : device_list)
//     {
//         std::cout << device.first << std::endl;
//         if (device.first.find(target_device_name) != std::string::npos)
//         {
//             sn.push_back(device.second);
//             break;
//         }     
//     }
    
//     return sn;
// }

StereoDriver::StereoDriver(std::string dev_sn_str) : initialised(false), _dev_sn_str(dev_sn_str)
{
    // High CPU usage issue https://github.com/IntelRealSense/librealsense/issues/2037

    std::cout << "Driver for SN: " << dev_sn_str << std::endl;

    if (init())
    {
        // setOption(RS2_OPTION_LASER_POWER,0);
        _pipe = new rs2::pipeline(_ctx); // Jun 2019: This line has to be called after init(), otherwise T265 will not be detected
    }      
    else
    {
        std::cerr << "StereoDriver() constructor for device sn: " << dev_sn_str << " failed." << std::endl;
        return;
    }
    // _cblist_stereo.clear();

    initialised = true;
}

StereoDriver::~StereoDriver()
{
    if (initialised)
    {
        stopPipe();
        delete _pipe;
        delete _stereo;
        delete _dev;
    }
    
    std::cout << "Stereo driver stopped..." << std::endl;
}

// Initialise using the serial number
bool StereoDriver::init()
{
    // First, create a rs2::context.
    // The context represents the current platform with respect to connected devices
    rs2::device selected_device;

    // Using the context we can get all connected devices in a device list
    // _ctx.query_devices();
    auto devices = _ctx.query_devices();
    
    if (devices.size() == 0)
    {
        std::cerr << "No device connected, please connect a RealSense device" << std::endl;
        return false;
    }
    else
    {
        // std::cout << "Found the following devices:" << std::endl;

        // device_list is a "lazy" container of devices which allows
        //The device list provides 2 ways of iterating it
        //The first way is using an iterator (in this case hidden in the Range-based for loop)
        int index = 0;
        bool rs2_found = false;
        for (rs2::device device : devices)
        {
            const std::string name = device.get_info(RS2_CAMERA_INFO_NAME);
            const std::string sn = device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
            // std::cout << "  " << index << " : " << name << " (" << sn << ")" << std::endl;

            if (_dev_sn_str == sn)
            {
                std::cout << "found RealSense Camera: " << name << " (" << sn << ")" << std::endl;
                _dev_name_str = name;
                rs2_found =  true;
                break;
            }

            if(!rs2_found)
                index++;
        }

        if (!rs2_found) // no realsense device present
        {
            std::cerr << "Camera " << _dev_sn_str <<" was NOT found, failed to initialise" << std::endl;
            return false;
        }

        // Update the selected device
        selected_device = devices[index];
    }

    // std::cout << "=======================================" << std::endl;
    _dev = new auto(selected_device);


    //// select the IR stereo sensors from the first detected realsense device
    std::vector<rs2::sensor> sensors = _dev->query_sensors();

    // std::cout << "Device consists of " << sensors.size() << " sensors:\n" << std::endl;
    
    // We can now iterate the sensors and print their names
    int index = 0;
    int found_ir_stereo = -1;
    int found_fisheye_stereo = -1;
    int found_color = -1;

    for (rs2::sensor sensor : sensors)
    {
        auto sensor_name = get_sensor_name(sensor);
        // std::cout << "  " << index << " : " << sensor_name << std::endl;

        // determine stereo sensor type
        if (sensor_name == "Stereo Module")
            found_ir_stereo = index;
        else if(sensor_name == "Tracking Module")
            found_fisheye_stereo = index;
        else if(sensor_name == "RGB Camera")
            found_color = index;

        index++;
    }

    if (!found_ir_stereo && !found_fisheye_stereo)
    {
        std::cerr << "No Stereo Sensor Found, Quitting" << std::endl;
        return false;
    }

    // auto stereo_sensor;
    if (found_ir_stereo >= 0){ // D400
        _stereo = new auto(sensors[found_ir_stereo]); // new auto is needed, otherwise segmentation fault
        _depth = new auto(_stereo->as<rs2::depth_sensor>());
        _stereo_stream_type = RS2_STREAM_INFRARED;
    }
    else if (found_fisheye_stereo >= 0)
    {
        _stereo = new auto(sensors[found_fisheye_stereo]); // T265
        _stereo_stream_type = RS2_STREAM_FISHEYE;
    }

    if (found_color >= 0)
        _color = new auto(sensors[found_color]);

    std::cout << "Stereo Sensor initialised successfully!" << std::endl;
    std::cout << "=======================================" << std::endl << std::endl;

    return true;
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

void StereoDriver::enableStereoStream(int width, int height, int hz, rs2_format stream_format)
{
    _cfg.enable_stream(_stereo_stream_type,1, width, height, stream_format, hz); // left
    _cfg.enable_stream(_stereo_stream_type,2, width, height, stream_format, hz); // right
    std::cout << "Enabled Both " << rs2_stream_to_string(_stereo_stream_type) << " Stream" << std::endl;
    std::cout << "Stereo Stream Format: " << rs2_format_to_string(stream_format) << std::endl;
}

void StereoDriver::enableColorStream(int width, int height, int hz, rs2_format stream_format)
{
    _cfg.enable_stream(RS2_STREAM_COLOR, width, height, stream_format, hz);
    std::cout << "Enabled Color Stream" << std::endl;
}

void StereoDriver::enableDepthStream(int width, int height, int hz, rs2_format stream_format)
{
    _cfg.enable_stream(RS2_STREAM_DEPTH, width, height, stream_format, hz);
    std::cout << "Enabled Depth Stream" << std::endl;
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

void StereoDriver::startPipe()
{

    // set to the specific SN device

    _cfg.enable_device(_dev_sn_str);

    // Start streaming with default recommended configuration

    if (! _cfg.can_resolve(std::shared_ptr<rs2_pipeline>(*_pipe)) )
    {
        std::cerr << "startPipe(): Cannot Resolve Config for the pipe" << std::endl;
        exit(-1);
    }

    rs2::pipeline_profile selection = _pipe->start(_cfg, std::bind(&StereoDriver::frameCallback,this, std::placeholders::_1));
    _profile = new auto(selection);

    std::string active_sn = _profile->get_device().get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

    assert (_dev_sn_str == active_sn); // the string turned on should be the one requested

    auto stream_profiles = _profile->get_streams();

    for (const auto& stream_profile : stream_profiles)
    {
        auto stream = stream_profile.stream_type();
        if (stream == _stereo_stream_type)
            is_streaming_stereo = true;
        else if (stream == RS2_STREAM_DEPTH)
            is_streaming_depth = true;
        else if (stream == RS2_STREAM_COLOR)
            is_streaming_color = true;
    }

    if (is_streaming_stereo)
    {
        auto stream_profile_left = _profile->get_stream(_stereo_stream_type,1);
        auto stream_profile_right = _profile->get_stream(_stereo_stream_type,2);
        auto video_profile_left = stream_profile_left.as<rs2::video_stream_profile>();
        auto video_profile_right = stream_profile_right.as<rs2::video_stream_profile>();
        

        std::cout << std::endl << "Realsense Stereo Intrinsics:" << std::endl;

        _stereo_left_intrinsics =  video_profile_left.get_intrinsics();
        _stereo_right_intrinsics = video_profile_right.get_intrinsics();

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
    }

    if (is_streaming_depth)
    {
        auto stream_profile_depth = _profile->get_stream(RS2_STREAM_DEPTH);
        auto video_profile_depth = stream_profile_depth.as<rs2::video_stream_profile>();

        _depth_intrinsics = video_profile_depth.get_intrinsics();

        std::cout << std::endl << "Obtained Depth Intrinsics" << std::endl;

        std::cout << "Depth scale: " << _depth->get_depth_scale() << " (meter = pixel * scale)"<< std::endl;
    }

    if (is_streaming_color)
    {
        
        auto stream_profile_color = _profile->get_stream(RS2_STREAM_COLOR);
        auto video_profile_color = stream_profile_color.as<rs2::video_stream_profile>();

        _color_intrinsics = video_profile_color.get_intrinsics();

        std::cout << std::endl << "Obtained Color Intrinsics" << std::endl;
    }
   
}

void StereoDriver::get_stereo_intrinsics(rs2_intrinsics& stereo_left_intrinsics, rs2_intrinsics& stereo_right_intrinsics)
{
    stereo_left_intrinsics = _stereo_left_intrinsics;
    stereo_right_intrinsics = _stereo_left_intrinsics;
}

rs2_extrinsics StereoDriver::get_extrinsics_left_to_right() const
{
    return _extrinsics_left_to_right;
}

float StereoDriver::get_baseline() const
{
    return _baseline;
}

rs2_intrinsics StereoDriver::get_depth_intrinsics() const
{
    return _depth_intrinsics;
}

rs2_intrinsics StereoDriver::get_color_intrinsics() const
{
    return _color_intrinsics;
}

void StereoDriver::stopPipe()
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
    std::lock_guard<std::mutex> lock(callback_stereo_mutex);
    
    // std::cout << "frameCallback()" << std::endl;
    // With callbacks, all synchronized stream will arrive in a single frameset
    if(rs2::frameset dataset = frame.as<rs2::frameset>())
    {
        num_framesets++;

        // obtain metadata
        double meta_time = dataset.get_timestamp()/1000;
        uint64_t meta_seq = dataset.get_frame_number();

        // std::cout << uint64_t(meta_time * 1e9) << std::endl;

        // Check for frame seq jitter
        if (num_framesets > 1){
            if (last_frameset_seq == meta_seq)
            {
                std::cerr << "SN" << _dev_sn_str << "Duplicated frameCallback frameset seq = " << meta_seq << ", Skipping" << std::endl;
                return;
            }
            if (last_frameset_seq + 1 != meta_seq){
                std::cerr << "SN" << _dev_sn_str << ": USB Backend misses (" << meta_seq - last_frameset_seq - 1 << ") frame(s). Current = " << meta_seq << " Previous = " << last_frameset_seq << std::endl;
            }
        }
        last_frameset_seq = meta_seq;

        if (num_framesets == 1)
        {
            auto meta_timedomain = dataset.get_frame_timestamp_domain();
            std::cout << "First Frameset: " << last_frameset_seq << ", time = " << (uint64_t) (meta_time * 1e9) << ", with time domain "<< meta_timedomain << std::endl;
        }


        // Check if depth frame is present
        rs2::depth_frame frame_depth = dataset.get_depth_frame();
        if (frame_depth)
        {
            // std::cout << uint64_t(frame_depth.get_timestamp()*1e6) << "depth" << std::endl;
            assert(meta_time == frame_depth.get_timestamp()/1000); // frameset timestamp is always the first sensor, depth sensor
            assert(meta_seq == frame_depth.get_frame_number());

            const int depth_bpp = frame_depth.get_bytes_per_pixel();
            const int depth_width = frame_depth.get_width();
            const int depth_height = frame_depth.get_height();
            char* depth_data = new char[depth_width*depth_height*depth_bpp];
            memcpy(depth_data, frame_depth.get_data(), depth_width*depth_height*depth_bpp);

            const float depth_scale = _depth->get_depth_scale();

            DepthDataType data = {"Depth", depth_data, depth_width, depth_height, depth_bpp, depth_scale, meta_time, meta_seq};
            for (callbackDepth& cb : _cblist_depth){
                (cb)(data);
            }
        }

        // Check if color frame is present
        rs2::video_frame frame_color = dataset.get_color_frame();
        if (frame_color)
        {
            // std::cout << uint64_t(frame_color.get_timestamp()*1e6) << "color" << std::endl;
            // assert(meta_time == frame_color.get_timestamp()/1000);
            // assert(meta_seq == frame_color.get_frame_number());

            double time = frame_color.get_timestamp()/1000;
            uint64_t seq = frame_color.get_frame_number();

            const int color_bpp = frame_color.get_bytes_per_pixel();
            const int color_width = frame_color.get_width();
            const int color_height = frame_color.get_height();
            char* color_data = new char[color_width*color_height*color_bpp];
            memcpy(color_data, frame_color.get_data(), color_width*color_height*color_bpp);

            ColorDataType data = {"Color", color_data, color_width, color_height, color_bpp, time, seq};
            for (callbackColor& cb : _cblist_color){
                (cb)(data);
            }
        }

        // Check if stereo frame is present
        rs2::video_frame frame_left = dataset.get_infrared_frame(1);
        if (!frame_left)
            frame_left = dataset.get_fisheye_frame(1);

        rs2::video_frame frame_right = dataset.get_infrared_frame(2);
        if (!frame_right)
            frame_right = dataset.get_fisheye_frame(2);

        if (frame_left && frame_right)
        {
            double time_left = frame_left.get_timestamp()/1000;
            double time_right = frame_right.get_timestamp()/1000;

            uint64_t seq_left = frame_left.get_frame_number();
            uint64_t seq_right = frame_right.get_frame_number();

            if (seq_left != seq_right)
                std::cerr << "seq_left != seq_right" << std::endl;

            int w = frame_left.get_width();
            int h = frame_left.get_height();

            int meta_exposure = 0;
            if (frame_left.supports_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE))
                meta_exposure = frame_left.get_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE);
            else if(num_framesets == 1)
                std::cout << "[ERROR] Stereo does not support metadata actual exposure get" << std::endl;

            int meta_gain = -1;
            if (frame_left.supports_frame_metadata(RS2_FRAME_METADATA_GAIN_LEVEL))
                meta_gain = frame_left.get_frame_metadata(RS2_FRAME_METADATA_GAIN_LEVEL);
            else if(num_framesets == 1)
                std::cout << "[ERROR] Stereo does not support metadata gain level get" << std::endl;
        

            const int bpp = frame_left.get_bytes_per_pixel();
            char* irleft = new char[w*h*bpp];
            memcpy(irleft,frame_left.get_data(),w*h*bpp);

            char* irright = new char[w*h*bpp];
            memcpy(irright,frame_right.get_data(),w*h*bpp);

            StereoDataType data = {"Stereo", irleft, irright, w, h, bpp, time_left, time_right, seq_left, seq_right, meta_exposure, meta_gain, false};
            for (callbackStereo& cb : _cblist_stereo){
                (cb)(data);
            }
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
            imuBuffer.pushGyro(data, std::bind(&StereoDriver::imuCallback, this, std::placeholders::_1));
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
            imuBuffer.pushAccel(data);
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

void StereoDriver::setOption(rs2_option option, const float value, bool force)
{
    if (_stereo->supports(option))
        _stereo->set_option(option,value); 
    else
    {
        std::cerr << "The sensor does not support option " << rs2_option_to_string(option) << std::endl;
        if (force)
            throw std::runtime_error("setOption() failed: does not support option");
    }

    // std::this_thread::sleep_for(std::chrono::milliseconds(2));

    if (force)
    {
        float value_get = getOption(option);
        if (value_get != value)
            throw std::runtime_error("setOption() failed: value assertion, requested " + std::to_string(value) + ", but gotten "  + std::to_string(value_get));
    }
        
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

void StereoDriver::disableAE()
{
    setOption(RS2_OPTION_ENABLE_AUTO_EXPOSURE,0);

    if (getOption(RS2_OPTION_ENABLE_AUTO_EXPOSURE) != 0)
    {
        std::cout << "AE is failed to be disabled" << std::endl;
        exit(-1);
    }
    else
        std::cout << "AE is disabled" << std::endl;
}

rs2::option_range StereoDriver::getOptionRange(rs2_option option)
{
    return _stereo->get_option_range(option); 
}

void StereoDriver::registerCallback(callbackStereo cb)
{
    _cblist_stereo.push_back(cb);
}

void StereoDriver::registerCallback(callbackDepth cb)
{
    _cblist_depth.push_back(cb);
}

void StereoDriver::registerCallback(callbackColor cb)
{
    _cblist_color.push_back(cb);
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
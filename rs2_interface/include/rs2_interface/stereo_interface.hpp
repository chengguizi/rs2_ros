// ROS Node for Realsense D415 Streams
// Cheng Huimin, June 2018
//
// Class of the physical sensor for callback operations

// https://github.com/IntelRealSense/librealsense/wiki/Projection-in-RealSense-SDK-2.0
// - Left and right infrared images are rectified by default (Y16 format is not)

#ifndef STEREO_INTERFACE_HPP
#define STEREO_INTERFACE_HPP

#include <librealsense2/rs.hpp>
#include <thread>
#include <chrono>
#include <mutex>
#include <functional>
#include <vector>

class StereoDriver {

public:

    struct StereoDataType{
        uint64_t sensor_time;
        void* left;
        void* right;
        int width;
        int height;
        double time_left;
        double time_right;
        uint64_t seq_left;
        uint64_t seq_right;
    };

    struct PoseDataType{

    };

    struct IMUDataType{

    };

    typedef std::function<void(StereoDataType)> callbackStereo; 
                        // time since epoch, left & right image data, width, height, hardware time left & right, hardware sequence left & right
    StereoDriver(std::string dev_name_str = "RealSense D415", int laser_power = 150);
    ~StereoDriver();
    void setOption(rs2_option option, float value);
    float getOption(rs2_option option);

    void enableAE(uint32_t meanIntensitySetPoint);
    void disableAE(){setOption(RS2_OPTION_ENABLE_AUTO_EXPOSURE,0);};
    rs2::option_range getOptionRange(rs2_option option);
    void enablePoseMotionStream();
    void startStereoPipe(int width, int height, int hz, rs2_stream stream_type = RS2_STREAM_INFRARED, rs2_format stream_format = RS2_FORMAT_Y8);
    rs2_intrinsics get_intrinsics() const; // only call this after pipe started
    rs2_extrinsics get_extrinsics_left_to_right() const;
    float get_baseline() const;
    void stopStereoPipe();
    void registerCallback(callbackStereo cb);
private:
    void init();
    void frameCallback(const rs2::frame& frame);

    std::string _dev_name_str;
    rs2::context _ctx;
    rs2::config _cfg;
    rs2::pipeline* _pipe;
    rs2::device* _dev;
    rs2::sensor* _stereo;
    rs2::pipeline_profile* _profile;

    rs2_intrinsics _intrinsics;
    rs2_extrinsics _extrinsics_left_to_right;
    float _baseline;
    int _laser_power; // range 0 - 360, step 30. default 150
    int _w,_h;

    // bool _isStreaming;

    std::mutex callback_stereo_mutex;
    // std::thread _thread;

    std::vector<callbackStereo> _cblist_stereo;
};


#endif /* STEREO_INTERFACE_HPP */

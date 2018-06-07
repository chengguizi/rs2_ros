#ifndef IRSTEREO_INTERFACE_HPP
#define IRSTEREO_INTERFACE_HPP
// ROS Node for Realsense D415 Streams
// Cheng Huimin, June 2018
//
// Class of the physical sensor for callback operations

// https://github.com/IntelRealSense/librealsense/wiki/Projection-in-RealSense-SDK-2.0
// - Left and right infrared images are rectified by default (Y16 format is not)

#include <librealsense2/rs.hpp>
#include <thread>
#include <chrono>
#include <functional>
#include <vector>

class IrStereoDriver {

public:
    typedef const std::function<void(uint64_t, void*,void*, int, int, double, double, uint64_t, uint64_t)> callbackType; 
                        // time since epoch, left & right image data, width, height, hardware time left & right, hardware sequence left & right
    IrStereoDriver();
    ~IrStereoDriver();
    void setOption(rs2_option option, float value);
    void startPipe();
    void stopPipe();
    void registerCallback(callbackType &cb);
private:
    void init();
    void process();
    rs2::pipeline* _pipe;
    rs2::device* _dev;
    rs2::depth_stereo_sensor* _stereo;
    rs2::pipeline_profile* _profile;
    bool _isStreaming;

    std::thread _thread;

    std::vector<callbackType*> _cblist;
};


#endif /* IRSTEREO_INTERFACE_HPP */

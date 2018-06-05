// ROS Node for Realsense D415 Streams
// Cheng Huimin, June 2018
//
// Class of the physical sensor for callback operations
#ifndef IRSTEREO_H
#define IRSTEREO_H

#include <librealsense2/rs.hpp>
#include <thread>
#include <chrono>
#include <functional>
#include <vector>
// #include <librealsense2/h/rs_option.h>
// #include <librealsense2/hpp/rs_types.hpp>


class IrStereoDriver {

public:
    typedef const std::function<void(void*,void*, int, int)> callbackType; // left image data, right image data
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

#endif /* IRSTEREO_H */

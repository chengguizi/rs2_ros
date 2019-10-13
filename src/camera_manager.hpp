#ifndef CAMERA_MANAGER_HPP
#define CAMERA_MANAGER_HPP

#include <vector>

#include <thread>
#include <chrono>

#include <mutex>
#include <condition_variable>

#include <rs2_interface/stereo_interface.hpp>
#include "ros_publisher.hpp"

#include <opencv2/opencv.hpp>   // Include OpenCV API

#include <ros/ros.h>


class ExposureControl;


class CameraParam{

public:
    std::string topic_ns;
    std::string type;
    std::string camera_sn;

    std::string do_hardware_sync = "none";
    bool do_publish_stereo = true;
    bool do_publish_depth = false;
    bool do_publish_color = false;
    bool do_publish_poseimu = false;
    bool do_alternate_laser_emitter = false;

    int width = 0,height = 0,hz = 0;
    int laser_power = 0;
    std::string auto_exposure_mode = "internal";
    int auto_exposure_mean_intensity_setpoint = 1800;

    std::string pose_coordinate;

    // Exposure control param
    int initial_exposure, initial_gain;
    int exposure_max, gain_max;
    double exposure_change_rate;
    int exposure_target_mean;
    int exposure_dead_region;
    double exposure_a_min, exposure_a_max, exposure_c;

    rs2::option_range exposure_range, gain_range;

    static std::vector<std::string> camera_list;
    static std::vector<std::string>  loadCameras(const ros::NodeHandle& nh);
    void loadParam(const std::string& topic_ns);
    void loadExposureControlParam(const std::string& type);
};


class CameraManager
{

public:
    
    CameraManager() = delete;
    CameraManager(const std::string& topic_ns);
    bool isInitialised(){return initialised;};
    ~CameraManager();

    void setSyncMode();
    
    void processFrame();
    void processStereoFrame(); // for running in an independent thread: for jitter detection, exposure control and topic publishing
    void processDepthFrame();
    void processColorFrame();
    void startPipe(){
        std::cout << param.topic_ns << ": Starting Pipe..." << std::endl;
        sys->startPipe();};

private:
    bool initialised;
    StereoDriver* sys;
    StereoCameraPublisher* pub_stereo;
    ImagePublisher* pub_depth;
    ImagePublisher* pub_color;
    IMUPublisher* pub_imu;
    PosePublisher* pub_pose;
    ros::Publisher pub_stats;

    ExposureControl* expo_ctl;

    CameraParam param;
    
    struct MtxStereoFrame{
        std::mutex inProcess;
        std::condition_variable cv;
        StereoDriver::StereoDataType frame;
    }mtxStereoFrame;

    struct MtxDepthFrame{
        std::mutex inProcess;
        std::condition_variable cv;
        std::function<rs2_intrinsics()> get_intrinsics;
        StereoDriver::DepthDataType frame;
    }mtxDepthFrame;

    struct MtxColorFrame{
        std::mutex inProcess;
        std::condition_variable cv;
        StereoDriver::ColorDataType frame;
        std::function<rs2_intrinsics()> get_intrinsics;
    }mtxColorFrame;

    sensor_msgs::CameraInfo cameraInfo_left, cameraInfo_right, cameraInfo_depth, cameraInfo_color;

    void getCameraInfo(const rs2_intrinsics& intrinsics, sensor_msgs::CameraInfo& camerainfo, float baseline = 0.0);

    template <class T, class M, class C>
    void setFrame(const T& frame, M* mtxFrame, C* cameraInfo);
    void setStereoFrame(const StereoDriver::StereoDataType& frame);

    void callbackSyncedIMU_t265(const StereoDriver::SyncedIMUDataType& data);
    void callbackPose_t265(const StereoDriver::PoseDataType& data);
    void callbackSyncedIMU_d400(const StereoDriver::SyncedIMUDataType& data);
};


#endif /* CAMERA_MANAGER_HPP */

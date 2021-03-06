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

#include <map>

#include <cmath>

//debug
#include <cassert>
#include <iomanip>
#include <iostream>

class StereoDriver {

public:

    struct StereoDataType{
        std::string name;
        char* left;
        char* right;
        int width;
        int height;
        int bpp;
        double time_left; // System Time Domain
        double time_right; // System Time Domain
        uint64_t seq_left;
        uint64_t seq_right;
        int exposure;
        int gain;
        bool is_published;
    };

    struct DepthDataType{
        std::string name;
        char* data;
        int width;
        int height;
        int bpp; // bytes per pixel
        float depth_scale;
        double time;
        uint64_t seq;
    };

    struct ColorDataType{
        std::string name;
        char* data;
        int width;
        int height;
        int bpp; // bytes per pixel
        double time;
        uint64_t seq;
    };

    struct PoseDataType{
        double timestamp;
        uint64_t seq;
        rs2_pose pose;
    };

    struct GyroDataType{
        double timestamp;
        uint64_t seq;
        float x,y,z;
    };

    struct AccelDataType{
        double timestamp;
        uint64_t seq;
        float x,y,z;
    };

    struct SyncedIMUDataType{
        double timestamp;
        uint64_t seq;
        float ax,ay,az; // accelerometer
        float gx,gy,gz; // gyroscope
    };

    typedef std::function<void(const StereoDataType&)> callbackStereo;
    typedef std::function<void(const DepthDataType&)> callbackDepth;
    typedef std::function<void(const ColorDataType&)> callbackColor;
    typedef std::function<void(const PoseDataType&)> callbackPose; 
    typedef std::function<void(const GyroDataType&)> callbackGyro; 
    typedef std::function<void(const AccelDataType&)> callbackAccel;
    typedef std::function<void(const SyncedIMUDataType&)> callbackIMU;

                        // time since epoch, left & right image data, width, height, hardware time left & right, hardware sequence left & right
    static std::map<std::string, std::string> getDeviceList(std::string target_device_name = std::string());
    // static std::vector<std::string> getSNfromName(std::string target_device_name);
    
    StereoDriver(std::string dev_sn_str = "");
    bool isInitialised(){return initialised;};
    ~StereoDriver();
    void setOption(rs2_option option, const float value, bool force = true);
    float getOption(rs2_option option);
    std::string getDeviceName(){return _dev_name_str;};

    void enableAE(uint32_t meanIntensitySetPoint);
    void disableAE();
    rs2::option_range getOptionRange(rs2_option option);
    void enablePoseMotionStream();
    void enableStereoStream(int width = 0, int height = 0, int hz = 0, rs2_format stream_format = RS2_FORMAT_ANY);
    void enableColorStream(int width = 0, int height = 0, int hz = 0, rs2_format stream_format = RS2_FORMAT_ANY);
    void enableDepthStream(int width = 0, int height = 0, int hz = 0, rs2_format stream_format = RS2_FORMAT_ANY);
    void startPipe();
    void get_stereo_intrinsics(rs2_intrinsics& stereo_left_intrinsics, rs2_intrinsics& stereo_right_intrinsics); // only call this after pipe started
    rs2_extrinsics get_extrinsics_left_to_right() const;
    float get_baseline() const;
    rs2_intrinsics get_depth_intrinsics() const;
    rs2_intrinsics get_color_intrinsics() const;
    void stopPipe();
    void registerCallback(callbackStereo cb);
    void registerCallback(callbackDepth cb);
    void registerCallback(callbackColor cb);
    void registerCallback(callbackGyro cb);
    void registerCallback(callbackAccel cb);
    void registerCallback(callbackIMU cb);
    void registerCallback(callbackPose cb);
private:
    
    // The following struct assumes accel is always arriving slower than gyro, and at a lower frequency than gyro
    struct IMUBuffer{
        const static uint buffer_size = 256; // size of char
        GyroDataType gyro[buffer_size];
        AccelDataType last_accel, curr_accel;

        unsigned char begin = 0; // at the position of the first data
        unsigned char end = 0; // one position pass the last data

        IMUBuffer(){
            last_accel.seq = 0;
        }

        void pushGyro(GyroDataType data, std::function<void(const SyncedIMUDataType&)> cb){
            static uint64_t accel_synced = 0;
            gyro[end++] = data;

            if (accel_synced != last_accel.seq && gyro[end-1].timestamp > curr_accel.timestamp) // gyro readings are at least up-to-date with accel readings
            {
                syncAndPush(cb);
                accel_synced = last_accel.seq;
            }
        }
        void pushAccel(AccelDataType accel){

            last_accel = curr_accel;
            curr_accel = accel;
        }

        void syncAndPush(std::function<void(const SyncedIMUDataType&)> cb){
            // publish all imu data in the buffer

            // std::cout  << "begin = " << gyro[begin].seq << ", end = " << gyro[end-1].seq << std::endl;
            double duration = curr_accel.timestamp - last_accel.timestamp;

            if (duration <= 0.0)
            {
                std::cerr << "IMU timestamp jitter detected,  curr vs. last = " << std::setprecision(15) << curr_accel.timestamp << ", " << last_accel.timestamp << std::endl;
                std::cerr << duration << std::endl;
                return;
            }

            double grad_x = (curr_accel.x - last_accel.x) / duration;
            double grad_y = (curr_accel.y - last_accel.y) / duration;
            double grad_z = (curr_accel.z - last_accel.z) / duration;

            assert(std::isfinite(grad_x) && std::isfinite(grad_y) && std::isfinite(grad_z));
            
            // last_accel.timestamp is always older than the gyro[begin]
            for(; begin != end && gyro[begin].timestamp <= curr_accel.timestamp ; begin++){ 
                double lapse = (gyro[begin].timestamp - last_accel.timestamp);

                // std::cout << std::setprecision(15) << gyro[begin].timestamp << std::endl << last_accel.timestamp << std::endl << gyro[end - 1].timestamp << std::endl << std::endl;
                if (lapse < 0.0)
                    continue;

                SyncedIMUDataType imu_data;
                imu_data.timestamp = gyro[begin].timestamp;
                imu_data.seq = gyro[begin].seq;
                imu_data.gx = gyro[begin].x;
                imu_data.gy = gyro[begin].y;
                imu_data.gz = gyro[begin].z;

                imu_data.ax = lapse * grad_x + last_accel.x;
                imu_data.ay = lapse * grad_y + last_accel.y;
                imu_data.az = lapse * grad_z + last_accel.z;

                cb(imu_data);
            }
        }
    }imuBuffer;

    bool initialised;
    bool init();
    uint64_t num_framesets = 0, num_pose_frames = 0, num_gyro_frames = 0, num_accel_frames = 0;
    uint64_t last_stereo_seq = 0, last_depth_seq = 0, last_color_seq = 0;
    uint64_t last_pose_seq = 0, last_gyro_seq = 0, last_accel_seq = 0;
    void frameCallback(const rs2::frame& frame);
    void imuCallback(const SyncedIMUDataType& data);

    std::string _dev_sn_str, _dev_name_str;
    static rs2::context _ctx; // related to LIBUVC interface claiming?
    rs2::config _cfg;
    rs2::pipeline* _pipe;
    rs2::device* _dev;
    rs2::sensor* _stereo = nullptr;
    rs2::depth_sensor* _depth = nullptr;
    rs2::sensor* _color;
    rs2::pipeline_profile* _profile;
    rs2_stream _stereo_stream_type;

    rs2_intrinsics _stereo_left_intrinsics;
    rs2_intrinsics _stereo_right_intrinsics;
    rs2_intrinsics _depth_intrinsics;
    rs2_intrinsics _color_intrinsics;
    rs2_extrinsics _extrinsics_left_to_right;
    float _baseline;

    // bool _initial_reset = true;
    bool is_streaming_stereo = false;
    bool is_streaming_depth = false;
    bool is_streaming_color = false;

    std::mutex callback_stereo_mutex;
    // std::thread _thread;

    std::vector<callbackStereo> _cblist_stereo;
    std::vector<callbackDepth> _cblist_depth;
    std::vector<callbackColor> _cblist_color;
    std::vector<callbackPose> _cblist_pose;
    std::vector<callbackGyro> _cblist_gyro;
    std::vector<callbackAccel> _cblist_accel;
    std::vector<callbackIMU> _cblist_imu;
};


#endif /* STEREO_INTERFACE_HPP */

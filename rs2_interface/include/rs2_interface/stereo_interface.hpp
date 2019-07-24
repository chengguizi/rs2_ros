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

class StereoDriver {

public:

    struct StereoDataType{
        uint64_t mid_shutter_time_estimate; // Generally need not be used, for reference only
        void* left;
        void* right;
        int width;
        int height;
        double time_left; // System Time Domain
        double time_right; // System Time Domain
        uint64_t seq_left;
        uint64_t seq_right;
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

    typedef std::function<void(StereoDataType)> callbackStereo; 
    typedef std::function<void(PoseDataType)> callbackPose; 
    typedef std::function<void(GyroDataType)> callbackGyro; 
    typedef std::function<void(AccelDataType)> callbackAccel;
    typedef std::function<void(SyncedIMUDataType)> callbackIMU;

                        // time since epoch, left & right image data, width, height, hardware time left & right, hardware sequence left & right
    static std::map<std::string, std::string> getDeviceList(std::string target_device_name = std::string());
    // static std::vector<std::string> getSNfromName(std::string target_device_name);
    
    StereoDriver(std::string dev_sn_str = "");
    ~StereoDriver();
    void setOption(rs2_option option, float value);
    float getOption(rs2_option option);
    std::string getDeviceName(){return _dev_name_str;};

    void enableAE(uint32_t meanIntensitySetPoint);
    void disableAE(){setOption(RS2_OPTION_ENABLE_AUTO_EXPOSURE,0);};
    rs2::option_range getOptionRange(rs2_option option);
    void enablePoseMotionStream();
    void enableStereoStream(int width = 0, int height = 0, int hz = 0, rs2_format stream_format = RS2_FORMAT_ANY);
    void startPipe();
    rs2_intrinsics get_intrinsics() const; // only call this after pipe started
    rs2_extrinsics get_extrinsics_left_to_right() const;
    float get_baseline() const;
    void stopPipe();
    void registerCallback(callbackStereo cb);
    void registerCallback(callbackGyro cb);
    void registerCallback(callbackAccel cb);
    void registerCallback(callbackIMU cb);
private:
    
    // The following struct assumes accel is always arriving slower than gyro, and at a lower frequency than gyro
    struct IMUBuffer{
        const static uint buffer_size = 256; // size of char
        GyroDataType gyro[buffer_size];
        AccelDataType last_accel;

        unsigned char begin = 0; // at the position of the first data
        unsigned char end = 0; // one position pass the last data

        IMUBuffer(){
            last_accel.seq = 0;
        }

        void pushGyro(GyroDataType data){
            gyro[end++] = data;
        }
        void update(AccelDataType accel, std::function<void(const SyncedIMUDataType&)> cb){

            if (last_accel.seq != 0){ // it is properly initialised 
                // publish all imu data in the buffer

                // std::cout  << "begin = " << gyro[begin].seq << ", end = " << gyro[end-1].seq << std::endl;
                double duration = accel.timestamp - last_accel.timestamp;

                double grad_x = (accel.x - last_accel.x) / duration;
                double grad_y = (accel.y - last_accel.y) / duration;
                double grad_z = (accel.z - last_accel.z) / duration;
                
                // last_accel.timestamp is always older than the gyro[begin]
                for(; gyro[begin].timestamp < accel.timestamp && begin != end ; begin++){ 
                    double lapse = (gyro[begin].timestamp - last_accel.timestamp);

                    SyncedIMUDataType imu_data;
                    imu_data.timestamp = gyro[begin].timestamp;
                    imu_data.timestamp = gyro[begin].seq;
                    imu_data.gx = gyro[begin].x;
                    imu_data.gy = gyro[begin].y;
                    imu_data.gz = gyro[begin].z;

                    imu_data.ax = lapse * grad_x + last_accel.x;
                    imu_data.ay = lapse * grad_y + last_accel.y;
                    imu_data.az = lapse * grad_z + last_accel.z;

                    cb(imu_data);
                }
            }

            last_accel = accel;
        }
    }imuBuffer;

    bool init();
    void frameCallback(const rs2::frame& frame);
    void imuCallback(const SyncedIMUDataType& data);

    std::string _dev_sn_str, _dev_name_str;
    static rs2::context _ctx; // related to LIBUVC interface claiming?
    rs2::config _cfg;
    rs2::pipeline* _pipe;
    rs2::device* _dev;
    rs2::sensor* _stereo;
    rs2::pipeline_profile* _profile;
    rs2_stream _stereo_stream_type;

    rs2_intrinsics _intrinsics;
    rs2_extrinsics _extrinsics_left_to_right;
    float _baseline;
    int _laser_power; // range 0 - 360, step 30. default 150
    int _w,_h;

    // bool _isStreaming;

    std::mutex callback_stereo_mutex;
    // std::thread _thread;

    std::vector<callbackStereo> _cblist_stereo;
    std::vector<callbackPose> _cblist_pose;
    std::vector<callbackGyro> _cblist_gyro;
    std::vector<callbackAccel> _cblist_accel;
    std::vector<callbackIMU> _cblist_imu;
};


#endif /* STEREO_INTERFACE_HPP */

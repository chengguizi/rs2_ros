// ROS Node for Realsense D415 Streams
// Cheng Huimin, July 2018
//
// ROS Interface for publishing images

#ifndef ROS_PUBLISHER_H
#define ROS_PUBLISHER_H

#include <ros/ros.h>
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <sensor_msgs/CameraInfo.h>

namespace image_transport
{
    class ImageTransport;
    class CameraPublisher;
}

class StereoCameraPublisher
{
    public:
        StereoCameraPublisher();
        StereoCameraPublisher(const ros::NodeHandle& nh);

        void publish(cv::Mat imageLeft_cv, cv::Mat imageRight_cv, sensor_msgs::CameraInfo cameraInfo_left, sensor_msgs::CameraInfo cameraInfo_right, ros::Time sensor_timestamp, uint64_t seq);
    private:
        ros::NodeHandle _nh;
        image_transport::ImageTransport* _it;
        image_transport::CameraPublisher* _pubLeft;
        image_transport::CameraPublisher* _pubRight;
        void init();
};

class IMUPublisher
{
    public:
        IMUPublisher();
        IMUPublisher(const ros::NodeHandle& nh);

    void publish(const float gyro[3], const float accel[3], const ros::Time timestamp, const uint64_t seq);

    private:
        ros::NodeHandle _nh;
        ros::Publisher _pub;
};

#endif /* ROS_PUBLISHER_H */

// ROS Node for Realsense D415 Streams
// Cheng Huimin, June 2018
//
//
#include "ros_publisher.hpp"

#include <image_transport/image_transport.h>
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>

#include <iostream>

#define BUFFER_SIZE 20


StereoCameraPublisher::StereoCameraPublisher()
{
    StereoCameraPublisher(ros::NodeHandle("~"));
}


StereoCameraPublisher::StereoCameraPublisher(const ros::NodeHandle& nh) : _nh(nh)
{
    _it = new image_transport::ImageTransport(_nh);
    _pubLeft = new auto( _it->advertiseCamera("left/image_rect_raw",BUFFER_SIZE));
    _pubRight = new auto( _it->advertiseCamera("right/image_rect_raw",BUFFER_SIZE));
    std::cout << "Stereo Publisher initialised." << std::endl;
}

void StereoCameraPublisher::publish(cv::Mat imageLeft_cv, cv::Mat imageRight_cv, sensor_msgs::CameraInfo cameraInfo_left,
        sensor_msgs::CameraInfo cameraInfo_right,  ros::Time sensor_timestamp, uint64_t seq)
{
    std_msgs::Header header;
    header.stamp = sensor_timestamp;
    header.seq = seq;

    // add header to the cameraInfo
    cameraInfo_left.header = header;
    cameraInfo_right.header = header;

    // std::cout << "Publishing: " << sensor_timestamp << std::endl;

    // convert to pointer format
    sensor_msgs::CameraInfoConstPtr cameraInfoPtr_left = boost::make_shared<sensor_msgs::CameraInfo>(cameraInfo_left);
    sensor_msgs::CameraInfoConstPtr cameraInfoPtr_right = boost::make_shared<sensor_msgs::CameraInfo>(cameraInfo_right);

    // publish left image
    cv_bridge::CvImage imageLeft_bridge = cv_bridge::CvImage(header, \
                sensor_msgs::image_encodings::MONO8, imageLeft_cv);

    _pubLeft->publish(imageLeft_bridge.toImageMsg(),cameraInfoPtr_left);

    // publish right image
    cv_bridge::CvImage imageRight_bridge = cv_bridge::CvImage(header, \
                sensor_msgs::image_encodings::MONO8, imageRight_cv);

    _pubRight->publish(imageRight_bridge.toImageMsg(),cameraInfoPtr_right);
}

IMUPublisher::IMUPublisher()
{
    IMUPublisher(ros::NodeHandle("~"));
}

IMUPublisher::IMUPublisher(const ros::NodeHandle& nh) : _nh(nh)
{
    _pub = _nh.advertise<sensor_msgs::Imu>("imu",BUFFER_SIZE);
    std::cout << "Imu Publisher initialised." << std::endl;
}

void IMUPublisher::publish(float gyro[3], float accel[3], ros::Time timestamp, uint64_t seq)
{
    sensor_msgs::Imu data;

    data.header.stamp = timestamp;
    data.header.seq = seq;
    data.angular_velocity.x = gyro[0];
    data.angular_velocity.y = gyro[1];
    data.angular_velocity.z = gyro[2];

    data.linear_acceleration.x = accel[0];
    data.linear_acceleration.y = accel[1];
    data.linear_acceleration.z = accel[2];

    _pub.publish(data);
    
}
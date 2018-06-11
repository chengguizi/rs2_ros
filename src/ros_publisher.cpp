// ROS Node for Realsense D415 Streams
// Cheng Huimin, June 2018
//
//
#include "ros_publisher.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <iostream>

#define BUFFER_SIZE 3


StereoOdometerPublisher::StereoOdometerPublisher()
{
    ros::NodeHandle _nh("~");
    init();
}


StereoOdometerPublisher::StereoOdometerPublisher(ros::NodeHandle& nh) : _nh("~")
{
    init();
}


StereoOdometerPublisher::~StereoOdometerPublisher()
{
    std::cout << "StereoOdometerPublisher() destructor." << std::endl;
}

void StereoOdometerPublisher::init()
{
    _it = new image_transport::ImageTransport(_nh);
    _pubLeft = new auto( _it->advertiseCamera("left/image_rect_raw",BUFFER_SIZE));
    _pubRight = new auto( _it->advertiseCamera("right/image_rect_raw",BUFFER_SIZE));
    std::cout << "Publisher initialised." << std::endl;
}

void StereoOdometerPublisher::publish(cv::Mat imageLeft_cv, cv::Mat imageRight_cv, sensor_msgs::CameraInfo cameraInfo_left,
        sensor_msgs::CameraInfo cameraInfo_right,  ros::Time sensor_timestamp, uint64_t seq)
{
    std_msgs::Header header;
    header.stamp = sensor_timestamp;
    header.seq = seq;

    // add header to the cameraInfo
    cameraInfo_left.header = header;
    cameraInfo_right.header = header;

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
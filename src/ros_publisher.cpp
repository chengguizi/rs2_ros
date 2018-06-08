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
    _pubLeft = new auto( _it->advertise("left",BUFFER_SIZE));
    _pubRight = new auto( _it->advertise("right",BUFFER_SIZE));
    std::cout << "Publisher initialised." << std::endl;
}

void StereoOdometerPublisher::publish(cv::Mat imageLeft_cv, cv::Mat imageRight_cv, ros::Time sensor_timestamp, uint64_t seq)
{
    std_msgs::Header header;
    header.stamp = sensor_timestamp;
    header.seq = seq;
    cv_bridge::CvImage imageLeft_bridge = cv_bridge::CvImage(header, \
                sensor_msgs::image_encodings::MONO8, imageLeft_cv);


    sensor_msgs::ImagePtr imageLeft_msg = imageLeft_bridge.toImageMsg();
    
    _pubLeft->publish(imageLeft_msg);
}
// ROS Node for Realsense D415 Streams
// Cheng Huimin, July 2018
//
// ROS Interface for Receiving Images

#ifndef ROS_SUBSCRIBER_HPP
#define ROS_SUBSCRIBER_HPP

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <image_transport/subscriber_filter.h>

class StereoCameraSubscriber{

public:

private:
    image_transport::SubscriberFilter left_sub_, right_sub_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> left_info_sub_, right_info_sub_;

    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactPolicy;
    typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
    
};

#endif /* ROS_SUBSCRIBER_HPP */

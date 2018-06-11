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

class StereoOdometerPublisher
{
    public:
        StereoOdometerPublisher();
        StereoOdometerPublisher(ros::NodeHandle& nh);
        ~StereoOdometerPublisher();

        void publish(cv::Mat imageLeft_cv, cv::Mat imageRight_cv, sensor_msgs::CameraInfo cameraInfo_left, sensor_msgs::CameraInfo cameraInfo_right, ros::Time sensor_timestamp, uint64_t seq);
    private:
        ros::NodeHandle _nh;
        image_transport::ImageTransport* _it;
        image_transport::CameraPublisher* _pubLeft;
        image_transport::CameraPublisher* _pubRight;
        void init();
};

#endif /* ROS_PUBLISHER_H */

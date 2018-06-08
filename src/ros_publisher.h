#ifndef ROS_PUBLISHER_H
#define ROS_PUBLISHER_H

#include <ros/ros.h>
#include <opencv2/opencv.hpp>   // Include OpenCV API

namespace image_transport
{
    class ImageTransport;
    class Publisher;
}

class StereoOdometerPublisher
{
    public:
        StereoOdometerPublisher();
        StereoOdometerPublisher(ros::NodeHandle& nh);
        ~StereoOdometerPublisher();

        void publish(cv::Mat imageLeft_cv, cv::Mat imageRight_cv, ros::Time sensor_timestamp, uint64_t seq);
    private:
        ros::NodeHandle _nh;
        image_transport::ImageTransport* _it;
        image_transport::Publisher* _pubLeft;
        image_transport::Publisher* _pubRight;
        void init();
};

#endif /* ROS_PUBLISHER_H */

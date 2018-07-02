
#include "stereo_processor.h"

#include "exposure_ctl.hpp"

#include <cv_bridge/cv_bridge.h>

#include <iostream>

class FPGABridge : public StereoProcessor{

public:
    FPGABridge() : StereoProcessor("raw",10){

    }

    ExposureControl exposurectl;
private:
    void imageCallback(
        const sensor_msgs::ImageConstPtr l_image_msg,
		const sensor_msgs::ImageConstPtr r_image_msg,
		const sensor_msgs::CameraInfoConstPtr l_info_msg,
		const sensor_msgs::CameraInfoConstPtr r_info_msg
    )
    {
        std::cout << "frame: " << l_info_msg->header.seq << std::endl;

        static int seq = -1;
        if (seq < 0 ) // initialisation
        {
            seq = l_info_msg->header.seq;
        }else if (seq + 1 != l_info_msg->header.seq)
        {
            ROS_WARN_STREAM("imageCallback(): Missed (" << l_info_msg->header.seq - (seq + 1) << ") frame(s)." );
        }

        seq = l_info_msg->header.seq;

        //ROS_ASSERT(l_info_msg->header.seq == r_info_msg->header.seq);

        cv_bridge::CvImageConstPtr l_cv_ptr, r_cv_ptr;
        l_cv_ptr = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::MONO8); // CV_8UC1
        r_cv_ptr = cv_bridge::toCvShare(r_image_msg, sensor_msgs::image_encodings::MONO8);

        int width = l_image_msg->width;
        int height =  l_image_msg->height;

        cv::Mat cv_leftImg_source(cv::Size(width, height), CV_8UC1, (void *)l_cv_ptr->image.data, cv::Mat::AUTO_STEP);
		cv::Mat cv_rightImg_source(cv::Size(width, height), CV_8UC1, (void *)r_cv_ptr->image.data, cv::Mat::AUTO_STEP);

        exposurectl.calcHistogram(cv_leftImg_source,10000,100);
        exposurectl.showHistogram();
        cvWaitKey(1);
    }
};

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "rs2_fpga_bridge");
    ros::NodeHandle nh("~");

    FPGABridge bridge;

    ros::spin();

    return 0;
}
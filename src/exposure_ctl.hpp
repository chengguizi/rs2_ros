// ROS Node for Realsense D415 Streams
// Cheng Huimin, July 2018
//
// Exposure Control Based on Histogram

#ifndef EXPOSURE_CTL_HPP
#define EXPOSURE_CTL_HPP

#include <opencv2/opencv.hpp>   // calcHist

class ExposureControl{

public:
    void calcHistogram(cv::Mat img, int exposure_usec, int gain, int histSize = 256, bool normalisedtoOne = true);
    int EstimateMeanLuminance();
    const cv::Mat getHist(){return hist;}
    void showHistogram(int exposure_usec = 0, int gain = 0);

    // cv::Mat DeVignetting(cv::Mat src);

private:
    const int p_width = 16;

    int img_width, img_height;

    cv::Mat hist;
    double bkBrightness;
    double P_acc;

    cv::Mat histPeaks;
    int histSize;

    struct Peak{
        bool valid;
        int idx;
        float value;
    };
    Peak peak1, peak2; // peak1 darker than peak2

    struct Curve{
        double a;
        double b;
        double c;
    }W_dark, W_bright;

    double weightDarkPeak, weightBrightPeak;

    void findPeaks();
    void calcWeights(); // must be called after findPeaks()

};

#endif /* EXPOSURE_CTL_HPP */

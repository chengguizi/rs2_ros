// ROS Node for Realsense D415 Streams
// Cheng Huimin, July 2018
//
// Exposure Control Based on Histogram

#ifndef EXPOSURE_CTL_HPP
#define EXPOSURE_CTL_HPP

#include <opencv2/opencv.hpp>   // calcHist

class ExposureControl{

public:
    struct Param{
        int exposure_max, exposure_min;
        int exposure_step;
        int gain_max, gain_min;
        int gain_step;
        double exposure_change_rate;
        int exposure_target_mean;
        int exposure_dead_region;
        double a_min, a_max; // range of suppression
        double c; // centre point of the parabola
    };

    ExposureControl(const Param& param) : param(param) {};

    void calcHistogram(cv::Mat img, int exposure_usec, int gain, int histSize = 256);
    int EstimateMeanLuminance(); // this requires calcHistogram()
    void updateExposureGain(const int& MeanLuminance, const int& exposure_usec,const int& gain, int& exposure_usec_next, int& gain_next, bool do_smoothing);

    const cv::Mat getHist(){return hist;}
    void getIntensityRange(int &min, int &max){min = intensity_min; max = intensity_max;}
    void showHistogram(int exposure_usec = 0, int gain = 0);


private:

    Param param;

    const int p_width = 16;

    int img_width, img_height;

    int intensity_min, intensity_max;

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

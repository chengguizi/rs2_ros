// ROS Node for Realsense D415 Streams
// Cheng Huimin, July 2018
//
// Exposure Control Based on Histogram

#include "exposure_ctl.hpp"

#include <utility>
#include <cmath>
#include <cassert>
#include <string>

#include <ros/ros.h>


void ExposureControl::calcHistogram(cv::Mat img, int exposure_usec, int gain, int histSize, bool normalisedtoOne)
{
    this->histSize = histSize;
    cv::calcHist(&img,1 /*num of images*/, 0 /*channels*/, cv::Mat() /*mask*/, 
        hist, 1 /*output dimensions*/,  &histSize/* histogram size*/, 0 /*ranges*/ );

    normalize(hist, hist,1,0,cv::NORM_L1); // normalised histogram
    // assert( fabs(cv::sum(hist)[0] - 1) < 1e-3 );

    histPeaks = cv::Mat(hist.size(),CV_32FC1);

    for (int i = 0 ; i < histSize ; i++)
    {
        float sum = 0;
        for (int j = 0; j < p_width && i+j < histSize ; j++)
        {
            sum += hist.at<float>(i+j);
        }
        histPeaks.at<float>(i) = sum;
    }
    // SHOULD NOT NORMALISE PEAKS
    //normalize(histPeaks, histPeaks,1,0,cv::NORM_L1);

    img_width = img.cols;
    img_height = img.rows;
    P_acc = cv::sum(img)[0] / (img_width * img_height);
    
    const double k = 5e3;
    double raw_brightness = k * P_acc / (exposure_usec * gain);

    // logistic function
    double f_logistic = 1.0 / ( 1.0 + std::exp( - (raw_brightness - 6.0)) ); // input shouldbe around 0~1, output is strictly 0 ~ 1

    bkBrightness = f_logistic; //std::min(1.0, f_logistic);
    // std::cout << "bkBrightness: " << bkBrightness << std::endl;
    assert (bkBrightness>=0 && bkBrightness<=1);

    const double c = 0.6;
    const double a_min = 0.15;
    const double a_max = 0.8;
    W_dark.c = W_bright.c = c; // arbituary constant

    W_dark.a = a_max - bkBrightness*(a_max-a_min);
    W_bright.a = a_min + bkBrightness*(a_max-a_min);

    W_dark.b = (1 - W_dark.a) / std::pow(1 - W_dark.c, 2.0);
    W_bright.b = (1 - W_bright.a) / std::pow(1 - W_bright.c, 2.0);

    findPeaks();
    calcWeights();
}

void ExposureControl::findPeaks()
{
    peak1 = {};
    peak2 = {};

    enum Slope{
        ASCENDING,
        DECENDING
    };

    Slope lastSlope = ASCENDING;
    float pre_min = 0;
    for (int i = 1; i < histSize - 1; i++)
    {
        float pre = histPeaks.at<float>(i-1);
        float cur = histPeaks.at<float>(i);
        if (cur < pre)
        {
            if (lastSlope == ASCENDING && pre > pre_min*1.05) // maxima detected
            {
                if ( pre > peak1.value)
                {
                    peak2 = peak1;
                    peak1.idx = i-1;
                    peak1.value = pre;
                    peak1.valid = true;
                }else if (pre > peak2.value)
                {
                    peak2.idx = i-1;
                    peak2.value = pre;
                    peak2.valid = true;
                }
            }
            lastSlope = DECENDING;
        }else if (cur > pre){
            if (lastSlope == DECENDING)
                pre_min = pre;
            lastSlope = ASCENDING;  
        }
    }

    // after the loop, peak1 is the highest peak, peak2 is the 2nd highest

    if (peak1.idx > peak2.idx) // peak1 is always on the left of peak2
        std::swap(peak1,peak2);

    // check if peak1 is indeed dark
    const int threshold = 5;
    if (peak1.valid && peak1.idx > 128 - threshold) // means both peak1 && peak2 bright
    {
        peak1.valid = false;
    }

    if (peak2.valid && peak2.idx < 128 + threshold) // means both peak1 && peak2 dark
    {
        peak2.valid = false;
    }

}

void ExposureControl::calcWeights()
{
    assert( ( peak1.value - 1.0 )< 1.0e-3 );

    if (peak1.valid)
        weightDarkPeak = std::min(1.0, W_dark.a + W_dark.b * std::pow(peak1.value - W_dark.c, 2.0));
    else
        weightDarkPeak = 1.0;

    assert(  ( peak2.value -  1.0) < 1.0e-3 );

    if (peak2.valid)
        weightBrightPeak = std::min(1.0, W_bright.a + W_bright.b * std::pow(peak2.value - W_bright.c, 2.0));
    else
        weightBrightPeak = 1.0;

    if (weightDarkPeak < 1)
        ROS_INFO_STREAM_THROTTLE(5,  "level " << peak1.idx  << " is suppressed to " << std::fixed << std::setprecision(2) << weightDarkPeak);

    if ( weightBrightPeak < 1 )
        ROS_INFO_STREAM_THROTTLE(5,  "level "  << peak2.idx << " is suppressed to " << std::fixed << std::setprecision(2) << weightBrightPeak);

    // if (weightDarkPeak < 1 or weightBrightPeak < 1)
        // std::cout << "weightDarkPeak= " << weightDarkPeak << "@ " << peak1.idx << "value=" << peak1.value
        //     << ", weightBrightPeak= " << weightBrightPeak << "@ " <<  peak2.idx << "value=" << peak2.value << std::endl;
}

int ExposureControl::EstimateMeanLuminance()
{
    double P_acc_dark = 0;
    double P_acc_bright = 0;

    for (int i=0; i< p_width && i + peak1.idx < histSize; i++)
        P_acc_dark += (i + peak1.idx)*hist.at<float>(i + peak1.idx);
    
    for (int i=0; i< p_width && i + peak2.idx < histSize; i++)
        P_acc_bright += (i + peak2.idx)*hist.at<float>(i + peak2.idx);


    double luminanceExcludingRONI = P_acc - P_acc_dark*(1-weightDarkPeak) - P_acc_bright*(1-weightBrightPeak);
    double sizeExcludingRONI =  (1 - peak1.value*(1-weightDarkPeak) - peak2.value*(1-weightBrightPeak)); // normalised

    double MeanLuminance = luminanceExcludingRONI / sizeExcludingRONI;

    // if ( false ) // detect wrong stuff!
    // {
    //     std::cout  << "P_acc: " << P_acc << ", P_acc_dark: " << P_acc_dark <<  ", P_acc_bright: " << P_acc_bright << std::endl;
    // }

    ROS_INFO_STREAM_THROTTLE(30, "MeanLuminance: " << (int)MeanLuminance);

    return (int)MeanLuminance;
}

void ExposureControl::showHistogram(int exposure_usec, int gain)
{
    // initialised to be all white
    cv::Mat histImage = cv::Mat::ones(200 /*rows or height*/ , histSize*2 /*cols or width*/, CV_8UC3)*255; 
    //cv::Mat hist = this->hist
    // normalised 
    // normalize(hist, hist, 0, histImage.rows, cv::NORM_MINMAX, CV_32F);

    cv::Mat hist = this->hist * 200.0;
    histImage = cv::Scalar::all(255);
    int binW = cvRound((double)histImage.cols/histSize);

    for( int i = 0; i < histSize; i++ )
        rectangle( histImage, cv::Point(i*binW, histImage.rows),
                   cv::Point((i+1)*binW, histImage.rows - cvRound(hist.at<float>(i))),
                   cv::Scalar::all(0), -1, 8, 0 );
    imshow("histogram", histImage);

    hist = this->histPeaks * 200.0;
    histImage = cv::Scalar::all(255);
    binW = cvRound((double)histImage.cols/histSize);
    for( int i = 0; i < histSize; i++ )
    {
        cv::Scalar color = (i == peak1.idx || i == peak2.idx ) ? cv::Scalar(255,0,0) : cv::Scalar::all(0);
        rectangle( histImage, cv::Point(i*binW, histImage.rows),
                   cv::Point((i+1)*binW - 1, histImage.rows - cvRound(hist.at<float>(i))),
                   color, -1, 8, 0 );
        if (i == peak1.idx && peak1.valid)
            rectangle( histImage, cv::Point(i*binW, histImage.rows - cvRound(hist.at<float>(i))+1),
                   cv::Point((i+1)*binW - 1, 0),
                   cv::Scalar(100,100,100), -1, 8, 0 );
        
        if (i == peak2.idx && peak2.valid)
            rectangle( histImage, cv::Point(i*binW, histImage.rows - cvRound(hist.at<float>(i))+1),
                   cv::Point((i+1)*binW - 1, 0),
                   cv::Scalar(200,200,200), -1, 8, 0 );
        
        // put text for exposure and gain
        cv::putText (histImage, "expo:" +  std::to_string(exposure_usec), cv::Point(histSize*2 - 200, 25), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar::all(80));
        cv::putText (histImage, "gain:" +  std::to_string(gain), cv::Point(histSize*2 - 200, 50), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar::all(80));
    }
        
    imshow("histogram Peaks", histImage);
    cvWaitKey(1);
}

cv::Mat Devignetting(cv::Mat src)
{
    // cv::Mat kernel
}
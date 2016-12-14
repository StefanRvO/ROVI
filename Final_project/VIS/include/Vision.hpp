#ifndef VISION_H
#define VISION_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <string>


class Vision
{
private:
    /*
    static cv::Mat hsvImg;
    static const int hSlider = 180;
    static int hSliderMin;
    static int hSliderMax;

    static const int sSlider = 255;
    static int sSliderMin;
    static int sSliderMax;

    static const int vSlider = 255;
    static int vSliderMin;
    static int vSliderMax;*/

    static void displayImage(std::string name, cv::Mat &image);
    cv::Mat getThresholdImage2(const cv::Mat &inImg, cv::Scalar redThresh, cv::Scalar blueThresh);
    cv::Mat applyHsvThreshold(const cv::Mat &inImg, const cv::Scalar minThresh, const cv::Scalar maxThresh);
    std::vector<std::vector<cv::Point>> getContours(cv::Mat inImg, float compactThresh, int areaTresh);
    cv::Point2f getCOG(std::vector<cv::Point> contour);
    //static void on_trackbar( int, void* );
    //void applyHsvTrackbar(const cv::Mat &inImg);
public:
    Vision();
    std::vector<cv::Point2f> trackPicture(cv::Mat inImg);
    cv::Mat getVisionViewImage(cv::Mat inImg, std::vector<cv::Point2f> contourCenters);
};

#endif // VISION_H

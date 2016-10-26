#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>

class DftUtilities
{
  private:
    cv::Mat image;
    int rows;
    int cols;
    int imgRows;
    int imgCols;
    cv::Mat newImage;
    cv::Mat_<float> imgs[2];
    cv::Mat_<cv::Vec2f> img_dft;
    cv::Mat_<float> magnitude, phase;
    cv::Mat_<float> magnitudel;   // Logarithm of magnitude
    void dftshift(cv::Mat_<float>& magnitude);
    float butterworthFilter(int x, int y, int v, int u, float d0, int order);

  public:
    DftUtilities();
    void computeDFT(cv::Mat image);
    void setMagnitude(cv::Mat_<float> magnitude);
    cv::Mat_<float> getMagnitude();
    cv::Mat_<float> getDFTImage();
    void plotDFT();
    void plotImage();
    cv::Mat getImage();
    void applyFilter(int v, int u, float d0, int order);

};

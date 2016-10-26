#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <algorithm>

using namespace std;
using namespace cv;

class SpatialDomain
{
  private:
    Mat getHistogram(const Mat &image);
    unsigned char applyMedianFilter(const Mat &image, int &xPos, int &yPos,
    int &maskSize, std::vector<unsigned char> &values, float percenttile);
    unsigned char applyAdaptiveMedianFilter(const Mat &image, int &xPos, int &yPos,
        int init_mask, int &max_mask, std::vector<unsigned char> &values);
    std::vector<unsigned char> to_ignore;
    bool ignore_pixel(unsigned char pixel);

    unsigned char applyAdaptiveMedianFilter_B(int Z_xy, int Z_min, int Z_max, int Z_median);


  public:
    SpatialDomain();
    Mat histEqualize(const Mat &image);
    Mat createHistogramImage(const Mat &image);
    Mat medianFilter(const Mat &image, int maskSize, float percenttile = 0.5);
    Mat adaptiveMedianFilter(const Mat &image, int init_mask, int max_mask);
    void add_ignored_pixel(unsigned char val) { to_ignore.push_back(val); }
    //void plotHistogram();
};

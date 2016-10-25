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
    std::vector<unsigned char> to_ignore;
    bool ignore_pixel(unsigned char pixel);
  public:
    SpatialDomain();
    Mat histEqualize(const Mat &image);
    Mat createHistogramImage(const Mat &image);
    Mat medianFilter(const Mat &image, int maskSize, float percenttile = 0.5);
    void add_ignored_pixel(unsigned char val) { to_ignore.push_back(val); }
    //void plotHistogram();
};

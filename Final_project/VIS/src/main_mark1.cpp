#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include "helper_funcs.hpp"
#include "Vision.hpp"

using namespace std;


int main(int argc, char** argv)
{
    srand(time(NULL));
    // Read the picture and show it
    cv::Mat img = cv::imread(argv[1]);
    Vision findCircles;

    findCircles.trackPicture(img);

}

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include "helper_funcs.hpp"
#include "Vision.hpp"
#include <chrono>

using namespace std::chrono;

using namespace std;


int main(int argc, char** argv)
{
    Vision findCircles;

    for(uint32_t i = 1; i < argc; i++)
    {

        cv::Mat img = cv::imread(argv[i]);
        auto start_time = duration_cast< nanoseconds >
            (system_clock::now().time_since_epoch());
        auto markers = findCircles.trackPicture(img);

        auto end_time = duration_cast< nanoseconds >
            (system_clock::now().time_since_epoch());
        std::cout << i << "\t" << markers.size() << "\t" << (end_time - start_time).count() << std::endl;
    }

}

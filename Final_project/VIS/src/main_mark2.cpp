#include "LineFinding.hpp"
#include <chrono>

/** @function main */
using namespace cv;
using namespace std::chrono;

int main( int argc, char** argv )
{
 //perform canny and sobol to find edges and their gradient.
 //Perform houghs line transform on canny edges to find lines
 //verify the lines by checking the colour on each side using the gradient.
 //They need to be black on one side and white on another (within some threshold.)
 for(uint32_t i = 1; i < argc; i++)
 {

     cv::Mat img = cv::imread(argv[i]);
     auto start_time = duration_cast< nanoseconds >
         (system_clock::now().time_since_epoch());
     LineFinding linefinder(img);
     auto markers = linefinder.get_marker_points(&img);
     auto end_time = duration_cast< nanoseconds >
         (system_clock::now().time_since_epoch());
     std::cout << i << "\t" << markers.size() << "\t" << (end_time - start_time).count() << std::endl;
 }
 //displayImage(img, "test2");


 return 0;
}

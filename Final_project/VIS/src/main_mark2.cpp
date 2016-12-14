#include "LineFinding.hpp"
/** @function main */
using namespace cv;
int main( int argc, char** argv )
{
 //perform canny and sobol to find edges and their gradient.
 //Perform houghs line transform on canny edges to find lines
 //verify the lines by checking the colour on each side using the gradient.
 //They need to be black on one side and white on another (within some threshold.)
 cv::Mat img = cv::imread(argv[1]);
 LineFinding linefinder(img);
 auto markers = linefinder.get_marker_points();
 for(uint8_t i = 0; i < markers.size(); i++)
 {
     cv::circle(img, markers[i],  5, Scalar( (i * 1000) % 256,i * 60,255 - i * 60),  CV_FILLED);
 }
 displayImage(img, "test");
 waitKey(0);

  return 0;
}

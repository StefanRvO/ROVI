#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <vector>
#include <cmath>
#include <vector>

using namespace std;
using namespace cv;

// Global variables
cv::Mat hsvImg;

const int hSlider = 180;
int hSliderMin = 0;
int hSliderMax = 180;

const int sSlider = 255;
int sSliderMin = 0;
int sSliderMax = 255;

const int vSlider = 255;
int vSliderMin = 0;
int vSliderMax = 255;
void on_trackbar( int, void* )
{
    // Segmentate image from HSV trackbar values
    cv::Mat dstImg;
    cv::inRange(hsvImg, cv::Scalar(hSliderMin, sSliderMin, vSliderMin), cv::Scalar(hSliderMax, sSliderMax, vSliderMax), dstImg);
    imshow( "HSV Colour Segmentation", dstImg);
}

cv::Mat applyHSV(cv::Mat &src_img)
{
    cv::Mat dstImg;
    cv::inRange(hsvImg, cv::Scalar(hSliderMin, sSliderMin, vSliderMin), cv::Scalar(hSliderMax, sSliderMax, vSliderMax), dstImg);
    return src_img;
}

cv::Mat applyHsvTrackbar(const cv::Mat &inImg)
{
    cv::cvtColor(inImg, hsvImg, CV_BGR2HSV);    // Convert image to HSV

    // Create a window with trackbars that allow you to find the HSV values
    namedWindow("Colour Segmentation", cv::WINDOW_AUTOSIZE);

    cv::createTrackbar( "Hue min", "Colour Segmentation", &hSliderMin, hSlider, on_trackbar );
    cv::createTrackbar( "Hue max", "Colour Segmentation", &hSliderMax, hSlider, on_trackbar );
    cv::createTrackbar( "S min", "Colour Segmentation", &sSliderMin, sSlider, on_trackbar );
    cv::createTrackbar( "S max", "Colour Segmentation", &sSliderMax, sSlider, on_trackbar );
    cv::createTrackbar( "V min", "Colour Segmentation", &vSliderMin, vSlider, on_trackbar );
    cv::createTrackbar( "V max", "Colour Segmentation", &vSliderMax, vSlider, on_trackbar );

    cv::waitKey(0);
}

void displayImage(const Mat &image, string name)
{
  namedWindow(name, WINDOW_NORMAL);
  cv::imshow(name, image);
  cv::resizeWindow(name, 600,600);
}


using namespace cv;





cv::Mat performCanny(cv::Mat in_image)
{
    GaussianBlur( in_image, in_image, Size(3,3), 0, 0, BORDER_DEFAULT );

    int edgeThresh = 1;
    int low_thres = 75;
    int ratio = 3;
    int kernel_size = 3;
    cv::Mat detected_edges;
    cv::cvtColor(in_image, detected_edges, CV_BGR2GRAY);
    Canny( detected_edges, detected_edges, low_thres, low_thres*ratio, kernel_size );
    return detected_edges;
}

cv::Mat performSobel(cv::Mat in_image)
{
    cv::Mat BW_image;
    cv::Mat grad;
    GaussianBlur( in_image, in_image, Size(3,3), 0, 0, BORDER_DEFAULT );
    cv::cvtColor(in_image, BW_image, CV_BGR2GRAY);
    int scale = 1;
    int delta = 0;
    int ddepth = CV_16S;
    cv::Mat grad_x, grad_y;
    cv::Mat abs_grad_x, abs_grad_y;
    /// Gradient X
    cv::Sobel( BW_image, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
    cv::convertScaleAbs( grad_x, abs_grad_x );
  /// Gradient Y
    cv::Sobel( BW_image, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
    convertScaleAbs( grad_y, abs_grad_y );
    addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );
    std::cout << (int)abs_grad_x.at<uchar>(683, 98) << std::endl;
    std::cout << (int)abs_grad_y.at<uchar>(683, 98) << std::endl;

    return grad;
}
/** @function main */
int main( int argc, char** argv )
{
 //perform canny and sobol to find edges and their gradient.
 //Perform houghs line transform on canny edges to find lines
 //verify the lines by checking the colour on each side using the gradient.
 //They need to be black on one side and white on another (within some threshold.)
 cv::Mat img = cv::imread(argv[1]);
  cv::Mat detected_edges = performCanny(img);
  displayImage(detected_edges, "test");
  cv::Mat cdst;
  cvtColor(detected_edges, cdst, CV_GRAY2BGR);

  /*vector<Vec2f> lines;
  HoughLines(detected_edges, lines, 1, CV_PI/180, 100, 0, 0 );
  for( size_t i = 0; i < lines.size(); i++ )
  {
     float rho = lines[i][0], theta = lines[i][1];
     Point pt1, pt2;
     double a = cos(theta), b = sin(theta);
     double x0 = a*rho, y0 = b*rho;
     pt1.x = cvRound(x0 + 1000*(-b));
     pt1.y = cvRound(y0 + 1000*(a));
     pt2.x = cvRound(x0 - 1000*(-b));
     pt2.y = cvRound(y0 - 1000*(a));
     line( cdst, pt1, pt2, Scalar(0,0,255), 3, CV_AA);
 }*/
 std::vector<Vec4i> lines;
 HoughLinesP(detected_edges, lines, 1, CV_PI/180, 50, 200, 100 );
 for( size_t i = 0; i < lines.size(); i++ )
 {
   Vec4i l = lines[i];
   line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
 }
  cv::Mat dst;
  dst = Scalar::all(0);
  img.copyTo( dst, detected_edges);
  displayImage(dst, "test2");
 // cv::Mat gradient = performSobel(img);
  displayImage(performSobel(img), "Test3");
  displayImage(cdst, "test4");
  waitKey(0);
  return 0;
  }

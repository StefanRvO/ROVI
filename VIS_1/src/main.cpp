#include <opencv2/opencv.hpp>
#include "DftUtilities.hpp"

using namespace cv;

int main()
{
  DftUtilities dftutilities;

  cv::Mat img = cv::imread("lena.bmp", CV_LOAD_IMAGE_GRAYSCALE);
  cv::imshow("Old image", img);

  dftutilities.computeDFT(img);
  dftutilities.plotDFT();

  // High-pass filter: remove the low frequency parts in the middle of the spectrum
  const int sizef = 50;
  cv::Mat_<float> magnitude = dftutilities.getMagnitude();
  magnitude(cv::Rect(magnitude.cols/2-sizef/2, magnitude.rows/2-sizef/2, sizef, sizef)) = 0.0f;
  dftutilities.setMagnitude(magnitude);

  dftutilities.plotImage();

  cv::waitKey();

  return 0;
}

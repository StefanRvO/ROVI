#include "DftUtilities.hpp"
#include <cmath>
DftUtilities::DftUtilities()
{
}

void DftUtilities::computeDFT(cv::Mat img)
{
  image = img;

  //Pad the image with borders. getOptimalDFTSize() gives the most optimal image size to calculate fastest.
  //The multiplication by 2, is to avoid wraparounds. See G&W page 251,252 and 263 and dft tutorial. (Typicly A+B-1 ~ 2A is used)
  rows = cv::getOptimalDFTSize(2*image.rows);
  cols = cv::getOptimalDFTSize(2*image.cols);
  imgRows = image.rows;
  imgCols = image.cols;
  cv::copyMakeBorder(image,image,0,rows-imgRows,0,cols-imgCols,cv::BORDER_CONSTANT,cv::Scalar(0));  // Pad image in the bottom and right side

  // Make 2 matrices(array) one for the real and imaginary part. First channel is the picture, second channel is zeroes
  // Merge the 2 matrices into a single 2-channel picture
  imgs[0] = image.clone();
  imgs[1] = cv::Mat_<float>(image.rows, image.cols, 0.0f);
  cv::merge(imgs, 2, img_dft);

  // Compute DFT
  cv::dft(img_dft, img_dft);

  // Split into real and imaginary matrices
  cv::split(img_dft, imgs);

  // Compute magnitude/phase
  cv::cartToPolar(imgs[0], imgs[1], magnitude, phase);

  // Shift quadrants for viewability
  dftshift(magnitude);
}

void DftUtilities::setMagnitude(cv::Mat_<float> magnitude)
{
  this->magnitude = magnitude;
}

cv::Mat_<float> DftUtilities::getMagnitude()
{
  return magnitude;
}

void DftUtilities::plotDFT()
{
  // Take logarithm of magnitude
  magnitudel = magnitude + 1.0f;
  cv::log(magnitudel, magnitudel);

  cv::normalize(magnitudel, magnitudel, 0.0, 1.0, CV_MINMAX); // Normalize magnitude before plotting
  cv::namedWindow("Magnitude", CV_WINDOW_NORMAL);
  cv::imshow("Magnitude", magnitudel);
  cv::resizeWindow("Magnitude", 500,500);
}

cv::Mat_<float> DftUtilities::getDFTImage()
{
  magnitudel = magnitude + 1.0f;
  cv::log(magnitudel, magnitudel);
  cv::normalize(magnitudel, magnitudel, 0.0, 1.0, CV_MINMAX);
  return magnitudel;
}

void DftUtilities::plotImage()
{
  // Shift back quadrants of the spectrum
  dftshift(magnitude);

  // Compute complex DFT output from magnitude/phase
  cv::polarToCart(magnitude, phase, imgs[0], imgs[1]);

  // Merge DFT into one image and restore
  cv::merge(imgs, 2, img_dft);
  cv::dft(img_dft, newImage, cv::DFT_INVERSE + cv::DFT_SCALE + cv::DFT_REAL_OUTPUT);

  //Cut away the borders
  newImage = newImage(cv::Rect(0,0,imgCols,imgRows));

  // Plot image
  cv::normalize(newImage, newImage, 0.0, 1.0, CV_MINMAX);
  cv::imshow("New image", newImage);
}

cv::Mat DftUtilities::getImage()
{
  // Shift back quadrants of the spectrum
  dftshift(magnitude);

  // Compute complex DFT output from magnitude/phase
  cv::polarToCart(magnitude, phase, imgs[0], imgs[1]);

  // Merge DFT into one image and restore
  cv::merge(imgs, 2, img_dft);
  cv::dft(img_dft, newImage, cv::DFT_INVERSE + cv::DFT_SCALE + cv::DFT_REAL_OUTPUT);

  //Cut away the borders
  newImage = newImage(cv::Rect(0,0,imgCols,imgRows));

  // Normalize and return image
  cv::normalize(newImage, newImage, 0.0, 1.0, CV_MINMAX);
  return newImage;
}

void DftUtilities::dftshift(cv::Mat_<float>& magnitude)
{
  const int cx = magnitude.cols/2;
  const int cy = magnitude.rows/2;

  cv::Mat_<float> tmp;
  cv::Mat_<float> topLeft(magnitude, cv::Rect(0, 0, cx, cy));
  cv::Mat_<float> topRight(magnitude, cv::Rect(cx, 0, cx, cy));
  cv::Mat_<float> bottomLeft(magnitude, cv::Rect(0, cy, cx, cy));
  cv::Mat_<float> bottomRight(magnitude, cv::Rect(cx, cy, cx, cy));

  topLeft.copyTo(tmp);
  bottomRight.copyTo(topLeft);
  tmp.copyTo(bottomRight);

  topRight.copyTo(tmp);
  bottomLeft.copyTo(topRight);
  tmp.copyTo(bottomLeft);
}

void DftUtilities::applyFilter(int v, int u, float d0, int order)
{
  // The filter coordinates is moved to origo
  for(int x = -magnitude.cols / 2; x < magnitude.cols / 2; x++)
  {
    for(int y = -magnitude.rows / 2; y< magnitude.rows / 2; y++)
    {
      magnitude.at<float>(y + magnitude.rows / 2 , x + magnitude.cols / 2 ) *= butterworthFilter(x,y,v,u,d0,order);
    }
  }
}

float DftUtilities::butterworthFilter(int x, int y, int v, int u, float d0, int order)
{
  float new_x = x - v;
  float new_y = y - u;
  float equ_1 = sqrt((float)new_x * (float)new_x + (float)new_y * (float)new_y) / d0;
  return 1./(1. + pow(equ_1, -order * 2));
}

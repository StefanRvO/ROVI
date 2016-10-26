#include "SpatialDomain.hpp"

SpatialDomain::SpatialDomain()
{

}


Mat SpatialDomain::createHistogramImage(const Mat &image)
{
    int histSize = 256; // number of bins
    int numOfIm = 1;
    int *channels = nullptr;
    int dimensions = 1;
    float minMaxValue[] = {0.0, 256.0}; //the min and max value of the bins
    const float* ranges[] = {minMaxValue};  //Array of min-max value arrays, so every dimension is assigned min and max
    Mat histogram;
    bool uniform = true;
    bool accumulate = false;

    // Compute the histogram
    calcHist(&image, numOfIm, channels, Mat(), histogram, dimensions, &histSize, ranges, uniform, accumulate);

    int histWidth = 512;
    int histHeight = 400;
    int binWidth = cvRound( (double)histWidth/(double)histSize);
    Mat histImage(histHeight, histWidth, CV_8UC3, Scalar( 0,0,0) );
    for(uint16_t i = 0; i < 256; i++)
    {
        std::cout << histogram.at<float>(i) << std::endl;
    }
    /// Normalize the result to [ 0, histImage.rows ]
    normalize(histogram, histogram, 0, histImage.rows, NORM_MINMAX, -1, Mat() );

    std::cout << "\n\n\n\n" << std::endl;
    /// Draw histogram
    for( int i = 1; i < histSize + 1; i++ )
    {
          line( histImage, Point( binWidth*(i-1), histHeight - cvRound(histogram.at<float>(i-1)) ) ,
                           Point( binWidth*(i), histHeight - cvRound(histogram.at<float>(i)) ),
                           Scalar( 255, 0, 0), 2, 8, 0  );
    }

    return histImage;
}

/*Mat SpatialDomain::histEqualize(const Mat &image)
{
  int MN = image.rows * image.cols; // Number of pixels
  int numOfBins = 256; // number of bins/intensity levels

  Mat histogram = getHistogram(image);
  Mat lookup(1, numOfBins, CV_8U);
  Mat result;
  int sum = 0;

  // Create cummulative histogram
  for(int i = 0; i<numOfBins; i++)
  {
    sum += histogram.at<float>(i);
    lookup.at<uchar>(i) = sum * (numOfBins-1)/MN;
  }

  LUT(image,lookup, result); //apply the lookup table
  return result;
}*/

Mat SpatialDomain::medianFilter(const Mat &image, int maskSize, float percenttile)
{
  Mat newImage = image.clone();
  Mat paddedImage = image.clone();
  std::vector<unsigned char> values(maskSize * maskSize);
  cv::copyMakeBorder(paddedImage, paddedImage, maskSize/2, maskSize/2, maskSize/2, maskSize/2, cv::BORDER_REPLICATE);

  for(int x = maskSize/2; x<paddedImage.cols-(maskSize/2); x++)
  {
    for(int y = maskSize/2; y<paddedImage.rows-(maskSize/2); y++)
    {
      //if(x % 50 == 0) std::cout << x << "\t" << y << "\t" << newImage.cols << "\t" << newImage.rows << std::endl;
      unsigned char pixelValue = applyMedianFilter(paddedImage, x, y, maskSize, values, percenttile);
      newImage.at<uchar>(y-maskSize/2, x-maskSize/2) = pixelValue;
    }
  }
  return newImage;
}


unsigned char SpatialDomain::applyMedianFilter(const Mat &image, int &xPos, int &yPos,
    int &maskSize, std::vector<unsigned char> &values, float percenttile)
{
  //return image.at<uchar>(0 + yPos, 0 + xPos);
  values.clear();
  uint8_t cnt = 0;
  for(int x = -maskSize/2; x <= maskSize / 2; x++)
  {
    for(int y = -maskSize/2; y <= maskSize / 2; y++)
    {
      //std::cout << (int)cnt << "\t" << values.size() << std::endl;
      auto tmp = image.at<uchar>(y + yPos, x + xPos);
      if(this->ignore_pixel(tmp)) continue;
      values.push_back(tmp);
    }
  }
  std::sort(values.begin(), values.end());
  if(values.size() == 0) return image.at<uchar>(yPos, xPos); // Return pixel value if no pixel in filter mask.
  return values[std::min((size_t)(values.size() * percenttile), values.size() - 1)];
}


bool SpatialDomain::ignore_pixel(unsigned char pixel)
{
  for(auto &val : this->to_ignore)
    if(val == pixel) return true;
  return false;
}

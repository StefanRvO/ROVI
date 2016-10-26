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
    /// Normalize the result to [ 0, histImage.rows ]
    normalize(histogram, histogram, 0, histImage.rows, NORM_MINMAX, -1, Mat() );

    /// Draw histogram
    for( int i = 0; i < histSize + 1; i++ )
    {
          line( histImage, Point( binWidth*(i), histHeight) ,
                           Point( binWidth*(i), histHeight - cvRound(histogram.at<float>(i)) ),
                           Scalar( 100, 100, 100), 2, 8, 0  );
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

Mat SpatialDomain::adaptiveMedianFilter(const Mat &image, int init_mask, int max_mask)
{
  Mat newImage = image.clone();
  Mat paddedImage = image.clone();
  std::vector<unsigned char> values(max_mask * max_mask);
  cv::copyMakeBorder(paddedImage, paddedImage, max_mask/2, max_mask/2, max_mask/2, max_mask/2, cv::BORDER_REPLICATE);

  for(int x = max_mask/2; x<paddedImage.cols-(max_mask/2); x++)
  {
    for(int y = max_mask/2; y<paddedImage.rows-(max_mask/2); y++)
    {
      //if(x % 50 == 0) std::cout << x << "\t" << y << "\t" << newImage.cols << "\t" << newImage.rows << std::endl;
      unsigned char pixelValue = applyAdaptiveMedianFilter(paddedImage, x, y, init_mask, max_mask, values);
      newImage.at<uchar>(y-max_mask/2, x-max_mask/2) = pixelValue;
    }
  }
  return newImage;
}


unsigned char SpatialDomain::applyMedianFilter(const Mat &image, int &xPos, int &yPos,
    int &maskSize, std::vector<unsigned char> &values, float percenttile)
{
  //return image.at<uchar>(0 + yPos, 0 + xPos);
  values.clear();
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


unsigned char SpatialDomain::applyAdaptiveMedianFilter(const Mat &image, int &xPos, int &yPos,
    int init_mask, int &max_mask, std::vector<unsigned char> &values)
{
    values.clear();
    for(int x = -init_mask / 2; x <= init_mask / 2; x++)
    {
      for(int y = -init_mask / 2; y <= init_mask / 2; y++)
      {
        //std::cout << (int)cnt << "\t" << values.size() << std::endl;
        auto tmp = image.at<uchar>(y + yPos, x + xPos);
        values.push_back(tmp);
      }
    }
    std::sort(values.begin(), values.end());
    int Z_min = values.front();
    int Z_max = values.back();
    int Z_med = values[values.size() / 2];
    int Z_xy  = image.at<uchar>(yPos, xPos);
    int A1 = Z_med - Z_min;
    int A2 = Z_med - Z_max;
    if(A1 > 0 && A2 < 0) return applyAdaptiveMedianFilter_B(Z_xy, Z_min, Z_max, Z_med);
    init_mask += 2;
    //Recurse. This is very ineffective, but who cares?
    if(init_mask <= max_mask)
        return applyAdaptiveMedianFilter(image, xPos, yPos, init_mask, max_mask, values);
    else
        return Z_med;
}
unsigned char SpatialDomain::applyAdaptiveMedianFilter_B(int Z_xy, int Z_min, int Z_max, int Z_median)
{
    int B1 = Z_xy - Z_min;
    int B2 = Z_xy - Z_max;
    if(B1 > 0 && B2 < 0) return Z_xy;
    return Z_median;
}

#include <opencv2/opencv.hpp>
#include "DftUtilities.hpp"
#include "SpatialDomain.hpp"
#include <string>

using namespace cv;

void displayImage(const Mat &image, string name)
{
  namedWindow(name, WINDOW_NORMAL);
  cv::imshow(name, image);
  cv::resizeWindow(name, 600,600);
}

int main(int argc, char* argv[])
{
  std::string out_dir = "./";
  DftUtilities dftutilities;
  SpatialDomain spatialdomain;


  // Exercise 1
  /*cv::Mat img = cv::imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
  displayImage(img, "Original image");

  Mat histogram_org = spatialdomain.createHistogramImage(img);
  cv::imwrite(out_dir + std::string("historgram_org_1.png"), histogram_org);
  Mat median_filtered = spatialdomain.medianFilter(img, 5, 0.5);
  Mat median_filtered_90 = spatialdomain.medianFilter(img, 5, 0.9);
  spatialdomain.add_ignored_pixel(0);
  Mat median_filtered_ignore = spatialdomain.medianFilter(img, 5, 0.5);

  cv::imwrite(out_dir + std::string("median_filtered_1.png"), median_filtered);
  cv::imwrite(out_dir + std::string("median_filtered_90_1.png"), median_filtered_90);
  cv::imwrite(out_dir + std::string("median_filtered_ignore.png"), median_filtered_ignore);
  cv::imwrite(out_dir + std::string("median_filtered_1_hist.png"), spatialdomain.createHistogramImage(median_filtered));
  cv::imwrite(out_dir + std::string("median_filtered_90_hist.png"), spatialdomain.createHistogramImage(median_filtered_90));
  cv::imwrite(out_dir + std::string("median_filtered_ignore_hist.png"), spatialdomain.createHistogramImage(median_filtered_ignore));


  //cv::waitKey();
  */
  // Exercise2
  cv::Mat img2 = cv::imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
  Mat histogram_org = spatialdomain.createHistogramImage(img2);
  displayImage(img2, "Image 2");
  displayImage(histogram_org, "Original histogram");
  spatialdomain.add_ignored_pixel(0);
  spatialdomain.add_ignored_pixel(255);
  Mat median_filtered = spatialdomain.medianFilter(img2, 5, 0.5);
  Mat histogram_filtered = spatialdomain.createHistogramImage(median_filtered);
  displayImage(median_filtered, "filtred image");
  displayImage(histogram_filtered, "Histogram filtered");


  cv::waitKey();
  return 0;
}

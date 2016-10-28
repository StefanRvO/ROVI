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

void fix_image_2(std::string path, std::string out_dir);

int main(int argc, char* argv[])
{
    try
    {
      std::string out_dir = "./";
      std::string img_folder = argv[1];
      fix_image_2(img_folder + "/Image2.png", out_dir);
    }
    catch(...)
    {
        std::cout << "Give the path to the folder containing the images as argument!" << std::endl;
    }

  return 0;
}


void fix_image_2(std::string path, std::string out_dir)
{
    DftUtilities dftutilities;
    SpatialDomain spatialdomain;
    cv::Mat img2 = cv::imread(path, CV_LOAD_IMAGE_GRAYSCALE);
    Mat histogram_org_2 = spatialdomain.createHistogramImage(img2);
    cv::imwrite(out_dir + std::string("histogram_org_2.png"), histogram_org_2);
    Mat adapt_median_filtered_2 = spatialdomain.adaptiveMedianFilter(img2, 3, 31);
    Mat adapt_median_filtered_2_bil;
    double sigmaSpace = 10;
    double sigmaColor = 10;
    cv::bilateralFilter(adapt_median_filtered_2, adapt_median_filtered_2_bil, 0, sigmaColor, sigmaSpace);

    Mat adapt_histogram_filtered_2 = spatialdomain.createHistogramImage(adapt_median_filtered_2);
    Mat adapt_histogram_median_filtered_2_bil = spatialdomain.createHistogramImage(adapt_median_filtered_2_bil);
    cv::imwrite(out_dir + std::string("adapt_median_filtered_2_bil.png"), adapt_median_filtered_2_bil);
    cv::imwrite(out_dir + std::string("adapt_histogram_median_filtered_2_bil.png"), adapt_histogram_median_filtered_2_bil);

    cv::imwrite(out_dir + std::string("adapt_median_filtered_2.png"), adapt_median_filtered_2);
    cv::imwrite(out_dir + std::string("adapt_histogram_filtered_2.png"), adapt_histogram_filtered_2);
    Mat median_filtered_2 = spatialdomain.medianFilter(img2, 5, 0.5);
    median_filtered_2 = spatialdomain.medianFilter(median_filtered_2, 5, 0.5);
    Mat histogram_filtered_2 = spatialdomain.createHistogramImage(median_filtered_2);
    cv::imwrite(out_dir + std::string("median_filtered_2.png"), median_filtered_2);
    cv::imwrite(out_dir + std::string("histogram_filtered_2.png"), histogram_filtered_2);
    spatialdomain.add_ignored_pixel(0);
    spatialdomain.add_ignored_pixel(255);
    Mat median_filtered_2_ignore = spatialdomain.medianFilter(img2, 5, 0.5);
    Mat histogram_filtered_2_ignore= spatialdomain.createHistogramImage(median_filtered_2_ignore);
    cv::imwrite(out_dir + std::string("median_filtered_2_ignore.png"), median_filtered_2_ignore);
    cv::imwrite(out_dir + std::string("histogram_filtered_2_ignore.png"), histogram_filtered_2_ignore);
}

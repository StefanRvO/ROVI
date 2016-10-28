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

void fix_image_1(std::string path, std::string out_dir);


int main(int argc, char* argv[])
{
    try
    {
      std::string out_dir = "./";
      std::string img_folder = argv[1];
      fix_image_1(img_folder + "/Image1.png", out_dir);
    }
    catch(...)
    {
        std::cout << "Give the path to the folder containing the images as argument!" << std::endl;
    }
  return 0;
}

void fix_image_1(std::string path, std::string out_dir)
{
    DftUtilities dftutilities;
    SpatialDomain spatialdomain;
    cv::Mat img = cv::imread(path, CV_LOAD_IMAGE_GRAYSCALE);
    //displayImage(img, "Original image");

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
}

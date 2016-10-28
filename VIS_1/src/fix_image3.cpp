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

void fix_image_3(std::string path, std::string out_dir);


int main(int argc, char* argv[])
{
    try
    {
      std::string out_dir = "./";
      std::string img_folder = argv[1];
      fix_image_3(img_folder + "/Image3.png", out_dir);
    }
    catch(...)
    {
        std::cout << "Give the path to the folder containing the images as argument!" << std::endl;
    }

  return 0;
}

void fix_image_3(std::string path, std::string out_dir)
{
    DftUtilities dftutilities;
    SpatialDomain spatialdomain;
    cv::Mat img3 = cv::imread(path, CV_LOAD_IMAGE_GRAYSCALE);
    cv::Mat uniform_area = img3(cv::Rect(10,10, 100, 100));
    cv::Mat uniform_histogram = spatialdomain.createHistogramImage(uniform_area);
    Mat histogram_org_3 = spatialdomain.createHistogramImage(img3);
    cv::imwrite(out_dir + std::string("uniform_area_3.png"), uniform_area);
    cv::imwrite(out_dir + std::string("uniform_histogram_3.png"), uniform_histogram);
    cv::imwrite(out_dir + std::string("histogram_org_3.png"), histogram_org_3);

    //Apply bilateral filter.
    double sigmaSpace = 20;
    double sigmaColor = 20;
    cv::Mat filtered_3;
    cv::bilateralFilter(img3, filtered_3, 0, sigmaColor, sigmaSpace);
    //spatialdomain.shift_intensity(filtered_3, -40, 255, 0);
    Mat histogram_filtered_3 = spatialdomain.createHistogramImage(filtered_3);
    Mat uniform_area_filtered = filtered_3(cv::Rect(10,10, 100, 100));
    Mat uniform_area_filtered_hist = spatialdomain.createHistogramImage(uniform_area_filtered);

    cv::imwrite(out_dir + std::string("filtered_3.png"), filtered_3);
    cv::imwrite(out_dir + std::string("histogram_filtered_3.png"), histogram_filtered_3);
    cv::imwrite(out_dir + std::string("uniform_area_filtered_3.png"), uniform_area_filtered);
    cv::imwrite(out_dir + std::string("uniform_area_filtered_hist_3.png"), uniform_area_filtered_hist);
}

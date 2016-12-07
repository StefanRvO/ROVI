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
void fix_image_2(std::string path, std::string out_dir);
void fix_image_3(std::string path, std::string out_dir);
void fix_image_4(std::string path, std::string out_dir);


int main(int argc, char* argv[])
{
  try
  {
      std::string out_dir = "./";
      std::string img_folder = argv[1];
      fix_image_1(img_folder + "/Image1.png", out_dir);
      fix_image_2(img_folder + "/Image2.png", out_dir);
      fix_image_3(img_folder + "/Image3.png", out_dir);
      fix_image_4(img_folder + "/Image4_1.png", out_dir);
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

void fix_image_4(std::string path, std::string out_dir)
{
    DftUtilities dftutilities;
    SpatialDomain spatialdomain;
    cv::Mat img4 = cv::imread(path, CV_LOAD_IMAGE_GRAYSCALE);
    // Compute and plot DFT
    dftutilities.computeDFT(img4);
    cv::Mat frequencyPlot = dftutilities.getDFTImage();
    frequencyPlot.convertTo(frequencyPlot, CV_8U, 255.0);
    cv::imwrite(out_dir + std::string("original_DFT.png"), frequencyPlot);
    // Apply filter
    dftutilities.applyFilter(202,211,100,3);
    dftutilities.applyFilter(-202,-211,100,3);
    dftutilities.applyFilter(605,-619,100,3);
    dftutilities.applyFilter(-605,619,100,3);

    cv::Mat frequencyPlot_filtered = dftutilities.getDFTImage();
    frequencyPlot_filtered.convertTo(frequencyPlot_filtered, CV_8U, 255.0);

    cv::imwrite(out_dir + std::string("filtered_DFT.png"), frequencyPlot_filtered);

    cv::Mat filtered_4 = dftutilities.getImage();
    filtered_4.convertTo(filtered_4, CV_8U, 255.0);

    cv::imwrite(out_dir + std::string("filtered_image_4.png"), filtered_4);
}

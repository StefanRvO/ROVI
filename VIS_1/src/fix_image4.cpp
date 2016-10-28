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

void fix_image_4(std::string path, std::string out_dir);

int main(int argc, char* argv[])
{
    try
    {
      std::string out_dir = "./";
      std::string img_folder = argv[1];
      fix_image_4(img_folder + "/Image4_1.png", out_dir);
    }
    catch(...)
    {
        std::cout << "Give the path to the folder containing the images as argument!" << std::endl;
    }

  return 0;
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

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cmath>

using namespace std;

// Global variables
cv::Mat hsvImg;

const int hSlider = 180;
int hSliderMin = 0;
int hSliderMax = 180;

const int sSlider = 255;
int sSliderMin = 0;
int sSliderMax = 255;

const int vSlider = 255;
int vSliderMin = 0;
int vSliderMax = 255;


void displayImage(string name, cv::Mat &image)
{
  cv::namedWindow(name, cv::WINDOW_NORMAL);
  cv::imshow(name, image);
  cv::resizeWindow(name, 600,600);
}

cv::Mat getThresholdImage(const cv::Mat &inImg)
{
    cv::Mat finalImage = cv::Mat::zeros(inImg.size(), CV_8UC1);    // Create a matrix with zeroes of same size as inImg but only 1 channel
    float meanThreshold = 0.08*255;
    cv::Scalar lowerThresh;
    cv::Scalar upperThresh;

    // Create rectangles bounding the desired colors in the image and push these to a vector
    vector<cv::Rect> colorAreas;
    cv::Rect rect1(650, 160, 50, 50);
    cv::Rect rect2(820, 160, 50, 50);
    //cv::Rect rect3(740, 240, 30, 30);

    colorAreas.push_back(rect1);
    colorAreas.push_back(rect2);
    //colorAreas.push_back(rect3);

    cv::Mat rectImg;
    for(unsigned int i = 0; i<colorAreas.size(); i++)
    {
        rectImg = inImg(colorAreas[i]); // Extract the rectangle from the image
        cv::Scalar colorMean = cv::mean(rectImg);   // Compute the mean of the pixels in the rectangle, returns a mean for each color channel

        // Create a lower- and a upper-threshold in which the pixels may deviate from the mean threshold
        lowerThresh = colorMean;
        upperThresh = colorMean;
        for(int i = 0; i<3; i++)
        {
            lowerThresh[i] -= meanThreshold;
            upperThresh[i] += meanThreshold;
        }

        // Check each pixel and set the pixel value to {0,0,0} if outside the thresholds, otherwise {1,1,1}
        cv::Mat threshImg;
        cv::inRange(inImg, lowerThresh, upperThresh, threshImg);

        finalImage = finalImage + threshImg;    // Combine all the thresholded images together as one
    }

    return finalImage;
}


// Copy of the other function with hardcoded colorMeans
cv::Mat getThresholdImage2(const cv::Mat &inImg, cv::Scalar redThresh, cv::Scalar blueThresh)
{
    cv::Mat finalImage = cv::Mat::zeros(inImg.size(), CV_8UC1);    // Create a matrix with zeroes of same size as inImg but only 1 channel
    float meanThreshold = 0.1*255;
    cv::Scalar lowerThresh;
    cv::Scalar upperThresh;

    // Hardcoded means
    vector<cv::Scalar> colorMean = {redThresh, blueThresh};

    for(unsigned int i = 0; i<colorMean.size(); i++)
    {
        // Create a lower- and a upper-threshold in which the pixels may deviate from the mean threshold
        lowerThresh = colorMean[i];
        upperThresh = colorMean[i];
        for(int i = 0; i<3; i++)
        {
            lowerThresh[i] -= meanThreshold;
            upperThresh[i] += meanThreshold;
        }

        // Check each pixel and set the pixel value to {0,0,0} if outside the thresholds, otherwise {1,1,1}
        cv::Mat threshImg;
        cv::inRange(inImg, lowerThresh, upperThresh, threshImg);

        finalImage = finalImage + threshImg;    // Combine all the thresholded images together as one
    }

    return finalImage;
}


// HSV Trackbar handler
void on_trackbar( int, void* )
{
    // Segmentate image from HSV trackbar values
    cv::Mat dstImg;
    cv::inRange(hsvImg, cv::Scalar(hSliderMin, sSliderMin, vSliderMin), cv::Scalar(hSliderMax, sSliderMax, vSliderMax), dstImg);
    displayImage( "HSV Colour Segmentation", dstImg);
}


cv::Mat applyHsvTrackbar(const cv::Mat &inImg)
{
    cv::cvtColor(inImg, hsvImg, CV_BGR2HSV);    // Convert image to HSV

    // Create a window with trackbars that allow you to find the HSV values
    namedWindow("Colour Segmentation", cv::WINDOW_AUTOSIZE);

    cv::createTrackbar( "Hue min", "Colour Segmentation", &hSliderMin, hSlider, on_trackbar );
    cv::createTrackbar( "Hue max", "Colour Segmentation", &hSliderMax, hSlider, on_trackbar );
    cv::createTrackbar( "S min", "Colour Segmentation", &sSliderMin, sSlider, on_trackbar );
    cv::createTrackbar( "S max", "Colour Segmentation", &sSliderMax, sSlider, on_trackbar );
    cv::createTrackbar( "V min", "Colour Segmentation", &vSliderMin, vSlider, on_trackbar );
    cv::createTrackbar( "V max", "Colour Segmentation", &vSliderMax, vSlider, on_trackbar );

    cv::waitKey(0);
}


cv::Mat applyHsvThreshold(const cv::Mat &inImg, const cv::Scalar minThresh, const cv::Scalar maxThresh)
{
    cv::Mat dstImg;
    cv::cvtColor(inImg, hsvImg, CV_BGR2HSV);    // Convert image to HSV
    cv::inRange(hsvImg, minThresh, maxThresh, dstImg);
    return dstImg;
}


/*
*   Dialate and erodes the image, where after it extracts the contours within a given compact and area threshold
*   The fitting contours are herafter returned as a vector
*/
std::vector<std::vector<cv::Point>> getContours(cv::Mat inImg, float compactThresh, int areaTresh)
{
    // Dilate and erode
    cv::Mat kernel = cv::Mat::ones(3,3,CV_8UC1);
    cv::dilate(inImg,inImg,kernel);
    cv::erode(inImg,inImg,kernel);
    kernel = cv::Mat::ones(13,13,CV_8UC1);
    cv::erode(inImg,inImg,kernel);
    cv::dilate(inImg,inImg,kernel);

    // Find contours
    std::vector<std::vector<cv::Point> > contours;
    std::vector<std::vector<cv::Point> > acceptedContours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours( inImg, contours, hierarchy, CV_RETR_LIST, cv::CHAIN_APPROX_NONE);

    // Draw the contours which have an area within certain limits
    for(unsigned int i = 0; i< contours.size(); i++)
    {
        // Calculate compactness
        float compactness = (4*M_PI * cv::contourArea(contours[i])) / (cv::arcLength(contours[i], true) * cv::arcLength(contours[i], true));

        // Check for how much circle it is and the area size
        if(compactness > compactThresh && cv::contourArea(contours[i]) > areaTresh)
        {
            acceptedContours.push_back(contours[i]);
        }
    }
    return acceptedContours;
}

/*
* Calculates and returns the center of gravity of a contour
*/
cv::Point2f getCOG(std::vector<cv::Point> contour)
{
    /// Get the moment of the contour
    cv::Moments mu = moments( contour, false );

    // Calculate and return the center of gravity
    return cv::Point2f( mu.m10/mu.m00 , mu.m01/mu.m00);
}

int main(int argc, char** argv)
{
    // Read the picture and show it
    cv::Mat img = cv::imread(argv[1]);
    displayImage("Original", img);

    applyHsvTrackbar(img);

    cv::Mat blueHsvThreshImg = applyHsvThreshold(img, cv::Scalar(110, 60, 35), cv::Scalar(130, 200, 155));
    cv::Mat redHsvThreshImg = applyHsvThreshold(img, cv::Scalar(0, 145, 110), cv::Scalar(50, 220, 215));
    //displayImage("HSV segmentation", redHsvThreshImg);
    //cv::waitKey(0);


    /*
    // Apply color threshold on the image
    cv::Mat threshImg = getThresholdImage2(img, cv::Scalar(32.2732, 47.724, 133.686), cv::Scalar(60.772, 32, 26.8812));
    imshow("Thresholded image", threshImg);
    //cv::waitKey(0);*/


    std::vector<std::vector<cv::Point>> redContours = getContours(redHsvThreshImg, 0.75, 2000);
    std::vector<std::vector<cv::Point>> blueContours = getContours(blueHsvThreshImg, 0.75, 2000);
    std::vector<cv::Point2f> contourCenters;

    // Calculate the center of gravity for the red contour and push it to contourCenters
    contourCenters.push_back(getCOG(redContours[0]));

    // Calculate the center of gravity for the blue contours and push them to contourCenters
    for(unsigned int i = 0; i< blueContours.size(); i++)
    {
        cv::Point2f point = getCOG(blueContours[i]);
        contourCenters.push_back(point);
    }


    cv::Point2f bluePoint = contourCenters[0];  // set initial to the red circles center

    // Find the blue circle diagonal to the red circle
    for(unsigned int i = 1; i< contourCenters.size(); i++)
    {
        // If the distance between the red and blue COG is bigger than the last distance then save the new COG
        if(cv::norm(contourCenters[0]-contourCenters[i]) > cv::norm(contourCenters[0]-bluePoint))
            bluePoint = contourCenters[i];
    }

    // Draw the red circles COG and the blue circle which is diagonal COG
    cv::Mat drawing = cv::Mat::zeros(redHsvThreshImg.size(), CV_8UC3);
    cv::circle(drawing, contourCenters[0], 5, cv::Scalar(255, 255, 255));
    cv::circle(drawing, bluePoint, 5, cv::Scalar(255, 255, 255));
    /*
    // Draw the center of gravities
    cv::Mat drawing = cv::Mat::zeros(redHsvThreshImg.size(), CV_8UC3);
    for(unsigned int i = 0; i< contourCenters.size(); i++)
    {
        cv::circle(drawing, contourCenters[i], 5, cv::Scalar(255, 255, 255));
    }*/

    /*
    // OBS stolen find contours which have a area larger than 2000
    cv::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours( segmentedImg, contours, hierarchy, CV_RETR_LIST, cv::CHAIN_APPROX_NONE);

    // Draw the contours which have an area within certain limits
    cv::Mat drawing = cv::Mat::zeros(segmentedImg.size(), CV_8UC3);
    for(unsigned int i = 0; i< contours.size(); i++)
    {
        float compactness = (4*M_PI * cv::contourArea(contours[i])) / (cv::arcLength(contours[i], true) * cv::arcLength(contours[i], true));

        // Check for how much circle it is and the area size
        if(compactness > 0.75 && cv::contourArea(contours[i]) > 2000)    // 1000 var godt til blå
        {

            cv::Scalar color = cv::Scalar(255, 255, 255);
            cv::drawContours( drawing, contours, i, color );
        }
    }*/

    //cv::namedWindow( "Contours");
    displayImage( "Contours", drawing );
    cv::waitKey(0);
}


/* Test on all images

for i in ../images/marker_color_hard/*.png; do
    ./main $i
done

*/

#include "Vision.hpp"
#define ROBWORK

Vision::Vision()
{
    /*
    hSliderMin = 0;
    hSliderMax = 180;

    sSliderMin = 0;
    sSliderMax = 255;

    vSliderMin = 0;
    vSliderMax = 255;*/

}

void Vision::displayImage(std::string name, cv::Mat &image)
{
  cv::namedWindow(name, cv::WINDOW_NORMAL);
  cv::imshow(name, image);
  cv::resizeWindow(name, 600,600);
}

/*
// Copy of the other function with hardcoded colorMeans
cv::Mat Vision::getThresholdImage2(const cv::Mat &inImg, cv::Scalar redThresh, cv::Scalar blueThresh)
{
    cv::Mat finalImage = cv::Mat::zeros(inImg.size(), CV_8UC1);    // Create a matrix with zeroes of same size as inImg but only 1 channel
    float meanThreshold = 0.1*255;
    cv::Scalar lowerThresh;
    cv::Scalar upperThresh;

    // Hardcoded means
    std::vector<cv::Scalar> colorMean = {redThresh, blueThresh};

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
}*/


cv::Mat Vision::applyHsvThreshold(const cv::Mat &inImg, const cv::Scalar minThresh, const cv::Scalar maxThresh)
{
    cv::Mat dstImg;
    cv::inRange(HSV_image, minThresh, maxThresh, dstImg);
    return dstImg;
}


/*
*   Dialate and erodes the image, where after it extracts the contours within a given compact and area threshold
*   The fitting contours are herafter returned as a vector
*/
std::vector<std::vector<cv::Point>> Vision::getContours(cv::Mat inImg, float compactThresh, int areaTresh)
{
    // Dilate and erode
    cv::Mat kernel = cv::Mat::ones(3,3,CV_8UC1);
    cv::dilate(inImg,inImg,kernel);
    cv::erode(inImg,inImg,kernel);
    kernel = cv::Mat::ones(13,13,CV_8UC1);
    cv::erode(inImg,inImg,kernel);
    cv::dilate(inImg,inImg,kernel);

    /*// Save the images
    if(first_run)
        imwrite("Mark1_Red_DialateErode.png", inImg);
    else
        imwrite("Mark1_Blue_DialateErode.png", inImg);
    */


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

    // Show a image of the accepted contours
    /// Draw contours
    /*cv::Mat drawing = cv::Mat::zeros(inImg.size(), CV_8UC3 );
    for(unsigned int i = 0; i<acceptedContours.size(); i++)
    {
         cv::Scalar color = cv::Scalar(255, 0, 0);
         cv::drawContours( drawing, acceptedContours, i, color, 2, 8, hierarchy, 0, cv::Point() );
    }*/

    // Save the images
    /*if(first_run)
    {
        first_run = false;
        imwrite("Mark1_Red_ContoursFound.png", drawing);
    }
    else
        imwrite("Mark1_Blue_ContoursFound.png", drawing);
    */

    return acceptedContours;
}

/*
* Calculates and returns the center of gravity of a contour
*/
cv::Point2f Vision::getCOG(std::vector<cv::Point> contour)
{
    /// Get the moment of the contour
    cv::Moments mu = moments( contour, false );

    // Calculate and return the center of gravity
    return cv::Point2f( mu.m10/mu.m00 , mu.m01/mu.m00);
}



std::vector<cv::Point2f> Vision::trackPicture(cv::Mat &inImg)
{
    #ifdef ROBWORK
    cvtColor(inImg, HSV_image, /*CV_BGR2HSV*/  CV_RGB2HSV);
    #else
    cvtColor(inImg, HSV_image, CV_BGR2HSV  /*CV_RGB2HSV*/);
    #endif

    std::vector<cv::Point2f> finalContoursCOG;
    std::vector<cv::Point2f> blueContourCOG;

    // Apply HSV thresholds
    #ifndef ROBWORK
    cv::Mat blueHsvThreshImg = applyHsvThreshold(inImg, cv::Scalar(110, 60, 35), cv::Scalar(130, 200, 155));
    cv::Mat redHsvThreshImg = applyHsvThreshold(inImg, cv::Scalar(0, 145, 110), cv::Scalar(50, 220, 215));
    #else
    cv::Mat blueHsvThreshImg = applyHsvThreshold(inImg, cv::Scalar(110, 60, 35), cv::Scalar(130, 255, 255));
    cv::Mat redHsvThreshImg = applyHsvThreshold(inImg, cv::Scalar(0, 145, 110), cv::Scalar(50, 255, 255));
    #endif
    //imwrite("Mark1_Blue_thresh_image.png", blueHsvThreshImg);
    //imwrite("Mark1_Red_thresh_image.png", redHsvThreshImg);

    // Get the red and blue circles as contours
    std::vector<std::vector<cv::Point>> redContours = getContours(redHsvThreshImg, 0.75, 2000);
    std::vector<std::vector<cv::Point>> blueContours = getContours(blueHsvThreshImg, 0.75, 2000);


    if(redContours.size() > 0)
    {
      // Calculate the center of gravity for the red contour and push it to finalContourCOG
      cv::Point2f redContourCOG = getCOG(redContours[0]);
      finalContoursCOG.push_back(redContourCOG);

      // Calculate the center of gravity for the blue contours and push them to blueContourCenters
      for(unsigned int i = 0; i< blueContours.size(); i++)
      {
          cv::Point2f point = getCOG(blueContours[i]);
          blueContourCOG.push_back(point);
      }


      // Find the blue circle diagonal to the red circle
      if(blueContourCOG.size() != 0)
      {
          cv::Point2f bluePoint = redContourCOG;  // set initial to the red circles center
          auto blue_pt_itt = blueContourCOG.begin();

          for(auto itt = blueContourCOG.begin(); itt != blueContourCOG.end(); ++itt)
          {
              // If the distance between the red and blue COG is bigger than the last distance then save the new COG
              if(cv::norm(redContourCOG-*itt) > cv::norm(redContourCOG-bluePoint))
              {
                  bluePoint = *itt;
                  blue_pt_itt = itt;
              }
          }
          blueContourCOG.erase(blue_pt_itt);

          finalContoursCOG.push_back(bluePoint);    // Push the blue diagonal circle to final contour cogs
      }


      // Find the two other blue circles in the same order every time and push to finalContourCOG
      if(blueContourCOG.size() == 2)
        sort_points(finalContoursCOG, blueContourCOG, finalContoursCOG);


      // Draw COG in the original image
      /*for(unsigned int i = 0; i<finalContoursCOG.size(); i++)
      {
          // Draw a circle with a random color
          cv::circle(inImg, finalContoursCOG[i], 10, cv::Scalar(50 + rand() % 150, 50 + rand() % 150, 50 + rand() % 150), -1);

          // Save images
          if(i == 0)
            imwrite("Marker1_Red_Cog.png", inImg);
      }

      imwrite("Marker1_Final_COGS.png", inImg);*/

      return finalContoursCOG;


    }

    return  std::vector<cv::Point2f>();
}


void Vision::sort_points(std::vector<cv::Point2f> knownCOGS, std::vector<cv::Point2f> unknownCOGS, std::vector<cv::Point2f> &finalCOGS)
{
    cv::Point2f direction = get_direction_vector(knownCOGS[0], knownCOGS[1], false);

    cv::Point2f perp_direction(-direction.y, direction.x);
    cv::Point2f perp_point = knownCOGS[0] + perp_direction * 50;
    //line(img, center1, perp_point, Scalar(255,0,255), 5);

    if(get_distance(unknownCOGS[0], perp_point) >
       get_distance(unknownCOGS[0], knownCOGS[0]))
    {
       finalCOGS.push_back(unknownCOGS[0]);
       finalCOGS.push_back(unknownCOGS[1]);
    }
    else
    {
        finalCOGS.push_back(unknownCOGS[1]);
        finalCOGS.push_back(unknownCOGS[0]);
    }
}

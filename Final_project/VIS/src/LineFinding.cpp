#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <vector>
#include <cmath>
#include <vector>
#include <algorithm>
#include <cstdlib>
#include "helper_funcs.hpp"

using namespace std;
using namespace cv;





cv::Mat performCanny(cv::Mat in_image, int threshold, float ratio = 3, bool convert = true, bool blur = true)
{
    if(blur)
        GaussianBlur( in_image, in_image, Size(7,7), 0, 0, BORDER_DEFAULT );

    //int edgeThresh = 1;
    int kernel_size = 3;
    cv::Mat detected_edges;
    if(convert)
    {
        cv::cvtColor(in_image, detected_edges, CV_BGR2GRAY);
        Canny( detected_edges, detected_edges, threshold, threshold*ratio, kernel_size );
    }
    else
    {
        Canny( in_image , detected_edges, threshold, threshold*ratio, kernel_size );
    }
    return detected_edges;
}

cv::Mat applyHsvThreshold(const cv::Mat &inImg, const cv::Scalar minThresh, const cv::Scalar maxThresh)
{
    cv::Mat dstImg;
    cv::Mat hsvImg;
    cv::cvtColor(inImg, hsvImg, CV_BGR2HSV);    // Convert image to HSV
    cv::inRange(hsvImg, minThresh, maxThresh, dstImg);
    return dstImg;
}

/** @function main */
int main( int argc, char** argv )
{
 //perform canny and sobol to find edges and their gradient.
 //Perform houghs line transform on canny edges to find lines
 //verify the lines by checking the colour on each side using the gradient.
 //They need to be black on one side and white on another (within some threshold.)
 cv::Mat img = cv::imread(argv[1]);
  cv::Mat detected_edges = performCanny(img, 50);
  //displayImage(detected_edges, "test");
  cv::Mat cdst;
  cv::Mat cdst2;
  cv::Mat cdst3;


 cv::Mat black_areas = applyHsvThreshold(img, cv::Scalar(0, 0, 0), cv::Scalar(255, 130, 100));

 cv::Mat kernel;
 kernel = cv::Mat::ones(3,3,CV_8UC1);
 displayImage(black_areas, "test_black2");

 cv::dilate(black_areas,black_areas,kernel);
 cv::erode(black_areas,black_areas,kernel);

 displayImage(black_areas, "test_black1");

 cv::Mat white_areas = applyHsvThreshold(img, cv::Scalar(0, 0, 100), cv::Scalar(255, 75, 255));
 cv::Mat big_white_areas = white_areas;

 cv::dilate(white_areas,white_areas,kernel);
 cv::erode(white_areas,white_areas,kernel);

 displayImage(white_areas, "test_white1");

 kernel = cv::Mat::ones(10,10   ,CV_8UC1);
 cv::dilate(big_white_areas,big_white_areas,kernel);
 cv::erode(big_white_areas,big_white_areas,kernel);
 cv::dilate(big_white_areas,big_white_areas,kernel);



std::vector<std::vector<cv::Point> > contours;
std::vector<cv::Vec4i> hierarchy;
cv::Mat canny_countour = performCanny(img, 15,3, true, false);

for(int32_t x = 0; x < big_white_areas.cols; x++)
    for(int32_t y = 0; y < big_white_areas.rows; y++)
    {
        //std::cout << (int)big_white_areas.at<uchar>(x,y) << std::endl;

        if(big_white_areas.at<uchar>(y,x) == 0)
            canny_countour.at<uchar>(y,x) = 0;

    }

displayImage(canny_countour, "canny");

kernel = cv::Mat::ones(5,5   ,CV_8UC1);
cv::dilate(canny_countour,canny_countour,kernel);

displayImage(canny_countour, "canny2");

cv::findContours( canny_countour, contours, hierarchy, CV_RETR_LIST, cv::CHAIN_APPROX_NONE);


// Draw the contours which have an area within certain limits
cv::Mat drawing = cv::Mat::zeros(big_white_areas.size(), CV_8UC3);
std::vector<std::vector<cv::Point> > approx_countour;
approx_countour.resize(contours.size());
float max_area_contour = 0;
int max_contour = 0;
for(unsigned int i = 0; i< contours.size(); i++)
{
    double epsilon = 0.06*cv::arcLength(contours[i],true);
    cv::approxPolyDP(contours[i],approx_countour[i],epsilon,false);
    cv::Scalar color = cv::Scalar(rand() %150 + 50 , rand() %150 + 50 ,rand() %150 + 50);
    float approx_area = cv::contourArea(approx_countour[i]);
    float perimeter = cv::arcLength(approx_countour[i], true);
    if(approx_area >= (perimeter/4. * perimeter/4.) * 0.8 and  approx_area <= (perimeter/4. * perimeter/4.) * 1.2)
    {
        if(approx_area > max_area_contour)
        {
            max_area_contour = approx_area;
            max_contour = i;
        }
    }
}
cv::drawContours( drawing, approx_countour, max_contour, cv::Scalar(255, 255,255),CV_FILLED );
kernel = cv::Mat::ones(70,70   ,CV_8UC1);
cv::dilate(drawing,drawing,kernel);


cv::namedWindow( "Contours");
imshow( "Contours", drawing );
cv::waitKey(0);




 cv::cvtColor(drawing, drawing, CV_BGR2GRAY);



 for(int32_t x = 0; x < big_white_areas.cols; x++)
     for(int32_t y = 0; y < big_white_areas.rows; y++)
     {
         //std::cout << (int)big_white_areas.at<uchar>(x,y) << std::endl;

         if(big_white_areas.at<uchar>(y,x) == 0 or drawing.at<uchar>(y,x) == 0)
             detected_edges.at<uchar>(y,x) = 0;

     }

     cvtColor(detected_edges, cdst, CV_GRAY2BGR);
     cvtColor(detected_edges, cdst2, CV_GRAY2BGR);
     cvtColor(detected_edges, cdst3, CV_GRAY2BGR);

 displayImage(detected_edges, "test_white3424");

 //Remove all edges not within white regions

 displayImage(big_white_areas, "test_white2");




 //displayImage(black_areas, "test_black");
 //displayImage(white_areas, "test_white");

 std::vector<Vec4i> lines;
 HoughLinesP(detected_edges, lines, 1, CV_PI/180., 75, 150, 100 );
 std::vector<Vec4i> sorted_lines;


bool done_removing_lines = false;
bool removed_line = false;
for( size_t i = 0; i < lines.size(); i++ )
{
  Vec4i l = lines[i];
  line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(rand() %150 + 50 , rand() %150 + 50 ,rand() %150 + 50), 3, CV_AA);
}


while(!done_removing_lines)
{
    for(auto it_outer = std::begin(lines); it_outer != std::end(lines); ++it_outer)
    {
        Vec4i &line_outer = *it_outer;
        Point2f outer_a = Point(line_outer[0], line_outer[1]);
        Point2f outer_b = Point(line_outer[2], line_outer[3]);

        removed_line = false;
        for(auto it_inner = it_outer + 1  ; it_inner != std::end(lines); ++it_inner)
        {
            //if(it_inner == it_outer) continue;
            if(removed_line) break;
            Vec4i &line_inner = *it_inner;
            Point2f inner_a =  Point(line_inner[0], line_inner[1]);
            Point2f inner_b =  Point(line_inner[2], line_inner[3]);

            float distance = get_distance(outer_a, outer_b, inner_a, inner_b);
            float angle_diff = abs(get_angle_diff(inner_a, inner_b, outer_a, outer_b));
            //std::cout << angle_diff << std::endl;
            if(distance < 5)
            {
                //std::cout << "A:" << angle_diff << std::endl;
                //std::cout << "D:" << distance << std::endl;
                //std::cout << outer_a << "\t" << outer_b  << "\t" << inner_a << "\t" << inner_b << "\t" << std::endl;
                if(angle_diff < CV_PI/180 * 5 )
                {
                    //Find the points in the two lines which is furthest from each other, and make a new line from them
                    Point2f new_line_a;
                    Point2f new_line_b;
                    concat_lines(inner_a, inner_b, outer_a, outer_b, &new_line_a, &new_line_b);
                    line_outer[0] = new_line_a.x; line_outer[1] = new_line_a.y;
                    line_outer[2] = new_line_b.x; line_outer[3] = new_line_b.y;
                    //std::cout << inner_a << "\t" << inner_b << "\t" << outer_a << "\t" << outer_b << "\t" << new_line_a << "\t" << new_line_b << std::endl;
                    lines.erase(it_inner);
                    removed_line = true;
                    break;
                }
            }
        }
        if(removed_line) break;
    }
    if(!removed_line) done_removing_lines= true;
}

for(auto &line : lines)
{
    Point2f midpoint = Point2f((line[0] + line[2])/2, (line[1] + line[3])/2);
    cv::circle(cdst, midpoint, 5, Scalar(255,0,255));
    //std::cout << (int)line[0] << "\t" << (int)line[1] << "\t" << (int)line[2] << "\t"  << (int)line[3] << "\t"  << std::endl;
    Point2f direction = get_direction_vector(Point2f(line[0],line[1]), Point2f(line[2], line[3]));
    Point2f perpendicular = Point2f(-direction.y, direction.x);
    bool white_left = false;
    bool black_left = false;
    bool white_right = false;
    bool black_right = false;

    for(int8_t i = 0; i < 10; i++)
    {
        Point2f point_left = midpoint + perpendicular * (int)i;
        Point2f point_right = midpoint + perpendicular * -(int)i;
        white_left |= white_areas.at<uchar>(point_left);
        black_left |= black_areas.at<uchar>(point_left);
        white_right |= white_areas.at<uchar>(point_right);
        black_right |= black_areas.at<uchar>(point_right);
    //std::cout << (int)black_areas.at<uchar>(point_right) << std::endl;
    cv::circle(cdst, point_left, 3, Scalar(127,0,255));
    cv::circle(cdst, point_right, 3, Scalar(127,0,255));
   }
    if((white_left && black_right) || (black_left && white_right) or true)
    {
        sorted_lines.push_back(line);
    }
}

//Only keep lines with a minimum of 2 perpendicular intersections and 2 parralel lines
std::vector<Vec4i> intersect_prunned;

std::cout << sorted_lines.size() << std::endl;
for(auto it_outer = std::begin(sorted_lines); it_outer != std::end(sorted_lines); ++it_outer)
{
    Vec4i line_outer = *it_outer;
    Point2f outer_a = Point(line_outer[0], line_outer[1]);
    Point2f outer_b = Point(line_outer[2], line_outer[3]);
    uint16_t par_lines_cnt = 0;
    uint16_t perp_intersect_cnt = 0;
    int i = 0;
    for(auto it_inner = std::begin(sorted_lines); it_inner != std::end(sorted_lines); ++it_inner)
    {
        if(it_inner == it_outer) continue;
        Vec4i line_inner = *it_inner;
        Point2f inner_a =  Point(line_inner[0], line_inner[1]);
        Point2f inner_b =  Point(line_inner[2], line_inner[3]);

        float angle_diff = abs(get_angle_diff(inner_a, inner_b, outer_a, outer_b));
        std::cout << angle_diff  << std::endl;
        if(angle_diff < CV_PI/180 * 15 )
        {
            par_lines_cnt += 1;
        }

        else if(angle_diff > CV_PI/2 - CV_PI/180 * 17 and angle_diff < CV_PI/2 + CV_PI/180 * 17 and
            does_intersect(outer_a, outer_b, inner_a, inner_b)) //Check if lines are perpendicular +-5 deg.
        {
            perp_intersect_cnt += 1;
        }
    }
    std::cout << line_outer << "\t" << par_lines_cnt << "\t" << perp_intersect_cnt << std::endl;
    if(par_lines_cnt >= 2 and perp_intersect_cnt >= 2)
    {
        intersect_prunned.push_back(*it_outer);
    }
}

for( size_t i = 0; i < sorted_lines.size(); i++ )
{
  Vec4i line = sorted_lines[i];
  Point2f midpoint = Point2f((line[0] + line[2])/2, (line[1] + line[3])/2);
  cv::circle(cdst3, midpoint, 5, Scalar(255,0,255));

  cv::line( cdst3, Point(line[0], line[1]), Point(line[2], line[3]), Scalar(rand() %150 + 50, rand() %150 + 50,rand() %150 + 50), 3, CV_AA);
}

sorted_lines = intersect_prunned;


for( size_t i = 0; i < sorted_lines.size(); i++ )
{
  Vec4i line = sorted_lines[i];
  Point2f midpoint = Point2f((line[0] + line[2])/2, (line[1] + line[3])/2);
  cv::circle(cdst2, midpoint, 5, Scalar(255,0,255));

  cv::line( cdst2, Point(line[0], line[1]), Point(line[2], line[3]), Scalar(rand() %150 + 50, rand() %150 + 50,rand() %150 + 50), 3, CV_AA);
}

  cv::Mat dst;
  dst = Scalar::all(0);
  img.copyTo( dst, detected_edges);
  //displayImage(dst, "test2");
 // cv::Mat gradient = performSobel(img);
  //displayImage(performSobel(img), "Test3");
  displayImage(cdst, "test4");
  displayImage(cdst3, "test5");

  displayImage(cdst2, "test6");

  waitKey(0);
  return 0;
  }

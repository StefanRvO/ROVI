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
#include "LineFinding.hpp"
#include <set>

using namespace std;
using namespace cv;


LineFinding::LineFinding(Mat in_image)
: org_image(in_image)
{
    return;
}

bool LineFinding::is_parallel(float angle, float max_diff)
{
    if(angle < max_diff)
        return true;
    return false;
}
bool LineFinding::is_perpendicular(float angle, float max_diff)
{
    if(angle > CV_PI/2 - max_diff and angle < CV_PI/2 + max_diff)
        return true;
    return false;
}


cv::Mat LineFinding::performCanny(cv::Mat in_image, int threshold, float ratio, bool convert, bool blur, int blur_image)
{
    if(blur)
        GaussianBlur( in_image, in_image, Size(blur_image,blur_image), 0, 0, BORDER_DEFAULT );

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

cv::Mat LineFinding::applyHsvThreshold(const cv::Mat &inImg, const cv::Scalar minThresh, const cv::Scalar maxThresh)
{
    cv::Mat dstImg;
    cv::Mat hsvImg;
    cv::cvtColor(inImg, hsvImg, CV_BGR2HSV);    // Convert image to HSV
    cv::inRange(hsvImg, minThresh, maxThresh, dstImg);
    return dstImg;
}

void LineFinding::detect_areas()
{
    //Marker series
    //black_areas = applyHsvThreshold(org_image, cv::Scalar(0, 0, 0), cv::Scalar(255, 130, 100));
    //white_areas = applyHsvThreshold(org_image, cv::Scalar(0, 0, 100), cv::Scalar(255, 75, 255));
    //Robwork
    black_areas = applyHsvThreshold(org_image, cv::Scalar(0, 0, 0), cv::Scalar(255, 130, 100));
    white_areas = applyHsvThreshold(org_image, cv::Scalar(0, 0, 210), cv::Scalar(255, 30, 255));

    big_white_areas = white_areas;
    cv::Mat kernel;
    kernel = cv::Mat::ones(3,3,CV_8UC1);
    cv::dilate(black_areas,black_areas,kernel);
    cv::erode(black_areas,black_areas,kernel);
    cv::dilate(white_areas,white_areas,kernel);
    cv::erode(white_areas,white_areas,kernel);
    kernel = cv::Mat::ones(10,10   ,CV_8UC1);
    cv::dilate(big_white_areas,big_white_areas,kernel);
    cv::erode(big_white_areas,big_white_areas,kernel);
    cv::dilate(big_white_areas,big_white_areas,kernel);

}

cv::Mat LineFinding::find_largest_square()
{
    //Find the largest square-like structure in the image, and return a binary image with that marked
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    //markers
    //cv::Mat canny_countour = performCanny(org_image, 15,3, true, true, 7);
    //Robwork
    cv::Mat canny_countour = performCanny(org_image, 80,2, true, true, 13);

    for(int32_t x = 0; x < big_white_areas.cols; x++)
        for(int32_t y = 0; y < big_white_areas.rows; y++)
        {
            //std::cout << (int)big_white_areas.at<uchar>(x,y) << std::endl;
            if(big_white_areas.at<uchar>(y,x) == 0)
                canny_countour.at<uchar>(y,x) = 0;
        }
    //displayImage(canny_countour, "test_canny");

    cv::Mat kernel = cv::Mat::ones(5,5   ,CV_8UC1);
    cv::dilate(canny_countour,canny_countour,kernel);
    cv::findContours( canny_countour, contours, hierarchy, CV_RETR_LIST, cv::CHAIN_APPROX_NONE);

    cv::Mat bigest_square = cv::Mat::zeros(big_white_areas.size(), CV_8UC3);
    std::vector<std::vector<cv::Point> > approx_countour;
    approx_countour.resize(contours.size());
    float max_area_contour = 0;
    int max_contour = 0;
    for(unsigned int i = 0; i< contours.size(); i++)
    {
        double epsilon = 0.06*cv::arcLength(contours[i],true);
        cv::approxPolyDP(contours[i],approx_countour[i],epsilon,false);
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

    cv::drawContours( bigest_square, approx_countour, max_contour, cv::Scalar(255, 255,255),CV_FILLED );
    kernel = cv::Mat::ones(70,70   ,CV_8UC1);
    cv::dilate(bigest_square,bigest_square,kernel);

    //cv::namedWindow( "Contours");
    cv::cvtColor(bigest_square, bigest_square, CV_BGR2GRAY);
    bigest_square_contour = approx_countour[max_contour];
    return bigest_square;
}


void LineFinding::create_edges()
{
    //Find edges for later line detection
    detected_edges = performCanny(org_image, 50);
    cv::Mat largest_square = find_largest_square();

    for(int32_t x = 0; x < largest_square.cols; x++)
        for(int32_t y = 0; y < largest_square.rows; y++)
        {
         if(big_white_areas.at<uchar>(y,x) == 0 or largest_square.at<uchar>(y,x) == 0)
             detected_edges.at<uchar>(y,x) = 0;
        }

}

void LineFinding::combine_double_lines(vector<Vec4i> &lines)
{
    bool done_removing_lines = false;
    bool removed_line = false;

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

}

std::vector<Marker_candidate> LineFinding::marker_candidates(std::vector<Point2f> points, float max_distance, float min_distance,
    float max_ratio, float min_ratio)
{
    std::vector<Marker_candidate> markers;
    flann::KDTreeIndexParams indexParams;

    flann::Index kdtree(Mat(points).reshape(1), indexParams);
    for(auto it_outer = std::begin(points); it_outer != std::end(points); ++it_outer)
    {
        //Find the four closest point to every point.
        vector<float> query;
        query.push_back((*it_outer).x);
        query.push_back((*it_outer).y);
        std::vector<int> indices(4);
        std::vector<float> dists(4);
        kdtree.knnSearch(query, indices, dists, 4);
        Marker_candidate mrk(points[indices[0]], points[indices[1]], points[indices[2]], points[indices[3]]);
        if(mrk.max_distance < max_distance and mrk.min_distance > min_distance and
            mrk.max_distance/mrk.min_distance < max_ratio and mrk.max_distance/mrk.min_distance > min_ratio )
        {
            markers.push_back(mrk);
        }

    }
    return markers;
}



std::vector<cv::Vec4i> LineFinding::remove_lines_outside_area(const std::vector<cv::Vec4i> &lines)
{
   std::vector<Vec4i> sorted_lines;
   for(auto &line : lines)
   {
       Point2f midpoint = Point2f((line[0] + line[2])/2, (line[1] + line[3])/2);
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
      }
       if((white_left && black_right) || (black_left && white_right) or true)
       {
           sorted_lines.push_back(line);
       }
   }
   return sorted_lines;
}

std::vector<cv::Vec4i> LineFinding::remove_lines_geometry(const std::vector<cv::Vec4i> &lines)
{
    //Only keep lines with a minimum of 2 perpendicular intersections and 2 parralel lines
    std::vector<Vec4i> intersect_prunned;

    for(auto it_outer = std::begin(lines); it_outer != std::end(lines); ++it_outer)
    {
        Vec4i line_outer = *it_outer;
        Point2f outer_a = Point(line_outer[0], line_outer[1]);
        Point2f outer_b = Point(line_outer[2], line_outer[3]);
        uint16_t par_lines_cnt = 0;
        uint16_t perp_intersect_cnt = 0;
        for(auto it_inner = std::begin(lines); it_inner != std::end(lines); ++it_inner)
        {
            if(it_inner == it_outer) continue;
            Vec4i line_inner = *it_inner;
            Point2f inner_a =  Point(line_inner[0], line_inner[1]);
            Point2f inner_b =  Point(line_inner[2], line_inner[3]);

            float angle_diff = abs(get_angle_diff(inner_a, inner_b, outer_a, outer_b));
            if(is_parallel(angle_diff))
            {
                par_lines_cnt += 1;
            }

            else if(is_perpendicular(angle_diff) and does_intersect(outer_a, outer_b, inner_a, inner_b)) //Check if lines are perpendicular +-5 deg.
            {
                perp_intersect_cnt += 1;
            }
        }
        //std::cout << line_outer << "\t" << par_lines_cnt << "\t" << perp_intersect_cnt << std::endl;
        if(par_lines_cnt >= 2 and perp_intersect_cnt >= 2)
        {
            intersect_prunned.push_back(*it_outer);
        }
    }
    return intersect_prunned;
}

std::vector<Point2f> LineFinding::get_intersection_points(const std::vector<Vec4i> &lines)
{
    std::vector<Point2f> points;
    for(auto it_outer = std::begin(lines); it_outer != std::end(lines); ++it_outer)
    {
        Vec4i line_outer = *it_outer;
        Point2f outer_a = Point(line_outer[0], line_outer[1]);
        Point2f outer_b = Point(line_outer[2], line_outer[3]);
        for(auto it_inner = std::begin(lines); it_inner != std::end(lines); ++it_inner)
        {
            if(it_inner == it_outer) continue;
            Vec4i line_inner = *it_inner;
            Point2f inner_a =  Point(line_inner[0], line_inner[1]);
            Point2f inner_b =  Point(line_inner[2], line_inner[3]);
            float angle_diff = abs(get_angle_diff(inner_a, inner_b, outer_a, outer_b));
            if(is_perpendicular(angle_diff, CV_PI/180 * 17))
            {
                points.push_back(find_intersection(inner_a, inner_b, outer_a, outer_b));
            }
        }
    }
    return points;

}

void LineFinding::sort_points(std::vector<Marker_candidate> &cand1, std::vector<Marker_candidate> &cand2,
    int index_offset)
{
    Point2f direction = get_direction_vector(cand1[0].center, cand1[1].center, false);
    Point2f center1 = (cand1[0].center + cand1[1].center) / 2.;
    Point2f center2 = (cand2[0].center + cand2[1].center) / 2.;
    //line(img, cand1[0].center, cand1[1].center,  Scalar(255,0,255), 5);
    //make a point perpendicular to the line connecting the small markers
    Point2f perp_direction(-direction.y, direction.x);
    Point2f perp_point = center1 + perp_direction * 50;
    //line(img, center1, perp_point, Scalar(255,0,255), 5);

    if(get_distance(center2, perp_point) >
       get_distance(center2, center1))
    {
       cand1[0].marker_id = 0 + index_offset;
       cand1[1].marker_id = 1 + index_offset;
    }
    else
    {
        cand1[0].marker_id = 1 + index_offset;
        cand1[1].marker_id = 0 + index_offset;
    }
}

void LineFinding::find_small_markers(std::vector<Marker_candidate> &small_marker_candidates, std::vector<Marker_candidate> &big_markers)
{
    std::vector<Marker_candidate> new_markers;
    Point2f square_center = getCOG(bigest_square_contour);
    if(small_marker_candidates.size() == 0) return;
    std::vector<Point2f> small_marker_points;
    std::vector<int> indices(1);
    std::vector<float> dists(1);

    for(auto &marker : small_marker_candidates) small_marker_points.push_back(marker.center);

    flann::KDTreeIndexParams indexParams;

    flann::Index kdtree(Mat(small_marker_points).reshape(1), indexParams);
    vector<float> query;
    query.push_back(square_center.x);
    query.push_back(square_center.y);
    kdtree.knnSearch(query, indices, dists, 1);
    Marker_candidate closest_candidate = small_marker_candidates[indices[0]];
    new_markers.push_back(closest_candidate);
    //float closest_candidate_dist = get_distance(closest_candidate.center, square_center);
    //Find the closest candidate_
    for(uint16_t i = 2; i <= small_marker_candidates.size(); i++)
    {
        indices.resize(i);
        dists.resize(i);
        kdtree.knnSearch(query, indices, dists, i);
        Marker_candidate test_mark = small_marker_candidates[indices[i - 1]];
        float marker_dist = get_distance(test_mark.center, closest_candidate.center);
        float center_dist = get_distance(test_mark.center, square_center);
        //std::cout << test_mark.center << std::endl;
        //std::cout << marker_dist << "\t" << closest_candidate_dist << "\t" << center_dist << "\t" << std::endl;
        if(marker_dist < center_dist)
            continue;
        new_markers.push_back(test_mark);
        small_marker_candidates = new_markers;
        return;
    }
    small_marker_candidates =  new_markers;
    return;
}

void LineFinding::find_big_markers(std::vector<Marker_candidate> &small_marker_candidates, std::vector<Marker_candidate> &big_markers)
{
    std::vector<Marker_candidate> new_markers;
    Point2f square_center = getCOG(bigest_square_contour);
    auto direction = get_direction_vector(small_marker_candidates[0].center, small_marker_candidates[1].center);
    if(big_markers.size() < 2) return;
    if(small_marker_candidates.size() != 2) return;


    //Find the markers closest to square center,
    //which fullfills the following: position is on the opsite of the center compared to the small markers
    //the second marker is closer to the center than to the other marker (like with the small markers).
    Point2f center_up = square_center + direction * 10;
    Point2f center_down = square_center + direction * -10;

    for(auto &point : big_markers)
    {
        if(is_left(center_up, center_down, point.center) ^ is_left(center_up, center_down, small_marker_candidates[0].center))
        new_markers.push_back(point);
    }
    big_markers = new_markers;
    new_markers.clear();
    //find small marker direction vector

    std::vector<Point2f> big_marker_points;
    std::vector<int> indices(1);
    std::vector<float> dists(1);

    for(auto &marker : big_markers) big_marker_points.push_back(marker.center);

    flann::KDTreeIndexParams indexParams;

    flann::Index kdtree(Mat(big_marker_points).reshape(1), indexParams);
    vector<float> query;
    query.push_back(square_center.x);
    query.push_back(square_center.y);
    kdtree.knnSearch(query, indices, dists, 1);

    Marker_candidate closest_candidate = big_markers[indices[0]];
    new_markers.push_back(closest_candidate);
    //float closest_candidate_dist = get_distance(closest_candidate.center, square_center);
    //Find the closest candidate_
    for(uint16_t i = 2; i <= big_markers.size(); i++)
    {
        indices.resize(i);
        dists.resize(i);
        kdtree.knnSearch(query, indices, dists, i);
        Marker_candidate test_mark = big_markers[indices[i - 1]];
        float marker_dist = get_distance(test_mark.center, closest_candidate.center);
        float center_dist = get_distance(test_mark.center, square_center);
        //std::cout << test_mark.center << std::endl;
        //std::cout << marker_dist << "\t" << closest_candidate_dist << "\t" << center_dist << "\t" << std::endl;
        if(marker_dist < center_dist)
            continue;
        new_markers.push_back(test_mark);
        big_markers = new_markers;
        return;
    }
    big_markers =  new_markers;
    return;

}

std::vector<cv::Point2f> LineFinding::get_marker_points()
{
    detect_areas();
    create_edges();
   //Detect lines in the image
   std::vector<Vec4i> lines;
   HoughLinesP(detected_edges, lines, 1, CV_PI/180., 75, 100, 100 );
   combine_double_lines(lines);
   lines = remove_lines_outside_area(lines);
   lines = remove_lines_geometry(lines);

    //Collect intersection points
    auto intersection_points = get_intersection_points(lines);
    //Make points unique
    remove_duplicates(intersection_points);

    //auto square_mid = getCOG(bigest_square_contour);

    auto small_marker_candidates = marker_candidates(intersection_points, 20, 5, 2, 1);
    auto big_marker_candidates = marker_candidates(intersection_points, 100, 5, 8, 2.5);
    remove_duplicates(small_marker_candidates);
    remove_duplicates(big_marker_candidates);
    find_small_markers(small_marker_candidates, big_marker_candidates);
    find_big_markers(small_marker_candidates, big_marker_candidates);
    if(small_marker_candidates.size() != 2) return std::vector<cv::Point2f>();
    if(big_marker_candidates.size() != 2) return std::vector<cv::Point2f>();

    sort_points(small_marker_candidates, big_marker_candidates, 0);
    sort_points(big_marker_candidates, small_marker_candidates, 2);

    if(small_marker_candidates[0].marker_id > small_marker_candidates[1].marker_id)
        std::swap(small_marker_candidates[0], small_marker_candidates[1]);
    if(big_marker_candidates[0].marker_id > big_marker_candidates[1].marker_id)
        std::swap(big_marker_candidates[0], big_marker_candidates[1]);
    std::vector<cv::Point2f> markers;
    markers.push_back(small_marker_candidates[0].center);
    markers.push_back(small_marker_candidates[1].center);
    markers.push_back(big_marker_candidates[0].center);
    markers.push_back(big_marker_candidates[1].center);

    return markers;
}

/** @function main */
int main( int argc, char** argv )
{
 //perform canny and sobol to find edges and their gradient.
 //Perform houghs line transform on canny edges to find lines
 //verify the lines by checking the colour on each side using the gradient.
 //They need to be black on one side and white on another (within some threshold.)
 cv::Mat img = cv::imread(argv[1]);
 LineFinding linefinder(img);
 auto markers = linefinder.get_marker_points();
 for(uint8_t i = 0; i < markers.size(); i++)
 {
     cv::circle(img, markers[i],  5, Scalar( (i * 1000) % 256,i * 60,255 - i * 60),  CV_FILLED);
 }
 displayImage(img, "test");
 waitKey(0);

  return 0;
}

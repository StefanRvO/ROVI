#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cmath>
#include <string>
struct Marker_candidate;



float get_distance(cv::Point2f line_a, cv::Point2f line_b, cv::Point2f p);
float get_distance(cv::Point2f p1, cv::Point2f p2);
float get_distance(cv::Point2f line1_a, cv::Point2f line1_b, cv::Point2f line2_a, cv::Point2f line2_b);
cv::Point2f get_direction_vector(cv::Point2f line_a, cv::Point2f line_b, bool fix_sign = true);
float get_angle(cv::Point2f line1_a, cv::Point2f line1_b);
float get_angle_diff(cv::Point2f line1_a, cv::Point2f line1_b, cv::Point2f line2_a, cv::Point2f line2_b);
cv::Point2f find_intersection(cv::Point2f line1_a, cv::Point2f line1_b, cv::Point2f line2_a, cv::Point2f line2_b);
bool does_intersect(cv::Point2f line1_a, cv::Point2f line1_b, cv::Point2f line2_a, cv::Point2f line2_b);
void concat_lines(cv::Point2f line1_a, cv::Point2f line1_b, cv::Point2f line2_a, cv::Point2f line2_b, cv::Point2f *new_line_a, cv::Point2f *new_line_b);
void displayImage(const cv::Mat &image, std::string name);
void displayImage(std::string name, const cv::Mat &image);

cv::Point2f getCOG(std::vector<cv::Point> contour);
void remove_duplicates(std::vector<cv::Point2f>& vec);
void remove_duplicates(std::vector<Marker_candidate>& vec);
bool is_left(cv::Point2f line_a, cv::Point2f line_b, cv::Point2f p);



struct Marker_candidate
{
    Marker_candidate(cv::Point2f _pt1, cv::Point2f _pt2, cv::Point2f _pt3, cv::Point2f _pt4)
    : pt1(_pt1), pt2(_pt2), pt3(_pt3), pt4(_pt4)
    {
        max_distance = 0;
        min_distance = 10000000000000;
        center.x = (pt1.x + pt2.x + pt3.x + pt4.x) / 4.;
        center.y = (pt1.y + pt2.y + pt3.y + pt4.y) / 4.;
        float distance = get_distance(pt1, pt2);
        if(distance > max_distance ) max_distance = distance;
        if(distance < min_distance) min_distance = distance;
        distance = get_distance(pt1, pt3);
        if(distance > max_distance ) max_distance = distance;
        if(distance < min_distance) min_distance = distance;
        distance = get_distance(pt1, pt4);
        if(distance > max_distance ) max_distance = distance;
        if(distance < min_distance) min_distance = distance;
        //std::cout << pt1 << "\t" << pt2 << "\t" << pt3 << "\t" << pt4 << std::endl;
        //std::cout << max_distance << "\t" << min_distance << std::endl;
    }
    cv::Point2f pt1;
    cv::Point2f pt2;
    cv::Point2f pt3;
    cv::Point2f pt4;
    cv::Point2f center;
    float max_distance;
    float min_distance;
    int marker_id;
};


bool operator==(const Marker_candidate &pt1, const Marker_candidate &pt2);

namespace std
{
    template<>
    struct hash<cv::Point2f>
    {
        size_t operator()(cv::Point2f const& pt) const
        {
            return (size_t)(pt.x*100 + pt.y);
        }
    };
}

namespace std
{
    template<>
    struct hash<Marker_candidate>
    {
        size_t operator()(Marker_candidate const& pt) const
        {
            return (size_t)(pt.center.x*100 + pt.center.y);
        }
    };
}

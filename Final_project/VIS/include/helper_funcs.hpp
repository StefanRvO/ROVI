#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cmath>
#include <string>

float get_distance(cv::Point2f line_a, cv::Point2f line_b, cv::Point2f p);
float get_distance(cv::Point2f p1, cv::Point2f p2);
float get_distance(cv::Point2f line1_a, cv::Point2f line1_b, cv::Point2f line2_a, cv::Point2f line2_b);
cv::Point2f get_direction_vector(cv::Point2f line_a, cv::Point2f line_b);
float get_angle(cv::Point2f line1_a, cv::Point2f line1_b);
float get_angle_diff(cv::Point2f line1_a, cv::Point2f line1_b, cv::Point2f line2_a, cv::Point2f line2_b);
cv::Point2f find_intersection(cv::Point2f line1_a, cv::Point2f line1_b, cv::Point2f line2_a, cv::Point2f line2_b);
bool does_intersect(cv::Point2f line1_a, cv::Point2f line1_b, cv::Point2f line2_a, cv::Point2f line2_b);
void concat_lines(cv::Point2f line1_a, cv::Point2f line1_b, cv::Point2f line2_a, cv::Point2f line2_b, cv::Point2f *new_line_a, cv::Point2f *new_line_b);
void displayImage(const cv::Mat &image, std::string name);

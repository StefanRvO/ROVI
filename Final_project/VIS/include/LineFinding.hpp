#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cmath>
#include <string>
#include "helper_funcs.hpp"



class LineFinding
{
    public:
        LineFinding(cv::Mat in_img);
        cv::Mat performCanny(cv::Mat in_image, int threshold, float ratio = 3, bool convert = true, bool blur = true, int blur_image = 7);
        cv::Mat applyHsvThreshold(const cv::Scalar minThresh, const cv::Scalar maxThresh);
        void detect_areas();
        std::vector<cv::Point2f> get_marker_points(cv::Mat *img = nullptr);
        cv::Mat find_largest_square(bool &success);
        void create_edges(bool &success);
        void combine_double_lines(std::vector<cv::Vec4i> &lines);
        std::vector<cv::Vec4i> remove_lines_outside_area(const std::vector<cv::Vec4i> &lines);
        std::vector<cv::Vec4i> remove_lines_geometry(const std::vector<cv::Vec4i> &lines);
        bool is_parallel(float angle, float max_diff = CV_PI/180 * 15);
        bool is_perpendicular(float angle, float max_diff = CV_PI/180 * 17);
        std::vector<cv::Point2f> get_intersection_points(const std::vector<cv::Vec4i> &lines);
        std::vector<Marker_candidate> marker_candidates(std::vector<cv::Point2f> points,
            float max_distance, float min_distance, float max_ratio, float min_ratio);
        void find_small_markers(std::vector<Marker_candidate> &small_marker_candidates, std::vector<Marker_candidate> &big_markers);
        void find_big_markers(std::vector<Marker_candidate> &small_marker_candidates, std::vector<Marker_candidate> &big_markers);
        void sort_points(std::vector<Marker_candidate> &cand1, std::vector<Marker_candidate> &cand2,
            int index_offset);
    private:
        cv::Mat HSV_image;
        cv::Mat org_image;
        cv::Mat black_areas;
        cv::Mat white_areas;
        cv::Mat big_white_areas;
        cv::Mat detected_edges;
        std::vector<cv::Point> bigest_square_contour;
        cv::Mat largest_square;
        cv::Mat canny_countour;
        cv::Mat *user_img;
};

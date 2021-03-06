#include "helper_funcs.hpp"
#include <unordered_set>
using namespace std;
using namespace cv;

float get_distance(Point2f line_a, Point2f line_b, Point2f p)
{
    double num = abs((line_b.x - line_a.x) * (line_a.y - p.y) - (line_a.x - p.x) * (line_b.y - line_a.y));
    double denum = sqrt(pow((line_b.y - line_a.y), 2) + pow((line_b.x - line_a.x), 2));
    return num/denum;
}
float get_distance(Point2f p1, Point2f p2)
{
    Point2f diff = p1 - p2;
    return sqrt(diff.x * diff.x + diff.y * diff.y);
}
float get_distance(Point2f line1_a, Point2f line1_b, Point2f line2_a, Point2f line2_b)
{
    //We find the minimum of the four possible distances
    float maximum = get_distance(line1_a, line1_b, line2_a);
    maximum = max(maximum, get_distance(line1_a, line1_b, line2_b));
    maximum = max(maximum, get_distance(line2_a, line2_b, line1_a));
    maximum = max(maximum, get_distance(line2_a, line2_b, line1_b));
    return maximum;
}

Point2f get_direction_vector(Point2f line_a, Point2f line_b, bool fix_sign)
{
    Point2f direction = Point2f(line_a.x - line_b.x, line_a.y - line_b.y);
    double mag = sqrt(direction.x * direction.x + direction.y * direction.y);
    direction.x /= mag;
    direction.y /= mag;
    //Make sure the direction have always the same sign
    if(fix_sign and direction.x < 0) return -direction;
    return direction;
}
float get_angle(Point2f line1_a, Point2f line1_b)
{
    Point2f direction = get_direction_vector(line1_a, line1_b);
    return fmod(std::atan(direction.y/direction.x) + CV_PI * 2,CV_PI);
}

float get_angle_diff(Point2f line1_a, Point2f line1_b, Point2f line2_a, Point2f line2_b)
{
    Point2f direction1 = get_direction_vector(line1_a, line1_b);
    Point2f direction2 = get_direction_vector(line2_a, line2_b);
    float angle1 = fmod(std::atan2(direction1.y,direction1.x) + CV_PI, CV_PI);
    float angle2 = fmod(std::atan2(direction2.y,direction2.x) + CV_PI, CV_PI);

    float tmp = min(abs(angle1 - angle2), abs(angle2 - angle1));
    if(tmp > CV_PI/2) return CV_PI - tmp;
    return tmp;
}

Point2f find_intersection(Point2f line1_a, Point2f line1_b, Point2f line2_a, Point2f line2_b)
{
    Point2f x = line2_a - line1_a;
    Point2f d1 = line1_b - line1_a;
    Point2f d2 = line2_b - line2_a;

    float cross = d1.x*d2.y - d1.y*d2.x;
    if (abs(cross) < /*EPS*/1e-8)
        return Point2f(-1,-1);

    double t1 = (x.x * d2.y - x.y * d2.x)/cross;
    Point2f r = line1_a + d1 * t1;
    return r;
}
bool does_intersect(Point2f line1_a, Point2f line1_b, Point2f line2_a, Point2f line2_b)
{
    //Checks if the lines intersect and the intersection point is between the start and end points
    Point2f intersection = find_intersection(line1_a, line1_b, line2_a, line2_b);
    //std::cout << intersection << "\t" << line1_a << "\t" << line1_b << "\t" << line2_a << "\t" << line2_b << std::endl;

    if(intersection == Point2f(-1,-1)) return false;
    if(intersection.y > max(line1_a.y, line1_b.y) or intersection.y > max(line2_a.y, line2_b.y))
        return false;
    if(intersection.y < min(line1_a.y, line1_b.y) or intersection.y < min(line2_a.y, line2_b.y))
        return false;
    if(intersection.x < min(line1_a.x, line1_b.x) or intersection.x < min(line2_a.x, line2_b.x))
        return false;
    if(intersection.x > max(line1_a.x, line1_b.x) or intersection.x > max(line2_a.x, line2_b.x))
        return false;

    return true;

}

void concat_lines(Point2f line1_a, Point2f line1_b, Point2f line2_a, Point2f line2_b, Point2f *new_line_a, Point2f *new_line_b)
{
    Point2f points[4] = {line1_a, line1_b, line2_a, line2_b};
    float max_dist = 0;
    for(uint8_t i = 0; i < 4; i++)
        for(uint8_t j = 0; j < 4; j++)
        {
            float dist = get_distance(points[i], points[j]);
            if(dist > max_dist )
            {
                max_dist = dist;
                *new_line_a = points[i];
                *new_line_b = points[j];
            }
        }
    return;
}

cv::Point2f getCOG(std::vector<cv::Point> contour)
{
    /// Get the moment of the contour
    cv::Moments mu = moments( contour, false );

    // Calculate and return the center of gravity
    return cv::Point2f( mu.m10/mu.m00 , mu.m01/mu.m00);
}



void displayImage(const Mat &image, string name)
{
  namedWindow(name, WINDOW_NORMAL);
  cv::imshow(name, image);
  cv::resizeWindow(name, 1000,1000);
}

void displayImage(string name, const Mat &image)
{
  namedWindow(name, WINDOW_NORMAL);
  cv::imshow(name, image);
  cv::resizeWindow(name, 1000,1000);
}



//Shamelessly stolen from stackoverflow
//http://stackoverflow.com/questions/25197805/how-to-delete-repeating-coordinates-of-vectorpoint2f
void remove_duplicates(std::vector<Point2f>& vec)
{
    for(auto &point : vec)
    {
        point.x = (int)point.x;
        point.y = (int)point.y;
    }
    std::unordered_set<Point2f> pointset;  // unordered_set is a hash table implementation

    auto itor = vec.begin();
    while (itor != vec.end())
    {
        if (pointset.find(*itor) != pointset.end())   // O(1) lookup time for unordered_set
        {
            itor = vec.erase(itor); // vec.erase returns the next valid iterator
        }
        else
        {
            pointset.insert(*itor);
            itor++;
        }
    }
}

void remove_duplicates(std::vector<Marker_candidate>& vec)
{
    for(auto &point : vec)
    {
        point.center.x = (int)point.center.x;
        point.center.y = (int)point.center.y;
    }
    std::unordered_set<Marker_candidate> pointset;  // unordered_set is a hash table implementation

    auto itor = vec.begin();
    while (itor != vec.end())
    {
        if (pointset.find(*itor) != pointset.end())   // O(1) lookup time for unordered_set
        {
            itor = vec.erase(itor); // vec.erase returns the next valid iterator
        }
        else
        {
            pointset.insert(*itor);
            itor++;
        }
    }
}

bool operator==(const Point2f& pt1, const Point2f& pt2)
{
    return ((pt1.x == pt2.x) && (pt1.y == pt2.y));
}

bool operator==(const Marker_candidate &pt1, const Marker_candidate &pt2)
{
    return ((pt1.center.x == pt2.center.x) && (pt1.center.y == pt2.center.y));
}
bool is_left(cv::Point2f line_a, cv::Point2f line_b, cv::Point2f p)
{
    return ((line_b.x - line_a.x) * (p.y - line_a.y) - (line_b.y - line_a.y) * (p.x - line_a.x) > 0);
}

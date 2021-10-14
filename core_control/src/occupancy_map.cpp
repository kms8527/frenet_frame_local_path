#include "occupancy_map.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

cv::Mat getPaddedROI(const cv::Mat &input, int top_left_x, int top_left_y, int width, int height)
{
    int bottom_right_x = top_left_x + width;
    int bottom_right_y = top_left_y + height;

    cv::Mat output;
    if (top_left_x < 0 || top_left_y < 0 || bottom_right_x > input.cols || bottom_right_y > input.rows)
    {
        // border padding will be required
        int border_left = 0, border_right = 0, border_top = 0, border_bottom = 0;

        if (top_left_x < 0)
        {
            width = width + top_left_x;
            border_left = -1 * top_left_x;
            top_left_x = 0;
        }
        if (top_left_y < 0)
        {
            height = height + top_left_y;
            border_top = -1 * top_left_y;
            top_left_y = 0;
        }
        if (bottom_right_x > input.cols)
        {
            width = width - (bottom_right_x - input.cols);
            border_right = bottom_right_x - input.cols;
        }
        if (bottom_right_y > input.rows)
        {
            height = height - (bottom_right_y - input.rows);
            border_bottom = bottom_right_y - input.rows;
        }

        cv::Rect R(top_left_x, top_left_y, width, height);
        copyMakeBorder(input(R), output, border_top, border_bottom, border_left, border_right, cv::BORDER_CONSTANT, cv::Scalar(100));
    }
    else
    {
        // no border padding required
        cv::Rect R(top_left_x, top_left_y, width, height);
        output = input(R);
    }
    return output;
}

cv::Mat detection_imgMap(double x, double y, double yaw)
{
    bool is_daegue = false;
    std::string name_map;
    if (is_daegue)
        name_map = "/home/a/k_city_ws/src/core_control/map/ImageMap(v3-expand).png";
    else
        name_map = "/home/a/k_city_ws/src/core_control/map/k_city_thres.png";
    // static cv::Mat map_origin = (cv::imread("/home/a/k_city_ws/src/core_control/map/ImageMap(final)_reverse.png", cv::IMREAD_GRAYSCALE));
    static cv::Mat map_origin = (cv::imread(name_map, cv::IMREAD_GRAYSCALE));
    //    for(int row = 0; row < map_origin.rows; row++)
    //    {
    //        for(int col = 0; col < map_origin.cols; col++)
    //        {
    //            if(map_origin.at<uchar>(row, col) == 0)
    //                map_origin.at<uchar>(row, col) = 0x64;
    //            else
    //                map_origin.at<uchar>(row, col) = 0x00;
    //        }
    //    }
    //    cv::imwrite("/home/a/k_city_ws/src/core_control/map/ImageMap(final)_reverse.png",map_origin);
    //  static bool a = false;
    //  if(!a)
    //  {
    //     cv::threshold(map_origin, map_origin, 10, 100, cv::THRESH_BINARY);
    //     // for (int i = 0; i < map_origin.rows * map_origin.cols; i++)
    //     // {
    //     //     if(map_origin.data[i] > 50 && map_origin.data[i] < 200 )
    //     //         map_origin.data[i] = 100;
    //     //     else
    //     //         map_origin.data[i] = 0;
    //     // }

    //      cv::imwrite("/home/a/k_city_ws/src/core_control/map/ImageMap(v3-expand)_thres.png",map_origin);
    //      a = true;
    //  }
    double max_x = 80;
    double max_y = 80;
    // dague
    double res = 0.0;
    int result_x = 0,
        result_y = 0;
    if (is_daegue)
    {
        res = 0.102616;
        result_x = (x / res) + 1910 - (max_x / res);
        result_y = -1.0 * (y / res) + 7109 - (max_y / res);
    }
    // kcity origin: [-182.4, -626.0, 0.0]
    else
    {
        res = 0.1;
        result_x = (x / res) + 1824 - (max_x / res);
        result_y = -1.0 * (y / res) + (10764 - 6260) - (max_y / res);
    }

    int result_w = (max_x / res) * 2;
    int result_h = (max_y / res) * 2;

    cv::Mat map_ROI = getPaddedROI(map_origin, result_x, result_y, result_w, result_h);
    //     cv::Point2f c_pt(result_w / 2, result_h / 2);
    //     double angle = yaw * 180 / 3.1415926535897;
    //     cv::Mat r = cv::getRotationMatrix2D(c_pt, (angle * -1), 1);
    //     cv::Mat map_rot_ROI;
    //     cv::warpAffine(map_ROI, map_rot_ROI, r, map_ROI.size());
    cv::Mat map_flip_ROI;
    cv::flip(map_ROI, map_flip_ROI, 0);
    // cv::Mat map_resize;
    // cv::resize(map_origin, map_resize, cv::Size(1000, 1000),cv::INTER_MAX);
    // cv::imshow("a", map_origin);
    // cv::waitKey();
    return map_flip_ROI;
}

// Include center point of your rectangle, size of your rectangle and the degrees of rotation
void DrawRotatedRectangle(cv::Mat &image, cv::Point centerPoint, cv::Size rectangleSize, double rotationDegrees)
{
    cv::Scalar color = cv::Scalar(100); // black

    // Create the rotated rectangle
    cv::RotatedRect rotatedRectangle(centerPoint, rectangleSize, rotationDegrees);

    // We take the edges that OpenCV calculated for us
    cv::Point2f vertices2f[4];
    rotatedRectangle.points(vertices2f);

    // Convert them so we can use them in a fillConvexPoly
    cv::Point vertices[4];
    for (int i = 0; i < 4; ++i)
    {
        vertices[i] = vertices2f[i];
    }

    // Now we can fill the rotated rectangle with our specified color
    cv::fillConvexPoly(image,
                       vertices,
                       4,
                       color);
}

OccupancyMap::OccupancyMap()
{
    res = 10;
    int height = 160 * res;
    int width = 160 * res;
    map.info.height = height;
    map.info.width = width;
    map.data.resize(map.info.height * map.info.width);
    map.header.frame_id = "world";
    map.info.resolution = 1;
}

OccupancyMap::~OccupancyMap() {}

void OccupancyMap::drawObstaclePolygons(geometry_msgs::Pose curr_pose)
{
    //     cv::Mat image = cv::Mat::zeros(map.info.height, map.info.width, CV_8UC1);
    cv::Mat image = detection_imgMap(curr_pose.position.x, curr_pose.position.y, 0);
    cv::resize(image, image, cv::Size(map.info.height, map.info.width));
    cv::threshold(image, image, 60, 100, cv::THRESH_BINARY);
    cv::Scalar color = cv::Scalar(100); // black
    for (int i = 0; i < polygons_obstacle.size(); i++)
    {
        std::vector<cv::Point> vertices;
        for (int j = 0; j < polygons_obstacle[i].size(); j++)
        {
            cv::Point p;
            p.x = (polygons_obstacle[i][j].x - curr_pose.position.x) * res + (double)map.info.width / 2.0;
            p.y = (polygons_obstacle[i][j].y - curr_pose.position.y) * res + (double)map.info.height / 2.0;
            // fill the polygon with our specified color
            vertices.push_back(p);
        }
        color = i > 90 ? cv::Scalar(10) : cv::Scalar(90 - i);
        cv::fillConvexPoly(image,
                           vertices,
                           color);
    }
    for (int i = 0; i < polygons_irregular.size(); i++)
    {
        std::vector<cv::Point> vertices;
        for (int j = 0; j < polygons_irregular[i].size(); j++)
        {
            cv::Point p;
            p.x = (polygons_irregular[i][j].x - curr_pose.position.x) * res + (double)map.info.width / 2.0;
            p.y = (polygons_irregular[i][j].y - curr_pose.position.y) * res + (double)map.info.height / 2.0;
            // fill the polygon with our specified color
            vertices.push_back(p);
        }
        color = cv::Scalar(91);
        cv::fillConvexPoly(image,
                           vertices,
                           color);
    }
    for (int i = 0; i < map.data.size(); i++)
        map.data[i] = static_cast<int8_t>(image.data[i]);
    // map.data[i] = (image.data[i] > 90) ? ((map.data[i] + 10 > 100) ? 100 : map.data[i] + 10) : ((map.data[i] - 10 < 0) ? 0 : map.data[i] - 10);
}

void OccupancyMap::addObstaclePolygon(const std::vector<geometry_msgs::Point> &polygon)
{
    polygons_obstacle.push_back(polygon);
}
double const &OccupancyMap::getResolution() { return res; }
void OccupancyMap::clearObstaclePolygons()
{
    polygons_obstacle.clear();
    polygons_obstacle.resize(0);
}
nav_msgs::OccupancyGrid const &OccupancyMap::getGridMap() { return map; }

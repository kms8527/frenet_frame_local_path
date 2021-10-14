#pragma once
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <vector>
#include <tf2/utils.h>

class OccupancyMap
{
private:
    // image size = 10083x8008
    nav_msgs::OccupancyGrid map;
    std::vector<std::vector<geometry_msgs::Point>> polygons_obstacle;
    std::vector<std::vector<geometry_msgs::Point>> polygons_irregular;
    //resolution
    double res;

public:
    OccupancyMap();
    ~OccupancyMap();
    void drawObstaclePolygons(geometry_msgs::Pose curr_pose);
    void addObstaclePolygon(const std::vector<geometry_msgs::Point> &polygon);
    void clearObstaclePolygons();

    nav_msgs::OccupancyGrid const &getGridMap();
    double const &getResolution();
};

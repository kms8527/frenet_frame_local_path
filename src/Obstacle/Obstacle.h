#ifndef FRENETOPTIMALTRAJECTORY_OBSTACLE_H
#define FRENETOPTIMALTRAJECTORY_OBSTACLE_H

#include <eigen3/Eigen/Dense>
#include <vector>
#include "geometry_msgs/Point.h"
using namespace Eigen;

class Obstacle {
public:
    std::pair<Vector2f, Vector2f> bbox;
    std::vector<geometry_msgs::Point> obstacle_points;
    Obstacle(Vector2f first_point, Vector2f second_point,
             double obstacle_clearance, std::vector<geometry_msgs::Point> obstacle_points);
    bool isSegmentInObstacle(Vector2f &p1, Vector2f &p2);
    bool isPointNearObstacle(Vector2f &p, double radius);
    double getArea();
};

#endif //FRENETOPTIMALTRAJECTORY_OBSTACLE_H

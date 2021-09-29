// Author: Edward Fang
// Email: edward.fang@berkeley.edu
//
// This code is adapted from
// https://github.com/AtsushiSakai/PythonRobotics/tree/
// master/PathPlanning/FrenetOptimalTrajectory.
// Its author is Atsushi Sakai.
//
// Reference Papers:
// - [Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet
// Frame]
// (https://www.researchgate.net/profile/Moritz_Werling/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame/links/54f749df0cf210398e9277af.pdf)
// - [Optimal trajectory generation for dynamic street scenarios in a Frenet
// Frame] (https://www.youtube.com/watch?v=Cj6tAQe7UCY)

#ifndef FRENET_OPTIMAL_TRAJECTORY_FRENET_OPTIMAL_TRAJECTORY_H
#define FRENET_OPTIMAL_TRAJECTORY_FRENET_OPTIMAL_TRAJECTORY_H

#include "src/CubicSpline/CubicSpline2D.h"
#include "src/FrenetOptimalTrajectory/FrenetPath.h"
#include "src/Obstacle/Obstacle.h"
#include "py_cpp_struct.h"

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <mutex>
#include <vector>
#include <ros/ros.h>
#include "frenet_local_path/frenet_input.h"
#include "frenet_local_path/waypoint.h"
#include "pcl/PCLPointCloud2.h"

#include "src/Polynomials/QuarticPolynomial.h"
#include "src/Polynomials/QuinticPolynomial.h"
#include "src/utils.h"

#include <chrono>
#include <iostream>
#include <thread>
#include <cmath>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include "tf2/utils.h"

using namespace std;
using namespace Eigen;

class FrenetOptimalTrajectory {


private:
    ros::Subscriber condition_sub;
    ros::Publisher frenet_path_pub;
    FrenetInitialConditions *fot_ic;
    FrenetHyperparameters *fot_hp;

    mutex *mu;
    FrenetPath *best_frenet_path;
    CubicSpline2D *csp;
    vector<Obstacle *> obstacles;


//    vector<double> x, y;
    vector<double> llx, lly, urx,ury; // obstacle lower left x , lower left y , upper right x, upper right y
    vector<frenet_local_path::waypoint> global_path;

    vector<FrenetPath *> frenet_paths;
    void calc_frenet_paths(int start_di_index, int end_di_index,
                           bool multithreaded);
    void threaded_calc_all_frenet_paths();

public:
    FrenetOptimalTrajectory(ros::NodeHandle &nh);
    ~FrenetOptimalTrajectory();
    FrenetPath *getBestPath();
    void setObstacles();
    void addObstacle(Vector2f first_point, Vector2f second_point); // first_point = obstacle's left_bottom_point[x,y] second_point = right_upper_point[x,y]
    void FrenetInitialConditionsCallback(const frenet_local_path::frenet_input &msg);
    void visualizePath();
    void visualizePath(const FrenetPath* frenet_path);
    void resetParam();
};

#endif // FRENET_OPTIMAL_TRAJECTORY_FRENET_OPTIMAL_TRAJECTORY_H

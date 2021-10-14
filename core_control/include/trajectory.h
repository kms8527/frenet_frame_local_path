#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <vector>
#include <cmath>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#define FILE_NAME "/home/a/trapezoidal.xls"

struct Motion
{
    double a; //acceleration
    double v; //velocity
    double p; //position - 여기서는 이동거리
};

class Trajectory
{
private:
    std::vector<Motion> motion;
    double ta = 0;          //현재링크 등가속시간
    double tc = 0;          //현재링크 등속시간
    double tb = 0;          //현재링크 감속시간
    double linktime;        //링크 하나를 지나는 예상시간
    double maxAcceleration; // (max Acceleration)

    void coeff(double vm, double v1, double v2, double dist);
    void trapezoidal(double localtime, double vm, double v1, double v2, double dist);

public:
    Trajectory(const double max_acc = 0.8) { setMaxAcc(max_acc); }
    void setMaxAcc(const double max_acc) { maxAcceleration = max_acc; }
    bool speedProfileDesign(double vm, double v1, double v2, double dist);
    void clearMotion();
    void getMotion(std::vector<Motion> &motion);
    void getTrajectory(pcl::PointCloud<pcl::PointXYZI> &path);
};

#endif // TRAJECTORY_H

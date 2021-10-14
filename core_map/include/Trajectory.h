#pragma once 

#include "hdmap_info.h"
#include <vector>
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
    std::vector<a3_link> path;
    std::vector<Motion> motion;
    std::vector<size_t> motionSize;
    double dt; //Sampling 시간간격
    double ta = 0; //현재링크 등가속시간
    double tc = 0; //현재링크 등속시간
    double tb = 0; //현재링크 감속시간
    double linktime; //링크 하나를 지나는 예상시간
//    double lastpos; //최종위치

    void coeff(double vm, double v1, double v2, double dist);
    void trapezoidal(double localtime, double vm, double v1, double v2, double prevpos, double lastpos);

public:
    Trajectory() {}
    void setPath(std::vector<a3_link> path);
    void remakePrevProfile(size_t i, double vm, double v1, double v2, double dist, double lastpos);
    void speedProfileDesign(double initalSpeed);
    void clearMotion();
    void getMotion(std::vector<Motion>& motion);
    void getTime(double& time);
};

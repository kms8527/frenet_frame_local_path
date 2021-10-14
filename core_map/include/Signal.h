#pragma once 
#include <ros/ros.h>
#include <string>
#include <vector>

struct IntersectionData
{
    int signalGroup;
    std::string linkID;
};

struct SignalData
{
    int signalGroup;
    double dist;
    int status;
    // 0,1   - Unavailable
    // 2,3   - RED
    // 4,5,6 - GREEN
    // 7,8,9 - YELLOW
    int minEndTime;
};

class Intersection
{
public:
    int intersectionID;
    std::vector<IntersectionData> data;
};

class Signal
{
public:
    ros::Time recvTime;
    int intersectionID;
    std::vector<SignalData> data;
};

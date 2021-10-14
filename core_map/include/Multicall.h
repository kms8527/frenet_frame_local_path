#pragma once 

#include <fstream>
#include <algorithm>
#include "Dijkstra.h"

#define MISSION_STATE_AVAILABLE     0
#define MISSION_STATE_UNAVAILABLE   1
#define MISSION_STATE_IMPOSSIBLE    2
#define MISSION_STATE_CLEAR         3

#define INCLUDE_NOTTING     0
#define INCLUDE_LEFT_TURN   1
#define INCLUDE_TUNNEL      2
#define INCLUDE_IDM         3
#define INCLUDE_IRREGULAR   4

struct Mission
{
    int missionID;
    int status;
    int point;
    unsigned short distance;
    int irrID;
    std::vector<int> includeMission;
    pcl::PointXY src;
    pcl::PointXY dst;
};

struct Irregular
{
    int irrID;
    pcl::PointXY p[4];
};

struct MissionPoint
{
    int id;
    bool src;
    bool dst;
    pcl::PointXY location;
};

class Multicall
{
private:
    unsigned int id;
    HDMap* map;
    std::vector<std::vector<double>>* table;
    Dijkstra dijkstra;
public:
    Multicall();
    void initMulticall(HDMap* map, std::vector<std::vector<double>>* table);
    double makeCombination(size_t currNode, std::vector<size_t> nodes, std::vector<int>& result);
    void sortCombination(std::vector<size_t> links, std::vector<size_t> linkPoint, std::vector<int>& combination);
    void optimizeRoute(std::vector<Mission> missionList, pcl::PointXY currentPos, int remainSeat, pcl::PointCloud<pcl::PointXY>& solution, std::vector<MissionPoint>& order, std::vector<int>& selectedMission);
    void laneChangeCheck(int& startLinkIndex, int& endLinkIndex, int startPointIndex, int endPointIndex, double& startLength, double& endLength);
    bool checkSameBundle(int startLinkIndex, int endLinkIndex);
};

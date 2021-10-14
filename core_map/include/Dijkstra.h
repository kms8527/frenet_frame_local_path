#pragma once 

#include "HDMap.h"
#include "Trajectory.h"
#include <queue>
#define TIME_MAX 999999
#define LANECHANGE_START_THRESH 25.0
#define LANECHANGE_END_THRESH 15.0

struct Node
{
    //Adjlist의 Head값에 쓰일 변수들
    c1_node thisNodeInfo;
    std::vector<Node> adjNode;
    Node* fromNode;
    double cost;

    //adjNode에 쓰일 변수들 - adj노드는 해당 adj노드로 가는 link정보까지 포함
    Node* thisNode; //원래 노드의 주소
    a3_link* thisLinkInfo;//해당 링크의 정보
};

class Adjlist
{
public:
    Adjlist() {}
    Adjlist(int size)
    {
        head.resize(size);
    }
    std::vector<Node> head;

    void makeGraph(std::vector<c1_node>& node, std::vector<a3_link>& link);
    void clearGraph();
};

class Dijkstra
{
private:
    HDMap* map;
    Adjlist graph;
    Trajectory trajectory;
    std::vector<std::vector<double>> *table;
    std::vector<a3_link> m_globalPath;
public:
    void initDijkstra(HDMap* map, std::vector<std::vector<double>> *table);
    void clearDijkstra();
    void findCloseLink(pcl::PointXY start, pcl::PointXY end, int& startLinkIndex, int& endLinkIndex, int& startPointIndex, int& endPointIndex);
    void findStartChangeLink(std::string linkid, pcl::PointXY p, std::vector<int>& linkIndex, std::vector<int>& linkPointIndex);
    void findEndChangeLink(std::string linkid, pcl::PointXY p, std::vector<int>& linkIndex, std::vector<int>& linkPointIndex);
    bool findPath(size_t startNode, size_t endNode, std::vector<a3_link>& globalPath, double& totalDist, double& totalTime); // 기본 Node to Node Dijkstra
    bool findPath(pcl::PointXY p1, pcl::PointXY p2, int changeLane, std::vector<a3_link>& globalPath); // 기본 Point to Point Dijkstra
    bool findPath(pcl::PointXY p1, pcl::PointXY p2, double initalSpeed, int changeLane, std::vector<a3_link>& globalPath, std::vector<Motion>& motion); // Emergency Path용 Dijkstra (Trajectory) 포함
    bool findPath(size_t startNode, size_t endNode, std::vector<a3_link>& globalPath, std::vector<Motion>& motion); // Multicall Table 작성을 위한 Node to Node Dijkstra (Trajectory 포함)
    bool findPath(size_t startLinkIndex, size_t startPointIndex, size_t endLinkIndex, size_t endPointIndex, std::vector<a3_link>& globalPath); // 차선 변경하는게 유리한지 확인하기 위한 차선변경용 Dijkstra
    void editStartLink(a3_link& origin, a3_link target);
    void editEndLink(a3_link& origin, a3_link target);
    bool rewiring(std::vector<a3_link>&globalPath, int currentIndex, int nextNode, int changeLane);
    bool reverseWiring(std::vector<a3_link>&globalPath, int currentIndex, int prevNode); // 전방의 패스에서 목표지점으로 차선변경이 가능한지 체크
    void reshapePath(std::vector<a3_link>& globalPath, int changeLane);
    bool avoidObstacle(std::vector<a3_link>& globalPath, int changeLane);
    void trajectoryPlanning(std::vector<a3_link> globalPath, double initalSpeed);
    void getTrajectory(std::vector<Motion>& motion);
    void getPathTime(double& time);
    void getIrrDist(a3_link irrLink, double& startDist, double& endDist);
};

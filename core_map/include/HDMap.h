#pragma once 

#include "hdmap_info.h"
#include <vector>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <cmath>

class Tilemap
{
public:
    std::vector<std::vector<std::vector<int>>> tileMapNode;
    std::vector<std::vector<std::vector<int>>> tileMapLink;
    std::vector<std::vector<std::vector<int>>> tileMapLane;

    int tileSize;
    int cell_x;
    int cell_y;
    pcl::PointXY offset;

public:
    Tilemap();
};

class HDMap
{
public:
    pcl::PointXY offset;
    std::vector<a1_lane> m_lane;
    std::vector<a3_link> m_link;
    std::vector<c1_node> m_node;
    std::vector<a2_stop> m_stop;
    std::vector<std::vector<a3_link*>> bundle;
    Tilemap tilemap;
public:
    HDMap();
    void readC1Node(std::string path);
    void readA3Link(std::string path);
    void readA1Lane(std::string path);
    void readA2Stop(std::string path);
    void wayPointCheck();
    void readHDMap(std::string path);

    void getHDMap(std::vector<c1_node>& node, std::vector<a3_link>& link, std::vector<a1_lane>& lane, std::vector<a2_stop>& stop);
    void getTilemapIndex(pcl::PointXY p, int& index_x, int& index_y);
    void getLinkInfo(pcl::PointXY p, int& linkIndex, int& linkPointIndex);
    void getChangeLinkInfo(int bundleIndex, int from_index, std::vector<a3_link>& result);
    void getStartChangeLinkInfo(std::string linkid, pcl::PointXY p, std::vector<int>& linkIndex, std::vector<int>& linkPointIndex);
    void getEndChangeLinkInfo(std::string linkid, pcl::PointXY p, std::vector<int>& linkIndex, std::vector<int>& linkPointIndex);
    void getLaneInfo(pcl::PointXY p, int linkIndex, int& leftLaneIndex, int& rightLaneIndex);
    void getRoadInfo(pcl::PointXY p, int& linkIndex, int& linkPointIndex, int& leftLaneIndex, int& rightLaneIndex);
    void getNodeInfo(pcl::PointXY p, int& nodeIndex);
    void getIntersection(pcl::PointXY p, std::vector<a3_link>& intersectionLink);
    void getConnectedLink(int fromNode, std::vector<a3_link>& links);
    bool isIrrBundle(int bundleIndex);
    double checkSignalIntersection(std::string linkid);

    double min_x, min_y;
    double max_x, max_y;
};

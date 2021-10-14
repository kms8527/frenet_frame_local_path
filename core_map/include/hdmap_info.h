#pragma once 
// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>

#include <ros/ros.h>

#include <iostream>
#include <string>

struct Waypoint
{
    double x;
    double y;
    std::string change_left;
    std::string change_right;
};

struct c1_node
{
    pcl::PointXY xy;
    std::string nodeid;
    short nodetype;
    short date;
    std::string remark;
    std::string its_nodeid;
    std::string hdufid;

    size_t thisIndex;
};

struct a3_link
{
    std::vector<Waypoint> waypoint;
    std::string linkid;
    std::string fromnode;
    std::string tonode;
    std::string length; //값이 이상해서 안썼는데 필요하면 stof써서 float으로 바꿔서 사용
    short roadtype;
    std::string roadno;
    short speed;
    short lane;
    short code;
    std::string gid; //이것도 필요하면 stoi써서 int으로 바꿔서 사용...
    std::string date;
    std::string remark;
    std::string its_linkid;
    std::string hdufid;
    double cost;
    double timecost;
    bool isStop;
    
    //빠른탐색을 위한 Node 인덱스 저장...
    size_t from_index;
    size_t to_index;
    //1,2,3,4 차선 나란한 차선 묶음
    size_t bundle_index, my_index;
    //차선변경 링크의 출발, 도착 인덱스 저장
    bool change_lane;
    size_t src_index;
    size_t dst_index;
    //비정형 구간을 포함
    bool isIrr;
    int startIrr;
    int endIrr;
};

struct a1_lane
{
    pcl::PointCloud<pcl::PointXY> xy;
    std::string r_link; //오른쪽 A3_LINK ID
    std::string l_link; //왼쪽 A3_LINK ID
    std::string lanetype; //색상 - 황색1, 백색2, 청색3
                  //겹수 - 단선1, 겹선2
                  //형태 - 실선1, 점선2, 좌점혼선3, 우점혼선4

    std::string lanecode; //중앙선01, 유턴구역선02, 차선03, 버스전용04 등...
    std::string barrier; //형태 - 녹지대01, 가드레일02, 콘크리트방호벽03, 콘크리트연석04
    std::string lno; //선번호
    std::string code; //종별코드 - 선1, 시설물(중앙분리대, 연석, 가드레일 등)2
    std::string date; //취득날짜
    std::string remark; //특이 case 설명
    std::string hdufid;

    //빠른 탐색을 위한 R_Link, L_Link 인덱스 저장...
    int r_index;
    int l_index;
};

struct a2_stop
{
    pcl::PointCloud<pcl::PointXY> xy;
    std::string linkid;
    short code;
    std::string date;
    std::string remark;
    std::string hdufid;
};

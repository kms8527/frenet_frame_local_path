#pragma once

// STL
#include <algorithm>
#include <fstream>
#include <thread>
#include <mutex>
#include <vector>
#include <string>
#include <string.h>
#include <stdio.h>

// ROS
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>

// Novatel
#include <novatel_gps_msgs/NovatelPosition.h>
#include <novatel_gps_msgs/Inspvax.h>

//PCL
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

// Custom Message
#include <core_map/road_info.h>
#include <core_map/global_path.h>
#include <core_map/traffic_light_info.h>
#include <core_map/RequestPath.h>
#include <core_map/Bsd.h>
#include <core_map/CheckProfit.h>
#include <core_map/Intersection.h>
#include <core_map/FrontIrr.h>
#include <core_control/AvanteData.h>
#include <core_map/SimCallReq.h>

// TCP
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/sockios.h>
#include <netinet/in.h>
#include <arpa/inet.h>

// OBU V2X
#include "dsrc_msg_id.h"
#include "obu_message.h"
#include "MessageFrame.h"
#include "Signal.h"

// My Header
#include "Dijkstra.h"
#include "Multicall.h"

// Bit Operation
const unsigned char nBit[4] = {0x01, 0x02, 0x04, 0x08};

enum state
{
    MISSION_INIT,
    WAIT_ORDER,
    MISSION_CHECK,
    WAIT_GETTING_ON,
    WAIT_GETTING_OFF,
};

class Coremap
{
private:
    typedef visualization_msgs::MarkerArray Markerarray;
    typedef core_map::RequestPath RequestPath;
    typedef core_map::CheckProfit CheckProfit;
    typedef core_map::Bsd Bsd;
    typedef core_map::FrontIrr FrontIrr;

    ros::NodeHandle nh;
    int runType;
    bool runOptimizer;
    int mouse_flag;
    pcl::PointXY mouseSrc;
    pcl::PointXY mouseDst;

    //Member Class
    HDMap hdmap;
    Dijkstra dijkstra;
    Multicall multiCall;
    // HDMap Offset
    double offset_x;
    double offset_y;

    //Subscriber
    ros::Subscriber clicked_point_sub;
    ros::Subscriber v2x_clear_sub;   //V2X Mission Clear
    ros::Subscriber bestpos_sub;
    ros::Subscriber inspvax_sub;
    ros::Subscriber avante_sub;

    //Simulation
    ros::Subscriber wave_sub;
    ros::Subscriber lte_sub;
    ros::Subscriber getting_on_sub;
    ros::Subscriber getting_off_sub;
    ros::ServiceClient sim_call_req_srv;

    //Publisher
    ros::Publisher rviz_v2x_call_pub; // v2x 미션 콜 visualize
    ros::Publisher rviz_v2x_map_pub;
    ros::Publisher irr_pub;     // 비정형 구간 pub
    ros::Publisher mission_pub; // V2X Mission 디버깅 bag 취득용
    ros::Publisher v2x_pub;     // V2X Wave 디버깅 bag 취득용
    ros::Publisher close_traffic_light_pub;

    ros::Publisher rviz_map_pub;  // HDMap Publish
    ros::Publisher path_pub;      // 보내는 전체 경로 core_map/global_path
    ros::Publisher real_path_pub; // 실제 이상적 경로 PCL
    ros::Publisher fsm_pub;       // FSM State publish
    ros::Publisher pcl_map;

    //Service
    ros::ServiceServer bsdSrv;
    ros::ServiceServer emergencyPathSrv;
    ros::ServiceServer checkProfitSrv;
    ros::ServiceServer intersectionSrv;
    ros::ServiceServer frontIrrSrv;

    //Thread
    std::mutex m_mutex;
    std::thread m_roadInfo_th;
    std::thread m_wave_th;
    std::thread m_lte_recv_th;
    std::thread m_lte_send_th;

    std::vector<Intersection> m_v2xMAP;
    std::vector<Signal> m_v2xSPAT;
    core_map::global_path m_finalPath;
    std::vector<a3_link> m_finalLink;

    //Thread Data
    int m_fsmState;      // FSM State
    bool m_penalty;
    ros::Time m_penaltyTime;
    ros::Time m_reqTime; // Request Time
    int m_currentMissionIndex;
    unsigned char m_sequence; // TCP Header Sequence

    // MultiCall Data
    std::vector<std::vector<double>> table;
    std::vector<MissionPoint> order;
    std::vector<core_map::global_path> multiCallPath;
    std::vector<Mission> missionList;
    std::vector<std_msgs::ColorRGBA> colors;
    int colorIndex;

    //Novatel Position, Heading
    double m_latitude;
    double m_longitude;
    float m_elevation;
    unsigned short m_heading;
    unsigned char m_speed;
    // TF Position Listener
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
    pcl::PointXY m_globalPos; //위치 정보 Listener

    // OBU TCP Socket
    sockaddr_in addrLTE;
    int sockLTE;
    sockaddr_in addrWAVE;
    int sockWAVE;

public:
    Coremap(ros::NodeHandle nh);

    //utill
    void initColor();
    void initWAVE();
    void initLTE();
    void resetMarker(Markerarray &marker, ros::Publisher pub);
    void cvtMsg(a3_link src, core_map::a3_link &msg);
    bool optimizeRoute(pcl::PointXY currPos, int remainSeat);
    void makeTable(std::string tablePath);
    void initTable(std::string tablePath);
    void transGps(double lat, double lon, double &east, double &north);
    void getTotalDist(pcl::PointCloud<pcl::PointXY> points, double &dist);
    void getBsdLink(pcl::PointXY p, core_map::links& tempBsd);
    //Callback Function
    void clickedPointCallback(const geometry_msgs::PoseStamped &msg);
    void missionStateCallback(const std_msgs::Int8 &msg);
    void bestposCallback(const novatel_gps_msgs::NovatelPosition::ConstPtr &msg);
    void inspvaxCallback(const novatel_gps_msgs::Inspvax::ConstPtr &msg);
    void avanteCallback(const core_control::AvanteData::ConstPtr &msg);
    void waveCallback(const std_msgs::String &msg);
    void lteCallback(const std_msgs::String &msg);
    void gettingOnCallback(const std_msgs::Int8 &msg);
    void gettingOffCallback(const std_msgs::Int8 &msg);
    //Publisher
    void visualizeMap(std::vector<c1_node> node, std::vector<a3_link> link, std::vector<a1_lane> lane, std::vector<a2_stop> stop);
    void visualizeMission(pcl::PointCloud<pcl::PointXY> srcPoints, pcl::PointCloud<pcl::PointXY> dstPoints);
    void visualizeSelectedMission(std::vector<MissionPoint> order);
    void visualizePath(core_map::global_path path);
    //Mouse Path Request
    bool mousePathReq(pcl::PointXY src, pcl::PointXY dst);
    //Service
    bool srvBsdLink(Bsd::Request &req, Bsd::Response &res);
    bool srvEmergencyPath(RequestPath::Request &req, RequestPath::Response &res);
    bool srvCheckProfit(CheckProfit::Request &req, CheckProfit::Response &res);
    bool srvIntersection(core_map::Intersection::Request &req, core_map::Intersection::Response &res);
    bool srvFrontIrr(FrontIrr::Request &req, FrontIrr::Response &res);
    void setGlobalPath(std::vector<Motion> motion, std::vector<a3_link> path, core_map::global_path &res, bool visualize);
    //Thread Function
    void roadInfo_thread();
    void wave_thread();
    void lte_recv_thread();
    void lte_send_thread();
    //OBU WAVE Function
    void recvSPAT(MessageFrame_t *msgFrame);
    void recvMAP(MessageFrame_t *msgFrame);
    //OBU LTE Function
    void access_mission_server();
    void callRequest(int missionID);
    void checkAccessRestriction(unsigned char *payload);
    void checkAccessPermission(unsigned char *payload);
    void checkLocationMessage(unsigned char *payload);
    void checkCallList(unsigned char *payload);
    void checkCallListForSim(unsigned char *payload);
    void checkCallResponse(unsigned char *payload);
    void checkGettingOn(unsigned char *payload);
    void checkGettingOff(unsigned char *payload);
    void checkMissionComplete(unsigned char *payload);
    void checkMissionGivingUp(unsigned char *payload);
    void startingPointArrive();
    void endPointArrive();
};

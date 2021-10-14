#pragma once
/// STD
#include <fstream>
#include <iostream>
#include <vector>
#include <functional>
#include <thread>
#include <mutex>

// user name get
#include <stdlib.h>

/// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int8.h>

//tf2
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <tf2/utils.h>

//geometry, navi msgs
#include <geometry_msgs/PoseArray.h>
// obstacles
#include <visualization_msgs/MarkerArray.h>
#include <polygon_msgs/polygonArray.h>

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

//Novatel
#include <novatel_gps_msgs/NovatelPosition.h>
#include <novatel_gps_msgs/Inspvax.h>

//avanate data msgs
#include <core_control/AvanteData.h>

// hd map
#include <core_map/RequestPath.h>
#include <core_map/traffic_light_info.h>
#include <core_map/CheckProfit.h>
#include <core_map/Bsd.h>
#include <core_map/Intersection.h>
#include <core_map/FrontIrr.h>

//morai new
#include <cbnu_msgs/AllObjectsData.h>
#include <cbnu_msgs/VehicleCmd.h>
#include <cbnu_msgs/VehicleTlm.h>
#include <morai_msgs/GPSMessage.h>

#include "av_control.h"
#include "local_path.h"
#include "occupancy_map.h"

/// Can_Parsing
#include "can_parsing.h"

/// PCAN
#include "PCANBasic.h"

const std::string kTopicObstacles = "/obj_data";
const std::string kTopicMissionState = "/core/control/mission_state";
const std::string kTopicAvanteData = "/core/control/avante_data";
const std::string kTopicLookAheadPoint = "/core/control/look_ahead_point";
const std::string kTopicPathGps = "/core/control/path_gps";
const std::string kTopicPathLocal = "/core/control/path_local";
const std::string kTopicMap = "/core/control/map";
const std::string kTopicMyMap = "/core/control/my_map";
const std::string kTopicGlobalPath = "/core/map/global_path";
const std::string kTopicFsmMission = "/core/map/mission_fsm";
const std::string kTopicTrafficInfo = "/core/v2x/traffic_signal";
const std::string kTopicIrregular = "/core/v2x/irregular_loc";

const std::string kSrvicePathEmergency = "/core/map/emergency_path_srv";
const std::string kSrviceCheckProfit = "/core/map/check_profit_srv";
const std::string kSrviceBsd = "/core/map/bsd_srv";
const std::string kServiceIntersection = "/core/map/intersection_srv";
const std::string kServiceFrontIrr = "/core/map/front_irr_srv";

const std::string kFrameWorld = "world";
const std::string kFrameGps = "gps";
const std::string kFramePos = "pos";
const std::string kFrameMap = "map";
const std::string kFrameVelodyne = "velodyne";

//morai
const std::string kTopicMoraiCmd = "/morai/cmd";
const std::string kTopicMoraiObjects = "/morai/objects";
const std::string kTopicMoraiTlm = "/morai/ego";
const std::string kTopicMoariGps = "/morai/gps";
using MoraiCmd = cbnu_msgs::VehicleCmd;
using MoraiTlm = cbnu_msgs::VehicleTlm;
using MoraiObjects = cbnu_msgs::AllObjectsData;
using MoraiGps = morai_msgs::GPSMessage;
enum class MoariGear : unsigned char
{
    M = 0,
    P = 1,
    R = 2,
    N = 3,
    D = 4
};

enum class FRAME_IDX : unsigned char
{
    GPS = 0,
    pos,
    map,
    velodyne
};
enum class TrafficSignState : char
{
    //0,1 unavailable
    //2,3 red
    //4,5,6 green
    //7,8,9 yellow
    unavailable = 1,
    red = 3,
    green = 6,
    yellow = 9
};
enum class MissionState : char
{
    error = -1,
    mission_wait = 0,
    mission_end = 1,
    mission_ing = 2
};
#define PCAN_NUM PCAN_USBBUS1
#define PCAN_BAUD PCAN_BAUD_500K

//ochang offset
// #define OFFSET_X -361050  //-361002.61+47+2.9-53
// #define OFFSET_Y -4065846 //-4065809.28-7+2.4-22.2//-361050, -4065846, x_offset: -361002.61, y_offset: -4065809.28
// //오창 형택이 맵 offset
// #define OFFSET_X -361047.4131372724
// #define OFFSET_Y -4065845.5535598844
//kipi offset
// #define OFFSET_X -445790.55599999998230487
// #define OFFSET_Y -3944958.8390000001527369
//kcity offset
#define OFFSET_X -302533.174487
#define OFFSET_Y -4124215.34631
//수성 알파시티 offset
// #define OFFSET_X -471194.505
// #define OFFSET_Y -3965610.5720000002
//image map offset
//#define OFFSET_X -471194.505 + 1881
//#define OFFSET_Y -3965610.5720000002 + 725

class CoreControl
{
public:
    CoreControl(ros::NodeHandle &node_handle);
    ~CoreControl();
    void mainLoop();

private:
    //ROS 노드 핸들
    ros::NodeHandle &nh;
    //컨트롤 클래스
    Control control;
    std::thread thread_control;
    std::mutex mutex_control;
    void threadControl();
    //LocalPath 클래스
    LocalPath local_path;
    StatePath state_path;
    StatePath state_path_next;
    //gps pose
    geometry_msgs::Pose gps_pose;
    std::mutex mutex_gps_pose;

    //전역경로
    core_map::global_path msg_path_global;

    //can data 받기
    std::thread thread_get_can;
    void threadCan();

    //(lat,lon) -> (x,y) 변환 함수
    void transGps(double lat, double lon, double &east, double &north);

    //이 밑은 sub, pub, morai관련 함수
    //형택이 전역경로 받기
    core_map::waypoint global_start_point;
    core_map::waypoint global_goal_point;
    MissionState mission_state;
    std_msgs::Int8 msg_mission_state;
    ros::Publisher pub_mission_state;
    ros::Subscriber sub_path_global_lic;
    ros::Subscriber sub_path_global_dague;
    ros::ServiceClient client_path_global;
    ros::ServiceClient client_intersection;
    void pubMissionState();
    void callbackPathGlobal(const core_map::global_path::ConstPtr &msg);
    bool callPathSrv(double src_x = 0, double src_y = 0, double dst_x = 0, double dst_y = 0);

    // 신호정보
    core_map::traffic_light_info traffic_info;
    ros::Subscriber sub_traffic_sign;
    void callbackTrafficInfo(const core_map::traffic_light_info::ConstPtr &msg);
    // 신호때문에 정지하면 true, 통과 가능하면 false;
    bool checkTrafficSignalStop(const core_map::traffic_light &traffic_light);

    //novatel 관련 sub
    ros::Subscriber sub_bestpos;
    ros::Subscriber sub_inspvax;
    void callbackBestpos(const novatel_gps_msgs::NovatelPosition::ConstPtr &msg);
    void callbackInspvax(const novatel_gps_msgs::Inspvax::ConstPtr &msg);

    //자동차 정보 pub, sub
    bool is_simulation;
    bool is_morai;
    ros::Publisher pub_avante_data;
    ros::Subscriber sub_avante_data;
    void pubAvanteData();
    void callbackAvanteData(const core_control::AvanteData::ConstPtr &msg);

    //위치 관련 pub
    PC_XYZI look_ahead_point;
    PC_XYZI radar_point;
    geometry_msgs::PoseArray path_gps;
    ros::Publisher pub_path_gps;
    ros::Publisher pub_look_ahead_point;
    void pubGpsPath();
    void pubLookAheadPoint();

    //경로 관련 pub
    PC_XYZI path_local;
    PC_XYZI path_lane_keeping;
    PC_XYZI path_lanechnage;
    ros::Publisher pub_path_local;
    void pubPathLocal();

    //Local Path 관련
    OccupancyMap occupancy_map;
    ros::Publisher pub_occupancy_map;
    ros::Publisher pub_occupancy_map_my;
    void pubOccupancyMap();

    //tf2
    tf2_ros::TransformBroadcaster br;
    tf2_ros::Buffer tf2_buffer;
    tf2_ros::TransformListener tf2_listener;
    std::vector<geometry_msgs::TransformStamped> transform_stampeds;
    void pubTf();
    void initTf();
    void tfListen();

    //object
    polygon_msgs::polygonArray objects;
    polygon_msgs::polygonArray msg_polygon_array_lidar;
    visualization_msgs::MarkerArray msg_polygon_irregular;
    ros::Subscriber sub_lidar_objects;
    ros::Subscriber sub_irregular;
    ros::Subscriber sub_state_mission_fsm;
    void callbackLidarObjects(const polygon_msgs::polygonArray &msg);
    void callbackIrregularLocation(const visualization_msgs::MarkerArray &msg);
    void callbackStateMissionFsm(const std_msgs::Int8 &msg);

    //morai 시뮬레이터 관련 sub, pub
    std::vector<std::vector<geometry_msgs::Point>> obstacle_morai;
    ros::Publisher pub_morai_cmd;
    ros::Subscriber sub_morai_tlm;
    ros::Subscriber sub_morai_gps;
    ros::Subscriber sub_morai_objects;
    void pubMoraiCmd(const bool &is_end = false);
    void callbackMoraiTlm(const MoraiTlm::ConstPtr &msg);
    void callbackMoraiGps(const MoraiGps::ConstPtr &msg);
    void callbackObjects(const MoraiObjects::ConstPtr &msg);

    double dis_traffic_stop;
    bool is_new_path_global;
    bool sign_stop;
    char state_mission_fsm;

    ros::ServiceClient client_check_profit;
    ros::ServiceClient client_bsd;
    ros::ServiceClient client_front_irr;
    void resetObject();

    std::mutex mutex_can;
    VehicleData vehicle_data;
    VehicleData vehicle_data_can;
    VehicleSetting vehicle_setting;
    VehicleControl vehicle_control;
    bool is_connect_can;
    bool is_pub_mission_state;
    double getDistanceStop(const double speed) { return (speed > 10) ? speed * 1.7805 - 6.0 : 10 * 1.7805 - 6.0; }
    double getDistanceAeb(const double speed) { return (speed > 10) ? speed * 0.3767 - 2.557 : 10 * 0.3767 - 2.557; }
    double getTimeStop(const double speed) { return (speed > 10) ? speed * 0.1 + 5.8 : 10 * 0.1 + 5.8; }
    double getTimeAeb(const double speed) { return (speed > 10) ? speed * 0.04 + 0.6 : 10 * 0.04 + 0.6; }
    /**
     * @brief 이 속도 이하 장애물 추월
     * 
     */
    double speed_slow_obejct;
    double gain_err_y;
    double gain_steer;
    double gain_look_ahead_distance;
    double offset_look_ahead_distance;
    double offset_yaw;
    double min_look_ahead_distance;
    double max_look_ahead_distance;

public:
    // Behavior Tree 대응 함수
    // Check 함수

    // 미션을 기다리고 있는지 확인
    bool btCheckMissionWait();
    // 전방장애물 확인
    bool btCheckFrontObject();
    // 코너(사거리) 확인
    bool btCheckNearCorner();
    // 정지선 근처 확인
    bool btCheckNearStopline();
    // 충돌 여부
    bool btCheckCollision();
    // 왼쪽차선 변경 가능 여부
    bool btCheckLaneChangeableLeft();
    // 오른쪽차선 변경 가능 여부
    bool btCheckLaneChangeableRight();
    // 신호등 통과 가능 여부
    // 미완성
    bool btCheckPassAbleTrafficLight();
    bool btCheckGlobalLaneChange();
    bool btCheckLeftChange();
    bool btCheckRightChange();
    bool btCheckExistPathGlobal();
    // 차선유지 state 체크
    bool btCheckStateLaneKeeping();
    // 왼쪽 차선변경 state 체크
    bool btCheckStateLaneChangeLeft();
    // 오른쪽 차선변경 state 체크
    bool btCheckStateLaneChangeRight();
    // 차선변경 끝났는지 체크!
    bool btCheckEndLaneChange();
    // 정지했는지 체크
    bool btCheckStop();
    bool btCheckArriveGoal();

    // 차선변경시 뒷 차량 확인
    bool btCheckCollisionBehindLeft();
    bool btCheckCollisionBehindRight();
    // 차선변경시 이득인지 확인
    bool btCheckProfitLaneChangeLeft();
    bool btCheckProfitLaneChangeRight();
    // 차선변경 실패확인
    bool btCheckFailLaneChange();
    // 비보호 좌,우회전 링크인지 확인
    bool btCheckUnprotectedTurn();
    bool btCheckImpassalbeUnprotectedTurn();
    // 차선변경을 무조건 해야하는곳 (다음링크가 없거나 마지막path일 때)
    bool btCheckMustLaneChange();

    bool btCheckObjectStaticLeftLane();
    bool btCheckObjectStaticRightLane();

    bool btCheckObjectNearGoal();
    /**
     * @brief 왼쪽차선에 비정형 장애물 확인
     * 
     * @return true 비정형 장애물 있음
     * @return false 
     */
    bool btCheckIrregularLeft();
    /**
     * @brief 오른쪽차선에 비정형 장애물 확인
     * 
     * @return true 비정형 장애물 있음
     * @return false 
     */
    bool btCheckIrregularRight();
    /**
     * @brief 내가 골지점과 얼마나 떨어져있는지 확인
     * 
     * @return true 내가 골지점과 가까움
     * @return false 
     */
    bool btCheckMeNearGoal();
    bool btCheckIrregularFront();
    // Action 함수

    // 초기화
    bool btActionInit();
    ;
    // 정지
    bool btActionStop();
    bool btActionObstacleStop();
    bool btActionStoplineStop();
    // AEB
    bool btActionEmergencyStop();
    // Global Path 따라가기
    bool btActionDrive();
    // Global Path 따라가는 중 ACCs
    bool btActionAcc();
    // 왼쪽으로 차선변경
    bool btActionLaneChangeLeft();
    // 오른쪽으로 차선변경
    bool btActionLaneChangeRight();
    bool btActionSetPath();
    // 좌회전 경로 생성
    bool btActionGenPathLaneChangeLeft();
    // 우회전 경로 생성
    bool btActionGenPathLaneChangeRight();
    // 차선유지 경로 생성
    bool btActionGenPathLaneKeeping();
    // state 차선유지로 변경
    bool btActionSetStateLaneKeeping();
    // state 왼쪽차선변경으로 변경
    bool btActionSetStateLaneChangeLeft();
    // state 오른쪽차선변경으로 변경
    bool btActionSetStateLaneChangeRight();
    // Data publish
    bool btActionPublishData();
    // 신호등 앞 정지 속도프로파일 생성
    bool btActionStopTrafficLight();
    //state mission
    bool btSetStateMissionGoing();
    bool btSetStateMissionArrive();
    bool btPublishStateMission();
    // Global Path 다시 요청
    bool btCallPathGlobal();
    bool btCallPathGlobalLeft();
    bool btCallPathGlobalRight();
    // 비보호 좌,우회전 속도프로파일 설계
    bool btGenSpeedUnprotectedTurn();
    // 차선변경 경로 속도프로파일 만들기
    bool btSpeedProfileLaneChange();
};

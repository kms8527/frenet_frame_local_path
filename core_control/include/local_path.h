#pragma once

#include <vector>

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <core_map/RequestPath.h>
#include <polygon_msgs/polygonArray.h>

#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2/utils.h>

#include "trajectory.h"
#include "av_control.h"

enum class LaneChangeState : char
{
    DEFAULT = 0,    //아무것도 안함.
    LEFT_SIGN,      //좌측 방향지시등 켜짐
    RIGHT_SIGN,     //우측 방향지시등 켜짐
    LEFT_CHANGING,  //좌측 차선 변경 중
    RIGHT_CHANGING, //우측 차선 변경 중
    END             //차선변경 종료
};

enum class ChangeableLanes : char
{
    NOT_EXIST = 0, //옆차선 없음
    LEFT,          //왼쪽차선 있음
    RIGHT,         //오른쪽 차선 있음
    BOTH           //양쪽차선 있음
};

enum class StatePath : char
{
    global = 0,
    lane_keeping,
    left_changing,
    right_changing,
    change_complte
};

enum class MsgType : char
{
    KCITY = 0,
    DAEGUE_AVANTE,
    DAEGUE_MAP
};

enum class Frenet : char
{
    frenet_initial =0,
    frenet_changing,
    frenet_complete
};

enum class StateCollision : char
{
    none = 0,
    static_object,
    dynamic_object,
    irregular
};

struct WayPointIndex
{
    int path,
        link,
        point;
    WayPointIndex(int _path = -1, int _link = -1, int _point = -1)
    {
        path = _path;
        link = _link;
        point = _point;
    }
    bool operator==(const WayPointIndex &w)
    {
        return path == w.path && link == w.link && point == w.point;
    }
};

struct UnprotectedLink
{
    std::string link_id;
    std::vector<core_map::point> bsd_points;
    double dis_check = -1.0;
};

class LocalPath
{
private:
    LaneChangeState lane_change_state;
    ChangeableLanes changeable_lanes;
    StatePath state_path;
    geometry_msgs::Pose curr_pose;
    geometry_msgs::Pose start_pose, goal_pose;
    // km/h
    double vehicle_speed;
    //minseong
    PC_XYZI frenet_path;


    nav_msgs::OccupancyGrid occ_map;

    PC_XYZI path_local;

    core_map::global_path msg_path_global;
    WayPointIndex idx_curr_path,
        idx_look_ahead,
        index_lane_change,
        idx_lane_change_start,
        idx_lane_change_mid,
        idx_lane_change_end;
    WayPointIndex idx_side_lane;
    StateCollision state_collision;

    int index_object;
    int index_object_front;
    std::vector<core_map::a3_link> path_behind;
    polygon_msgs::polygonArray msg_object;
    double time_lane_change;
    double max_speed_lane_change;
    core_map::point point_unprotected_left_turn[2];
    bool is_irr_same_link;
    double speed_min_lanechange;
    std::string linkid_intersection;
    UnprotectedLink unprotected_link;

    //베지어 커브 만들기
    std::vector<geometry_msgs::Pose> genBezierCurve(const std::vector<geometry_msgs::Pose> &poses, const int res);
    std::vector<geometry_msgs::Pose> genBezierCurve(const geometry_msgs::Pose &p0, const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2, const geometry_msgs::Pose &p3, const int res = 100);

    pcl::PointXYZI convertXYZI(const core_map::waypoint &wp)
    {
        pcl::PointXYZI p;
        p.x = wp.x;
        p.y = wp.y;
        p.intensity = wp.v * 3.6;
        return p;
    }

    // 글로벌 path 추종 경로
    // @param dis_path 경로길이
    PC_XYZI genGlobalFollowPath(const double dis_path);
    //global_path에서 현재위치 index 탐색
    WayPointIndex findCurrPathIndex(const geometry_msgs::Pose &pose);
    WayPointIndex findNextPointIndex(const WayPointIndex &idx);
    WayPointIndex findLookAheadPoint(const WayPointIndex &path_idx, const double look_ahead_distance = 0.0);
    int findNextLaneIndex(const WayPointIndex &idx);
    WayPointIndex findNextPathIndex(const WayPointIndex &idx);

    //해당 위치 검증
    bool isValidIndex(const WayPointIndex &idx);
    //옆차선 존재하는지 확인
    WayPointIndex findSideLanePoint(const WayPointIndex &idx, const bool &is_left);
    ChangeableLanes findLaneChangeable(const WayPointIndex &idx);

    // 차량과 충돌하는지 확인
    bool checkCollision(const geometry_msgs::Pose &pose);
    bool checkCollisionIdx(const WayPointIndex &idx);
    bool checkCollisionPath(const PC_XYZI &path);
    //nav_msgs::Path 로 변경
    nav_msgs::Path toNavPath(const PC_XYZI &path);
    nav_msgs::Path toNavPath(const core_map::a3_link &path);
    // 접선 계산
    double calcTangent(const WayPointIndex &idx);
    double calcTangent(const std::vector<geometry_msgs::PoseStamped> &vec_pose, const int index);
    double calcTangent(const PC_XYZI &pc, const int index);
    template <typename T, typename S>
    double calcTangent(const T &src, const S &dst);
    // double calcTangent(const pcl::PointXYZI &src, const pcl::PointXYZI &dst);
    // 두 점사이의 거리 계산
    template <typename T, typename S>
    double calcDistance(const T &src, const S &dst);
    template <typename T>
    double calcSumSqure(const T &src, const T &dst);
    double calcDistance(const WayPointIndex &src, const WayPointIndex &dst);
    double calcSumSqure(const WayPointIndex &src, const WayPointIndex &dst);
    WayPointIndex findPreviousPointIndex(const WayPointIndex &idx);
    WayPointIndex findLookBehindPoint(const WayPointIndex &path_idx, const double look_ahead_distance = 0.0);
    int findPreviousLaneIndex(const WayPointIndex &idx);

    std::vector<UnprotectedLink> unprotected_straight;
    std::vector<UnprotectedLink> unprotected_left;
    std::vector<UnprotectedLink> unprotected_right;

public:
    char frenet_state = 0;
    LocalPath();
    ~LocalPath();

    //글로벌패스에서 가져오는 함수들
    core_map::path const &getPath(const WayPointIndex &idx) { return msg_path_global.pathAry[idx.path]; }
    core_map::a3_link const &getLink(const WayPointIndex &idx) { return msg_path_global.pathAry[idx.path].links[idx.link]; }
    core_map::waypoint const &getWayPoint(const WayPointIndex &idx) { return msg_path_global.pathAry[idx.path].links[idx.link].waypointAry[idx.point]; }

    core_map::path const &getPath() { return msg_path_global.pathAry[idx_curr_path.path]; }
    core_map::a3_link const &getLink() { return msg_path_global.pathAry[idx_curr_path.path].links[idx_curr_path.link]; }
    core_map::waypoint const &getWayPoint() { return msg_path_global.pathAry[idx_curr_path.path].links[idx_curr_path.link].waypointAry[idx_curr_path.point]; }

    WayPointIndex const &getIndex() { return idx_curr_path; }

    // 위치 초기화 확인!
    bool initPose();
    bool requestNewPath;
    void genIntersectionStopPath(const double dis);
    bool is_exist_obstacle;
    bool frenet_flag;

    void resetIndex() { idx_curr_path = WayPointIndex(); }
    void setPose(const geometry_msgs::Pose &_pose) { curr_pose = _pose; }
    // km/h
    void setSpeed(const double _speed) { vehicle_speed = _speed; }
    void setOccMap(const nav_msgs::OccupancyGrid _map) { occ_map = _map; }
    void setPathGlobal(const core_map::global_path &_path) { msg_path_global = _path; }
    void setObject(polygon_msgs::polygonArray object) { msg_object = object; }
    void setPathBehind(std::vector<core_map::a3_link> &path) { path_behind = path; }
    void const setPathState(const StatePath state) { state_path = state; }
    void setFrenetPath(const PC_XYZI _frenet_path){frenet_path = _frenet_path;}
    void clearFrenetPath(){frenet_path.clear();}
    StatePath const &getPathState() { return state_path; }
    LaneChangeState const &getLaneChangeState() { return lane_change_state; }
    std::string const &getSideLaneTonode() { return getLink(idx_lane_change_mid).tonode; }
    std::string const &getTonodeCurrLane() { return getLink(idx_curr_path).tonode; }
    core_map::waypoint const &getLaneChangeEndPoint() { return getWayPoint(idx_lane_change_end); }
    int const &getIndexCollisionObstacle() { return index_object; }
    int const &getIndexObjectFront() { return index_object_front; }
    std::vector<UnprotectedLink> const &getUnprotectedStraight() { return unprotected_straight; }
    std::vector<UnprotectedLink> const &getUnprotectedLeft() { return unprotected_left; }
    std::vector<UnprotectedLink> const &getUnprotectedRight() { return unprotected_right; }
    UnprotectedLink const getUnprotectedLink();
    bool isUnprotectedIntersection(std::string linkid);
    void avoid_obstacle_path(PC_XYZI &path);
    std::string const getTonodeLeftLane()
    {
        WayPointIndex wp = findSideLanePoint(idx_curr_path, true);
        if (isValidIndex(wp))
            return getLink(wp).tonode;
        else
        {
            std::string not_found_tonode = "";
            return not_found_tonode;
        }
    }
    std::string const getTonodeRightLane()
    {
        WayPointIndex wp = findSideLanePoint(idx_curr_path, false);
        if (isValidIndex(wp))
            return getLink(wp).tonode;
        else
        {
            std::string not_found_tonode = "";
            return not_found_tonode;
        }
    }
    core_map::a3_link const getLinkLeft()
    {
        WayPointIndex wp = findSideLanePoint(idx_curr_path, true);
        if (isValidIndex(wp))
            return getLink(wp);
        else
        {
            core_map::a3_link not_found_link;
            return not_found_link;
        }
    }
    core_map::a3_link const getLinkRight()
    {
        WayPointIndex wp = findSideLanePoint(idx_curr_path, false);
        if (isValidIndex(wp))
            return getLink(wp);
        else
        {
            core_map::a3_link not_found_link;
            return not_found_link;
        }
    }
    /**
     * @brief Bsd포인트 설정
     * 
     */
    void genBsdPoints();

    // Behavior Tree
private:
    PC_XYZI path_global_follow;
    PC_XYZI path_lanechange_base;
    PC_XYZI path_lanechange;
    PC_XYZI path_lane_keeping;

public:
    PC_XYZI const &getPathGlobalFollow() { return path_global_follow; }
    PC_XYZI const &getPathLanechage() { return path_lanechange; }
    PC_XYZI const &getPathLaneKeeping() { return path_lane_keeping; }
    core_map::waypoint const getPointLeftLane();
    core_map::waypoint const getPointRightLane();
    PC_XYZI path_debug;

public:
    // 전방장애물 확인
    bool tickCheckFrontObject();
    // 정지선 근처 확인
    bool tickCheckNearStopline();
    // 충돌 여부
    bool tickCheckCollision();
    // 차선변경 가능 여부
    bool tickCheckLaneChangeable();
    // 왼쪽차선 변경 가능 여부
    bool tickCheckLaneChangeableLeft();
    // 오른쪽 차선 변경가능 여부
    bool tickCheckLaneChangeableRight();
    // gobal path가 차선변경 해야하는가?
    bool tickCheckGlobalLaneChange();
    // 왼쪽으로 차선변경 해야하는가?
    bool tickCheckLeftChange();
    // 차선변경 완료됐나?
    bool tickCheckEndLaneChange();
    // local path와 현재위치 사이가 먼가?
    bool checkFarPath();
    // 좌,우 차선 후방차량 확인
    bool checkCollisionBehindLeft();
    bool checkCollisionBehindRight();
    // irregular 확인
    bool checkIrregular();
    // 경로의 마지막인지 확인
    bool checkEndPath();
    // 차선변경 실패핬는지 확인
    bool tickCheckFailLaneChange();
    // 비보호 좌,우회전 확인
    bool checkUnprotected();
    bool checkUnprotectedNext();
    bool checkUnprotectedStraight();
    bool checkUnprotectedTurnLeft();
    bool checkUnprotectedTurnRight();
    // 비보호 좌회전 충돌체크
    bool checkCollisionUnprotectedTurn();

    bool checkMustLaneChange();

    // Behavior Tree action 함수
    // 왼쪽 차선변경 경로 생성
    bool genPathLaneChangeLeft();
    // 오른쪽 차선변경 경로 생성
    bool genPathLaneChangeRight();
    // 왼쪽 차선변경 경로 선택
    bool setLaneChangeLeft();
    // 오른쪽 차선변경 경로 선택
    bool setLaneChangeRight();
    // 차선유지 경로 생성
    bool genPathLaneKeeping();
    // dis 만큼 떨어진 곳에서 정지하게 속도프로파일 생성
    bool genSpeedProfileStop(const double dis_stop = 0);

    PC_XYZI genPathLaneChange(const bool &is_left_lane);
    PC_XYZI genSpeedProfileLaneChange();
    /**
     * @brief 왼쪽차선 정지장애물 확인
     * 
     * @return true 정지장애물 있음
     * @return false 정지장애물 없음
     */
    bool checkObjectStaticLeftLane();
    /**
     * @brief 오른쪽차선 정지장애물 확인
     * 
     * @return true 정지장애물 있음
     * @return false 정지장애물 없음
     */
    bool checkObjectStaticRightLane();
    /**
     * @brief 목표지점 주위에 장애물 확인
     * 
     * @return true 목표지점 주위에 장애물 있음
     * @return false 목표지점 주위에 장애물 없음
     */
    bool checkObjectnearGoal();
    /**
     * @brief 차선변경해야할때 속도프로파일 생성 링크끝 15m 앞에서 정지경로 생성
     * 
     * @param scale 차선변경 횟수
     */
    void genSpeedProfileMustLaneChange(const int scale = 1);
    /**
     * @brief 같은링크 전방에 비정형 장애물 확인.
     * 
     * @return true 
     * @return false 
     */
    bool checkIrregularFront();
    /**
     * @brief 내가 골 주변인지 확인
     * 
     * @return true 내가 골 주변임
     * @return false 
     */
    bool checkMeNearGoal();
};

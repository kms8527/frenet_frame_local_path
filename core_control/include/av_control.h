#pragma once

#include <iostream>
#include <geometry_msgs/Pose.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2/transform_datatypes.h>
#include <tf2/convert.h>
#include <tf2/utils.h>

#include "can_parsing.h"
#include "pid.h"
#include "PCANBasic.h"

const double kControlRate = 20.0;
const double kDegToRad = M_PI / 180.0;
const double kRadToDeg = 180.0 / M_PI;

using PC_XYZI = pcl::PointCloud<pcl::PointXYZI>;

#define PCAN_NUM PCAN_USBBUS1
#define PCAN_BAUD PCAN_BAUD_500K

enum class AEB_STATE : unsigned char
{
    DEFAULT = 0,  //아무것도 안함.
    FCW = 1,      //forward colision warning
    STAGE1 = 2,   //브레이크1단계
    STAGE2 = 3,   //브레이크2단계
    MAX_BRAKE = 4 //AEB
};

enum class GEAR_STATE : unsigned char
{
    D = 0x1, //Drive
    N = 0x2, //Nutral
    R = 0x4, //Reverse
    P = 0x8  //parking
};

enum class TURN_SIGNAL : unsigned char
{
    NONE = 0x0,
    HAZARD = 0x1, //비상등
    LEFT = 0x2,
    RIGHT = 0x4
};

enum class StateDrive : char
{
    DRIVE = 0,
    ACC,
    AEB
};

class Control
{
private:
    /// Vehicle info
    VehicleData vehicle_data;
    VehicleSetting vehicle_setting;
    VehicleControl vehicle_control;

    geometry_msgs::Pose curr_pose;

    /// Target
    //km/h
    double target_speed;
    //deg
    double target_angle;
    double del_angle;
    /// 차간거리
    double L;
    bool is_morai;

    /// PID objects
    PID pid_vehicle_acc;
    PID pid_vehicle_brake;
    PID pid_purepersuit;

    unsigned char can_alive_count;

    PC_XYZI local_path;
    int path_index;
    int look_ahead_index;

    bool is_pose_init;

    AEB_STATE aeb_state;

    double gain_steer;
    double gain_err_y;
    double gain_look_ahead_distance;
    double offset_look_ahead_distance;
    double min_look_ahead_distance;
    double max_look_ahead_distance;

    // 정지판단. AEB와 출발지점, 도착지점에 사용
    bool stop_sign;
    // 정지할때 sin 파형으로 acc값 넣어야함
    double genBrakeAcc();
    // 정지시간
    int stop_count;
    double front_object_distance;
    double front_object_speed;
    StateDrive state_drive;

    void setPcanMsg(TPCANMsg &msg, int id, int length, const unsigned char *data = NULL);
    double purePersuit(double curr_x, double curr_y, double curr_yaw, double curr_speed, pcl::PointCloud<pcl::PointXYZI> &local_path);
    //현재위치 탐색.
    int findCurrPathIndex(double curr_x, double curr_y, const pcl::PointCloud<pcl::PointXYZI> &local_path);
    //LookAheadPoint의 인덱스 탐색.
    int findLookAheadPointIndex(double curr_x, double curr_y, double look_ahead_distance, int path_index, const pcl::PointCloud<pcl::PointXYZI> &local_path);
    //Param
    //v_f : 내 차 속도
    //v_relative : 앞차와의 상대속도
    //s : 앞차와의 거리
    double acc(double v_f, double v_relative, double s);
    //Param
    //v_ego : 내 차 속도
    //v_relative : 앞차와의 상대속도
    //dis : 앞차와의 거리
    double aeb(double v_ego, double v_relative, double dis);

    double calSteer(const double curr_speed);
    bool checkInitialized();
    void writeCanMsg();

public:
    Control();
    ~Control();
    void RunOnce();
    bool test_lane_change;

    void setIsMorai(const bool morai) { is_morai = morai; }
    void setCanData(const VehicleData _vehicle_data) { vehicle_data = _vehicle_data; }
    void setEps(const unsigned char *data) { vehicle_data.eps.setValue(data); }
    void setEps(const EpsStat &eps_state) { vehicle_data.eps = eps_state; }
    void setAccData(const unsigned char *data) { vehicle_data.acc.setValue(data); }
    void setAccData(const AccStat &acc_state) { vehicle_data.acc = acc_state; }
    void setWheel(const unsigned char *data) { vehicle_data.wheel.setValue(data); }
    void setWheel(const WheelSpeed &wheel_speed) { vehicle_data.wheel = wheel_speed; }
    void setYawAndBrake(const unsigned char *data) { vehicle_data.yaw_and_brake.setValue(data); }
    void setRadar(const unsigned char *data) { vehicle_data.radar.setValue(data); }
    void setPose(const geometry_msgs::Pose &pose)
    {
        curr_pose = pose;
        is_pose_init = true;
    }
    void setPath(const PC_XYZI &path) { local_path = path; }
    void setL(const int _L) { L = _L; }
    void setStopSign(const bool stop) { stop_sign = stop; }
    void activateAeb()
    {
        aeb_state = AEB_STATE::MAX_BRAKE;
        stop_sign = true;
    }
    void setFrontObject(const double obj_distance, const double obj_speed)
    {
        front_object_distance = obj_distance;
        front_object_speed = obj_speed;
    }
    void setAcc() { state_drive = StateDrive::ACC; }
    void setGainErrY(const double gain) { gain_err_y = gain; }
    void setGainSteer(const double gain) { gain_steer = gain; }
    void setGainLookAhaedDistance(const double gain) { gain_look_ahead_distance = gain; }
    void setOffsetLookAheadDistance(const double offset) { offset_look_ahead_distance = offset; }
    void setMinLookAheadDistance(const double min) { min_look_ahead_distance; }
    void setMaxLookAheadDistance(const double max) { max_look_ahead_distance; }

    const VehicleSetting &getSetting() { return vehicle_setting; }
    const VehicleControl &getControl() { return vehicle_control; }
    const VehicleData &getVehicleData() { return vehicle_data; }
    const double &getTargetSpeed() { return target_speed; }
    const double &getTargetAngle() { return target_angle; }
    const int &getLookAheadPoint() { return look_ahead_index; }
    const double getStopTime() { return stop_count / kControlRate; }
    const double getSpeed() { return (vehicle_data.wheel.rear_left + vehicle_data.wheel.rear_right + vehicle_data.wheel.front_left + vehicle_data.wheel.front_right) / 4.0; }

    void setMorai(double *data);
    void setMoraiObject(double *data);
    void setMoraiLaneChange(int *data);
};

#include "av_control.h"

Control::Control()
    : target_speed(0.0),
      target_angle(0.0),
      del_angle(0.0),
      path_index(0),
      is_pose_init(false),
      look_ahead_index(-1),
      aeb_state(AEB_STATE::DEFAULT),
      can_alive_count(0),
      L(2.7),
      stop_count(0),
      stop_sign(false)
{
    TPCANStatus pcan_status = CAN_Initialize(PCAN_NUM, PCAN_BAUD, 0, 0, 0);
    fprintf(stderr, "Initialize CAN: %i\n", (int)pcan_status);
    is_morai = true;
    pid_vehicle_acc = PID(0.2, 0, 0.02, 1 / kControlRate, -3.0, 1.5); //5.0, 4.0, 1.0, avante 0.3, 0, 0.1
    // if(is_morai)
    // {
    //     gain_steer = 15.0;
    //     pid_vehicle_acc = PID(0.2, 4.0 / 1000.0, 0, 1 / kControlRate, -3.0, 1.5);
    // }
    // else
    // {
    //     //0.1, 0.0, 0.0
    //     pid_vehicle_acc = PID(0.2, 0 , 0, 1 / kControlRate, -3.0, 1.0); //5.0, 4.0, 1.0, avante 0.3, 0, 0.1
    //     gain_steer = 20.0541461947;
    // }
    pid_purepersuit = PID(0.2, 0, 0, 1 / kControlRate, -5.0, 5.0);
    test_lane_change = false;
    gain_err_y = 0.0;
    gain_look_ahead_distance = 0.3;
    offset_look_ahead_distance = 3.0;
    min_look_ahead_distance = 4.0;
    max_look_ahead_distance = 25.0;
    // gain_steer = 20.0541461947;
    gain_steer = 25.0;
}

Control::~Control() {}

void Control::setPcanMsg(TPCANMsg &msg, int id, int length, const unsigned char *data)
{
    msg.ID = id;
    msg.LEN = length;
    for (int i = 0; i < msg.LEN; i++)
    {
        if (data == NULL)
            msg.DATA[i] = 0;
        else
            msg.DATA[i] = data[i];
    }
}

bool Control::checkInitialized()
{
    if (is_pose_init == false)
    {
        fprintf(stderr, "gps not init\n");
        return false;
    }
    if (local_path.size() == 0)
    {
        // fprintf(stderr, "path not init\n");
        return false;
    }
    return true;
}

void Control::RunOnce()
{
    if (!checkInitialized())
        return;
    //km/h
    double curr_speed = 0.0;
    double max_speed = 40.0;
    //현재속도는 바퀴 4개 속도 평균
    curr_speed = getSpeed();

    //steer값 계산
    double target_steer = calSteer(curr_speed);
    //steer값 설정
    vehicle_control.setSteerAngle((short)(target_steer));

    //경로에서 현재속도가 5km/h 이상이면 Look ahead point 속도, 이하면 현재 위치 속도 사용.
    // target_speed = (curr_speed > 1) ? local_path[look_ahead_index].intensity : local_path[path_index].intensity;
    target_speed = local_path[look_ahead_index].intensity;
    // 속도범위 0~40 제한
    target_speed = target_speed < 0.1 ? 0 : (target_speed > max_speed ? max_speed : target_speed);
    // 목표속도가 0. 이하면 정지
    stop_sign = target_speed < 0.1 ? true : false;

    // 앞에 장애물이 있으면 acc사용
    double acc_power_pid = 0;
    // if(front_object_distance > 0)
    //     acc_power_pid = acc(curr_speed, front_object_speed, front_object_distance);
    // // if(vehicle_data.radar.radar_object_distant < 50 && vehicle_data.radar.radar_object_state == 1 && curr_speed < target_speed)
    // //     acc_power_pid = acc(curr_speed, vehicle_data.radar.radar_object_relative_speed, vehicle_data.radar.radar_object_distant);
    // else
    acc_power_pid = pid_vehicle_acc.update_new(curr_speed, target_speed, 1.0);

    //모라이 시뮬레이터면 acc변화량 제한
    static double acc_power = acc_power_pid;
    if (is_morai)
    {
        // acc_power 변화량 최대 1초에 1로 제한
        if (fabs(acc_power_pid - acc_power) > 1.0 / kControlRate /*&& copysign(1.0, acc_power_pid) == copysign(1.0, acc_power)*/)
            acc_power = acc_power < acc_power_pid ? acc_power + 1.0 / kControlRate : acc_power - 1.0 / kControlRate;
        else
            acc_power = acc_power_pid;
    }
    acc_power = acc_power_pid;
    //속도가 10km/h 이상일 때 감속은 2배값을 넣는다.
    // if(curr_speed > 5)
    acc_power = acc_power < 0 ? acc_power * 2.0 : acc_power;
    // 속도가 10km/h 이하일때 가속은 0.5배
    // else
    //     acc_power = acc_power > 0 ? acc_power / 2.0 : acc_power;

    // 0.5초이상 정지한 후 출발할때 1값 넣기
    static bool start_condition = false;
    if (stop_count / kControlRate > 0.5)
    {
        acc_power = 1;
        start_condition = false;
    }
    // if(test_lane_change)
    // {
    //    acc_power = acc_power > 0.0 ? acc_power : 0.0;
    // }
    // else
    // {
    //     start_condition = true;
    // }

    // 정지신호 들어오면 0.5초 이상 정지할때까지 acc에 -1입력
    if (stop_sign)
    {
        if (stop_count < 1)
            acc_power = acc_power > -1.0 ? -1.0 : acc_power;
        else
            //genBrakeAcc 쓸지 안쓸지 생각해봐야함.. -0.01은 2초뒤 출발
            acc_power = genBrakeAcc();
        // acc_power = is_morai ? -1 : -0.01;
    }
    //aeb
    double aeb_power = acc_power;
    // double aeb_power = aeb(curr_speed, vehicle_data.radar.radar_object_relative_speed, vehicle_data.radar.radar_object_distant);

    //AEB 체크.
    if (aeb_state != AEB_STATE::MAX_BRAKE)
        vehicle_setting.setVehicleSetting(true, false, 250, true, false, static_cast<int>(GEAR_STATE::D), static_cast<int>(TURN_SIGNAL::NONE), can_alive_count);
    //AEB 작동 후 속도가 0이면  AEB풀음
    else if (aeb_state == AEB_STATE::MAX_BRAKE && curr_speed < 0.2 && stop_sign == false)
    {
        fprintf(stderr, "AEB stop\n");
        //AEB 끝나면 acc제어를 풀어서 차량 aeb상태를 강제로 푸는 방법
        vehicle_setting.setVehicleSetting(false, false, 250, false, false, static_cast<int>(GEAR_STATE::D), static_cast<int>(TURN_SIGNAL::NONE), can_alive_count);
        acc_power = -1;
        aeb_state = AEB_STATE::DEFAULT;
    }
    //AEB 작동
    else
    {
        fprintf(stderr, "AEB!!!\n");
        vehicle_setting.setVehicleSetting(false, false, 250, true, true, static_cast<int>(GEAR_STATE::D), static_cast<int>(TURN_SIGNAL::NONE), can_alive_count);
        acc_power = aeb_power;
    }
    if (vehicle_data.acc.control_stat != 2 && vehicle_data.acc.control_stat != 3)
        acc_power = 0;
    vehicle_control.setAcceleration(acc_power);

    //avante pcan
    writeCanMsg();

    can_alive_count = (can_alive_count == 255) ? 0 : can_alive_count + 1;
    stop_count = curr_speed < 0.01 ? stop_count + 1 : 0;
}

double Control::genBrakeAcc()
{
    static bool flag_increase = false;
    static double brake_power = -1.0;
    // -25 ~ 20 까지 sin파형으로 만듦
    if (brake_power > 0.5)
        flag_increase = false;
    else if (brake_power < -0.05)
        flag_increase = true;

    brake_power = flag_increase ? brake_power + 0.01 : brake_power - 0.01;
    return brake_power;
}

void Control::writeCanMsg()
{
    TPCANStatus pcan_status;
    TPCANMsg pcan_control_msgs;
    setPcanMsg(pcan_control_msgs, 0x156, 8, vehicle_setting.getCanData());
    pcan_status = CAN_Write(PCAN_NUM, &pcan_control_msgs);

    setPcanMsg(pcan_control_msgs, 0x157, 8, vehicle_control.getCanData());
    pcan_status = CAN_Write(PCAN_NUM, &pcan_control_msgs);
}

double Control::calSteer(const double curr_speed)
{
    double target_steer = 0.0;
    double pure_pursuit_angle = purePersuit(curr_pose.position.x, curr_pose.position.y, tf2::getYaw(curr_pose.orientation), curr_speed, local_path);
    del_angle = fabs(pure_pursuit_angle - target_angle);
    geometry_msgs::TransformStamped transform;
    transform.transform.translation.x = curr_pose.position.x;
    transform.transform.translation.y = curr_pose.position.y;
    transform.transform.rotation = curr_pose.orientation;
    tf2::Transform tfGpsToWorld;
    tf2::fromMsg(transform.transform, tfGpsToWorld);
    tfGpsToWorld = tfGpsToWorld.inverse();
    geometry_msgs::TransformStamped gpsToWorldTransform;
    gpsToWorldTransform.transform = tf2::toMsg(tfGpsToWorld);

    geometry_msgs::Pose pose_path_index;
    pose_path_index.position.x = local_path.at(path_index).x;
    pose_path_index.position.y = local_path.at(path_index).y;
    pose_path_index.orientation.w = 1;
    tf2::doTransform(pose_path_index, pose_path_index, gpsToWorldTransform);
    double err_y = 0.0;
    if (pose_path_index.position.y > 0.0)
        err_y = hypot(pose_path_index.position.x, pose_path_index.position.y);
    else
        err_y = -hypot(pose_path_index.position.x, pose_path_index.position.y);
    del_angle += gain_err_y * err_y;
    if (is_morai)
    {
        //각도 변화량 최대 1초에 15도 로 제한
        // double limit_angle = 15.0 / kControlRate;
        // if (del_angle > limit_angle)
        //     target_angle = target_angle < pure_pursuit_angle ? target_angle += limit_angle : target_angle -= limit_angle;
        // else
        target_angle = pure_pursuit_angle * 4;
        target_steer = target_angle;
        //범위 -450 ~ 450 제한
        target_steer = target_steer < -30 ? -30 : (target_steer > 30 ? 30 : target_steer);
    }
    else
    {
        target_angle = pure_pursuit_angle;
        target_steer = target_angle * gain_steer;
        //범위 -490 ~ 490 제한
        target_steer = target_steer < -490 ? -490 : (target_steer > 490 ? 490 : target_steer);
    }
    return target_steer;
}

double Control::purePersuit(double curr_x, double curr_y, double curr_yaw, double curr_speed, pcl::PointCloud<pcl::PointXYZI> &     local_path)
{
    //purePersuit 목표점
    double look_ahead_x = 0.0,
           look_ahead_y = 0.0,
           point_radian = 0.0, //목표점과의 각도
        goal_angle = 0.0,      //목표 각도값
        alpha = 0.0,           //차량의 헤딩과 목표점 각도 차이
        k = 1;                 //HyperParameter(Gain) 높이면 반응 느려짐 avante:0.5 // 1 고정이 맞는것 같다.. k는 그냥 단순히 곡률일뿐 게인이 아님.

    //현재 path index 구하기
    path_index = findCurrPathIndex(curr_pose.position.x, curr_pose.position.y, local_path);
    //얼마나 앞의 점을 목표점으로 계산할지.
    double look_ahead_distance = offset_look_ahead_distance + gain_look_ahead_distance * curr_speed;
    look_ahead_distance = (look_ahead_distance < min_look_ahead_distance) ? min_look_ahead_distance : look_ahead_distance;
    look_ahead_distance = (look_ahead_distance > max_look_ahead_distance) ? max_look_ahead_distance : look_ahead_distance;

    look_ahead_index = findLookAheadPointIndex(curr_x, curr_y, look_ahead_distance, path_index, local_path);
    if (look_ahead_index == -1)
    {
        // fprintf(stderr, "Last Point@!@!@!@\n\n");
        look_ahead_index = local_path.points.size() - 1;
    }
    look_ahead_x = local_path.points[look_ahead_index].x;
    look_ahead_y = local_path.points[look_ahead_index].y;

    // gps to map tf 가져와서 역변환
    // geometry_msgs::TransformStamped transform;
    // transform.transform.translation.x = curr_pose.position.x;
    // transform.transform.translation.y = curr_pose.position.y;
    // transform.transform.rotation = curr_pose.orientation;
    // geometry_msgs::Pose look_ahead_pose;
    // look_ahead_pose.position.x = look_ahead_x;
    // look_ahead_pose.position.y = look_ahead_y;
    // look_ahead_pose.orientation.w = 1;
    // tf2::Transform tfGpsToWorld;
    // tf2::fromMsg(transform.transform, tfGpsToWorld);
    // tfGpsToWorld = tfGpsToWorld.inverse();
    // geometry_msgs::TransformStamped gpsToWorldTransform;
    // gpsToWorldTransform.transform = tf2::toMsg(tfGpsToWorld);

    // tf2::doTransform(look_ahead_pose, look_ahead_pose, gpsToWorldTransform);

    point_radian = atan2(look_ahead_y - curr_y, look_ahead_x - curr_x);
    alpha = point_radian - curr_yaw;
    // point_radian = atan2(look_ahead_pose.position.y, look_ahead_pose.position.x);
    // alpha = point_radian;
    goal_angle = atan2(2 * L * sin(alpha), (k * look_ahead_distance)) * kRadToDeg;
    // fprintf(stderr,"gx, gy: (%3.2lf,%3.2lf), alpha:%3.3f, curr angle::%3.3f", path_index, look_ahead_x - curr_x, look_ahead_y - curr_y, alpha* kRadToDeg, curr_yaw * kRadToDeg);
    // fprintf(stderr,"point angle: %2.2f, goal_angle: %2.2lf\n", point_radian * kRadToDeg, goal_angle);
    return goal_angle;
}

//현재위치 탐색.
int Control::findCurrPathIndex(double curr_x, double curr_y, const pcl::PointCloud<pcl::PointXYZI> &local_path)
{
    int curr_path_index = -1;
    double min_distance = DBL_MAX;
    for (int i = 0; i < local_path.points.size(); i++)
    {
        double dis = hypot(curr_x - local_path.points[i].x, curr_y - local_path.points[i].y);
        if (dis < min_distance)
        {
            min_distance = dis;
            curr_path_index = i;
        }
    }
    return curr_path_index;
}

//LookAheadPoint의 인덱스 탐색.
int Control::findLookAheadPointIndex(double curr_x, double curr_y, double look_ahead_distance, int path_index, const pcl::PointCloud<pcl::PointXYZI> &local_path)
{
    for (int i = path_index; i < local_path.points.size(); i++)
    {
        double dis = hypot(curr_x - local_path.points[i].x, curr_y - local_path.points[i].y);
        if (dis > look_ahead_distance)
        {
            return i;
        }
    }
    return -1;
}

//Param
//v_f : 내 차 속도
//v_relative : 앞차와의 상대속도
//s : 앞차와의 거리
double Control::acc(double v_f, double v_relative, double s)
{
    v_f = v_f * 1000 / 3600; // km/h -> m/s
    double x_0 = 2 + 5.4,    //완전히 멈췄을 때 안전거리 origin : 2m + car width(4.5)
        t_0 = 1.8,           //0보다 큰 값
        c_v = 0.05,          //상대속도 게인값(m/s)
        k_v = 0.58,          //속도게인 0.58
        k_s = 0.1,           //거리게인 0.1
        l_p = 4.5,           //앞차의 길이
        v_p,                 //앞차 속도
        t_h,                 //time_headway
        x_des,               //계산된 안전거리.
        s_d,                 //거리 설정값
        a_sc;                //결과 가속도값.

    v_p = v_f + v_relative; //앞차속도
    //앞차속도 0.5km/h이하면 0으로 간주.
    if (v_p * 3.6 < 0.5 && v_p > 0)
        v_p = 0;
    t_h = t_0 + c_v * (v_p - v_f);              //time_headway
    x_des = t_h * v_f + x_0;                    //계산된 안전거리.1
    s_d = x_des - l_p;                          //거리 설정값
    a_sc = k_v * (v_p - v_f) + k_s * (s - s_d); //결과 가속도값.

    //앞차 속도를 고려한 time headway
    //    double c_a = 0.3, //
    //        a_p;
    //    t_h = t_0 + c_v * (v_p - v_f) + c_a * a_p;

    //constant time headway
    //sfae: 2.4 normal: 1.8 agrresive: 1.2
    //    t_h = 1.8;
    //    x_des = t_h * v_f + x_0;                    //계산된 안전거리.
    //    s_d = x_des - l_p;                          //거리 설정값
    //    a_sc = k_v * (v_p - v_f) + k_s * (s - s_d); //결과 가속도값.
    // fprintf(stderr,"v_p:%2.2f, v_f:%2.2f, s_diff:%2.5f \n",v_p,v_f,s-s_d);
    // fprintf(stderr,"front_car: %2.1f, x_des: %2.1f, t_h: %2.2f, s_d: %2.1f, a_sc: %2.1f\n", s, x_des, t_h, s_d, a_sc);

    //범위 -3~1.5 제한
    a_sc = a_sc < -3.0 ? -3.0 : (a_sc > 0.5 ? 0.5 : a_sc);
    return a_sc;
}

// aeb
// @param v_ego  내 차 속도
// @param v_relative  앞차와의 상대속도
// @param dis  앞차와의 거리
double Control::aeb(double v_ego, double v_relative, double dis)
{
    v_ego = v_ego * 1000 / 3600;             // km/h -> m/s
    double aeb_offset = 2 + 4,               //aeb가 작동할 거리 offset+ car_width
        ttc,                                 //충돌까지의 시간
        decel[4] = {-0.1, -0.5, -1.5, -3.0}, //브레이크 밟았을 경우 감속도 단계
        time_to_React,                       //운전자 반응속도
        aeb_time_margin = 0.5,               //시간 마진(게인값)
        stop_time[4],                        //감속단계별 정지시간
        result_acc;                          //가속도값
    bool fcw_active = false,                 //충돌경고
        stop = false;                        //차량 정지 여부
    if (stop_count / kControlRate >= 1.0)
        stop = true;

    v_relative = vehicle_data.radar.radar_object_relative_speed; //상대 속도
    //내차속도가 0이 아니면 ttc 계산
    if (v_ego != 0)
        ttc = dis / v_relative; //충돌까지의 시간
    else
        ttc = 999;
    fprintf(stderr, "stop_time: ");
    for (int i = 0; i < 4; i++)
    {
        stop_time[i] = -(v_ego / decel[i]) + aeb_time_margin;
        fprintf(stderr, "%2.1f ", stop_time[i]);
    }
    stop_time[3] = 0.0395 * v_ego + 0.6 + aeb_time_margin;

    fprintf(stderr, "dis: %2.1f, v_r: %2.1f, ttc: %2.1f\t", dis, v_relative * 3.6, ttc);
    switch (aeb_state)
    {
    case AEB_STATE::DEFAULT:
        fprintf(stderr, "aeb: DEFAULT\n");
        if ((fabs(ttc) < stop_time[0]) && ttc < 0 && vehicle_data.radar.radar_object_state != 0)
            aeb_state = AEB_STATE::FCW;
        break;

    case AEB_STATE::FCW:
        fprintf(stderr, "aeb: FCW\n");
        fcw_active = true;
        if ((fabs(ttc) < stop_time[1]) && ttc < 0 && vehicle_data.radar.radar_object_state != 0)
            aeb_state = AEB_STATE::STAGE1;
        else if (ttc >= 1.2 * stop_time[1])
            aeb_state = AEB_STATE::DEFAULT;
        break;

    case AEB_STATE::STAGE1:
        fprintf(stderr, "aeb: STAGE1\n");
        fcw_active = true;
        result_acc = decel[1];
        if ((fabs(ttc) < stop_time[2]) && ttc < 0 && vehicle_data.radar.radar_object_state != 0)
            aeb_state = AEB_STATE::STAGE2;
        else if (stop == true)
            aeb_state = AEB_STATE::DEFAULT;
        break;

    case AEB_STATE::STAGE2:
        fprintf(stderr, "aeb: STAGE2\n");
        fcw_active = true;
        result_acc = decel[2];
        if ((fabs(ttc) < stop_time[3]) && ttc < 0 && vehicle_data.radar.radar_object_state != 0)
            aeb_state = AEB_STATE::MAX_BRAKE;
        else if (stop == true)
            aeb_state = AEB_STATE::DEFAULT;
        break;

    case AEB_STATE::MAX_BRAKE:
        fprintf(stderr, "aeb: MAX_BRAKE\n");
        //        result_acc = decel[3];
        result_acc = -6;
        fcw_active = true;
        if (stop == true)
            aeb_state = AEB_STATE::DEFAULT;
        break;

    default:
        break;
    }
    return result_acc;
}

void Control::setMorai(double *data)
{
    vehicle_data.acc.speed = data[0];       //speed,
    vehicle_data.eps.steer_angle = data[1]; //steering_angle,
    // data[2];//steering_angular_velocity,
    vehicle_data.wheel.front_left = data[0];  //wheel_velocity_fl,
    vehicle_data.wheel.front_right = data[0]; //wheel_velocity_fr,
    vehicle_data.wheel.rear_left = data[0];   //wheel_velocity_rl,
    vehicle_data.wheel.rear_right = data[0];  //wheel_velocity_rr,
    //  data[7];//yaw
}

void Control::setMoraiObject(double *data)
{
    vehicle_data.radar.radar_object_distant = data[0];
    vehicle_data.radar.radar_object_relative_speed = data[1]; //상대 속도
    vehicle_data.radar.radar_object_state = static_cast<unsigned char>(data[2]);
}

void Control::setMoraiLaneChange(int *data)
{
    if (data[2] == 1)
    {
        vehicle_data.acc.turn_left_enable = data[0];
        vehicle_data.acc.turn_right_enable = data[1];
    }
    else
    {
        vehicle_data.acc.turn_left_enable = 0;
        vehicle_data.acc.turn_right_enable = 0;
    }
}

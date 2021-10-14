#include "core_control.h"

CoreControl::CoreControl(ros::NodeHandle &node_handle) : nh(node_handle),
                                                         tf2_listener(tf2_buffer),
                                                         mission_state(MissionState::error),
                                                         state_path(StatePath::lane_keeping),
                                                         state_path_next(StatePath::lane_keeping)
{
    nh.param("/core/contorl/is_simulation", is_simulation, true);
    nh.param("/core/control/is_morai", is_morai, true);
    nh.param("/core/control/speed_slow_obejct", speed_slow_obejct, 18.0);
    nh.param("/core/control/gain_err_y", gain_err_y, 20.0);
    nh.param("/core/control/gain_steer", gain_steer, 25.0);
    nh.param("/core/control/gain_look_ahead_distance", gain_look_ahead_distance, 0.3);
    nh.param("/core/control/offset_look_ahead_distance", offset_look_ahead_distance, 3.0);
    nh.param("/core/control/min_look_ahead_distance", min_look_ahead_distance, 4.0);
    nh.param("/core/control/max_look_ahead_distance", max_look_ahead_distance, 25.0);
    nh.param("/core/control/offset_yaw", offset_yaw, 0.75);

    control.setGainErrY(gain_err_y);
    control.setGainSteer(gain_steer);
    control.setGainLookAhaedDistance(gain_look_ahead_distance);
    control.setOffsetLookAheadDistance(offset_look_ahead_distance);
    control.setMinLookAheadDistance(min_look_ahead_distance);
    control.setMaxLookAheadDistance(max_look_ahead_distance);
    //novatel gps 데이터 sub
    // sub_bestpos = nh.subscribe("/bestpos",1,&CoreControl::callbackBestpos, this);
    // sub_inspvax = nh.subscribe("/inspvax",1,&CoreControl::callbackInspvax, this);

    //형택이 전역경로 받기
    client_path_global = nh.serviceClient<core_map::RequestPath>(kSrvicePathEmergency);
    sub_path_global_dague = nh.subscribe(kTopicGlobalPath, 1, &CoreControl::callbackPathGlobal, this);
    pub_mission_state = nh.advertise<std_msgs::Int8>(kTopicMissionState, 1);
    sub_traffic_sign = nh.subscribe(kTopicTrafficInfo, 1, &CoreControl::callbackTrafficInfo, this);
    sub_state_mission_fsm = nh.subscribe(kTopicFsmMission, 1, &CoreControl::callbackStateMissionFsm, this);

    //min seong frenet local path
    pub_frenet_state = nh.advertise<core_map::frenet_input>(kTopicfrenetInput,1);
    sub_frenet_path = nh.subscribe(kTopicfrenetpath, 1, &CoreControl::callbackfrenetpath, this);
    if (!is_simulation)
        //자동차 정보 pub
        pub_avante_data = nh.advertise<core_control::AvanteData>(kTopicAvanteData, 1);
    else
        sub_avante_data = nh.subscribe(kTopicAvanteData, 1, &CoreControl::callbackAvanteData, this);

    //위치 관련 pub
    pub_path_gps = nh.advertise<geometry_msgs::PoseArray>(kTopicPathGps, 1);
    pub_look_ahead_point = nh.advertise<sensor_msgs::PointCloud2>(kTopicLookAheadPoint, 1);

    //경로 관련 pub
    pub_path_local = nh.advertise<sensor_msgs::PointCloud2>(kTopicPathLocal, 1);

    //Local Path 관련
    pub_occupancy_map_my = nh.advertise<nav_msgs::OccupancyGrid>(kTopicMyMap, 1);

    //tf관련
    initTf();

    // 장애물 관련
    sub_lidar_objects = nh.subscribe(kTopicObstacles, 1, &CoreControl::callbackLidarObjects, this);

    // 초기값 설정 임시..
    ////////////////////////////////////////////
    gps_pose.position.x = 1; //21
    gps_pose.position.y = 1; //41
    gps_pose.position.z = 0;
    tf2::Quaternion q;
    q.setRPY(0, 0, 90.0 * M_PI / 180.0);
    gps_pose.orientation = tf2::toMsg(q);
    ////////////////////////////////////////////

    //Morai 관련 sub, pub
    if (is_morai)
    {
        sub_morai_tlm = nh.subscribe(kTopicMoraiTlm, 1, &CoreControl::callbackMoraiTlm, this);
        sub_morai_gps = nh.subscribe(kTopicMoariGps, 1, &CoreControl::callbackMoraiGps, this);
        sub_morai_objects = nh.subscribe(kTopicMoraiObjects, 1, &CoreControl::callbackObjects, this);
        pub_morai_cmd = nh.advertise<MoraiCmd>(kTopicMoraiCmd, 1);
        control.setL(2.854);
        control.setIsMorai(is_morai);
    }
    else
        control.setL(2.7);

    dis_traffic_stop = -1;
    is_new_path_global = false;
    sign_stop = false;

    sub_irregular = nh.subscribe(kTopicIrregular, 1, &CoreControl::callbackIrregularLocation, this);
    client_bsd = nh.serviceClient<core_map::Bsd>(kSrviceBsd);
    client_check_profit = nh.serviceClient<core_map::CheckProfit>(kSrviceCheckProfit);
    client_intersection = nh.serviceClient<core_map::Intersection>(kServiceIntersection);
    client_front_irr = nh.serviceClient<core_map::FrontIrr>(kServiceFrontIrr);

    if (!is_simulation)
        thread_get_can = std::thread(&CoreControl::threadCan, this);
    thread_control = std::thread(&CoreControl::threadControl, this);
    is_connect_can = true;
    is_pub_mission_state = false;
}

void CoreControl::get_frenet_path()
{
    core_map::frenet_input msg;
    msg.cur_pos = gps_pose;

//    msg.obs_pos = obstacle_morai;

    for(int i=0; i<obstacle_morai.size(); i++){
        core_map::frenet_obstacle tmp;
        for(int j =0; j < obstacle_morai[i].size(); j++)
        {
        //[1] : upper right point // [3] : lower left point
//        msg.obs_pos.push_back(obstacle_morai[i]);
//          msg.obs_pos.push_back(obstacle_morai[i])
            tmp.Points.push_back(obstacle_morai[i][j]);
        }
        msg.obs_pos.push_back(tmp);
    }
    WayPointIndex idx = local_path.getIndex();

    msg.cur_speed = vehicle_data.getSpeedDouble() / 3.6; // m/s
    msg.target_speed = vehicle_data.getSpeedDouble() / 3.6;
//    for(int i=0; i<msg_path_global.pathAry.size(); i++){ //need to minimize global path array
//        for(int j=0; j<msg_path_global.pathAry[i].links.size();j++){
//            for(int k=0; k<msg_path_global.pathAry[i].links[j].waypointAry.size(); k++){
//                msg.waypointAry.push_back(msg_path_global.pathAry[i].links[j].waypointAry[k]);
//            }
//        }
//    }

    if (path_local.size() == 0)
        std::cout<< "0" << std::endl;
    for(size_t i =0; i<path_local.size(); i++){
        core_map::waypoint waypoint;
        waypoint.x = path_local[i].x;
        waypoint.y = path_local[i].y;
        msg.waypointAry.push_back(waypoint);
    }

    msg.frenet_state = local_path.frenet_state;
//    if (local_path.path_local.size() ==0)
//        msg.frenet_state = 0;
    pub_frenet_state.publish(msg);
}

void CoreControl::callbackfrenetpath(const core_map::frenet_output &msg){

    PC_XYZI frenet_path;
    pcl::fromROSMsg(msg.frenet_path, frenet_path); //roi
//    pcl::PCLPointCloud2 cloud_ROI;
//    for (size_t i = 0; i< msg.frenet_path.points.size(); i++){
//        pcl::PointXYZI p;
//        p.x = msg.frenet_path.points[i].x;
//        p.y = msg.frenet_path.points[i].y;
//        p.z = 0;
//        p.intensity = 0;
//        frenet_path.push_back(p);
//    }

//    frenet_path = msg;
    local_path.setFrenetPath(frenet_path);
    if (local_path.frenet_state == 1 && msg.frenet_state ==0)
        local_path.frenet_state = 0; // 0 :initial  , 1: changing
}

CoreControl::~CoreControl()
{
}

void CoreControl::threadControl()
{
    double control_rate = 50;
    ros::Rate r(control_rate);
    while (ros::ok())
    {
        ros::Time start = ros::Time::now();
        VehicleData vehicle_data_tmp;
        {
            std::unique_lock<std::mutex> lock_can(mutex_can);
            vehicle_data_tmp = vehicle_data_can;
        }
        fprintf(stderr, "vehicle_data time: %lf", (ros::Time::now() - start).toSec());

        start = ros::Time::now();
        geometry_msgs::Pose control_pose;
        {
            std::unique_lock<std::mutex> lock_gps(mutex_gps_pose);
            control_pose = gps_pose;
        }
        fprintf(stderr, "gps_pose time: %lf", (ros::Time::now() - start).toSec());

        start = ros::Time::now();
        {
            std::unique_lock<std::mutex> lock_control(mutex_control);
            control.setPose(control_pose);
            control.setCanData(vehicle_data_tmp);
            control.RunOnce();
        }
        fprintf(stderr, "control time: %lf\n", (ros::Time::now() - start).toSec());

        r.sleep();
    }
}

void CoreControl::tfListen()
{
    geometry_msgs::TransformStamped transformStamped;
    try
    {
        transformStamped = tf2_buffer.lookupTransform(kFrameWorld, kFramePos, ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        fprintf(stderr, "%s\n", ex.what());
        ros::Duration(1.0).sleep();
        return;
    }
    geometry_msgs::Pose ndt_pose;
    ndt_pose.orientation = transformStamped.transform.rotation;
    ndt_pose.position.x = transformStamped.transform.translation.x - 1.0 * cos(tf2::getYaw(ndt_pose.orientation));
    ndt_pose.position.y = transformStamped.transform.translation.y - 1.0 * sin(tf2::getYaw(ndt_pose.orientation));
    ndt_pose.position.z = transformStamped.transform.translation.z;
    tf2::Quaternion q;
    q.setRPY(0, 0, tf2::getYaw(ndt_pose.orientation) + offset_yaw * M_PI / 180.0);
    ndt_pose.orientation = tf2::toMsg(q);
    {
        std::unique_lock<std::mutex> lock_gps(mutex_gps_pose);
        gps_pose = ndt_pose;
        lock_gps.unlock();
    }
    path_gps.poses.push_back(gps_pose);
}

bool CoreControl::checkTrafficSignalStop(const core_map::traffic_light &traffic_light)
{
    TrafficSignState state_traffic_sign;
    if (traffic_light.status <= static_cast<char>(TrafficSignState::unavailable))
        state_traffic_sign = TrafficSignState::unavailable;
    else if (traffic_light.status <= static_cast<char>(TrafficSignState::red))
        state_traffic_sign = TrafficSignState::red;
    else if (traffic_light.status <= static_cast<char>(TrafficSignState::green))
        state_traffic_sign = TrafficSignState::green;
    else
        state_traffic_sign = TrafficSignState::yellow;

    bool sign_stop_tmp = false;
    if (state_traffic_sign == TrafficSignState::green)
    {
        // km/h -> m/s
        double curr_speed = vehicle_data.getSpeedDouble();
        curr_speed = curr_speed < 1.0 ? 1.0 : curr_speed;
        double arrive_time = traffic_light.dist / curr_speed;
        // 도착시간이 남은시간보다 크면(더 오래걸리면) 신호등 앞 정지
        if (arrive_time + 0.5 > (traffic_light.minEndTime / 10.0))
            sign_stop_tmp = true;
        else
            sign_stop_tmp = false;
    }
    else if (state_traffic_sign == TrafficSignState::unavailable)
        sign_stop_tmp = false;
    else
        sign_stop_tmp = true;
    return sign_stop_tmp;
}

void CoreControl::pubAvanteData()
{
    core_control::AvanteData msg_avante_data;
    msg_avante_data.header.stamp = ros::Time::now();

    // Setting
    msg_avante_data.setting.can.id = 0x156;
    msg_avante_data.setting.can.len = 8;
    msg_avante_data.setting.can.data.resize(8);
    for (int i = 0; i < 8; i++)
        msg_avante_data.setting.can.data[i] = vehicle_setting.getCanData()[i];
    msg_avante_data.setting.eps_enable = vehicle_setting.eps_enable;
    msg_avante_data.setting.override_ignore = vehicle_setting.override_ignore;
    msg_avante_data.setting.eps_speed = vehicle_setting.eps_speed;
    msg_avante_data.setting.acc_enable = vehicle_setting.acc_enable;
    msg_avante_data.setting.aeb_enable = vehicle_setting.aeb_enable;
    msg_avante_data.setting.gear_change = vehicle_setting.gear_change;
    msg_avante_data.setting.turn_signal_enable = vehicle_setting.turn_signal_enable;
    msg_avante_data.setting.alive_count = vehicle_setting.alive_count;

    // Control
    msg_avante_data.control.can.id = 0x157;
    msg_avante_data.control.can.len = 8;
    msg_avante_data.control.can.data.resize(8);
    for (int i = 0; i < 8; i++)
        msg_avante_data.control.can.data[i] = vehicle_control.getCanData()[i];

    //control command
    msg_avante_data.control.steer_command = vehicle_control.steer_command;
    msg_avante_data.control.acceleration_command = vehicle_control.acceleration_command;

    //Acc
    msg_avante_data.acc.acc_control_board_status = vehicle_data.acc.control_board_stat;
    msg_avante_data.acc.acc_enable_status = vehicle_data.acc.enable;
    msg_avante_data.acc.acc_control_status = vehicle_data.acc.control_stat;
    msg_avante_data.acc.acc_user_can_err = vehicle_data.acc.user_can_err;
    msg_avante_data.acc.acc_vehicle_err = vehicle_data.acc.vehicle_err;
    msg_avante_data.acc.acc_err = vehicle_data.acc.err;
    msg_avante_data.acc.aeb_active = vehicle_data.acc.aeb_action;
    msg_avante_data.acc.vehicle_speed = vehicle_data.acc.speed;
    msg_avante_data.acc.turn_left = vehicle_data.acc.turn_left_enable;
    msg_avante_data.acc.turn_rigt = vehicle_data.acc.turn_right_enable;
    msg_avante_data.acc.harzard = vehicle_data.acc.harzard_enable;
    msg_avante_data.acc.gear_select = vehicle_data.acc.gear_select;
    msg_avante_data.acc.bsd_left = vehicle_data.acc.bsd_left;
    msg_avante_data.acc.bsd_right = vehicle_data.acc.bsd_right;
    msg_avante_data.acc.long_accel = vehicle_data.acc.long_accel;
    msg_avante_data.acc.acc_alive_count = vehicle_data.acc.alive_count;

    //Eps
    msg_avante_data.eps.eps_enable_status = vehicle_data.eps.enable;
    msg_avante_data.eps.eps_control_board_status = vehicle_data.eps.control_board_stat;
    msg_avante_data.eps.eps_control_status = vehicle_data.eps.control_stat;
    msg_avante_data.eps.eps_user_can_err = vehicle_data.eps.user_can_err;
    msg_avante_data.eps.eps_err = vehicle_data.eps.err;
    msg_avante_data.eps.eps_vehicle_can_err = vehicle_data.eps.vehicle_can_err;
    msg_avante_data.eps.eps_sas_err = vehicle_data.eps.sas_err;
    msg_avante_data.eps.override_ignore_status = vehicle_data.eps.override_ignore_stat;
    msg_avante_data.eps.override_status = vehicle_data.eps.override_stat;
    msg_avante_data.eps.steer_angle = vehicle_data.eps.steer_angle;
    msg_avante_data.eps.steer_drive_torque = vehicle_data.eps.steer_drv_torque;
    msg_avante_data.eps.steer_out_torque = vehicle_data.eps.steer_out_torque;
    msg_avante_data.eps.eps_alive_count = vehicle_data.eps.alive_count;

    //Radar
    msg_avante_data.radar.object_distance = vehicle_data.radar.radar_object_distant;
    msg_avante_data.radar.object_lateral_pos = vehicle_data.radar.radar_object_lateral_pos;
    msg_avante_data.radar.object_relative_speed = vehicle_data.radar.radar_object_relative_speed;
    msg_avante_data.radar.object_state = vehicle_data.radar.radar_object_state;

    //Whell Speed
    msg_avante_data.wheel_speed.front_left = vehicle_data.wheel.front_left;
    msg_avante_data.wheel_speed.front_right = vehicle_data.wheel.front_right;
    msg_avante_data.wheel_speed.rear_left = vehicle_data.wheel.rear_left;
    msg_avante_data.wheel_speed.rear_right = vehicle_data.wheel.rear_right;

    //etc
    msg_avante_data.yaw_rate = vehicle_data.yaw_and_brake.yaw_rate;
    msg_avante_data.lateral_accel = vehicle_data.yaw_and_brake.lat_accel;
    msg_avante_data.brake_pressure = vehicle_data.yaw_and_brake.brake_pressure;

    pub_avante_data.publish(msg_avante_data);
}

void CoreControl::callbackAvanteData(const core_control::AvanteData::ConstPtr &msg)
{
    AccStat acc_state;
    acc_state.control_board_stat = msg->acc.acc_control_board_status;
    acc_state.enable = msg->acc.acc_enable_status;
    acc_state.control_stat = msg->acc.acc_control_status;
    acc_state.user_can_err = msg->acc.acc_user_can_err;
    acc_state.vehicle_err = msg->acc.acc_vehicle_err;
    acc_state.err = msg->acc.acc_err;
    acc_state.aeb_action = msg->acc.aeb_active;
    acc_state.speed = msg->acc.vehicle_speed;
    acc_state.turn_left_enable = msg->acc.turn_left;
    acc_state.turn_right_enable = msg->acc.turn_rigt;
    acc_state.harzard_enable = msg->acc.harzard;
    acc_state.gear_select = msg->acc.gear_select;
    acc_state.bsd_left = msg->acc.bsd_left;
    acc_state.bsd_right = msg->acc.bsd_right;
    acc_state.long_accel = msg->acc.long_accel;
    acc_state.alive_count = msg->acc.acc_alive_count;

    EpsStat eps_state;
    eps_state.enable = msg->eps.eps_enable_status;
    eps_state.control_board_stat = msg->eps.eps_control_board_status;
    eps_state.control_stat = msg->eps.eps_control_status;
    eps_state.user_can_err = msg->eps.eps_user_can_err;
    eps_state.err = msg->eps.eps_err;
    eps_state.vehicle_can_err = msg->eps.eps_vehicle_can_err;
    eps_state.sas_err = msg->eps.eps_sas_err;
    eps_state.override_ignore_stat = msg->eps.override_ignore_status;
    eps_state.override_stat = msg->eps.override_status;
    eps_state.steer_angle = msg->eps.steer_angle;
    eps_state.steer_drv_torque = msg->eps.steer_drive_torque;
    eps_state.steer_out_torque = msg->eps.steer_out_torque;
    eps_state.alive_count = msg->eps.eps_alive_count;

    WheelSpeed wheel_speed;
    wheel_speed.front_left = msg->wheel_speed.front_left;
    wheel_speed.front_right = msg->wheel_speed.front_right;
    wheel_speed.rear_left = msg->wheel_speed.rear_left;
    wheel_speed.rear_right = msg->wheel_speed.rear_right;
    std::unique_lock<std::mutex> lock_can(mutex_can);
    vehicle_data_can.acc = acc_state;
    vehicle_data_can.eps = eps_state;
    vehicle_data_can.wheel = wheel_speed;
}

void CoreControl::pubGpsPath()
{
    if (path_gps.poses.empty())
        return;
    pub_path_gps.publish(path_gps);
    path_gps.header.frame_id = kFrameWorld;
}

void CoreControl::pubPathLocal()
{
    if (path_local.empty())
        return;
    path_local.resize(path_local.size());
    sensor_msgs::PointCloud2::Ptr path_local_msg_ptr(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(path_local, *path_local_msg_ptr);
    path_local_msg_ptr->header.frame_id = kFrameWorld;
    pub_path_local.publish(*path_local_msg_ptr);
}

void CoreControl::pubLookAheadPoint()
{
    std::unique_lock<std::mutex> lock_control(mutex_control);
    int index = control.getLookAheadPoint();
    lock_control.unlock();
    look_ahead_point.points.clear();
    if (index >= path_local.points.size() || index < 0)
        return;
    look_ahead_point.points.push_back(path_local.points[index]);
    sensor_msgs::PointCloud2::Ptr look_ahead_point_ptr(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(look_ahead_point, *look_ahead_point_ptr);
    // pcl::toROSMsg(local_path.path_debug, *look_ahead_point_ptr);
    look_ahead_point_ptr->header.frame_id = kFrameWorld;
    pub_look_ahead_point.publish(*look_ahead_point_ptr);
}

void CoreControl::pubOccupancyMap()
{
    nav_msgs::OccupancyGrid my_map = occupancy_map.getGridMap();
    double res = occupancy_map.getResolution();
    my_map.info.resolution = occupancy_map.getGridMap().info.resolution / res;
    my_map.info.origin.position.x = -1 * (int)(my_map.info.width / 2.0 / res) + gps_pose.position.x;
    my_map.info.origin.position.y = -1 * (int)(my_map.info.height / 2.0 / res) + gps_pose.position.y;
    my_map.header.frame_id = "world";
    pub_occupancy_map_my.publish(my_map);
}

void CoreControl::pubMissionState()
{
    if (state_mission_fsm == 3)
        msg_mission_state.data = 1;
    else if (state_mission_fsm == 4)
        msg_mission_state.data = 2;
    else
        msg_mission_state.data = 0;
    is_pub_mission_state = true;
    pub_mission_state.publish(msg_mission_state);
}

void CoreControl::pubTf()
{
    transform_stampeds[static_cast<int>(FRAME_IDX::GPS)].header.stamp = ros::Time::now();
    transform_stampeds[static_cast<int>(FRAME_IDX::GPS)].transform.translation.x = gps_pose.position.x;
    transform_stampeds[static_cast<int>(FRAME_IDX::GPS)].transform.translation.y = gps_pose.position.y;
    transform_stampeds[static_cast<int>(FRAME_IDX::GPS)].transform.rotation = gps_pose.orientation;

    transform_stampeds[static_cast<int>(FRAME_IDX::pos)].header.stamp = ros::Time::now();
    transform_stampeds[static_cast<int>(FRAME_IDX::pos)].transform.translation.x = gps_pose.position.x;
    transform_stampeds[static_cast<int>(FRAME_IDX::pos)].transform.translation.y = gps_pose.position.y;
    transform_stampeds[static_cast<int>(FRAME_IDX::pos)].transform.rotation = gps_pose.orientation;

    transform_stampeds[static_cast<int>(FRAME_IDX::map)].header.stamp = ros::Time::now();

    if (is_morai)
        transform_stampeds[static_cast<int>(FRAME_IDX::velodyne)].header.stamp = ros::Time::now();

    br.sendTransform(transform_stampeds);
}

void CoreControl::initTf()
{
    geometry_msgs::TransformStamped stamp;

    stamp.header.stamp = ros::Time::now();
    stamp.header.frame_id = kFrameWorld;
    stamp.child_frame_id = kFrameGps;
    stamp.transform.translation.x = 0.0;
    stamp.transform.translation.y = 0.0;
    stamp.transform.translation.z = 0.0;
    stamp.transform.rotation = gps_pose.orientation;
    transform_stampeds.push_back(stamp);

    stamp.header.stamp = ros::Time::now();
    stamp.header.frame_id = kFrameWorld;
    stamp.child_frame_id = kFramePos;
    stamp.transform.translation.x = 0.0;
    stamp.transform.translation.y = 0.0;
    stamp.transform.translation.z = 0.0;
    stamp.transform.rotation = gps_pose.orientation;
    transform_stampeds.push_back(stamp);

    stamp.header.stamp = ros::Time::now();
    stamp.header.frame_id = kFrameWorld;
    stamp.child_frame_id = kFrameMap;
    stamp.transform.translation.x = 0.0;
    stamp.transform.translation.y = 0.0;
    stamp.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    stamp.transform.rotation = tf2::toMsg(q);
    transform_stampeds.push_back(stamp);

    if (is_morai)
    {
        stamp.header.stamp = ros::Time::now();
        stamp.header.frame_id = kFramePos;
        stamp.child_frame_id = kFrameVelodyne;
        stamp.transform.translation.x = 1.0;
        stamp.transform.translation.y = 0.0;
        stamp.transform.translation.z = 1.5;
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        stamp.transform.rotation = tf2::toMsg(q);
        transform_stampeds.push_back(stamp);
    }

    transform_stampeds[static_cast<int>(FRAME_IDX::pos)].transform.rotation = gps_pose.orientation;
}

void CoreControl::callbackLidarObjects(const polygon_msgs::polygonArray &msg) { msg_polygon_array_lidar = msg; }
void CoreControl::callbackIrregularLocation(const visualization_msgs::MarkerArray &msg) { msg_polygon_irregular = msg; }
void CoreControl::callbackStateMissionFsm(const std_msgs::Int8 &msg) { state_mission_fsm = msg.data; }
void CoreControl::callbackTrafficInfo(const core_map::traffic_light_info::ConstPtr &msg) { traffic_info = *msg; }

void CoreControl::callbackPathGlobal(const core_map::global_path::ConstPtr &msg)
{
    msg_path_global = *msg;
    is_new_path_global = true;
    global_start_point = msg->pathAry.front().links[msg->pathAry.front().src].waypointAry.front();
    if (msg->pathAry.back().changeLane == false)
        global_goal_point = msg->pathAry.back().links[msg->pathAry.back().src].waypointAry.back();
    else
        global_goal_point = msg->pathAry.back().links[msg->pathAry.back().dst].waypointAry.back();
}

void CoreControl::callbackBestpos(const novatel_gps_msgs::NovatelPosition::ConstPtr &msg)
{
    //위도, 경도, 높이
    double lat = msg->lat,
           lon = msg->lon,
           height = msg->height;

    //위도, 경도, 높이, 분산
    double lat_sigma = msg->lat_sigma,
           log_sigma = msg->lon_sigma,
           height_sigma = msg->height_sigma;

    pcl::PointXYZI p;
    double x = 0.0, y = 0.0;
    transGps(lat, lon, x, y);
    p.x = x + OFFSET_X;
    p.y = y + OFFSET_Y;
    // path_gps.poses.clear();
    // path_gps.poses.push_back(p);
    gps_pose.position.x = p.x;
    gps_pose.position.y = p.y;
    std::unique_lock<std::mutex> lock_control(mutex_control);
    control.setPose(gps_pose);
    lock_control.unlock();

    // path_gps.poses.push_back(p);
    pubTf();
}

void CoreControl::callbackInspvax(const novatel_gps_msgs::Inspvax::ConstPtr &msg)
{
    //북 기준 시계방향으로 0~359. -> x축 기준 -180 ~ 180 변경
    double head = msg->azimuth;
    double heading = 90 - head;
    if (head > 270)
        heading += 360;
    tf2::Quaternion q;
    q.setRPY(0, 0, heading * M_PI / 180.0);
    gps_pose.orientation = tf2::toMsg(q);
    path_gps.poses.push_back(gps_pose);
}

void CoreControl::transGps(double lat, double lon, double &east, double &north)
{
    double Lat = lat;
    double Lon = lon;
    double Easting;
    double Northing;
    int Zone;
    char Letter = 'N';
    Zone = (int)floor(Lon / 6 + 31);

    Easting = 0.5 * log((1 + cos(Lat * M_PI / 180) * sin(Lon * M_PI / 180 - (6 * Zone - 183) * M_PI / 180)) / (1 - cos(Lat * M_PI / 180) * sin(Lon * M_PI / 180 - (6 * Zone - 183) * M_PI / 180))) * 0.9996 * 6399593.62 / pow((1 + pow(0.0820944379, 2) * pow(cos(Lat * M_PI / 180), 2)), 0.5) * (1 + pow(0.0820944379, 2) / 2 * pow((0.5 * log((1 + cos(Lat * M_PI / 180) * sin(Lon * M_PI / 180 - (6 * Zone - 183) * M_PI / 180)) / (1 - cos(Lat * M_PI / 180) * sin(Lon * M_PI / 180 - (6 * Zone - 183) * M_PI / 180)))), 2) * pow(cos(Lat * M_PI / 180), 2) / 3) + 500000;
    Easting = round(Easting * 100) * 0.01;
    Northing = (atan(tan(Lat * M_PI / 180) / cos((Lon * M_PI / 180 - (6 * Zone - 183) * M_PI / 180))) - Lat * M_PI / 180) * 0.9996 * 6399593.625 / sqrt(1 + 0.006739496742 * pow(cos(Lat * M_PI / 180), 2)) * (1 + 0.006739496742 / 2 * pow(0.5 * log((1 + cos(Lat * M_PI / 180) * sin((Lon * M_PI / 180 - (6 * Zone - 183) * M_PI / 180))) / (1 - cos(Lat * M_PI / 180) * sin((Lon * M_PI / 180 - (6 * Zone - 183) * M_PI / 180)))), 2) * pow(cos(Lat * M_PI / 180), 2)) + 0.9996 * 6399593.625 * (Lat * M_PI / 180 - 0.005054622556 * (Lat * M_PI / 180 + sin(2 * Lat * M_PI / 180) / 2) + 4.258201531e-05 * (3 * (Lat * M_PI / 180 + sin(2 * Lat * M_PI / 180) / 2) + sin(2 * Lat * M_PI / 180) * pow(cos(Lat * M_PI / 180), 2)) / 4 - 1.674057895e-07 * (5 * (3 * (Lat * M_PI / 180 + sin(2 * Lat * M_PI / 180) / 2) + sin(2 * Lat * M_PI / 180) * pow(cos(Lat * M_PI / 180), 2)) / 4 + sin(2 * Lat * M_PI / 180) * pow(cos(Lat * M_PI / 180), 2) * pow(cos(Lat * M_PI / 180), 2)) / 3);

    Northing = round(Northing * 100) * 0.01;

    east = Easting;
    north = Northing;
}

void CoreControl::threadCan()
{
    TPCANMsg can_msg;
    TPCANStatus Status;
    fd_set Fds;

    int fd;

    CAN_GetValue(PCAN_NUM, PCAN_RECEIVE_EVENT, &fd, sizeof(int));

    FD_ZERO(&Fds);
    FD_SET(fd, &Fds);
    ros::Time time_data_save = ros::Time::now();
    VehicleData vehicle_data_thread;
    while (select(fd + 1, &Fds, NULL, NULL, NULL) > 0)
    {
        Status = CAN_Read(PCAN_NUM, &can_msg, NULL);

        if (Status != PCAN_ERROR_OK)
        {
            fprintf(stderr, "Error 0x%x\n", (int)Status);
            break;
        }
        if ((ros::Time::now() - time_data_save).toSec() > 0.02)
        {
            time_data_save = ros::Time::now();
            std::unique_lock<std::mutex> lock_can_data(mutex_can);
            vehicle_data_can = vehicle_data_thread;
        }
        if (can_msg.ID == 0x710)
        {
            vehicle_data_thread.eps.setValue(can_msg.DATA);
        }
        else if (can_msg.ID == 0x711)
        {
            vehicle_data_thread.acc.setValue(can_msg.DATA);
        }
        else if (can_msg.ID == 0x712)
        {
            vehicle_data_thread.wheel.setValue(can_msg.DATA);
        }
        else if (can_msg.ID == 0x713)
        {
            vehicle_data_thread.yaw_and_brake.setValue(can_msg.DATA);
        }
        else if (can_msg.ID == 0x714)
        {
            vehicle_data_thread.radar.setValue(can_msg.DATA);
        }
    }
    if (!is_morai && !is_simulation)
        is_connect_can = false;
}

bool CoreControl::callPathSrv(double src_x, double src_y, double dst_x, double dst_y)
{
    fprintf(stderr, "Request Path Service!!");
    core_map::RequestPath srvPath;
    srvPath.request.src.x = src_x;
    srvPath.request.src.y = src_y;
    srvPath.request.dst.x = dst_x;
    srvPath.request.dst.y = dst_y;
    double curr_speed = vehicle_data.getSpeedDouble();
    srvPath.request.currSpeed = curr_speed / 3.6; // km/h -> m/s

    if (client_path_global.call(srvPath))
    {
        is_new_path_global = true;
        std::cout << "srv_paht find!" << std::endl;
        msg_path_global = srvPath.response.path;
        return true;
    }
    else
        return false;
}
/**
 * @brief 여러 장애물 메시지 하나로 통합
 * 
 */
void CoreControl::resetObject()
{
    objects = msg_polygon_array_lidar;
    // polygon_msgs::polygon tmp;
    // std::vector<geometry_msgs::Point> points;
    // geometry_msgs::Point p;
    // double x = -46.81;
    // double y = 3.14;
    // double scale_x = 10;
    // double scale_y = 0.9;
    // p.x = x + scale_x;
    // p.y = y + scale_y;
    // tmp.markers.points.push_back(p);
    // p.x = x + scale_x;
    // p.y = y - scale_y;
    // tmp.markers.points.push_back(p);
    // p.x = x - scale_x;
    // p.y = y - scale_y;
    // tmp.markers.points.push_back(p);
    // p.x = x - scale_x;
    // p.y = y + scale_y;
    // tmp.markers.points.push_back(p);
    // p.x = x + scale_x;
    // p.y = y + scale_y;
    // tmp.markers.points.push_back(p);
    // msg_polygon_array_lidar.polygon.clear();
    // msg_polygon_array_lidar.polygon.push_back(tmp);
    for (int i = 0; i < msg_polygon_array_lidar.polygon.size(); i++)
        occupancy_map.addObstaclePolygon(msg_polygon_array_lidar.polygon.at(i).markers.points);

    for (int i = 0; i < msg_polygon_irregular.markers.size(); i++)
    {
        occupancy_map.addObstaclePolygon(msg_polygon_irregular.markers.at(i).points);
        polygon_msgs::polygon msg_polygon;
        msg_polygon.markers = msg_polygon_irregular.markers.at(i);
        msg_polygon.class_name = "irregular";
        msg_polygon.global_x = 0.0;
        msg_polygon.global_y = 0.0;
        msg_polygon.velocity_x = 0.0;
        msg_polygon.velocity_y = 0.0;
        msg_polygon.speed = 0.0;
        objects.polygon.push_back(msg_polygon);
    }
    for (int i = 0; i < obstacle_morai.size(); i++)
    {
        occupancy_map.addObstaclePolygon(obstacle_morai.at(i));
        polygon_msgs::polygon msg_polygon;
        msg_polygon.class_name = "irregular";
        msg_polygon.markers.points = obstacle_morai.at(i);
        msg_polygon.global_x = 0.0;
        msg_polygon.global_y = 0.0;
        msg_polygon.velocity_x = 0.0;
        msg_polygon.velocity_y = 0.0;
        msg_polygon.speed = 20.0;
        objects.polygon.push_back(msg_polygon);
    }
}

bool CoreControl::btCheckMissionWait()
{
    if (mission_state == MissionState::mission_wait)
        return true;
    else
        return false;
}
bool CoreControl::btCheckFrontObject() { return local_path.tickCheckFrontObject(); }
bool CoreControl::btCheckNearCorner()
{
    double dis_max = 35;
    core_map::Intersection srv_intersection;
    int index_object = local_path.getIndexObjectFront();
    if (index_object >= 0 || index_object < objects.polygon.size())
    {
        srv_intersection.request.p.x = objects.polygon.at(index_object).global_x;
        srv_intersection.request.p.y = objects.polygon.at(index_object).global_y;
        srv_intersection.request.p.z = 0;
        if (client_intersection.call(srv_intersection))
        {
            if (srv_intersection.response.dist < dis_max && objects.polygon.at(index_object).class_name != "irregular")
                return true;
        }
        else
        {
            fprintf(stderr, "client_intersection call fail\n");
        }
    }
    return false;
}
bool CoreControl::btCheckNearStopline()
{
    double dis_max = 15;
    core_map::Intersection srv_intersection;
    int index_object = local_path.getIndexObjectFront();
    if (index_object < 0 || index_object >= objects.polygon.size())
        return false;
    srv_intersection.request.p.x = objects.polygon.at(index_object).global_x;
    srv_intersection.request.p.y = objects.polygon.at(index_object).global_y;
    srv_intersection.request.p.z = 0;
    if (client_intersection.call(srv_intersection))
    {
        if (srv_intersection.response.dist < dis_max)
            return true;
    }
    else
    {
        fprintf(stderr, "client_intersection call fail\n");
    }
    return false;
}
bool CoreControl::btCheckCollision() { return local_path.tickCheckCollision(); }
bool CoreControl::btCheckLaneChangeableLeft() { return local_path.tickCheckLaneChangeableLeft(); }
bool CoreControl::btCheckLaneChangeableRight() { return local_path.tickCheckLaneChangeableRight(); }
bool CoreControl::btCheckPassAbleTrafficLight()
{
    for (int i = 0; i < traffic_info.infoAry.size(); i++)
    {
        if (checkTrafficSignalStop(traffic_info.infoAry.at(i)))
        {
            dis_traffic_stop = traffic_info.infoAry.at(i).dist;
            return false;
        }
    }
    return true;
}
bool CoreControl::btCheckGlobalLaneChange()
{
    if (local_path.tickCheckGlobalLaneChange())
        return true;
    else
        return false;
}
bool CoreControl::btCheckLeftChange()
{
    return local_path.tickCheckLeftChange();
}
bool CoreControl::btCheckExistPathGlobal()
{
    if (msg_path_global.pathAry.empty())
    {
        return false;
    }
    else
    {
        // 새로운 경로가 들어오면 경로 리셋
        if (is_new_path_global)
        {
            path_lane_keeping.clear();
            path_lanechnage.clear();
            local_path.resetIndex();
            state_path = StatePath::lane_keeping;
            state_path_next = StatePath::lane_keeping;
            is_new_path_global = false;
        }
        return true;
    }
}
bool CoreControl::btCheckStateLaneKeeping()
{
    if (state_path == StatePath::lane_keeping)
        return true;
    else
        return false;
}
bool CoreControl::btCheckStateLaneChangeLeft()
{
    if (state_path == StatePath::left_changing)
        return true;
    else
        return false;
}
bool CoreControl::btCheckStateLaneChangeRight()
{
    if (state_path == StatePath::right_changing)
        return true;
    else
        return false;
}
bool CoreControl::btCheckEndLaneChange()
{
    return local_path.tickCheckEndLaneChange();
}
bool CoreControl::btCheckStop()
{
    std::unique_lock<std::mutex> lock_control(mutex_control);
    double curr_speed = control.getSpeed();
    double stop_time = control.getStopTime();
    lock_control.unlock();
    if (curr_speed < 0.05 && stop_time > 0.5)
        return true;
    else
        return false;
}
bool CoreControl::btCheckArriveGoal()
{
    double dis = hypot(gps_pose.position.x - global_goal_point.x, gps_pose.position.y - global_goal_point.y);
    std::unique_lock<std::mutex> lock_contorl(mutex_control);
    double time_stop = control.getStopTime();
    lock_contorl.unlock();
    if (dis < 5.0 && time_stop > 1)
        return true;
    else
        return false;
}
bool CoreControl::btCheckCollisionBehindLeft()
{
    core_map::Bsd request_bsd;
    core_map::point p;
    p.x = local_path.getLaneChangeEndPoint().x;
    p.y = local_path.getLaneChangeEndPoint().y;
    p.z = 0;
    request_bsd.request.p.push_back(p);
    if (client_bsd.call(request_bsd))
    {
        local_path.setPathBehind(request_bsd.response.bsdLink.front().link);

        return local_path.checkCollisionBehindLeft();
    }
    else
        return false;
}
bool CoreControl::btCheckCollisionBehindRight()
{
    core_map::Bsd request_bsd;
    core_map::point p;
    p.x = local_path.getLaneChangeEndPoint().x;
    p.y = local_path.getLaneChangeEndPoint().y;
    p.z = 0;
    request_bsd.request.p.push_back(p);
    if (client_bsd.call(request_bsd))
    {
        local_path.setPathBehind(request_bsd.response.bsdLink.front().link);

        return local_path.checkCollisionBehindRight();
    }
    else
        return false;
}
// 미완성
bool CoreControl::btCheckProfitLaneChangeLeft()
{
    // 마지막 path면 비정형은 피함.
    if (local_path.checkEndPath())
    {
        int index_object = local_path.getIndexObjectFront();
        if (index_object != -1)
        {
            if (objects.polygon.at(index_object).class_name == "irregular")
            {
                geometry_msgs::Point p = objects.polygon.at(index_object).markers.points.front();
                double dis_object = pow(gps_pose.position.x - p.x, 2) + pow(gps_pose.position.y - p.y, 2);
                double dis_goal = pow(gps_pose.position.x - global_goal_point.x, 2) + pow(gps_pose.position.y - global_goal_point.y, 2);
                if (dis_object < dis_goal)
                    return true;
            }
        }
        return false;
    }
    else
    {
        if (local_path.checkIrregularFront())
            return true;
        core_map::CheckProfit srv_check_profit;
        srv_check_profit.request.tonode.push_back(local_path.getTonodeCurrLane());
        srv_check_profit.request.tonode.push_back(local_path.getTonodeLeftLane());
        if (client_check_profit.call(srv_check_profit))
        {
            if (srv_check_profit.response.totalTime.at(1) > 99999.0)
                return false;
            int index_object = local_path.getIndexObjectFront();
            if (index_object >= 0 && index_object < objects.polygon.size())
            {
                if (objects.polygon.at(index_object).speed < speed_slow_obejct / 3.6)
                {
                    // 옆차선으로 가는게 차이가 별로 없으면 true
                    if (srv_check_profit.response.totalTime.at(1) - 10 < srv_check_profit.response.totalTime.at(0))
                        return true;
                }
            }
        }
        return false;
    }
}
// 미완성
bool CoreControl::btCheckProfitLaneChangeRight()
{
    // 마지막 path면 비정형은 피함.
    if (local_path.checkEndPath())
    {
        int index_object = local_path.getIndexObjectFront();
        if (index_object != -1)
        {
            if (objects.polygon.at(index_object).class_name == "irregular")
            {
                geometry_msgs::Point p = objects.polygon.at(index_object).markers.points.front();
                double dis_object = pow(gps_pose.position.x - p.x, 2) + pow(gps_pose.position.y - p.y, 2);
                double dis_goal = pow(gps_pose.position.x - global_goal_point.x, 2) + pow(gps_pose.position.y - global_goal_point.y, 2);
                if (dis_object < dis_goal)
                    return true;
            }
        }
        return false;
    }
    else
    {
        if (local_path.checkIrregularFront())
            return true;
        core_map::CheckProfit srv_check_profit;
        srv_check_profit.request.tonode.push_back(local_path.getTonodeCurrLane());
        srv_check_profit.request.tonode.push_back(local_path.getTonodeRightLane());
        if (client_check_profit.call(srv_check_profit))
        {
            if (srv_check_profit.response.totalTime.at(1) > 99999.0)
                return false;
            int index_object = local_path.getIndexObjectFront();
            if (index_object >= 0 && index_object < objects.polygon.size())
            {
                if (objects.polygon.at(index_object).speed < speed_slow_obejct / 3.6)
                {
                    // 옆차선으로 가는게 차이가 별로 없으면 true
                    if (srv_check_profit.response.totalTime.at(1) - 10 < srv_check_profit.response.totalTime.at(0))
                        return true;
                }
            }
        }
        return false;
    }
}
// 미완성
bool CoreControl::btCheckFailLaneChange()
{
    if (local_path.tickCheckFailLaneChange())
    {
        // if(control.getStopTime() > 50)
        return true;
    }
    return false;
}
// 미완성
bool CoreControl::btCheckUnprotectedTurn()
{
    return local_path.checkUnprotected();
}
// 미완성
bool CoreControl::btCheckImpassalbeUnprotectedTurn()
{
    UnprotectedLink link = local_path.getUnprotectedLink();
    if (link.link_id == "")
        return false;
    // 교차로 위일때
    if (link.dis_check < 0.0)
    {
        // 링크 위면 반 지나가면 검사안함
        if (local_path.getIndex().point > local_path.getLink().waypointAry.size() / 2.0)
            return false;
        else
        {
            core_map::Bsd request_bsd;
            request_bsd.request.p = link.bsd_points;
            if (client_bsd.call(request_bsd))
            {
                for (int iter_links = 0; iter_links < request_bsd.response.bsdLink.size(); iter_links++)
                {
                    local_path.setPathBehind(request_bsd.response.bsdLink.at(iter_links).link);
                    if (local_path.checkCollisionUnprotectedTurn())
                        return true;
                }
            }
        }
    }
    // 교차로 전링크 일 때
    else
    {
        double dis_min = INT_MAX;
        for (int iter_point = 0; iter_point < link.bsd_points.size(); iter_point++)
        {
            double dis = hypot(gps_pose.position.x - link.bsd_points.at(iter_point).x, gps_pose.position.y - link.bsd_points.at(iter_point).y);
            dis_min = dis < dis_min ? dis : dis_min;
        }

        double dis_stop_max = getDistanceStop(vehicle_data.getSpeedDouble());
        double dis_unprotected_marjin = 5.0;
        if (dis_min > dis_stop_max + dis_unprotected_marjin)
            return false;
        else
        {
            core_map::Bsd request_bsd;
            request_bsd.request.p = link.bsd_points;
            if (client_bsd.call(request_bsd))
            {
                for (int iter_links = 0; iter_links < request_bsd.response.bsdLink.size(); iter_links++)
                {
                    local_path.setPathBehind(request_bsd.response.bsdLink.at(iter_links).link);
                    if (local_path.checkCollisionUnprotectedTurn())
                        return true;
                }
            }
        }
    }
    return false;
}
// 미완성
bool CoreControl::btCheckMustLaneChange()
{
    // 다음 링크확인
    core_map::CheckProfit srv_check_profit;
    srv_check_profit.request.tonode.push_back(local_path.getTonodeCurrLane());
    srv_check_profit.request.tonode.push_back(local_path.getTonodeLeftLane());
    srv_check_profit.request.tonode.push_back(local_path.getTonodeRightLane());
    if (client_check_profit.call(srv_check_profit))
    {
        double time_curr_lane = srv_check_profit.response.totalTime.at(0);
        double time_left_lane = srv_check_profit.response.totalTime.at(1);
        double time_right_lane = srv_check_profit.response.totalTime.at(2);
        if (time_left_lane < time_curr_lane - 10 || time_right_lane < time_curr_lane - 10)
        {
            if (local_path.getPath().changeLane)
            {
                local_path.genSpeedProfileMustLaneChange();
                return true;
            }
        }
    }
    local_path.checkMustLaneChange();
    return false;
}
// 미완성
bool CoreControl::btCheckObjectStaticLeftLane()
{
    return local_path.checkObjectStaticLeftLane();
}
bool CoreControl::btCheckObjectStaticRightLane()
{
    return local_path.checkObjectStaticRightLane();
}
bool CoreControl::btCheckObjectNearGoal()
{
    return local_path.checkObjectnearGoal();
}
bool CoreControl::btCheckIrregularFront()
{
    return local_path.checkIrregularFront();
}
// 미완성
bool CoreControl::btCheckIrregularLeft()
{
    core_map::FrontIrr front_irr;
    front_irr.request.p.x = local_path.getPointLeftLane().x;
    front_irr.request.p.y = local_path.getPointLeftLane().y;
    if (front_irr.request.p.x > 0.0 && front_irr.request.p.y > 0.0)
    {
        if (client_front_irr.call(front_irr))
        {
            double dis_thres_front_irr = 200.0;
            if (front_irr.response.dist < dis_thres_front_irr && front_irr.response.dist >= 0.0)
                return true;
        }
    }
    return false;
}

bool CoreControl::btCheckIrregularRight()
{
    core_map::FrontIrr front_irr;
    front_irr.request.p.x = local_path.getPointRightLane().x;
    front_irr.request.p.y = local_path.getPointRightLane().y;
    if (front_irr.request.p.x > 0.0 && front_irr.request.p.y > 0.0)
    {
        if (client_front_irr.call(front_irr))
        {
            double dis_thres_front_irr = 200.0;
            if (front_irr.response.dist < dis_thres_front_irr && front_irr.response.dist >= 0.0)
                return true;
        }
    }
    return false;
}
bool CoreControl::btCheckMeNearGoal()
{
    return local_path.checkMeNearGoal();
}

bool CoreControl::btActionInit()
{
    if (!is_connect_can)
    {
        ros::Duration(1.0).sleep();
        fprintf(stderr, "can't connet can");
        return false;
    }
    tfListen();
    sign_stop = false;
    occupancy_map.clearObstaclePolygons();
    objects.polygon.clear();
    resetObject();
    local_path.setObject(objects);
    occupancy_map.drawObstaclePolygons(gps_pose);
    nav_msgs::OccupancyGrid my_map = occupancy_map.getGridMap();
    double res = occupancy_map.getResolution();
    my_map.info.resolution = my_map.info.resolution / res;

    {
        std::unique_lock<std::mutex> lock_control(mutex_control);
        vehicle_setting = control.getSetting();
        vehicle_control = control.getControl();
    }
    {
        std::unique_lock<std::mutex> lock_can(mutex_can);
        vehicle_data = vehicle_data_can;
    }
    local_path.path_debug.clear();
    local_path.setPathGlobal(msg_path_global);
    local_path.setPose(gps_pose);
    local_path.setOccMap(my_map);
    local_path.setSpeed(vehicle_data.getSpeedDouble());
    state_path = state_path_next;
    if (local_path.initPose())
        return true;
    else
    {
        // if(global_goal_point.x != 0)
        // {
        //     if(callPathSrv(gps_pose.position.x, gps_pose.position.y, global_goal_point.x, global_goal_point.y))
        //     {
        //         local_path.setPathGlobal(msg_global_path);
        //         if(local_path.initPose())
        //             return true;
        //     }
        // }
        return false;
    }
}
bool CoreControl::btActionStop()
{
    sign_stop = true;
    local_path.genSpeedProfileStop();
    return true;
}
// 미완성
bool CoreControl::btActionStoplineStop()
{
}
// 미완성 control에서 AEB 작동하는 것 확인해야함.
bool CoreControl::btActionEmergencyStop()
{
    std::unique_lock<std::mutex> lock_control(mutex_control);
    control.activateAeb();
    return true;
}
bool CoreControl::btActionDrive()
{
    sign_stop = false;
    // path_lane_keeping = local_path.getPathLaneKeeping();
    // state_path = StatePath::lane_keeping;
    return true;
}
bool CoreControl::btActionAcc()
{
    state_path = StatePath::lane_keeping;
    return true;
}
bool CoreControl::btActionLaneChangeLeft()
{
    if (local_path.setLaneChangeLeft())
    {
        state_path_next = StatePath::left_changing;
        return true;
    }
    else
        return false;
}
bool CoreControl::btActionLaneChangeRight()
{
    if (local_path.setLaneChangeRight())
    {
        state_path_next = StatePath::right_changing;
        return true;
    }
    else
        return false;
}
bool CoreControl::btActionSetPath()
{
    switch (state_path)
    {
    case StatePath::lane_keeping:
        path_lane_keeping = local_path.getPathLaneKeeping();
        path_local = path_lane_keeping;
        break;
    case StatePath::left_changing:
    case StatePath::right_changing:
        path_lanechnage = local_path.getPathLanechage();
        path_local = path_lanechnage;
        break;
    default:
        path_lane_keeping = local_path.getPathLaneKeeping();
        path_local = path_lane_keeping;
        break;
    }
    // 경로와 현재 위치가 멀면 초기화
    if (local_path.checkFarPath())
    {
        state_path_next = StatePath::lane_keeping;
        path_lane_keeping.clear();
        path_lanechnage.clear();
        return false;
    }
    if (sign_stop)
    {
        for (int i = 0; i < path_local.size(); i++)
            path_local.at(i).intensity = 0;
    }
    std::unique_lock<std::mutex> lock_control(mutex_control);
    control.setPath(path_local);
    lock_control.unlock();
    return true;
}
bool CoreControl::btActionGenPathLaneChangeLeft()
{
    if (local_path.genPathLaneChangeLeft())
    {
        path_lanechnage = local_path.getPathLanechage();
        return true;
    }
    else
        return false;
}
bool CoreControl::btActionGenPathLaneChangeRight()
{
    if (local_path.genPathLaneChangeRight())
    {
        path_lanechnage = local_path.getPathLanechage();
        return true;
    }
    else
        return false;
}
bool CoreControl::btActionGenPathLaneKeeping()
{
//    local_path.clearFrenetPath();
    get_frenet_path();
    fprintf(stderr,"pub frenet \n");
    if (local_path.genPathLaneKeeping())
    {
        path_lane_keeping = local_path.getPathLaneKeeping();
        return true;
    }
    else
        return false;
}
bool CoreControl::btActionSetStateLaneKeeping()
{
    state_path_next = StatePath::lane_keeping;
    local_path.setPathState(StatePath::lane_keeping);
    return true;
}
bool CoreControl::btActionSetStateLaneChangeLeft()
{
    state_path_next = StatePath::left_changing;
    local_path.setPathState(StatePath::left_changing);
    return true;
}
bool CoreControl::btActionSetStateLaneChangeRight()
{
    state_path_next = StatePath::right_changing;
    local_path.setPathState(StatePath::right_changing);
    return true;
}
bool CoreControl::btActionPublishData()
{
    if (!is_simulation)
        pubAvanteData();
    pubPathLocal();
    pubOccupancyMap();
    pubGpsPath();
    pubLookAheadPoint();
    if (is_morai)
        pubMoraiCmd();
    return true;
}
bool CoreControl::btActionStopTrafficLight()
{
    if (dis_traffic_stop > 0)
    {
        switch (state_path)
        {
        case StatePath::lane_keeping:
            if (local_path.genSpeedProfileStop(dis_traffic_stop - 3.0))
            {
                path_lane_keeping = local_path.getPathLaneKeeping();
                return true;
            }
            break;
        case StatePath::left_changing:
        case StatePath::right_changing:
            if (local_path.genSpeedProfileStop(dis_traffic_stop - 3.0))
            {
                path_lanechnage = local_path.getPathLanechage();
                return true;
            }
            break;
        default:
            break;
        }
    }
    return false;
}
bool CoreControl::btSetStateMissionGoing()
{
    mission_state = MissionState::mission_ing;
    is_pub_mission_state = false;
    return true;
}
bool CoreControl::btSetStateMissionArrive()
{
    mission_state = MissionState::mission_end;
    return true;
}
bool CoreControl::btPublishStateMission()
{
    if (!is_pub_mission_state)
        pubMissionState();
    return true;
}
bool CoreControl::btCallPathGlobal()
{
    core_map::RequestPath srv_request_path;
    srv_request_path.request.currSpeed = vehicle_data.acc.speed / 3.6;
    srv_request_path.request.src.x = gps_pose.position.x;
    srv_request_path.request.src.y = gps_pose.position.y;
    srv_request_path.request.dst.x = global_goal_point.x;
    srv_request_path.request.dst.y = global_goal_point.y;
    srv_request_path.request.changeLane = 0;
    if (client_path_global.call(srv_request_path))
    {
        msg_path_global = srv_request_path.response.path;
        is_new_path_global = true;
        return true;
    }
    else
        return false;
}
bool CoreControl::btCallPathGlobalLeft()
{
    core_map::RequestPath srv_request_path;
    srv_request_path.request.currSpeed = vehicle_data.acc.speed / 3.6;
    srv_request_path.request.src.x = gps_pose.position.x;
    srv_request_path.request.src.y = gps_pose.position.y;
    srv_request_path.request.dst.x = global_goal_point.x;
    srv_request_path.request.dst.y = global_goal_point.y;
    srv_request_path.request.changeLane = 1;
    if (client_path_global.call(srv_request_path))
    {
        msg_path_global = srv_request_path.response.path;
        is_new_path_global = true;
        return true;
    }
    else
        return false;
}
bool CoreControl::btCallPathGlobalRight()
{
    core_map::RequestPath srv_request_path;
    srv_request_path.request.currSpeed = vehicle_data.acc.speed / 3.6;
    srv_request_path.request.src.x = gps_pose.position.x;
    srv_request_path.request.src.y = gps_pose.position.y;
    srv_request_path.request.dst.x = global_goal_point.x;
    srv_request_path.request.dst.y = global_goal_point.y;
    srv_request_path.request.changeLane = 2;
    if (client_path_global.call(srv_request_path))
    {
        msg_path_global = srv_request_path.response.path;
        is_new_path_global = true;
        return true;
    }
    else
        return false;
}
// 미완성
bool CoreControl::btGenSpeedUnprotectedTurn()
{
    // if(local_path.checkUnprotectedTurnLeft())
    // {
    //     core_map::Bsd request_bsd;
    //     request_bsd.request.p.x = local_path.getLaneChangeEndPoint().x;
    //     request_bsd.request.p.y = local_path.getLaneChangeEndPoint().y;
    //     request_bsd.request.p.z = 0;
    //     if(client_bsd.call(request_bsd))
    //     {
    //         local_path.setPathBehind(request_bsd.response.links);

    //         return local_path.checkCollisionBehindRight();
    //     }
    //     else
    //         return true;
    // }
    // else
    //     return local_path.genSpeedProfileUnprotectedTurn(false);
}
// 미완성
bool CoreControl::btSpeedProfileLaneChange()
{
    PC_XYZI path_profile = local_path.genSpeedProfileLaneChange();
    if (!path_profile.empty())
    {
        path_lanechnage = path_profile;
        return true;
    }
    else
        return false;
}

void CoreControl::pubMoraiCmd(const bool &is_end)
{
    MoraiCmd msg_vehicle_cmd;
    VehicleControl vehicle_control = control.getControl();
    if (is_end)
    {
        msg_vehicle_cmd.longlCmdType = 2;
        msg_vehicle_cmd.acceleration = 0;
        // msg_vehicle_cmd.gear_cmd = static_cast<int>(MoariGear::P);
        msg_vehicle_cmd.steering = 0;
        // msg_vehicle_cmd.steering_angular_velocity_cmd =15;
        pub_morai_cmd.publish(msg_vehicle_cmd);
    }
    else
    {
        // msg_vehicle_cmd.longlCmdType = 2;
        // msg_vehicle_cmd.acceleration = vehicle_control.acceleration_command;
        // msg_vehicle_cmd.gear_cmd = static_cast<int>(MoariGear::D);
        msg_vehicle_cmd.steering = -control.getTargetAngle();
        // msg_vehicle_cmd.steering_angular_velocity_cmd =15;
        bool acc_control = false;
        if (acc_control)
        {
            msg_vehicle_cmd.longlCmdType = 3;
            msg_vehicle_cmd.acceleration = vehicle_control.acceleration_command;
        }
        else
        {
            msg_vehicle_cmd.longlCmdType = 1;
            msg_vehicle_cmd.accel = vehicle_control.acceleration_command;

            if (vehicle_control.acceleration_command > 0)
            {
                msg_vehicle_cmd.accel = vehicle_control.acceleration_command;
                msg_vehicle_cmd.brake = 0;
            }
            else
            {
                msg_vehicle_cmd.accel = 0;
                msg_vehicle_cmd.brake = -vehicle_control.acceleration_command;
            }
        }

        pub_morai_cmd.publish(msg_vehicle_cmd);
    }
}

void CoreControl::callbackMoraiTlm(const MoraiTlm::ConstPtr &msg)
{
    double data[10] = {
        msg->speed * 3.6,
        msg->steering_angle,
        msg->steering_angular_velocity,
        msg->wheel_velocity_fl,
        msg->wheel_velocity_fr,
        msg->wheel_velocity_rl,
        msg->wheel_velocity_rr,
        msg->ned_heading};
    std::unique_lock<std::mutex> lock_can(mutex_can);
    vehicle_data_can.acc.control_stat = 2;
    vehicle_data_can.acc.speed = msg->speed * 3.6;
    vehicle_data_can.wheel.front_left = vehicle_data_can.acc.speed;
    vehicle_data_can.wheel.front_right = vehicle_data_can.acc.speed;
    vehicle_data_can.wheel.rear_left = vehicle_data_can.acc.speed;
    vehicle_data_can.wheel.rear_right = vehicle_data_can.acc.speed;
    lock_can.unlock();
    double x = 0.0, y = 0.0;
    pcl::PointXYZI p;
    transGps(msg->ned_latitude, msg->ned_longitude, x, y);
    p.x = x + OFFSET_X;
    p.y = y + OFFSET_Y;

    gps_pose.position.x = p.x;
    gps_pose.position.y = p.y;
    //북 기준 반시계방향으로 0~359. -> x축 기준 -180 ~ 180
    double heading = 90 + msg->ned_heading;
    // heading = heading < -180 ? heading + 360 : heading;
    heading = msg->ned_heading;
    tf2::Quaternion q;
    q.setRPY(0, 0, heading * M_PI / 180.0);
    gps_pose.orientation = tf2::toMsg(q);
    // gps_pose.position.x -= 1.8 * cos(tf2::getYaw(gps_pose.orientation));
    // gps_pose.position.y -= 1.8 * sin(tf2::getYaw(gps_pose.orientation));
    path_gps.poses.push_back(gps_pose);
    pubTf();
}
void CoreControl::callbackMoraiGps(const MoraiGps::ConstPtr &msg)
{
}
void CoreControl::callbackObjects(const MoraiObjects::ConstPtr &msg)
{
    static double pre_relative_x = 0;
    static ros::Time _t = ros::Time::now();
    double min_x = 999.0;
    int index = 0;
    obstacle_morai.clear();
    if (msg->object_data_array.size() != 0)
    {
        for (int i = 0; i < msg->object_data_array.size(); i++)
        {
            if (min_x > msg->object_data_array[i].rel_pos_x)
            {
                min_x = msg->object_data_array[i].rel_pos_x;
                index = i;
            }
            double x = 0.0, y = 0.0;
            transGps(msg->object_data_array[i].global_pos_lat, msg->object_data_array[i].global_pos_lon, x, y);
            std::vector<geometry_msgs::Point> points;
            x += OFFSET_X;
            y += OFFSET_Y;
            geometry_msgs::Point p;
            double scale_x = 0.1;
            double scale_y = 0.1;
            double scale_z = 0.1;

            scale_x = msg->object_data_array.at(i).length * 0.5;
            scale_y = msg->object_data_array.at(i).width * 0.5;
            scale_z = msg->object_data_array.at(i).height;

            geometry_msgs::Point front_left, front_right, back_left, back_right;

            std::vector<geometry_msgs::Point> object_points;
            p.x = -scale_x;
            p.y = scale_y;
            object_points.push_back(p);

            p.x = scale_x;
            p.y = scale_y;
            object_points.push_back(p);

            p.x = scale_x;
            p.y = -scale_y;
            object_points.push_back(p);

            p.x = -scale_x;
            p.y = -scale_y;
            object_points.push_back(p);

            for (auto &point : object_points)
            {
                double cos_heading = cos(msg->object_data_array[i].global_heading / 180.0 * M_PI);
                double sin_heading = sin(msg->object_data_array[i].global_heading / 180.0 * M_PI);
                double tmp_x = point.x * cos_heading - point.y * sin_heading;
                double tmp_y = point.x * sin_heading + point.y * cos_heading;
                point.x = x + tmp_x;
                point.y = y + tmp_y;
            }

            object_points.push_back(object_points.front());
            obstacle_morai.push_back(object_points);
        }
    }
}

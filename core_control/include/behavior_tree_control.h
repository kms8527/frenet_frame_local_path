#pragma once
#include <iostream>
#include <unistd.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <core_control.h>
#include <time.h>

class BehaviorTreeControl
{
private:
    BT::BehaviorTreeFactory factory;
    BT::Tree tree;
    CoreControl core_control;
    bool printf_state;
    BT::NodeStatus checkMissionWait();
    BT::NodeStatus checkFrontObject();
    BT::NodeStatus checkNearStopLine();
    BT::NodeStatus checkNearCorner();
    BT::NodeStatus checkLaneChangeableLeft();
    BT::NodeStatus checkLaneChangeableRight();
    BT::NodeStatus checkCollision();
    BT::NodeStatus checkPassAbleTrafficLight();
    BT::NodeStatus checkGlobalLaneChange();
    BT::NodeStatus checkLeftChange();
    BT::NodeStatus checkExistPathGlobal();
    BT::NodeStatus checkStateLaneKeeping();
    BT::NodeStatus checkStateLaneChangeLeft();
    BT::NodeStatus checkStateLaneChangeRight();
    BT::NodeStatus checkEndLaneChange();
    BT::NodeStatus checkStop();
    BT::NodeStatus checkArriveGoal();
    BT::NodeStatus checkCollisionBehindLeft();
    BT::NodeStatus checkCollisionBehindRight();
    BT::NodeStatus checkProfitLaneChangeLeft();
    BT::NodeStatus checkProfitLaneChangeRight();
    BT::NodeStatus checkFailLaneChange();
    BT::NodeStatus checkUnprotected();
    BT::NodeStatus checkImpassalbeUnprotected();
    BT::NodeStatus checkMustLaneChange();
    BT::NodeStatus checkObjectStaticLeftLane();
    BT::NodeStatus checkObjectStaticRightLane();
    BT::NodeStatus checkObjectNearGoal();
    BT::NodeStatus checkIrregularLeft();
    BT::NodeStatus checkIrregularRight();
    BT::NodeStatus checkMeNearGoal();
    BT::NodeStatus checkIrregularFront();

    BT::NodeStatus init();
    BT::NodeStatus stop();
    BT::NodeStatus drive();
    BT::NodeStatus acc();
    BT::NodeStatus laneChangeRight();
    BT::NodeStatus laneChangeLeft();
    BT::NodeStatus stoplineStop();
    BT::NodeStatus setPath();
    BT::NodeStatus genPathLaneChangeLeft();
    BT::NodeStatus genPathLaneChangeRight();
    BT::NodeStatus genPathLaneKeeping();
    BT::NodeStatus setStateLaneKeeping();
    BT::NodeStatus setStateLaneChangeLeft();
    BT::NodeStatus setStateLaneChangeRight();
    BT::NodeStatus publishData();
    BT::NodeStatus stopTrafficLight();
    BT::NodeStatus setStateMissionGoing();
    BT::NodeStatus setStateMissionArrive();
    BT::NodeStatus publishStateMission();
    BT::NodeStatus callPathGlobal();
    BT::NodeStatus callPathGlobalLeft();
    BT::NodeStatus callPathGlobalRight();
    BT::NodeStatus genSpeedUnprotectedTurn();
    BT::NodeStatus speedProfileLaneChange();

public:
    BehaviorTreeControl(ros::NodeHandle nh, std::string file_name);
    ~BehaviorTreeControl();
    void tick();
};

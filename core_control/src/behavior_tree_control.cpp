#include "behavior_tree_control.h"

BehaviorTreeControl::BehaviorTreeControl(ros::NodeHandle nh, std::string file_name) : core_control(nh)
{
    factory.registerSimpleCondition("checkPassAbleTrafficLight", std::bind(&BehaviorTreeControl::checkPassAbleTrafficLight, this));
    factory.registerSimpleCondition("checkCollision", std::bind(&BehaviorTreeControl::checkCollision, this));
    factory.registerSimpleCondition("checkLaneChangeableLeft", std::bind(&BehaviorTreeControl::checkLaneChangeableLeft, this));
    factory.registerSimpleCondition("checkLaneChangeableRight", std::bind(&BehaviorTreeControl::checkLaneChangeableRight, this));
    factory.registerSimpleCondition("checkMissionWait", std::bind(&BehaviorTreeControl::checkMissionWait, this));
    factory.registerSimpleCondition("checkNearCorner", std::bind(&BehaviorTreeControl::checkNearCorner, this));
    factory.registerSimpleCondition("checkFrontObject", std::bind(&BehaviorTreeControl::checkFrontObject, this));
    factory.registerSimpleCondition("checkNearStopline", std::bind(&BehaviorTreeControl::checkNearStopLine, this));
    factory.registerSimpleCondition("checkGlobalLaneChange", std::bind(&BehaviorTreeControl::checkGlobalLaneChange, this));
    factory.registerSimpleCondition("checkLeftChange", std::bind(&BehaviorTreeControl::checkLeftChange, this));
    factory.registerSimpleCondition("checkRightChange", std::bind(&BehaviorTreeControl::checkRightChange, this));
    factory.registerSimpleCondition("checkExistPathGlobal", std::bind(&BehaviorTreeControl::checkExistPathGlobal, this));
    factory.registerSimpleCondition("checkStateLaneKeeping", std::bind(&BehaviorTreeControl::checkStateLaneKeeping, this));
    factory.registerSimpleCondition("checkStateLaneChangeLeft", std::bind(&BehaviorTreeControl::checkStateLaneChangeLeft, this));
    factory.registerSimpleCondition("checkStateLaneChangeRight", std::bind(&BehaviorTreeControl::checkStateLaneChangeRight, this));
    factory.registerSimpleCondition("checkEndLaneChange", std::bind(&BehaviorTreeControl::checkEndLaneChange, this));
    factory.registerSimpleCondition("checkStop", std::bind(&BehaviorTreeControl::checkStop, this));
    factory.registerSimpleCondition("checkArriveGoal", std::bind(&BehaviorTreeControl::checkArriveGoal, this));
    factory.registerSimpleCondition("checkCollisionBehindLeft", std::bind(&BehaviorTreeControl::checkCollisionBehindLeft, this));
    factory.registerSimpleCondition("checkCollisionBehindRight", std::bind(&BehaviorTreeControl::checkCollisionBehindRight, this));
    factory.registerSimpleCondition("checkProfitLaneChangeLeft", std::bind(&BehaviorTreeControl::checkProfitLaneChangeLeft, this));
    factory.registerSimpleCondition("checkProfitLaneChangeRight", std::bind(&BehaviorTreeControl::checkProfitLaneChangeRight, this));
    factory.registerSimpleCondition("checkFailLaneChange", std::bind(&BehaviorTreeControl::checkFailLaneChange, this));
    factory.registerSimpleCondition("checkUnprotected", std::bind(&BehaviorTreeControl::checkUnprotected, this));
    factory.registerSimpleCondition("checkImpassalbeUnprotected", std::bind(&BehaviorTreeControl::checkImpassalbeUnprotected, this));
    factory.registerSimpleCondition("checkMustLaneChange", std::bind(&BehaviorTreeControl::checkMustLaneChange, this));
    factory.registerSimpleCondition("checkObjectStaticLeftLane", std::bind(&BehaviorTreeControl::checkObjectStaticLeftLane, this));
    factory.registerSimpleCondition("checkObjectStaticRightLane", std::bind(&BehaviorTreeControl::checkObjectStaticRightLane, this));
    factory.registerSimpleCondition("checkObjectNearGoal", std::bind(&BehaviorTreeControl::checkObjectNearGoal, this));
    factory.registerSimpleCondition("checkIrregularLeft", std::bind(&BehaviorTreeControl::checkIrregularLeft, this));
    factory.registerSimpleCondition("checkIrregularRight", std::bind(&BehaviorTreeControl::checkIrregularRight, this));
    factory.registerSimpleCondition("checkMeNearGoal", std::bind(&BehaviorTreeControl::checkMeNearGoal, this));
    factory.registerSimpleCondition("checkIrregularFront", std::bind(&BehaviorTreeControl::checkIrregularFront, this));

    factory.registerSimpleAction("init", std::bind(&BehaviorTreeControl::init, this));
    factory.registerSimpleAction("acc", std::bind(&BehaviorTreeControl::acc, this));
    factory.registerSimpleAction("drive", std::bind(&BehaviorTreeControl::drive, this));
    factory.registerSimpleAction("laneChangeLeft", std::bind(&BehaviorTreeControl::laneChangeLeft, this));
    factory.registerSimpleAction("laneChangeRight", std::bind(&BehaviorTreeControl::laneChangeRight, this));
    factory.registerSimpleAction("stop", std::bind(&BehaviorTreeControl::stop, this));
    factory.registerSimpleAction("stoplineStop", std::bind(&BehaviorTreeControl::stoplineStop, this));
    factory.registerSimpleAction("setPath", std::bind(&BehaviorTreeControl::setPath, this));
    factory.registerSimpleAction("genPathLaneChangeLeft", std::bind(&BehaviorTreeControl::genPathLaneChangeLeft, this));
    factory.registerSimpleAction("genPathLaneChangeRight", std::bind(&BehaviorTreeControl::genPathLaneChangeRight, this));
    factory.registerSimpleAction("genPathLaneKeeping", std::bind(&BehaviorTreeControl::genPathLaneKeeping, this));
    factory.registerSimpleAction("setStateLaneKeeping", std::bind(&BehaviorTreeControl::setStateLaneKeeping, this));
    factory.registerSimpleAction("setStateLaneChangeLeft", std::bind(&BehaviorTreeControl::setStateLaneChangeLeft, this));
    factory.registerSimpleAction("setStateLaneChangeRight", std::bind(&BehaviorTreeControl::setStateLaneChangeRight, this));
    factory.registerSimpleAction("publishData", std::bind(&BehaviorTreeControl::publishData, this));
    factory.registerSimpleAction("stopTrafficLight", std::bind(&BehaviorTreeControl::stopTrafficLight, this));
    factory.registerSimpleAction("setStateMissionGoing", std::bind(&BehaviorTreeControl::setStateMissionGoing, this));
    factory.registerSimpleAction("setStateMissionArrive", std::bind(&BehaviorTreeControl::setStateMissionArrive, this));
    factory.registerSimpleAction("publishStateMission", std::bind(&BehaviorTreeControl::publishStateMission, this));
    factory.registerSimpleAction("callPathGlobal", std::bind(&BehaviorTreeControl::callPathGlobal, this));
    factory.registerSimpleAction("callPathGlobalLeft", std::bind(&BehaviorTreeControl::callPathGlobalLeft, this));
    factory.registerSimpleAction("callPathGlobalRight", std::bind(&BehaviorTreeControl::callPathGlobalRight, this));
    factory.registerSimpleAction("genSpeedUnprotectedTurn", std::bind(&BehaviorTreeControl::genSpeedUnprotectedTurn, this));
    factory.registerSimpleAction("speedProfileLaneChange", std::bind(&BehaviorTreeControl::speedProfileLaneChange, this));

    tree = factory.createTreeFromFile(file_name);
    printf_state = false;
    // static BT::StdCoutLogger logger_cout(tree);
#ifdef ZMQ_FOUND
    // Groot Visualize
    // static BT::PublisherZMQ publisher_zmq(tree);
#endif
}

BehaviorTreeControl::~BehaviorTreeControl()
{
}

void BehaviorTreeControl::tick()
{
    if (printf_state)
        fprintf(stderr, "[ tick_start ] \n\n\n");
    tree.tickRoot();
}

BT::NodeStatus BehaviorTreeControl::checkMissionWait()
{
    //call_mission_status
    ros::Time start = ros::Time::now();
    if (core_control.btCheckMissionWait())
    {
        if (printf_state)
            fprintf(stderr, "[ Mission: Start ] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[ Mission: Start ] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}
BT::NodeStatus BehaviorTreeControl::checkFrontObject()
{
    ros::Time start = ros::Time::now();
    if (core_control.btCheckFrontObject())
    {
        if (printf_state)
            fprintf(stderr, "[ checkFrontObject ] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[ checkFrontObject ] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}
BT::NodeStatus BehaviorTreeControl::checkNearStopLine()
{
    ros::Time start = ros::Time::now();
    if (core_control.btCheckNearStopline())
    {
        if (printf_state)
            fprintf(stderr, "[ checkNearStopLine ] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[ checkNearStopLine ] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}
BT::NodeStatus BehaviorTreeControl::checkNearCorner()
{
    ros::Time start = ros::Time::now();
    if (core_control.btCheckNearCorner())
    {
        if (printf_state)
            fprintf(stderr, "[ checkNearCorner ] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[ checkNearCorner ] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}
BT::NodeStatus BehaviorTreeControl::checkLaneChangeableLeft()
{
    ros::Time start = ros::Time::now();
    if (core_control.btCheckLaneChangeableLeft())
    {
        if (printf_state)
            fprintf(stderr, "[ checkLaneChangeableLeft ] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[ checkLaneChangeableLeft ] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}
BT::NodeStatus BehaviorTreeControl::checkCollision()
{
    ros::Time start = ros::Time::now();
    if (core_control.btCheckCollision())
    {
        if (printf_state)
            fprintf(stderr, "[ checkCollision ] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[ checkCollision ] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}
BT::NodeStatus BehaviorTreeControl::checkPassAbleTrafficLight()
{
    ros::Time start = ros::Time::now();
    if (core_control.btCheckPassAbleTrafficLight())
    {
        if (printf_state)
            fprintf(stderr, "[ checkPassAbleTrafficLight ] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[ checkPassAbleTrafficLight ] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}
BT::NodeStatus BehaviorTreeControl::checkGlobalLaneChange()
{
    ros::Time start = ros::Time::now();
    if (core_control.btCheckGlobalLaneChange())
    {
        if (printf_state)
            fprintf(stderr, "[ checkGlobalLaneChange ] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[ checkGlobalLaneChange ] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}
BT::NodeStatus BehaviorTreeControl::checkLeftChange()
{
    ros::Time start = ros::Time::now();
    if (core_control.btCheckLeftChange())
    {
        if (printf_state)
            fprintf(stderr, "[ checkLeftChange ] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[ checkLeftChange ] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}
BT::NodeStatus BehaviorTreeControl::checkRightChange()
{
    ros::Time start = ros::Time::now();
    if (core_control.btCheckRightChange())
    {
        if (printf_state)
            fprintf(stderr, "[ checkRightChange ] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[ checkRightChange ] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}
BT::NodeStatus BehaviorTreeControl::checkExistPathGlobal()
{
    ros::Time start = ros::Time::now();
    if (core_control.btCheckExistPathGlobal())
    {
        if (printf_state)
            fprintf(stderr, "[ CheckExistPathGlobal ] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[ CheckExistPathGlobal ] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}
BT::NodeStatus BehaviorTreeControl::checkStateLaneKeeping()
{
    ros::Time start = ros::Time::now();
    if (core_control.btCheckStateLaneKeeping())
    {
        if (printf_state)
            fprintf(stderr, "[ checkStateLaneKeeping ] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[ checkStateLaneKeeping ] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}
BT::NodeStatus BehaviorTreeControl::checkStateLaneChangeLeft()
{
    ros::Time start = ros::Time::now();
    if (core_control.btCheckStateLaneChangeLeft())
    {
        if (printf_state)
            fprintf(stderr, "[ checkStateLaneChangeLeft ] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[ checkStateLaneChangeLeft ] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}
BT::NodeStatus BehaviorTreeControl::checkStateLaneChangeRight()
{
    ros::Time start = ros::Time::now();
    if (core_control.btCheckStateLaneChangeRight())
    {
        if (printf_state)
            fprintf(stderr, "[ checkStateLaneChangeRight ] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[ checkStateLaneChangeRight ] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}
BT::NodeStatus BehaviorTreeControl::checkEndLaneChange()
{
    ros::Time start = ros::Time::now();
    if (core_control.btCheckEndLaneChange())
    {
        if (printf_state)
            fprintf(stderr, "[ checkEndLaneChange ] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[ checkEndLaneChange ] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}
BT::NodeStatus BehaviorTreeControl::checkLaneChangeableRight()
{
    ros::Time start = ros::Time::now();
    if (core_control.btCheckLaneChangeableRight())
    {
        if (printf_state)
            fprintf(stderr, "[ checkLaneChangeableRight ] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[ checkLaneChangeableRight ] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}
BT::NodeStatus BehaviorTreeControl::checkStop()
{
    ros::Time start = ros::Time::now();
    if (core_control.btCheckStop())
    {
        if (printf_state)
            fprintf(stderr, "[ checkStop ] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[ checkStop ] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}
BT::NodeStatus BehaviorTreeControl::checkArriveGoal()
{
    ros::Time start = ros::Time::now();
    if (core_control.btCheckArriveGoal())
    {
        if (printf_state)
            fprintf(stderr, "[ checkArriveGoal ] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[ checkArriveGoal ] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}
BT::NodeStatus BehaviorTreeControl::checkCollisionBehindLeft()
{
    ros::Time start = ros::Time::now();
    if (core_control.btCheckCollisionBehindLeft())
    {
        if (printf_state)
            fprintf(stderr, "[ checkCollisionBehindLeft ] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[ checkCollisionBehindLeft ] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}
BT::NodeStatus BehaviorTreeControl::checkCollisionBehindRight()
{
    ros::Time start = ros::Time::now();
    if (core_control.btCheckCollisionBehindRight())
    {
        if (printf_state)
            fprintf(stderr, "[ checkCollisionBehindRight ] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[ checkCollisionBehindRight ] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}
BT::NodeStatus BehaviorTreeControl::checkProfitLaneChangeLeft()
{
    ros::Time start = ros::Time::now();
    if (core_control.btCheckProfitLaneChangeLeft())
    {
        if (printf_state)
            fprintf(stderr, "[ checkProfitLaneChangeLeft ] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[ checkProfitLaneChangeLeft ] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}
BT::NodeStatus BehaviorTreeControl::checkProfitLaneChangeRight()
{
    ros::Time start = ros::Time::now();
    if (core_control.btCheckProfitLaneChangeRight())
    {
        if (printf_state)
            fprintf(stderr, "[ checkProfitLaneChangeRight ] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[ checkProfitLaneChangeRight ] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}
BT::NodeStatus BehaviorTreeControl::checkFailLaneChange()
{
    ros::Time start = ros::Time::now();
    if (core_control.btCheckFailLaneChange())
    {
        if (printf_state)
            fprintf(stderr, "[ checkFailLaneChange ] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[ checkFailLaneChange ] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}
BT::NodeStatus BehaviorTreeControl::checkUnprotected()
{
    ros::Time start = ros::Time::now();
    if (core_control.btCheckUnprotectedTurn())
    {
        if (printf_state)
            fprintf(stderr, "[ checkUnprotected ] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[ checkUnprotected ] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}
BT::NodeStatus BehaviorTreeControl::checkImpassalbeUnprotected()
{
    ros::Time start = ros::Time::now();
    if (core_control.btCheckImpassalbeUnprotectedTurn())
    {
        if (printf_state)
            fprintf(stderr, "[ checkImpassalbeUnprotected ] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[ checkImpassalbeUnprotected ] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}
BT::NodeStatus BehaviorTreeControl::checkMustLaneChange()
{
    ros::Time start = ros::Time::now();
    if (core_control.btCheckMustLaneChange())
    {
        if (printf_state)
            fprintf(stderr, "[ checkMustLaneChange ] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[ checkMustLaneChange ] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}
BT::NodeStatus BehaviorTreeControl::checkObjectStaticLeftLane()
{
    ros::Time start = ros::Time::now();
    if (core_control.btCheckObjectStaticLeftLane())
    {
        if (printf_state)
            fprintf(stderr, "[ checkObjectStaticLeftLane ] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[ checkObjectStaticLeftLane ] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}
BT::NodeStatus BehaviorTreeControl::checkObjectStaticRightLane()
{
    ros::Time start = ros::Time::now();
    if (core_control.btCheckObjectStaticRightLane())
    {
        if (printf_state)
            fprintf(stderr, "[ checkObjectStaticRightLane ] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[ checkObjectStaticRightLane ] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}
BT::NodeStatus BehaviorTreeControl::checkObjectNearGoal()
{
    ros::Time start = ros::Time::now();
    if (core_control.btCheckObjectNearGoal())
    {
        if (printf_state)
            fprintf(stderr, "[ checkObjectNearGoal ] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[ checkObjectNearGoal ] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}
BT::NodeStatus BehaviorTreeControl::checkIrregularLeft()
{
    ros::Time start = ros::Time::now();
    if (core_control.btCheckIrregularLeft())
    {
        if (printf_state)
            fprintf(stderr, "[ checkIrregularLeft ] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[ checkIrregularLeft ] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}
BT::NodeStatus BehaviorTreeControl::checkIrregularRight()
{
    ros::Time start = ros::Time::now();
    if (core_control.btCheckIrregularRight())
    {
        if (printf_state)
            fprintf(stderr, "[ checkIrregularRight ] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[ checkIrregularRight ] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}
BT::NodeStatus BehaviorTreeControl::checkMeNearGoal()
{
    ros::Time start = ros::Time::now();
    if (core_control.btCheckMeNearGoal())
    {
        if (printf_state)
            fprintf(stderr, "[ checkMeNearGoal ] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[ checkMeNearGoal ] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}
BT::NodeStatus BehaviorTreeControl::checkIrregularFront()
{
    ros::Time start = ros::Time::now();
    if (core_control.btCheckIrregularFront())
    {
        if (printf_state)
            fprintf(stderr, "[ checkIrregularFront ] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[ checkIrregularFront ] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}

BT::NodeStatus BehaviorTreeControl::init()
{
    ros::Time start = ros::Time::now();
    if (core_control.btActionInit())
    {
        if (printf_state)
            fprintf(stderr, "[ init ] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[ init ] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}
BT::NodeStatus BehaviorTreeControl::stop()
{
    ros::Time start = ros::Time::now();
    if (core_control.btActionStop())
    {
        if (printf_state)
            fprintf(stderr, "[ Stop ] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[ Stop ] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}
BT::NodeStatus BehaviorTreeControl::drive()
{
    ros::Time start = ros::Time::now();
    if (core_control.btActionDrive())
    {
        if (printf_state)
            fprintf(stderr, "[ Drive ] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[ Drive ] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}
BT::NodeStatus BehaviorTreeControl::acc()
{
    ros::Time start = ros::Time::now();
    if (core_control.btActionAcc())
    {
        if (printf_state)
            fprintf(stderr, "[ Acc ] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[ Acc ] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}
BT::NodeStatus BehaviorTreeControl::laneChangeRight()
{
    ros::Time start = ros::Time::now();
    if (core_control.btActionLaneChangeRight())
    {
        if (printf_state)
            fprintf(stderr, "[ LaneChange_R ] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[ LaneChange_R ] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}
BT::NodeStatus BehaviorTreeControl::laneChangeLeft()
{
    ros::Time start = ros::Time::now();
    if (core_control.btActionLaneChangeLeft())
    {
        if (printf_state)
            fprintf(stderr, "[ LaneChange_L ] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[ LaneChange_L ] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}
BT::NodeStatus BehaviorTreeControl::stoplineStop()
{
    ros::Time start = ros::Time::now();
    if (core_control.btActionStoplineStop())
    {
        if (printf_state)
            fprintf(stderr, "[Stopline Stop!!] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[Stopline Stop!!] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}
BT::NodeStatus BehaviorTreeControl::setPath()
{
    ros::Time start = ros::Time::now();
    if (core_control.btActionSetPath())
    {
        if (printf_state)
            fprintf(stderr, "[ Set Path!!] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[ Set Path!!] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}
BT::NodeStatus BehaviorTreeControl::genPathLaneChangeLeft()
{
    ros::Time start = ros::Time::now();
    if (core_control.btActionGenPathLaneChangeLeft())
    {
        if (printf_state)
            fprintf(stderr, "[ genPathLaneChangeLeft ] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[ genPathLaneChangeLeft ] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}
BT::NodeStatus BehaviorTreeControl::genPathLaneChangeRight()
{
    ros::Time start = ros::Time::now();
    if (core_control.btActionGenPathLaneChangeRight())
    {
        if (printf_state)
            fprintf(stderr, "[ genPathLaneChangeRight ] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[ genPathLaneChangeRight ] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}
BT::NodeStatus BehaviorTreeControl::genPathLaneKeeping()
{
    ros::Time start = ros::Time::now();
    if (core_control.btActionGenPathLaneKeeping())
    {
        if (printf_state)
            fprintf(stderr, "[ genPathLaneKeeping ] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[ genPathLaneKeeping ] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}
BT::NodeStatus BehaviorTreeControl::setStateLaneKeeping()
{
    ros::Time start = ros::Time::now();
    if (core_control.btActionSetStateLaneKeeping())
    {
        if (printf_state)
            fprintf(stderr, "[ setStateLaneKeeping ] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[ setStateLaneKeeping ] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}
BT::NodeStatus BehaviorTreeControl::setStateLaneChangeLeft()
{
    ros::Time start = ros::Time::now();
    if (core_control.btActionSetStateLaneChangeLeft())
    {
        if (printf_state)
            fprintf(stderr, "[ setStateLaneChangeLeft ] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[ setStateLaneChangeLeft ] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}
BT::NodeStatus BehaviorTreeControl::setStateLaneChangeRight()
{
    ros::Time start = ros::Time::now();
    if (core_control.btActionSetStateLaneChangeRight())
    {
        if (printf_state)
            fprintf(stderr, "[ setStateLaneChangeRight ] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[ setStateLaneChangeRight ] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}
BT::NodeStatus BehaviorTreeControl::publishData()
{
    ros::Time start = ros::Time::now();
    if (core_control.btActionPublishData())
    {
        if (printf_state)
            fprintf(stderr, "[ publishData ] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[ publishData ] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}
BT::NodeStatus BehaviorTreeControl::stopTrafficLight()
{
    ros::Time start = ros::Time::now();
    if (core_control.btActionStopTrafficLight())
    {
        if (printf_state)
            fprintf(stderr, "[ stopTrafficLight ] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[ stopTrafficLight ] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}
BT::NodeStatus BehaviorTreeControl::setStateMissionGoing()
{
    ros::Time start = ros::Time::now();
    if (core_control.btSetStateMissionGoing())
    {
        if (printf_state)
            fprintf(stderr, "[ setStateMissionGoing ] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[ setStateMissionGoing ] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}
BT::NodeStatus BehaviorTreeControl::setStateMissionArrive()
{
    ros::Time start = ros::Time::now();
    if (core_control.btSetStateMissionArrive())
    {
        if (printf_state)
            fprintf(stderr, "[ setStateMissionArrive ] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[ setStateMissionArrive ] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}
BT::NodeStatus BehaviorTreeControl::publishStateMission()
{
    ros::Time start = ros::Time::now();
    if (core_control.btPublishStateMission())
    {
        if (printf_state)
            fprintf(stderr, "[ publishStateMission ] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[ publishStateMission ] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}
BT::NodeStatus BehaviorTreeControl::callPathGlobal()
{
    ros::Time start = ros::Time::now();
    if (core_control.btCallPathGlobal())
    {
        if (printf_state)
            fprintf(stderr, "[ callPathGlobal ] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[ callPathGlobal ] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}
BT::NodeStatus BehaviorTreeControl::callPathGlobalLeft()
{
    ros::Time start = ros::Time::now();
    if (core_control.btCallPathGlobalLeft())
    {
        if (printf_state)
            fprintf(stderr, "[ callPathGlobalLeft ] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[ callPathGlobalLeft ] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}
BT::NodeStatus BehaviorTreeControl::callPathGlobalRight()
{
    ros::Time start = ros::Time::now();
    if (core_control.btCallPathGlobalRight())
    {
        if (printf_state)
            fprintf(stderr, "[ callPathGlobalRight ] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[ callPathGlobalRight ] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}
BT::NodeStatus BehaviorTreeControl::genSpeedUnprotectedTurn()
{
    ros::Time start = ros::Time::now();
    if (core_control.btGenSpeedUnprotectedTurn())
    {
        if (printf_state)
            fprintf(stderr, "[ genSpeedUnprotectedTurn ] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[ genSpeedUnprotectedTurn ] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}
BT::NodeStatus BehaviorTreeControl::speedProfileLaneChange()
{
    ros::Time start = ros::Time::now();
    if (core_control.btSpeedProfileLaneChange())
    {
        if (printf_state)
            fprintf(stderr, "[ speedProfileLaneChange ] true, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (printf_state)
            fprintf(stderr, "[ speedProfileLaneChange ] false, %lf\n", (ros::Time::now() - start).toSec());
        return BT::NodeStatus::FAILURE;
    }
}

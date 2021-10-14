#include "core_control.h"
#include "behavior_tree_control.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "control");
    ros::NodeHandle nh;
    ros::Rate ros_rate(kControlRate);

    std::string tree_file_name = "/home/a/k_city_ws/src/core_control/lc_bt.xml";
    BehaviorTreeControl bt_control(nh, tree_file_name);
    ros::Time pre_loop = ros::Time::now();
    ros::Time time_loop = ros::Time::now();
    while (ros::ok())
    {
        time_loop = ros::Time::now();
        fprintf(stderr, "pre_loop time: %lf\n", (ros::Time::now() - pre_loop).toSec());
        static ros::Time pre_time = ros::Time::now();
        fprintf(stderr, "run_time: %lf\n", (ros::Time::now() - pre_time).toSec());
        if ((ros::Time::now() - pre_time).toSec() > 0.5)
            fprintf(stderr, "to slow!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n\n\n");
        pre_time = ros::Time::now();
        ros::Time start = ros::Time::now();
        ros::spinOnce();
        fprintf(stderr, "spin time: %lf\n", (ros::Time::now() - start).toSec());
        start = ros::Time::now();
        bt_control.tick();
        fprintf(stderr, "tick time: %lf\n", (ros::Time::now() - start).toSec());
        start = ros::Time::now();
        if ((ros::Time::now() - pre_time).toSec() < 0.05)
            ros_rate.sleep();
        fprintf(stderr, "time loop: %lf\n", (ros::Time::now() - time_loop).toSec());
        pre_loop = ros::Time::now();
    }
    return 0;
}

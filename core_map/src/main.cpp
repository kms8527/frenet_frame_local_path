#include "Coremap.h"

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "core_map_node");
    ros::NodeHandle nh;
    Coremap coremap(nh);
    ros::spin();

    return 0;
}

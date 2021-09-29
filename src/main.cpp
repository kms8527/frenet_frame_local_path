#include "src/FrenetOptimalTrajectory/FrenetOptimalTrajectory.h"
#include "src/FrenetOptimalTrajectory/FrenetPath.h"
#include "src/FrenetOptimalTrajectory/py_cpp_struct.h"
#include <iostream>

using namespace std;

int main(int arg, char ** argv) {

    ros::init(arg, argv, "frenet_node"); //FrenetOptimalTrajectory
    ros::NodeHandle nh;
    FrenetOptimalTrajectory FrenetOptimalTrajectory(nh);
    ros::spin();

    return 0;

    // waypoint x pos
//    double wx [25]= //{0,50,150};
//    {132.67, 128.67, 124.67, 120.67, 116.67, 112.67, 108.67,
//                   104.67, 101.43,  97.77,  94.84,  92.89,  92.4 ,  92.4 ,
//                   92.4 ,  92.4 ,  92.4 ,  92.4 ,  92.4 ,  92.39,  92.39,
//                   92.39,  92.39,  92.39,  92.39};
//    //waypoint y pos
//    double wy [25]=//{0,0,0};
//    {195.14, 195.14, 195.14, 195.14, 195.14, 195.14, 195.14,
//                   195.14, 195.14, 195.03, 193.88, 191.75, 188.72, 185.32,
//                   181.32, 177.32, 173.32, 169.32, 165.32, 161.32, 157.32,
//                   153.32, 149.32, 145.32, 141.84};
//    // obstacle point
//    double o_llx[1] = {92.89};//{12};//
//    double o_lly[1] = {191.75};//{-2};//
//    double o_urx[1] = {92.89};//{14};//
//    double o_ury[1] = {191.75};//{4};//

//    // set up experiment
//    FrenetInitialConditions fot_ic = { *o_llx;
//    double *o_lly;
//    double *o_urx;
//    double *o_ury;
//        0 ,  //s0
//        7.10964962, // c_speed
//        -1.35277168, //c_d
//        -1.86, //c_d_d
//        0.0, // c_d_dd
//        10,   // target_speed
//        wx,
//        wy,
//        25,  // nw: int - number of waypoints
//        o_llx, //lower left x
//        o_lly,
//        o_urx, //upper right x
//        o_ury,
//        1    // no: int -number of obstacles
//    };
//    FrenetHyperparameters fot_hp = {
//        25.0, //max_speed
//        15.0, //max_accel
//        15.0, //max_curvature
//        5.0, //max_road_width_l
//        5.0, //max_road_width_r
//        0.5, //d_road_w
//        0.2, //dt
//        5.0, //maxt
//        2.0, //mint
//        0.5, //d_t_s
//        2.0, //n_s_sample
//        0.1, //obstacle_clearance
//        1.0, //kd
//        0.1, //kv
//        0.1, //ka
//        0.1, //kj
//        0.1, //kt
//        0.1, //ko
//        1.0, //klat
//        1.0, //klong
//        0    //number of thread
//    };

//    // run experiment
//    FrenetOptimalTrajectory fot = FrenetOptimalTrajectory(&fot_ic, &fot_hp);
//    FrenetPath* best_frenet_path = fot.getBestPath();
//    if (best_frenet_path) {
//        cout << "Success\n";
//        return 1;
//    }
//    cout << "Failure\n";
//    return 0;
}

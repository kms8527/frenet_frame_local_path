#include "src/FrenetOptimalTrajectory/FrenetOptimalTrajectory.h"


using namespace std;

FrenetHyperparameters frenet_path_hp = {
    20.0, //max_speed
    5.0, //max_accel
    50.0, //max_curvature
    0.0, //max_road_width_l (float): maximum road width to the left [m]
    5.0, //max_road_width_r (float): maximum road width to the right [m]
    0.5, //d_road_w (float): road width sampling discretization [m]
    0.1, //dt(float): road width sampling discretization [m]
    12.0, //maxt (float): max prediction horizon [s]
    9.0, //mint (float): min prediction horizon [s]
    1, //d_t_s (float): target speed sampling discretization [m/s]
    1.0, //n_s_sample (float): sampling number of target speed
    0.0, //obstacle_clearance (float): obstacle radius [m]
    1.0, //kd (float): positional deviation cost
    0.1, //kv (float): velocity cost
    0.1, //ka (float): acceleration cost
    0.1, //kj (float): acceleration cost
    0.1, //kt (float): time cost
    1.0, //ko (float): dist to obstacle cost
    0.1, //klat (float): lateral cost
    1.0, //klong (float): lateral cost
    0    //number of thread
};

// Compute the frenet optimal trajectory

FrenetOptimalTrajectory::FrenetOptimalTrajectory(ros::NodeHandle &nh){

    condition_sub = nh.subscribe("/core/control/frenet_path_input", 1, &FrenetOptimalTrajectory::FrenetInitialConditionsCallback, this);
    frenet_path_pub = nh.advertise<sensor_msgs::PointCloud2>("/core/frenet/_path", 1);
    result_pub = nh.advertise<frenet_local_path::frenet_output>("/core/frenet/result",1);
    pub_path_global = nh.advertise<sensor_msgs::PointCloud2>("/frenet/input_global_path",1);
//    nh.subscribe("", 1, &FrenetOptimalTrajectory::FrenetHyperparametersCallback, this);

    fot_hp = &frenet_path_hp;

}

bool FrenetOptimalTrajectory::isSamePath(vector<frenet_local_path::waypoint> &global_path,const vector<frenet_local_path::waypoint> &msg_path){
    int end_idx = global_path.size()-1;
    int end_idx2 = msg_path.size()-1;
    if (hypot(global_path[0].x-msg_path[0].x,global_path[0].y-msg_path[0].y) == 0 &&
            hypot(global_path[end_idx].x-msg_path[end_idx].x,global_path[end_idx2].y-msg_path[end_idx2].y) == 0)
        return true;
    else
        return false;
}

void FrenetOptimalTrajectory::FrenetInitialConditionsCallback(const frenet_local_path::frenet_input &msg){

    //    double* wx = new double[msg.waypointAry.size()];
    //    double* wy = new double[msg.waypointAry.size()];
    //    for (int i = 0; i< fot_ic->nw; i++){
    //        fot_ic->wx[i] = msg.waypointAry[i].x;
    //        fot_ic->wy[i] = msg.waypointAry[i].y;
    //    }


    if (msg.waypointAry.size() == 0 && global_path.empty()) {
        fprintf(stderr,"plz pub path \n");
        return;
    }
        // construct spline path
        auto start = chrono::high_resolution_clock::now();
        resetParam();
//        vector<frenet_local_path::waypoint> waypoint_list;
//        for (int i =0; i<msg.waypointAry.size(); i++){
//            waypoint_list.push_back(msg.waypointAry[i]);
//        }

//        obstacles_points = msg.obs_pos;

        for(size_t i =0; i< msg.obs_pos.size();i++){
//            vector<geometry_msgs::Point> obstacle;
//            for(size_t j =0; j< msg.obs_pos[i].Points.size(); j++){
//                geometry_msgs::Point tmp;
//                tmp.x = msg.obs_pos[i].Points[j].x;
//                tmp.x = msg.obs_pos[i].Points[j].y;
//                tmp.x = msg.obs_pos[i].Points[j].z;
//                obstacle.push_back(tmp);
//            }

//            obstacles_points.push_back(obstacle);

            obstacles_points.push_back(msg.obs_pos[i].Points);
        }
        frenet_state = msg.frenet_state;

        if (!global_path.empty()&& msg.frenet_state == 0){
            delete csp;
        }
        if(global_path.empty() || msg.frenet_state == 0){
//            global_path = msg.waypointAry;
            vector<frenet_local_path::waypoint> waypoint_list;
            for (size_t i =0; i < msg.waypointAry.size(); i+=2){
                waypoint_list.push_back(msg.waypointAry[i]);
            }
//            if (global_path.size() == 38)
//                fprintf(stderr,"stop \n");
            global_path = waypoint_list;
            raw_global_path = msg.waypointAry;
            pubglobalpath();
            fprintf(stderr, "path changed \n");
            if (global_path.size() < 2) {
                fprintf(stderr,"not enough waypoints \n");
                return;
            }
            csp = new CubicSpline2D(global_path);

        }
        else
            fprintf(stderr, "path keep \n");


//        double s0 = fot_ic->s0;
        double s = csp->find_s(msg.cur_pos.position.x,msg.cur_pos.position.y,0);
//        fot_ic->s0 = s;
//        double s = fot_ic->s0;
        double x = msg.cur_pos.position.x;
        double y = msg.cur_pos.position.y;
        double distance = norm(csp->calc_x(s) - x, csp->calc_y(s) - y);
        tuple<double, double> bvec ((csp->calc_x(s) - x) / distance,
                        (csp->calc_y(s) - y) / distance);

       // normal spline vector
        double x0 = csp->calc_x(s);
        double y0 = csp->calc_y(s);
        double x1 = csp->calc_x(s + 2);
        double y1 = csp->calc_y(s + 2);

        // unit vector orthog. to spline
        tuple<double, double> tvec (y1-y0, -(x1-x0));
        as_unit_vector(tvec);

       // compute tangent / normal car vectors
        double vx = msg.cur_speed * cos(tf2::getYaw(msg.cur_pos.orientation));
        double vy = msg.cur_speed * sin(tf2::getYaw(msg.cur_pos.orientation));
        tuple<double, double> fvec (vx, vy);
        as_unit_vector(fvec);
//        fprintf(stderr,"vx : %lf , vy : %lf", vx, vy);
        double c_d = copysign(distance, dot(tvec, bvec)); // lateral position c_d [m]
        double c_dd = -msg.cur_speed * dot(tvec, fvec);   // lateral speed c_d_d [m/s]
        const int no = msg.obs_pos.size()/2;              // lateral acceleration c_d_dd [m/s^2]
        FrenetInitialConditions tmp={
            s,
            msg.cur_speed,
            c_d,//-1.35277168
            c_dd, //-1.86
            0.0,
            5.0, // target_speedllx
            no
        };
        fot_ic = &tmp;


        for (int i = 0; i < fot_ic->no; i++){
            urx.push_back(msg.obs_pos[i].Points[1].x);
            ury.push_back(msg.obs_pos[i].Points[1].y);
            llx.push_back(msg.obs_pos[i+1].Points[3].x);
            lly.push_back(msg.obs_pos[i+1].Points[3].y);
        }

        // parse the waypoints and obstacles

    //    fot_ic = fot_ic_;
    //    fot_hp = fot_hp_;
        mu = new mutex();

    //    x.assign(fot_ic->wx, fot_ic->wx + fot_ic->nw);
    //    y.assign(fot_ic->wy, fot_ic->wy + fot_ic->nw);
        setObstacles();

        // make sure best_frenet_path is initialized
        best_frenet_path = nullptr;

        // exit if not enough waypoints


        // calculate the trajectories
        if (fot_hp->num_threads == 0) {
            // calculate how to split computation across threads

            int total_di_iter = static_cast<int>((fot_hp->max_road_width_l +
                                                  fot_hp->max_road_width_r) /
                                                 fot_hp->d_road_w) + 1; // account for the last index

            calc_frenet_paths(0, total_di_iter, false);

        } else { // if threading
            threaded_calc_all_frenet_paths();
        }

        // select the best path
        double mincost = INFINITY;
        int bfp_idx = 0;
//        for (FrenetPath *fp : frenet_paths) {
        for(int i =0; i < frenet_paths.size(); i++){
            if (frenet_paths[i]->cf <= mincost) { // search min cost
                mincost = frenet_paths[i]->cf;
                best_frenet_path = frenet_paths[i];
                bfp_idx = i;
                if (frenet_paths[i]->s[0]<s)
                    fprintf(stderr,"%.1lf , i : %d \n",frenet_paths[i]->s[0], i);

            }
        }
        auto end = chrono::high_resolution_clock::now();
        double run_time =chrono::duration_cast<chrono::nanoseconds>(end - start).count();
        run_time *= 1e-9;
        // cout << "Planning runtime " << run_time << "\n";

//        visualizePath(frenet_paths);
        if (best_frenet_path) {
             double min = INFINITY;
             size_t idx = 0;
             for (size_t i =0; i< raw_global_path.size()-1; i++){
                 double tmp = hypot(best_frenet_path->x.back()-raw_global_path[i].x,best_frenet_path->y.back()-raw_global_path[i].y);
                if (tmp < min)
                    min = tmp;
                    idx = i;
             }
             if (frenet_state == 1 && min < 0.1){
                 frenet_state = 0;
                 best_frenet_path->x.push_back(msg.waypointAry[idx+1].x);
                 best_frenet_path->y.push_back(msg.waypointAry[idx+1].y);
             }


            fprintf(stderr, "Sucess, runtime : %lf sec, s : %lf, frenet_state: %d \n", run_time, s, frenet_state);

            pubFrenetPath();
//             visualizePath();
//             visualizePath(frenet_paths);

//                if (fp->s[0]<s)
//                    fprintf(stderr,"%.1lf , bfp_idx : %d \n",fp->s[0], bfp_idx);

//             fprintf(stderr,"\n");
//            for (int i =0; best_frenet_path->x.size();i++){
//                fprintf(stderr, "x : %lf",best_frenet_path->x[i]);
//            }
//            fprintf(stderr,"\n");
//            for (int i =0; best_frenet_path->x.size();i++){
//                fprintf(stderr, "y : %lf",best_frenet_path->y[i]);
//            }
        }
        else{
            fprintf(stderr, "False \n");
            flag = true;
        }
        delete mu;

        for (FrenetPath *fp : frenet_paths) {
            delete fp;
        }

        for (Obstacle *ob : obstacles) {
            delete ob;
        }

}

void FrenetOptimalTrajectory::pubFrenetPath(){
    frenet_local_path::frenet_output result;
    pcl::PointCloud<pcl::PointXYZI> tmp;
    pcl::PointXYZI p;
    for (size_t i = 0; i< best_frenet_path->x.size(); i++){
        p.x = best_frenet_path->x[i];
        p.y = best_frenet_path->y[i];
        p.z = 0;
        p.intensity = 150;
        tmp.push_back(p);
    }

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(tmp, output);
    output.header.frame_id = "world";


    result.frenet_path = output;
    result.frenet_state = frenet_state;

    frenet_path_pub.publish(output);
    result_pub.publish(result);
}

void FrenetOptimalTrajectory::resetParam(){
    frenet_paths.clear();
    llx.clear();
    lly.clear();
    urx.clear();
    ury.clear();
    obstacles.clear();
    obstacles_points.clear();
//    global_path.clear();
}

void FrenetOptimalTrajectory::visualizePath(){
    pcl::PointCloud<pcl::PointXYZI> tmp;
    pcl::PointXYZI p;
    for (size_t i = 0; i< best_frenet_path->x.size(); i++){
        p.x = best_frenet_path->x[i];
        p.y = best_frenet_path->y[i];
        p.z = 0;
        p.intensity = 150;
        tmp.push_back(p);
    }

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(tmp, output);
    output.header.frame_id = "world";
    frenet_path_pub.publish(output);
}
void FrenetOptimalTrajectory::visualizePath(const vector<FrenetPath *> frenet_paths){
    pcl::PointCloud<pcl::PointXYZI> tmp;
    pcl::PointXYZI p;
    for (size_t i = 0; i< frenet_paths.size(); i++){
        for (size_t j = 0; j< frenet_paths[i]->x.size(); j++){
            p.x = frenet_paths[i]->x[j];
            p.y = frenet_paths[i]->y[j];
            p.z = 0;
            p.intensity = 255;
            tmp.push_back(p);
        }
    }

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(tmp, output);
    output.header.frame_id = "world";
    frenet_path_pub.publish(output);
}

void FrenetOptimalTrajectory::visualizePath(const FrenetPath * frenet_path){
    pcl::PointCloud<pcl::PointXYZI> tmp;
    pcl::PointXYZI p;
    for (size_t i = 0; i< frenet_path->x.size(); i++){

        p.x = frenet_path->x[i];
        p.y = frenet_path->y[i];
        p.z = 0;
        p.intensity = 255;
        tmp.push_back(p);

    }

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(tmp, output);
    output.header.frame_id = "world";
    frenet_path_pub.publish(output);
}

FrenetOptimalTrajectory::~FrenetOptimalTrajectory() {
}

// Return the best path
FrenetPath *FrenetOptimalTrajectory::getBestPath() { return best_frenet_path; }

/*
 * Spawn threads to calculate frenet paths
 * Called when multithreading.
 */
void FrenetOptimalTrajectory::threaded_calc_all_frenet_paths() {
    vector<thread> threads;

    // calculate how to split computation across threads
    int num_di_iter =
        static_cast<int>((fot_hp->max_road_width_l + fot_hp->max_road_width_r) /
                         fot_hp->d_road_w);
    num_di_iter = num_di_iter + 1; // account for the last index

    int iter_di_index_range =
        static_cast<int>(num_di_iter / fot_hp->num_threads);

    for (int i = 0; i < fot_hp->num_threads; i++) {
        if (i != fot_hp->num_threads - 1) {
            threads.push_back(thread(&FrenetOptimalTrajectory::calc_frenet_paths, this, i * iter_di_index_range, (i + 1) * iter_di_index_range, true));
        } else { // account for last thread edge case
            threads.push_back(
                thread(&FrenetOptimalTrajectory::calc_frenet_paths, this,
                       i * iter_di_index_range, num_di_iter, true));
        }
    }

    // wait for all threads to finish computation
    for (auto &t : threads) {
        t.join();
    }
}

/*
 * Calculate frenet paths
 * If running we are multithreading,
 * We parallelize on the outer loop, in terms of di
 * Iterates over possible values of di, from start index to end index
 * (exclusive). Then, computes the actual di value for path planning.
 * Mutex is only enabled when we are multithreading.
 */
void FrenetOptimalTrajectory::calc_frenet_paths(int start_di_index,
                                                int end_di_index,
                                                bool multithreaded) {
    double t, ti, tv;
    double lateral_deviation, lateral_velocity, lateral_acceleration,
        lateral_jerk;
    double longitudinal_acceleration, longitudinal_jerk;
    FrenetPath *fp, *tfp;
    int num_paths = 0;
    int num_viable_paths = 0;
    // double valid_path_time = 0;

    // initialize di, with start_di_index
    double di = -fot_hp->max_road_width_l + start_di_index * fot_hp->d_road_w;

    // generate path to each offset goal
    // note di goes up to but not including end_di_index*fot_hp->d_road_w
    while ((di < -fot_hp->max_road_width_l + end_di_index * fot_hp->d_road_w) &&
           (di <= fot_hp->max_road_width_r)) {

        ti = fot_hp->mint;
        // lateral motion planning
        while (ti <= fot_hp->maxt) {
            lateral_deviation = 0;
            lateral_velocity = 0;
            lateral_acceleration = 0;
            lateral_jerk = 0;

            fp = new FrenetPath(fot_hp);
            QuinticPolynomial lat_qp = QuinticPolynomial(
                fot_ic->c_d, fot_ic->c_d_d, fot_ic->c_d_dd, di, 0.0, 0.0, ti);

            QuinticPolynomial tmp_qp = QuinticPolynomial(
                fot_ic->c_d, fot_ic->c_d_d, fot_ic->c_d_dd, fot_hp->max_road_width_r, 0.0, 0.0, fot_hp->maxt);
            double tmp = tmp_qp.calc_point(fot_hp->maxt);

//            if (tmp <  -1 || tmp > 10)
//                break;

            // construct frenet path
            t = 0;
            while (t <= ti) {
                fp->t.push_back(t);
                fp->d.push_back(lat_qp.calc_point(t));
                fp->d_d.push_back(lat_qp.calc_first_derivative(t));
                fp->d_dd.push_back(lat_qp.calc_second_derivative(t));
                fp->d_ddd.push_back(lat_qp.calc_third_derivative(t));
                lateral_deviation += abs(lat_qp.calc_point(t));
                lateral_velocity += abs(lat_qp.calc_first_derivative(t));
                lateral_acceleration += abs(lat_qp.calc_second_derivative(t));
                lateral_jerk += abs(lat_qp.calc_third_derivative(t));
                t += fot_hp->dt;
            }

            // velocity keeping
            tv = fot_ic->target_speed - fot_hp->d_t_s * fot_hp->n_s_sample;
            while (tv <=
                   fot_ic->target_speed + fot_hp->d_t_s * fot_hp->n_s_sample) {
                longitudinal_acceleration = 0;
                longitudinal_jerk = 0;

                // copy frenet path
                tfp = new FrenetPath(fot_hp);
                tfp->t.assign(fp->t.begin(), fp->t.end());
                tfp->d.assign(fp->d.begin(), fp->d.end());
                tfp->d_d.assign(fp->d_d.begin(), fp->d_d.end());
                tfp->d_dd.assign(fp->d_dd.begin(), fp->d_dd.end());
                tfp->d_ddd.assign(fp->d_ddd.begin(), fp->d_ddd.end());

                QuarticPolynomial lon_qp = QuarticPolynomial(
                    fot_ic->s0, fot_ic->c_speed, 0.0, tv, 0.0, ti);

                // longitudinal motion
                for (double tp : tfp->t) {
                    tfp->s.push_back(lon_qp.calc_point(tp));
                    tfp->s_d.push_back(lon_qp.calc_first_derivative(tp));
                    tfp->s_dd.push_back(lon_qp.calc_second_derivative(tp));
                    tfp->s_ddd.push_back(lon_qp.calc_third_derivative(tp));
                    longitudinal_acceleration +=
                        abs(lon_qp.calc_second_derivative(tp));
                    longitudinal_jerk += abs(lon_qp.calc_third_derivative(tp));
                }
                tfp->tmp = tmp; // for debug // last point later deviation
                num_paths++;
                // delete if failure or invalid path
                bool success = tfp->to_global_path(csp);
                num_viable_paths++;
                if (!success) {
                    // deallocate memory and continue
                    delete tfp;
                    tv += fot_hp->d_t_s;
                    continue;
                }

                // auto start = chrono::high_resolution_clock::now();
//                visualizePath(tfp);
//                if (flag)
//                    visualizePath(tfp);
                bool valid_path = tfp->is_valid_path(obstacles);
                // auto end = chrono::high_resolution_clock::now();
                // valid_path_time +=
                // chrono::duration_cast<chrono::nanoseconds>(end -
                // start).count();
                if (!valid_path) {
                    // deallocate memory and continue
                    delete tfp;
                    tv += fot_hp->d_t_s;
                    continue;
                }

//                if (di !=0)
//                    std::cout<<"asdf";
//                if (di >1)
//                    std::cout<<"asdf";

                // lateral costs
                tfp->c_lateral_deviation = lateral_deviation;
                tfp->c_lateral_velocity = lateral_velocity;
                tfp->c_lateral_acceleration = lateral_acceleration;
                tfp->c_lateral_jerk = lateral_jerk;
                tfp->c_lateral = fot_hp->kd * tfp->c_lateral_deviation +
                                 fot_hp->kv * tfp->c_lateral_velocity +
                                 fot_hp->ka * tfp->c_lateral_acceleration +
                                 fot_hp->kj * tfp->c_lateral_jerk;

                // longitudinal costs
                tfp->c_longitudinal_acceleration = longitudinal_acceleration;
                tfp->c_longitudinal_jerk = longitudinal_jerk;
                tfp->c_end_speed_deviation =
                    abs(fot_ic->target_speed - tfp->s_d.back());
                tfp->c_time_taken = ti;
                tfp->c_longitudinal =
                    fot_hp->ka * tfp->c_longitudinal_acceleration +
                    fot_hp->kj * tfp->c_longitudinal_jerk +
                    fot_hp->kt * tfp->c_time_taken +
                    fot_hp->kd * tfp->c_end_speed_deviation;

                // obstacle costs
                tfp->c_inv_dist_to_obstacles =
                    tfp->inverse_distance_to_obstacles(obstacles);

                // final cost
                tfp->cf = fot_hp->klat * tfp->c_lateral +
                          fot_hp->klon * tfp->c_longitudinal +
                          fot_hp->ko * tfp->c_inv_dist_to_obstacles;

                if (multithreaded) {
                    // added mutex lock to prevent threads competing to write to
                    // frenet_path
                    mu->lock();
                    frenet_paths.push_back(tfp);
                    mu->unlock();
                } else {
                    frenet_paths.push_back(tfp);
                }
//                if (lat_qp.calc_point(fot_hp->maxt) >  1)

                tv += fot_hp->d_t_s;
            }
            ti += fot_hp->dt;
            // make sure to deallocate
            delete fp;
        }

        di += fot_hp->d_road_w;
    }
    // valid_path_time *= 1e-6;
    // cout << "NUM THREADS = " << fot_hp->num_threads << "\n"; // check if
    // Thread argument is passed down cout << "Found " << frenet_paths.size() <<
    // " valid paths out of " << num_paths << " paths; Valid path time " <<
    // valid_path_time << "\n";
}

void FrenetOptimalTrajectory::setObstacles() {
    // Construct obstacles
//    vector<double> llx(fot_ic->o_llx, fot_ic->o_llx + fot_ic->no);
//    vector<double> lly(fot_ic->o_lly, fot_ic->o_lly + fot_ic->no);
//    vector<double> urx(fot_ic->o_urx, fot_ic->o_urx + fot_ic->no);
//    vector<double> ury(fot_ic->o_ury, fot_ic->o_ury + fot_ic->no);

    for (int i = 0; i < fot_ic->no; i++) {
        addObstacle(Vector2f(llx[i], lly[i]), Vector2f(urx[i], ury[i]),i);
    }
}

void FrenetOptimalTrajectory::addObstacle(Vector2f first_point,
                                          Vector2f second_point, int i) {
    obstacles.push_back(new Obstacle(std::move(first_point),
                                     std::move(second_point),
                                     fot_hp->obstacle_clearance, obstacles_points[i]));
}

void FrenetOptimalTrajectory::pubglobalpath(){

    pcl::PointCloud<pcl::PointXYZI> tmp;
    pcl::PointXYZI p;
    for (size_t i = 0; i< global_path.size(); i++){
        p.x = global_path[i].x;
        p.y = global_path[i].y;
        p.z = 0;
        p.intensity = 0;
        tmp.push_back(p);
    }

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(tmp, output);
    output.header.frame_id = "world";
    pub_path_global.publish(output);

}

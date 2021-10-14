#include "local_path.h"

LocalPath::LocalPath() : vehicle_speed(0.0),
                         state_path(StatePath::lane_keeping),
                         is_exist_obstacle(false)
{
    start_pose.orientation.w = 1;
    goal_pose.orientation.w = 1;
    time_lane_change = 4;
    point_unprotected_left_turn[0].x = 26.5488;
    point_unprotected_left_turn[0].y = 184.239;
    point_unprotected_left_turn[1].x = 29.5389;
    point_unprotected_left_turn[1].y = 183.954;
    is_irr_same_link = false;
    speed_min_lanechange = 6.0;
    genBsdPoints();
}

LocalPath::~LocalPath() {}

std::vector<geometry_msgs::Pose> LocalPath::genBezierCurve(const std::vector<geometry_msgs::Pose> &poses, const int res)
{
    std::vector<geometry_msgs::Pose> curve;
    double t = 0.0;
    double u = 1.0 - t;
    double max_len = res / 2.0;
    for (int len = 0; len < max_len + 1; len++)
    {
        geometry_msgs::Pose p;
        p.position.x = 0.0;
        p.position.y = 0.0;
        t = (double)len / max_len;
        u = 1.0 - t;
        p.position.x += u * u * poses[0].position.x;
        p.position.y += u * u * poses[0].position.y;

        p.position.x += 2 * u * t * poses[1].position.x;
        p.position.y += 2 * u * t * poses[1].position.y;

        p.position.x += t * t * poses[2].position.x;
        p.position.y += t * t * poses[2].position.y;
        curve.push_back(p);
    }
    for (int len = 0; len < max_len + 1; len++)
    {
        geometry_msgs::Pose p;
        p.position.x = 0.0;
        p.position.y = 0.0;
        t = (double)len / max_len;
        u = 1.0 - t;
        p.position.x += u * u * poses[2].position.x;
        p.position.y += u * u * poses[2].position.y;

        p.position.x += 2 * u * t * poses[3].position.x;
        p.position.y += 2 * u * t * poses[3].position.y;

        p.position.x += t * t * poses[4].position.x;
        p.position.y += t * t * poses[4].position.y;
        curve.push_back(p);
    }
    for (size_t i = 1; i < curve.size() - 1; i++)
    {
        double tangent = calcTangent(curve[i - 1].position, curve[i + 1].position);
        if (tangent < M_PI && tangent > -M_PI)
        {
            tf2::Quaternion q;
            q.setRPY(0, 0, tangent);
            curve[i].orientation = tf2::toMsg(q);
        }
    }
    curve.front().orientation = curve[1].orientation;
    curve.back().orientation = curve[curve.size() - 2].orientation;

    return curve;
}

std::vector<geometry_msgs::Pose> LocalPath::genBezierCurve(const geometry_msgs::Pose &p0, const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2, const geometry_msgs::Pose &p3, const int res)
{
    std::vector<geometry_msgs::Pose> curve;
    double t = 0.0;
    double u = 1.0 - t;
    double max_len = res;
    for (int len = 0; len < max_len + 1; len++)
    {
        geometry_msgs::Pose p;
        p.position.x = 0.0;
        p.position.y = 0.0;
        t = (double)len / max_len;
        u = 1.0 - t;
        p.position.x += u * u * u * p0.position.x;
        p.position.y += u * u * u * p0.position.y;

        p.position.x += 3 * u * u * t * p1.position.x;
        p.position.y += 3 * u * u * t * p1.position.y;

        p.position.x += 3 * u * t * t * p2.position.x;
        p.position.y += 3 * u * t * t * p2.position.y;

        p.position.x += t * t * t * p3.position.x;
        p.position.y += t * t * t * p3.position.y;
        curve.push_back(p);
    }
    for (size_t i = 1; i < curve.size() - 1; i++)
    {
        double tangent = calcTangent(curve[i - 1].position, curve[i + 1].position);
        if (tangent < M_PI && tangent > -M_PI)
        {
            tf2::Quaternion q;
            q.setRPY(0, 0, tangent);
            curve[i].orientation = tf2::toMsg(q);
        }
    }
    curve.front().orientation = curve[1].orientation;
    curve.back().orientation = curve[curve.size() - 2].orientation;

    return curve;
}

void LocalPath::genIntersectionStopPath(const double dis)
{
    double accum_dis = 0.0;
    for (int i = 0; i + 1 < path_local.size(); i++)
    {
        if (accum_dis > dis - 2)
            path_local[i].intensity = 0;
        else
            accum_dis += hypot(path_local[i].x - path_local[i + 1].x, path_local[i].y - path_local[i + 1].y);
    }
    accum_dis = 0.0;
    for (int i = path_local.size() - 2; i >= 0; i--)
    {
        if (path_local[i].intensity < 10e-3)
            continue;
        else
        {
            accum_dis += hypot(path_local[i].x - path_local[i + 1].x, path_local[i].y - path_local[i + 1].y);
            double cal_speed = ((dis - 2 - 2) * 0.7);
            if (cal_speed < path_local.points[i].intensity)
                path_local.points[i].intensity = cal_speed;
        }
    }
}

bool LocalPath::initPose()
{
    idx_curr_path = findCurrPathIndex(curr_pose);
    //가까운 pathindex 못찾으면 처음부터 다시 찾음.
    if (idx_curr_path.path == -1)
    {
        idx_curr_path = findCurrPathIndex(curr_pose);
        //전체 경로에서 index 못찾으면 오류!
        if (idx_curr_path.path == -1)
        {
            fprintf(stderr, "Not found curr path index!\n");
            return false;
        }
    }
    return true;
}

WayPointIndex LocalPath::findCurrPathIndex(const geometry_msgs::Pose &pose)
{
    WayPointIndex curr_idx = idx_curr_path;
    WayPointIndex curr_idx_half = idx_curr_path;
    double min_dis = DBL_MAX;
    //처음 탐색은 전부 탐색, 두번째부터는 주변 탐색.
    int path_start = (idx_curr_path.path == -1) ? 0 : idx_curr_path.path,
        path_end = (idx_curr_path.path == -1) ? msg_path_global.pathAry.size() : idx_curr_path.path + 2;

    double dis_min = INT_MAX;
    //링크 검색
    for (int path_iter = path_start; path_iter < path_end; path_iter++)
    {
        WayPointIndex idx_path(path_start, 1, 0);
        if (isValidIndex(idx_path))
        {
            double dis = calcDistance(curr_pose.position, getWayPoint(idx_path));
            dis_min = (dis < dis_min) ? dis : dis_min;
            if (dis < dis_min)
            {
                dis_min = dis;
                path_start = (path_iter > 0) ? path_iter - 1 : 0;
                path_end = (path_iter + 1 < msg_path_global.pathAry.size()) ? path_iter + 1 : msg_path_global.pathAry.size();
            }
        }
    }
    std::vector<WayPointIndex> indice_min_distance;
    for (int path_iter = path_start; path_iter < path_end; path_iter++)
    {
        //path 범위 넘어가면 넘김
        if (path_iter < 0 || path_iter >= msg_path_global.pathAry.size())
            continue;
        // 0번은 91차선, 1,2,3,4 차선은 순서대로 들어있음.
        int link_start = 0,
            link_end = msg_path_global.pathAry[path_iter].links.size();
        for (int link_iter = link_start; link_iter < link_end; link_iter++)
        {
            if (!isValidIndex(WayPointIndex(path_iter, link_iter, 0)))
                continue;
            core_map::a3_link tmp = getLink(WayPointIndex(path_iter, link_iter, 0));
            int index_past_left = 0;
            int index_past_right = tmp.waypointAry.size();
            {
                dis_min = INT_MAX;
                int index_half = tmp.waypointAry.size() / 2.0;
                // 필요없을듯.
                // if(index_half + 1 < tmp.waypointAry.size())
                while (true)
                {
                    pcl::PointXYZI p_left = convertXYZI(getWayPoint(WayPointIndex(path_iter, link_iter, index_half)));
                    pcl::PointXYZI p_right = convertXYZI(getWayPoint(WayPointIndex(path_iter, link_iter, index_half + 1)));
                    double dis_left = pow(pose.position.x - p_left.x, 2) + pow(pose.position.y - p_left.y, 2);
                    double dis_right = pow(pose.position.x - p_right.x, 2) + pow(pose.position.y - p_right.y, 2);
                    if (dis_left < dis_right)
                    {
                        if (index_half == 0)
                        {
                            break;
                        }
                        else
                        {
                            if (fabs(dis_min - dis_left) < 10e-5)
                                break;
                            else
                            {
                                index_past_right = index_half;
                                index_half = (index_past_left + index_past_right) / 2.0;
                                dis_min = dis_left;
                            }
                        }
                    }
                    else
                    {
                        if (index_half == tmp.waypointAry.size() - 1)
                        {
                            break;
                        }
                        else
                        {
                            if (fabs(dis_min - dis_right) < 10e-5)
                            {
                                index_half++;
                                break;
                            }
                            else
                            {
                                index_past_left = index_half;
                                index_half = (index_past_left + index_past_right) / 2.0;
                                dis_min = dis_right;
                            }
                        }
                    }
                }
                indice_min_distance.push_back(WayPointIndex(path_iter, link_iter, index_half));
            }
        }
    }
    dis_min = INT_MAX;
    for (int iter_idx = 0; iter_idx < indice_min_distance.size(); iter_idx++)
    {
        double dis = calcDistance(curr_pose.position, getWayPoint(indice_min_distance.at(iter_idx)));
        if (getLink(indice_min_distance.at(iter_idx)).linkid[8] == 'I')
        {
            if (indice_min_distance.at(iter_idx).link != getPath(indice_min_distance.at(iter_idx)).src)
                dis = dis + 0.5;
        }
        if (dis < dis_min)
        {
            dis_min = dis;
            curr_idx_half = indice_min_distance.at(iter_idx);
        }
    }
    indice_min_distance.clear();
    indice_min_distance.push_back(curr_idx_half);
    WayPointIndex idx_check = findNextPointIndex(curr_idx_half);
    if (isValidIndex(idx_check))
    {
        indice_min_distance.push_back(idx_check);
        idx_check = findNextPointIndex(idx_check);
        if (isValidIndex(idx_check))
        {
            indice_min_distance.push_back(idx_check);
        }
    }
    idx_check = findPreviousPointIndex(curr_idx_half);
    if (isValidIndex(idx_check))
    {
        indice_min_distance.push_back(idx_check);
        idx_check = findPreviousPointIndex(idx_check);
        if (isValidIndex(idx_check))
        {
            indice_min_distance.push_back(idx_check);
        }
    }

    dis_min = INT_MAX;
    for (int iter_idx = 0; iter_idx < indice_min_distance.size(); iter_idx++)
    {
        double dis = calcDistance(curr_pose.position, getWayPoint(indice_min_distance.at(iter_idx)));
        if (getLink(indice_min_distance.at(iter_idx)).linkid[8] == 'I')
        {
            if (indice_min_distance.at(iter_idx).link != getPath(indice_min_distance.at(iter_idx)).src)
                dis = dis + 0.5;
        }
        if (dis < dis_min)
        {
            dis_min = dis;
            curr_idx_half = indice_min_distance.at(iter_idx);
        }
    }

    path_start = (idx_curr_path.path == -1) ? 0 : idx_curr_path.path;
    path_end = (idx_curr_path.path == -1) ? msg_path_global.pathAry.size() : idx_curr_path.path + 2;
    //링크 검색
    for (int path_iter = path_start; path_iter < path_end; path_iter++)
    {
        //path 범위 넘어가면 넘김
        if (path_iter < 0 || path_iter + 1 > msg_path_global.pathAry.size())
            continue;
        // 0번은 91차선, 1,2,3,4 차선은 순서대로 들어있음.
        int link_start = 0,
            link_end = msg_path_global.pathAry[path_iter].links.size();
        for (int link_iter = link_start; link_iter < link_end; link_iter++)
        {
            if (!isValidIndex(WayPointIndex(path_iter, link_iter, 0)))
                continue;
            core_map::a3_link tmp = getLink(WayPointIndex(path_iter, link_iter, 0));
            for (int point_iter = 0; point_iter < tmp.waypointAry.size(); point_iter++)
            {
                // {
                //     double dis_min = INT_MAX;
                //     int index_half = tmp.waypointAry.size() / 2.0;
                //     // 필요없을듯.
                //     // if(index_half + 1 < tmp.waypointAry.size())
                //     while(true)
                //     {
                //         pcl::PointXYZI p_left = convertXYZI(getWayPoint(WayPointIndex(path_iter, link_iter, index_half)));
                //         pcl::PointXYZI p_right = convertXYZI(getWayPoint(WayPointIndex(path_iter, link_iter, index_half + 1)));
                //         double dis_left = pow(pose.position.x - p_left.x, 2) + pow(pose.position.y - p_left.y, 2);
                //         double dis_right = pow(pose.position.x - p_right.x, 2) + pow(pose.position.y - p_right.y, 2);
                //         if(dis_left < dis_right)
                //         {
                //             if(index_half == 0)
                //             {
                //                 break;
                //             }
                //             else
                //             {
                //                 if(fabs(dis_min - dis_left) < 10e-5)
                //                     break;
                //                 else
                //                 {
                //                     index_half = index_half / 2.0;
                //                     dis_min = dis_left;
                //                 }
                //             }
                //         }
                //         else
                //         {
                //             if(index_half == tmp.waypointAry.size() -1)
                //             {
                //                 break;
                //             }
                //             else
                //             {
                //                 if(fabs(dis_min - dis_right) < 10e-5)
                //                 {
                //                     index_half++;
                //                     break;
                //                 }
                //                 else
                //                 {
                //                     index_half = (dis_right + tmp.waypointAry.size()) / 2.0;
                //                     dis_min = dis_right;
                //                 }
                //             }
                //         }
                //     }
                // }
                pcl::PointXYZI p = convertXYZI(getWayPoint(WayPointIndex(path_iter, link_iter, point_iter)));
                //거리 최소값 찾아서 현재위치 탐색.
                double dis = pow(pose.position.x - p.x, 2) + pow(pose.position.y - p.y, 2);
                // 교차로에서 src가 아닌경로는 페널티줌
                if (getLink(WayPointIndex(path_iter, link_iter, 0)).linkid[8] == 'I')
                    if (link_iter != getPath(WayPointIndex(path_iter, 0, 0)).src)
                        dis = dis + 1.5;
                if (dis < min_dis)
                {
                    min_dis = dis;
                    curr_idx.path = path_iter;
                    curr_idx.link = link_iter;
                    curr_idx.point = point_iter;
                    if (min_dis < 0.25)
                        return curr_idx;
                }
            }
        }
    }
    if (curr_idx.point != curr_idx_half.point)
    {
        curr_idx_half.point = curr_idx.point;
    }
    //최소거리가 3미터 이상이면 path 못찾음.
    if (min_dis > 4)
    {
        curr_idx = WayPointIndex();
        // fprintf(stderr,"Too far from the path\n");
    }
    return curr_idx;
}

PC_XYZI LocalPath::genGlobalFollowPath(const double dis_path)
{
    is_exist_obstacle = false;
    is_irr_same_link = false;
    PC_XYZI path;
    WayPointIndex next_point_idx_daegue = idx_curr_path;
    //Global path를 따라가는 경로를 생성함.
    double accum_dis = 0.0;
    double min_speed = 1.0;
    // 설정된 경로 길이까지 다음점 추가
    while (accum_dis < dis_path)
    {
        //다음점 탐색
        WayPointIndex curr_idx = next_point_idx_daegue;
        next_point_idx_daegue = findNextPointIndex(curr_idx);
        if (isValidIndex(next_point_idx_daegue))
        {
            accum_dis += calcDistance(curr_idx, next_point_idx_daegue);
            pcl::PointXYZI p = convertXYZI(getWayPoint(next_point_idx_daegue));
            if (is_exist_obstacle)
            {
                p.intensity = 0;
                path.points.push_back(p);
                continue;
            }
            // 충돌검사
            if (!checkCollisionIdx(next_point_idx_daegue))
                path.points.push_back(p);
            else
            {
                if (msg_object.polygon.at(index_object).class_name == "irregular")
                    is_irr_same_link = true;
                is_exist_obstacle = true;
                p.intensity = 0;
                path.points.push_back(p);
                PC_XYZI path_trj;
                path_trj = path;
                Trajectory trj;
                double tmp_speed = vehicle_speed > min_speed ? vehicle_speed : min_speed;
                double dis_safety = 5.0;
                if (trj.speedProfileDesign(getLink(curr_idx).speed / 3.6, tmp_speed / 3.6, 0, accum_dis - dis_safety))
                {
                    trj.getTrajectory(path_trj);
                    // 경로중에 낮은속도 선택
                    for (size_t iter = 0; iter < path.size(); iter++)
                        path.at(iter).intensity = path.at(iter).intensity < path_trj.at(iter).intensity ? path.at(iter).intensity : path_trj.at(iter).intensity;
                }
                else
                {
                    for (size_t iter = 0; iter < path.size(); iter++)
                        path.at(iter).intensity = 0;
                }
            }
        }
        // 경로가 재대로 만들어지지 않으면 경로의 끝에서 정지하게 만듬.
        else
        {
            Trajectory trj;
            if (!path.empty())
            {
                PC_XYZI path_trj;
                path_trj = path;
                double tmp_speed = vehicle_speed > min_speed ? vehicle_speed : min_speed;
                double dis_safety = 0.5;
                bool bool_slow_down = false;
                // 차선변경 해야하는곳에서 목표차선과 위치가 다를때
                int diff_lane = 0;
                if (getPath(idx_curr_path).changeLane == true)
                {
                    if (getPath(idx_curr_path).dst != idx_curr_path.link)
                    {
                        dis_safety = 13;
                        bool_slow_down = true;
                        diff_lane = abs(getPath(idx_curr_path).dst - idx_curr_path.link);
                        dis_safety *= diff_lane;
                    }
                }
                else
                {
                    if (getPath(idx_curr_path).src != idx_curr_path.link)
                    {
                        dis_safety = 13;
                        bool_slow_down = true;
                        diff_lane = abs(getPath(idx_curr_path).src - idx_curr_path.link);
                        dis_safety *= diff_lane;
                    }
                }
                if (getPath(curr_idx).changeLane == true)
                {
                    if (getPath(curr_idx).dst != curr_idx.link)
                    {
                        dis_safety = 13;
                        bool_slow_down = true;
                        diff_lane = abs(getPath(curr_idx).dst - curr_idx.link);
                        dis_safety *= diff_lane;
                    }
                }
                else
                {
                    if (getPath(curr_idx).src != curr_idx.link)
                    {
                        dis_safety = 13;
                        bool_slow_down = true;
                        diff_lane = abs(getPath(curr_idx).src - curr_idx.link);
                        dis_safety *= diff_lane;
                    }
                }
                // 속도를 줄여야하면 최대 가속도를 낮춰서 목적지에 천천히 도달하게함.
                if (bool_slow_down)
                    trj.setMaxAcc(0.5);
                if (trj.speedProfileDesign(getLink(curr_idx).speed / 3.6, tmp_speed / 3.6, 0, accum_dis - dis_safety))
                {
                    trj.getTrajectory(path_trj);
                    // 경로중에 낮은속도 선택
                    for (size_t iter = 0; iter < path.size(); iter++)
                    {
                        path.at(iter).intensity = path.at(iter).intensity < path_trj.at(iter).intensity ? path.at(iter).intensity : path_trj.at(iter).intensity;
                    }
                }
                else
                {
                    for (size_t iter = 0; iter < path.size(); iter++)
                        path.at(iter).intensity = 0;
                }
            }
            //멈추는 경로 뿌리고 함수 종료.
            return path;
        }
    }
    index_object_front = index_object;
    if (!path.empty() && !is_exist_obstacle)
    {
        PC_XYZI path_trj;
        path_trj = path;
        Trajectory trj;
        double tmp_speed = vehicle_speed > min_speed ? vehicle_speed : min_speed;
        double max_speed = 40.0;
        if (trj.speedProfileDesign(max_speed / 3.6, tmp_speed / 3.6, path.back().intensity / 3.6, accum_dis))
        {
            trj.getTrajectory(path_trj);
            // 경로중에 낮은속도 선택
            for (size_t iter = 0; iter < path.size(); iter++)
                path.at(iter).intensity = path.at(iter).intensity < path_trj.at(iter).intensity ? path.at(iter).intensity : path_trj.at(iter).intensity;
        }
        else
        {
            for (size_t iter = 0; iter < path.size(); iter++)
                path.at(iter).intensity = path.at(iter).intensity < path_trj.at(iter).intensity ? path.at(iter).intensity : path_trj.at(iter).intensity;
        }
    }
    return path;
}

WayPointIndex LocalPath::findNextPointIndex(const WayPointIndex &idx)
{
    WayPointIndex next_point_idx_daegue = idx;
    int next_lane_idx = -1;
    //현재링크 범위면 포인트 idx만 증가.
    if (next_point_idx_daegue.point + 1 < getLink(next_point_idx_daegue).waypointAry.size())
    {
        next_point_idx_daegue.point++;
    }
    //현재 링크의 마지막 포인트면 다음 링크로 넘어감.
    else
    {
        //다음링콰 연결된 차선찾기 / 마지막링크면 첫번째 링크와 연결된 차선 찾기
        next_lane_idx = findNextLaneIndex(next_point_idx_daegue);
        //다음 링크와 연결된 차선을 못찾으면 에러
        if (next_lane_idx == -1)
        {
            // fprintf(stderr,"next link not exist\n");
            WayPointIndex err_idx;
            return err_idx;
        }
        //마지막 링크가 아니면 PATH수 증가.
        if (next_point_idx_daegue.path + 1 < msg_path_global.pathAry.size())
            next_point_idx_daegue.path++;
        else
            next_point_idx_daegue.path = 0;

        next_point_idx_daegue.link = next_lane_idx;
        next_point_idx_daegue.point = 0;
    }
    return next_point_idx_daegue;
}

int LocalPath::findNextLaneIndex(const WayPointIndex &idx)
{
    int next_link = -1;
    double min_dis = DBL_MAX;
    WayPointIndex next_link_idx = idx;
    //다음링크 없으면 오류
    if (idx.path + 1 < msg_path_global.pathAry.size())
        next_link_idx.path = idx.path + 1;
    else
        return next_link;
    // next_link_idx = 0;
    // for (size_t j = 0; j < getPath(next_link_idx).links.size(); j++)
    // {
    //     //0
    //     if(msg_path_global.pathAry[next_link_idx.path].links[j].waypointAry.empty())
    //         continue;
    //     next_link_idx.link = j;
    //     //현재차로의 마지막 점과 다음 링크 차로들의 첫번째 점들의 거리를 비교한다.
    //     double dis = hypot(getLink(idx).waypointAry.back().x - getLink(next_link_idx).waypointAry.front().x, getLink(idx).waypointAry.back().y - getLink(next_link_idx).waypointAry.front().y);
    //     // src가 아니면 코스트 증가 (여러차선이 겹쳐졌을때 예외처리)
    //     if(getPath(idx).src != next_link)
    //         dis += 0.5;
    //     if(dis < 0.5 && dis < min_dis )
    //     {
    //         min_dis = dis;
    //         next_link = j;
    //     }
    // }
    std::vector<int> next_link_idxs;
    for (size_t link_iter = 0; link_iter < getPath(next_link_idx).links.size(); link_iter++)
    {
        // 링크 없으면 넘김
        if (msg_path_global.pathAry[next_link_idx.path].links[link_iter].waypointAry.empty())
            continue;
        // to node와 from node가 같으면 현재차선과 연결된 차선임.
        if (getLink(idx).tonode == getPath(next_link_idx).links[link_iter].fromnode)
            next_link_idxs.push_back(link_iter);
    }
    // 연결된 링크가 하나 이상이면 Global Path 링크를 선택함.
    if (next_link_idxs.size() == 0)
        return -1;
    else if (next_link_idxs.size() != 1)
        next_link = getPath(next_link_idx).src;
    else
        next_link = next_link_idxs.front();
    return next_link;
}

WayPointIndex LocalPath::findNextPathIndex(const WayPointIndex &idx)
{
    WayPointIndex index_next_path = idx;
    if (idx.path < msg_path_global.pathAry.size())
    {
        index_next_path.path++;
        index_next_path.point = 0;
        index_next_path.link = findNextLaneIndex(idx);
        if (index_next_path.link != -1)
            return index_next_path;
        else
        {
            index_next_path.link = getPath(index_next_path).src;
            if (isValidIndex(index_next_path))
                return index_next_path;
            else
                return WayPointIndex();
        }
        return WayPointIndex();
    }
    else
        return WayPointIndex();
}

bool LocalPath::isValidIndex(const WayPointIndex &idx)
{
    //링크범위, 차선범위, 웨이포인트 범위 검증
    if (idx.path >= 0 && idx.path < msg_path_global.pathAry.size())
        if (idx.link >= 0 && idx.link < getPath(idx).links.size())
        {
            if (idx.link != 0)
            {
                if (idx.point >= 0 && idx.point < getLink(idx).waypointAry.size())
                    return true;
            }
            // idx가 0일때는 링크가 존재하는지 확인
            else
            {
                if (!msg_path_global.pathAry[idx.path].links[0].waypointAry.empty())
                    if (idx.point >= 0 && idx.point < getLink(idx).waypointAry.size())
                        return true;
            }
        }
    return false;
}

WayPointIndex LocalPath::findSideLanePoint(const WayPointIndex &idx, const bool &is_left_lane)
{
    WayPointIndex side_lane_idx = idx;
    side_lane_idx.point = 0;
    //dst차선 선택.
    // side_lane_idx.link = getPathDaegue(idx).dst;
    if (is_left_lane)
        side_lane_idx.link--;
    else
        side_lane_idx.link++;
    double min_dis = DBL_MAX;
    if (!isValidIndex(side_lane_idx))
    {
        // fprintf(stderr, "not_exist_side_lane\n");
        WayPointIndex not_exist_side_lane;
        return not_exist_side_lane;
    }
    //옆차선에서 처음부터 끝까지 현재 위치와 가장 가까운점 찾기.
    WayPointIndex iter_idx = side_lane_idx;
    for (int iter = 0; iter < getLink(side_lane_idx).waypointAry.size(); iter++)
    {
        double dis = hypot(getWayPoint(idx).x - getWayPoint(iter_idx).x, getWayPoint(idx).y - getWayPoint(iter_idx).y);
        if (min_dis > dis)
        {
            min_dis = dis;
            side_lane_idx.point = iter;
        }
        iter_idx.point++;
    }
    return side_lane_idx;
}

WayPointIndex LocalPath::findLookAheadPoint(const WayPointIndex &path_idx_daegue, const double look_ahead_distance)
{
    WayPointIndex idx_look_ahead_tmp = path_idx_daegue;
    double dis = 0;

    //최대 찾는 점 1000개
    int max_iter = 1000;
    for (int i = 0; i < max_iter; i++)
    {
        //다음점 인덱스
        WayPointIndex next_idx = findNextPointIndex(idx_look_ahead_tmp);
        if (isValidIndex(next_idx))
        {
            core_map::waypoint curr_point = getWayPoint(idx_look_ahead_tmp);
            core_map::waypoint next_point = getWayPoint(next_idx);
            //거리 누적
            dis += hypot(next_point.x - curr_point.x, next_point.y - curr_point.y);
            //설정한 거리에 도달하면 인덱스 반환.
            if (dis >= look_ahead_distance)
            {
                return idx_look_ahead_tmp;
                break;
            }
            idx_look_ahead_tmp = next_idx;
        }
        else
        {
            //path가 원하는 look ahead 길이보다 짧거나 유효한 점이 아닐 때
            WayPointIndex err_idx;
            return err_idx;
            break;
        }
    }
    //마지막까지 몾찾으면 오류!
    WayPointIndex err_idx;
    return err_idx;
}

ChangeableLanes LocalPath::findLaneChangeable(const WayPointIndex &idx)
{
    ChangeableLanes cl;
    //현재링크의 왼쪽, 오른쪽 차선 존재 여부
    //1차로 초과면
    WayPointIndex left_lane_idx = idx;
    WayPointIndex right_lane_idx = idx;
    left_lane_idx.link = idx.link - 1;
    left_lane_idx.point = 0;
    right_lane_idx.link = idx.link + 1;
    right_lane_idx.point = 0;

    if (isValidIndex(left_lane_idx))
    {
        cl = ChangeableLanes::LEFT;
        if (isValidIndex(right_lane_idx))
            cl = ChangeableLanes::BOTH;
    }
    else if (isValidIndex(right_lane_idx))
        cl = ChangeableLanes::RIGHT;
    else
        cl = ChangeableLanes::NOT_EXIST;
    return cl;
}

bool LocalPath::checkCollision(const geometry_msgs::Pose &pose)
{
    // 차량 충돌범위를 앞, 중간, 뒤 점을 기준으로 원을그려 계산한다.
    // [ ] 모양을 000으로 계산
    std::vector<geometry_msgs::Point> collision_points;
    collision_points.resize(3);
    double radius = 2.0 / 2.0; //3.0
    double distance = 1.5;     //3.2
    double offset = 0.55;
    // front point
    collision_points[0].x = pose.position.x + (offset + distance) * cos(tf2::getYaw(pose.orientation));
    collision_points[0].y = pose.position.y + (offset + distance) * sin(tf2::getYaw(pose.orientation));
    // center point
    collision_points[1].x = pose.position.x + offset * cos(tf2::getYaw(pose.orientation));
    collision_points[1].y = pose.position.y + offset * sin(tf2::getYaw(pose.orientation));
    // rear point
    collision_points[2].x = pose.position.x + (offset - distance) * cos(tf2::getYaw(pose.orientation));
    collision_points[2].y = pose.position.y + (offset - distance) * sin(tf2::getYaw(pose.orientation));

    for (int i = 0; i < collision_points.size(); i++)
    {
        int x = (collision_points[i].x - curr_pose.position.x) / occ_map.info.resolution + (occ_map.info.width / 2.0);
        int y = (collision_points[i].y - curr_pose.position.y) / occ_map.info.resolution + (occ_map.info.height / 2.0);
        // 원점이동 -> 맵 중앙이동
        double mask_size = radius * 2.0 / occ_map.info.resolution;
        for (int mask_x = 0; mask_x < mask_size; mask_x++)
        {
            for (int mask_y = 0; mask_y < mask_size; mask_y++)
            {
                int px = x + mask_x - mask_size / 2.0;
                int py = y + mask_y - mask_size / 2.0;
                if (px + 2 > occ_map.info.width || px - 2 < 0 || py + 2 > occ_map.info.height || py - 2 < 0)
                    continue;
                // 패스위에 장애물이 있다면
                if (hypot(mask_x - mask_size / 2.0, mask_y - mask_size / 2.0) < radius / occ_map.info.resolution)
                {
                    if (occ_map.data[py * occ_map.info.width + px] != 0)
                    {
                        if (occ_map.data[py * occ_map.info.width + px] > 90)
                            state_collision = StateCollision::irregular;
                        else
                        {
                            index_object = 90 - occ_map.data[py * occ_map.info.width + px];
                            // 미완성 - dynamic 상대속도 고려 해야함.
                            if (hypot(msg_object.polygon[index_object].velocity_x, msg_object.polygon[index_object].velocity_y) > 100)
                                state_collision = StateCollision::dynamic_object;
                            else
                                state_collision = StateCollision::static_object;
                            // 미완성 - dynamic 상대속도 고려 해야함.
                            state_collision = StateCollision::static_object;
                        }
                        is_exist_obstacle = true;
                        return true;
                    }
                }
            }
        }
    }
    index_object = -1;
    return false;
}

bool LocalPath::checkCollisionIdx(const WayPointIndex &idx)
{
    pcl::PointXYZI p = convertXYZI(getWayPoint(idx));
    // 장애물 체크
    double tangent = calcTangent(idx);
    if (tangent < M_PI && tangent > -M_PI)
    {
        geometry_msgs::Pose tmp_pose;
        tmp_pose.position.x = p.x;
        tmp_pose.position.y = p.y;
        tf2::Quaternion q;
        q.setRPY(0, 0, tangent);
        tmp_pose.orientation = tf2::toMsg(q);
        if (checkCollision(tmp_pose))
        {
            // fprintf(stderr, "Collision Error\n");
            return true;
        }
    }
    else
        return false;
    return false;
}

bool LocalPath::checkCollisionPath(const PC_XYZI &path)
{
    nav_msgs::Path nav_path = toNavPath(path);
    for (size_t i = 0; i < nav_path.poses.size(); i++)
    {
        if (checkCollision(nav_path.poses[i].pose))
            return true;
    }
    return false;
}

nav_msgs::Path LocalPath::toNavPath(const PC_XYZI &path)
{
    nav_msgs::Path nav_path;
    nav_path.poses.resize(path.size());
    // for (size_t i = 1; i < path.size() - 1; i++)
    // {
    //     int pre_idx = i - 1,
    //         next_idx = i + 1;
    //     // 두 점 위치가 같으면 다른점 선택
    //     if(hypot(path[pre_idx].x - path[next_idx].x ,path[pre_idx].y - path[next_idx].y ) < 0.01)
    //     {
    //         if( i > 1 )
    //             pre_idx--;
    //         else if ( i + 1 < path.size())
    //             next_idx++;
    //     }
    //     double tangent = calcTangent(path[pre_idx], path[next_idx]);
    //     if(tangent < M_PI && tangent > -M_PI)
    //     {
    //         tf2::Quaternion q;
    //         q.setRPY(0, 0, tangent);
    //         nav_path.poses[i].pose.position.x = path[i].x;
    //         nav_path.poses[i].pose.position.y = path[i].y;
    //         nav_path.poses[i].pose.position.z = path[i].z;
    //         nav_path.poses[i].pose.orientation = tf2::toMsg(q);
    //     }
    // }
    for (int i = 0; i < path.size(); i++)
    {
        nav_path.poses.at(i).pose.position.x = path.at(i).x;
        nav_path.poses.at(i).pose.position.y = path.at(i).y;
        nav_path.poses.at(i).pose.position.z = path.at(i).z;
    }

    for (int i = 0; i < nav_path.poses.size(); i++)
    {
        double tangent = calcTangent(nav_path.poses, i);
        if (tangent < M_PI && tangent > -M_PI)
        {
            tf2::Quaternion q;
            q.setRPY(0, 0, tangent);
            nav_path.poses.at(i).pose.orientation = tf2::toMsg(q);
        }
        else
        {
            tf2::Quaternion q;
            q.setRPY(0, 0, 0);
            nav_path.poses.at(i).pose.orientation = tf2::toMsg(q);
        }
    }
    // nav_path.poses.front().pose.position.x = path.front().x;
    // nav_path.poses.front().pose.position.y = path.front().y;
    // nav_path.poses.front().pose.position.z = path.front().z;
    // nav_path.poses.front().pose.orientation = nav_path.poses.at(1).pose.orientation;
    // nav_path.poses.back().pose.position.x = path.back().x;
    // nav_path.poses.back().pose.position.y = path.back().y;
    // nav_path.poses.back().pose.position.z = path.back().z;
    // nav_path.poses.back().pose.orientation = nav_path.poses.at(nav_path.poses.size() - 2).pose.orientation;
    return nav_path;
}

nav_msgs::Path LocalPath::toNavPath(const core_map::a3_link &path)
{
    nav_msgs::Path nav_path;
    nav_path.poses.resize(path.waypointAry.size());
    // for (size_t i = 1; i < path.waypointAry.size() - 1; i++)
    // {
    //     double tangent = atan2(path.waypointAry[i+1].y - path.waypointAry[i-1].y, path.waypointAry[i+1].x -path.waypointAry[i-1].x);
    //     if(tangent < M_PI && tangent > -M_PI)
    //     {
    //         tf2::Quaternion q;
    //         q.setRPY(0, 0, tangent);
    //         nav_path.poses[i].pose.position.x = path.waypointAry[i].x;
    //         nav_path.poses[i].pose.position.y = path.waypointAry[i].y;
    //         nav_path.poses[i].pose.orientation = tf2::toMsg(q);
    //     }
    // }
    for (int i = 0; i < path.waypointAry.size(); i++)
    {
        nav_path.poses.at(i).pose.position.x = path.waypointAry.at(i).x;
        nav_path.poses.at(i).pose.position.y = path.waypointAry.at(i).y;
        nav_path.poses.at(i).pose.position.z = 0;
    }
    for (int i = 0; i < nav_path.poses.size(); i++)
    {
        double tangent = calcTangent(nav_path.poses, i);
        if (tangent < M_PI && tangent > -M_PI)
        {
            tf2::Quaternion q;
            q.setRPY(0, 0, tangent);
            nav_path.poses.at(i).pose.orientation = tf2::toMsg(q);
        }
        else
        {
            tf2::Quaternion q;
            q.setRPY(0, 0, 0);
            nav_path.poses.at(i).pose.orientation = tf2::toMsg(q);
        }
    }
    // nav_path.poses.front().pose.position.x = path.waypointAry.front().x;
    // nav_path.poses.front().pose.position.y = path.waypointAry.front().y;
    // nav_path.poses.front().pose.orientation = nav_path.poses.at(1).pose.orientation;
    // nav_path.poses.back().pose.position.x = path.waypointAry.back().x;
    // nav_path.poses.back().pose.position.y = path.waypointAry.back().y;
    // nav_path.poses.back().pose.orientation = nav_path.poses.at(nav_path.poses.size() - 2).pose.orientation;
    return nav_path;
}

double LocalPath::calcTangent(const WayPointIndex &idx)
{
    WayPointIndex next_point_idx = idx;
    // 다음점과 현재점이 같으면 그 다음점을 찾음
    while (calcDistance(idx, next_point_idx) < 0.01)
    {
        WayPointIndex idx_next_tmp = next_point_idx;
        idx_next_tmp = findNextPointIndex(next_point_idx);
        if (isValidIndex(idx_next_tmp))
            next_point_idx = idx_next_tmp;
        else
            break;
    }

    WayPointIndex pre_point_idx = idx;
    while (calcDistance(idx, pre_point_idx) < 0.01 && calcDistance(next_point_idx, pre_point_idx) < 0.01)
    {
        WayPointIndex idx_pre_tmp = pre_point_idx;
        idx_pre_tmp = findPreviousPointIndex(pre_point_idx);
        if (isValidIndex(idx_pre_tmp))
            pre_point_idx = idx_pre_tmp;
        else
            break;
    }
    if (next_point_idx.point != -1 && pre_point_idx.point != -1)
        return atan2(getWayPoint(next_point_idx).y - getWayPoint(pre_point_idx).y, getWayPoint(next_point_idx).x - getWayPoint(pre_point_idx).x);
    else if (next_point_idx.point == -1 && pre_point_idx.point != -1)
        return atan2(getWayPoint(idx).y - getWayPoint(pre_point_idx).y, getWayPoint(idx).x - getWayPoint(pre_point_idx).x);
    else if (next_point_idx.point != -1 && pre_point_idx.point == -1)
        return atan2(getWayPoint(next_point_idx).y - getWayPoint(idx).y, getWayPoint(next_point_idx).x - getWayPoint(idx).x);
    else
        return DBL_MIN;
}

double LocalPath::calcTangent(const std::vector<geometry_msgs::PoseStamped> &vec_pose, const int index)
{
    int index_pre = index,
        index_next = index;
    // 다음점과 현재점이 같으면 그 다음점을 찾음
    while (index_next + 1 < vec_pose.size() && calcDistance(vec_pose[index].pose.position, vec_pose[index_next].pose.position) < 0.01)
        index_next++;
    // 이전점과 현재점이 같으면 그 이전점을 찾음
    while (index_pre >= 0 && calcDistance(vec_pose[index].pose.position, vec_pose[index_pre].pose.position) < 0.01)
        index_pre--;

    if (index_next < vec_pose.size() && index_pre >= 0)
        return calcTangent(vec_pose[index_next].pose.position, vec_pose[index_pre].pose.position);
    else if (index_next >= vec_pose.size() && index_pre >= 0)
        return calcTangent(vec_pose[index].pose.position, vec_pose[index_pre].pose.position);
    else if (index_next < vec_pose.size() && index_pre < 0)
        return calcTangent(vec_pose[index_next].pose.position, vec_pose[index].pose.position);
    else
        return DBL_MIN;
}

double LocalPath::calcTangent(const PC_XYZI &pc, const int index)
{
    int index_pre = index,
        index_next = index;
    // 다음점과 현재점이 같으면 그 다음점을 찾음
    while (index_next + 1 < pc.size() && calcDistance(pc[index], pc[index_next]) < 0.01)
        index_next++;
    // 이전점과 현재점이 같으면 그 이전점을 찾음
    while (index_pre > 0 && (calcDistance(pc[index], pc[index_pre]) < 0.01 && calcDistance(pc[index_next], pc[index_pre]) < 0.01))
        index_pre--;

    if (index_next < pc.size() && index_pre >= 0)
        return calcTangent(pc[index_next], pc[index_pre]);
    else if (index_next >= pc.size() && index_pre >= 0)
        return calcTangent(pc[index], pc[index_pre]);
    else if (index_next < pc.size() && index_pre < 0)
        return calcTangent(pc[index_next], pc[index]);
    else
        return DBL_MIN;
}

template <typename T, typename S>
double LocalPath::calcTangent(const T &src, const S &dst)
{
    return atan2(dst.y - src.y, dst.x - src.x);
}

template <typename T, typename S>
double LocalPath::calcDistance(const T &src, const S &dst)
{
    return hypot(dst.x - src.x, dst.y - src.y);
}

template <typename T>
double LocalPath::calcSumSqure(const T &src, const T &dst)
{
    return pow(dst.x - src.x, 2) + pow(dst.y - src.y, 2);
}

double LocalPath::calcDistance(const WayPointIndex &src, const WayPointIndex &dst)
{
    core_map::waypoint wp1 = getWayPoint(src);
    core_map::waypoint wp2 = getWayPoint(dst);
    return hypot(wp2.x - wp1.x, wp2.y - wp1.y);
}

double LocalPath::calcSumSqure(const WayPointIndex &src, const WayPointIndex &dst)
{
    core_map::waypoint wp1 = getWayPoint(src);
    core_map::waypoint wp2 = getWayPoint(dst);
    return pow(wp2.x - wp1.x, 2) + pow(wp2.y - wp1.y, 2);
}

WayPointIndex LocalPath::findPreviousPointIndex(const WayPointIndex &idx)
{
    WayPointIndex pre_point_idx_daegue = idx;
    //현재링크 범위면 포인트 idx만 감소.
    if (pre_point_idx_daegue.point > 0)
        pre_point_idx_daegue.point--;
    //현재 링크의 첫번째 포인트면 이전 링크로 넘어감.
    else
    {
        // 현재링크와 연결된 이전링크인데스 찾기
        int pre_lane_idx = findPreviousLaneIndex(pre_point_idx_daegue);
        // 못찾으면 에러
        if (pre_lane_idx == -1)
        {
            fprintf(stderr, "previous link not exist\n");
            WayPointIndex err_idx;
            return err_idx;
        }
        //첫번째 Path가 아니면 path 감소.
        if (pre_point_idx_daegue.path > 0)
            pre_point_idx_daegue.path--;

        pre_point_idx_daegue.link = pre_lane_idx;
        pre_point_idx_daegue.point = getLink(pre_point_idx_daegue).waypointAry.size() - 1;
    }
    return pre_point_idx_daegue;
}

int LocalPath::findPreviousLaneIndex(const WayPointIndex &idx)
{
    int pre_link = -1;
    double min_dis = DBL_MAX;
    WayPointIndex pre_link_idx = idx;
    //이전path 없으면 오류
    if (idx.path > 0)
        pre_link_idx.path = idx.path - 1;
    else
        return pre_link;
    // pre_link_idx = 0;
    for (size_t j = 0; j < getPath(pre_link_idx).links.size(); j++)
    {
        //0
        if (msg_path_global.pathAry[pre_link_idx.path].links[j].waypointAry.empty())
            continue;
        pre_link_idx.link = j;
        //현재차로의 마지막 점과 다음 링크 차로들의 첫번째 점들의 거리를 비교한다.
        double dis = hypot(getLink(idx).waypointAry.front().x - getLink(pre_link_idx).waypointAry.back().x, getLink(idx).waypointAry.front().y - getLink(pre_link_idx).waypointAry.back().y);
        if (dis < 1 && dis < min_dis)
        {
            min_dis = dis;
            pre_link = j;
        }
    }
    return pre_link;
}

WayPointIndex LocalPath::findLookBehindPoint(const WayPointIndex &path_idx_daegue, const double look_ahead_distance)
{
    WayPointIndex look_behind_idx_daegue = path_idx_daegue;
    double dis = 0;

    //최대 찾는 점 1000개
    int max_iter = 1000;
    for (int i = 0; i < max_iter; i++)
    {
        //다음점 인덱스
        WayPointIndex previous_idx = findPreviousPointIndex(look_behind_idx_daegue);
        if (isValidIndex(previous_idx))
        {
            core_map::waypoint curr_point = getWayPoint(look_behind_idx_daegue);
            core_map::waypoint previous_point = getWayPoint(previous_idx);
            //거리 누적
            dis += hypot(previous_point.x - curr_point.x, previous_point.y - curr_point.y);
            //설정한 거리에 도달하면 인덱스 반환.
            if (dis >= look_ahead_distance)
            {
                return look_behind_idx_daegue;
                break;
            }
            look_behind_idx_daegue = previous_idx;
        }
        else
        {
            //path가 원하는 look ahead 길이보다 짧거나 유효한 점이 아닐 때
            WayPointIndex err_idx;
            return err_idx;
            break;
        }
    }
    //마지막까지 몾찾으면 오류!
    WayPointIndex err_idx;
    return err_idx;
}

bool LocalPath::tickCheckFrontObject()
{
    return is_exist_obstacle;
    // is_exist_obstacle = false;
    // return checkCollisionPath(path_lane_keeping);
}
// 미완성
bool LocalPath::tickCheckNearStopline()
{
    if (isValidIndex(idx_curr_path))
    {
        if (getPath(idx_curr_path).isStop)
            ;
        {
            core_map::waypoint point_stop_line = getLink(idx_curr_path).waypointAry.back();
            double dis = hypot(curr_pose.position.x - point_stop_line.x, curr_pose.position.y - point_stop_line.y);
            // 정지선과 20m 미만이면
            if (dis < 20)
                return true;
            else
                return false;
        }
    }
    return false;
}
bool LocalPath::tickCheckCollision()
{
    is_exist_obstacle = false;
    if (state_path == StatePath::lane_keeping)
        return checkCollisionPath(path_lane_keeping);
    else if (state_path == StatePath::left_changing || state_path == StatePath::right_changing)
        return checkCollisionPath(path_lanechange);
    else
        return false;
}

bool LocalPath::tickCheckLaneChangeableLeft()
{
    if (getLink(idx_curr_path).linkid[8] == 'I' || getLink(idx_curr_path).linkid.substr(8, 2) == "TT")
        return false;
    if (vehicle_speed < 5.1)
        time_lane_change = 5.0;
    else
        time_lane_change = 4.0;
    max_speed_lane_change = vehicle_speed;
    double dis_lane_change_start = 3.0 + 0.25 * vehicle_speed;
    // 차선변경시 제어점이 한번에 틀어지지 않게 하기 위해 시작점 거리둠
    index_lane_change = idx_curr_path;
    idx_lane_change_start = findLookAheadPoint(idx_curr_path, dis_lane_change_start);
    double v = vehicle_speed < 5.0 ? 5.0 / 3.6 : vehicle_speed / 3.6;
    double dis_lane_change = time_lane_change * v;
    idx_lane_change_mid = findLookAheadPoint(idx_curr_path, dis_lane_change_start + dis_lane_change / 2.0);
    // 교차로면 차선변경 불가능
    double num_split = 3;
    for (int i = 0; i < num_split; i++)
    {
        WayPointIndex idx_check = findLookAheadPoint(idx_curr_path, dis_lane_change_start + dis_lane_change / num_split * (double)i);
        if (isValidIndex(idx_check))
        {
            if (getLink(idx_check).linkid[8] == 'I' || getLink(idx_check).linkid.substr(8, 2) == "TT")
                return false;
        }
    }
    if (isValidIndex(idx_lane_change_mid))
    {
        idx_side_lane = findSideLanePoint(idx_curr_path, true);
        if (isValidIndex(idx_side_lane))
        {
            idx_lane_change_end = findLookAheadPoint(idx_side_lane, dis_lane_change_start + dis_lane_change);
            if (isValidIndex(idx_lane_change_end))
            {
                if (getLink(idx_lane_change_end).linkid[8] != 'I' && getLink(idx_lane_change_end).linkid.substr(8, 2) != "TT")
                    return true;
            }
        }
    }
    return false;
}

bool LocalPath::tickCheckLaneChangeableRight()
{
    if (getLink(idx_curr_path).linkid[8] == 'I' || getLink(idx_curr_path).linkid.substr(8, 2) == "TT")
        return false;
    if (vehicle_speed < 5.1)
        time_lane_change = 5.0;
    else
        time_lane_change = 4.0;
    max_speed_lane_change = vehicle_speed;
    double dis_lane_change_start = 3.0 + 0.25 * vehicle_speed;
    // 차선변경시 제어점이 한번에 틀어지지 않게 하기 위해 시작점 거리둠
    index_lane_change = idx_curr_path;
    idx_lane_change_start = findLookAheadPoint(idx_curr_path, dis_lane_change_start);
    double v = vehicle_speed < 5.0 ? 5.0 / 3.6 : vehicle_speed / 3.6;
    double dis_lane_change = time_lane_change * v;
    idx_lane_change_mid = findLookAheadPoint(idx_curr_path, dis_lane_change_start + dis_lane_change / 2.0);
    // 교차로면 차선변경 불가능
    double num_split = 3;
    for (int i = 0; i < num_split; i++)
    {
        WayPointIndex idx_check = findLookAheadPoint(idx_curr_path, dis_lane_change_start + dis_lane_change / num_split * (double)i);
        if (isValidIndex(idx_check))
        {
            if (getLink(idx_check).linkid[8] == 'I' || getLink(idx_check).linkid.substr(8, 2) == "TT")
                return false;
        }
    }
    if (isValidIndex(idx_lane_change_mid))
    {
        idx_side_lane = findSideLanePoint(idx_curr_path, false);
        if (isValidIndex(idx_side_lane))
        {
            idx_lane_change_end = findLookAheadPoint(idx_side_lane, dis_lane_change_start + dis_lane_change);
            if (isValidIndex(idx_lane_change_end))
            {
                if (getLink(idx_lane_change_end).linkid[8] != 'I' && getLink(idx_lane_change_end).linkid.substr(8, 2) != "TT")
                    return true;
            }
        }
    }
    return false;
}

bool LocalPath::tickCheckGlobalLaneChange()
{
    // 교차로가 아니고 차선변경하는
    if (getLink(idx_curr_path).linkid[8] != 'I')
    {
        // // 차선변경하는 곳이 아닐 때
        // if (getPath(idx_curr_path).dst == -1)
        // {
        //     // src와 현재 차선이 다르면 차선 복귀.
        //     if (idx_curr_path.link != getPath(idx_curr_path).src)
        //         return true;
        // }
        // else
        // {
        //     // 차선변경해야하는 부분에서 차선변경
        //     if (idx_curr_path.link != getPath(idx_curr_path).dst)
        //     {
        //         return true;
        //     }
        // }
        // 차선변경해야하는 부분에서 차선변경
        if (getPath(idx_curr_path).dst != -1 && idx_curr_path.link != getPath(idx_curr_path).dst)
            return true;
    }
    return false;
}
bool LocalPath::tickCheckLeftChange()
{
    // 차선변경하는 곳이 아닐 때
    if (getPath(idx_curr_path).dst == -1)
    {
        // src와 현재 차선이 다르면 차선 복귀.
        if (idx_curr_path.link > getPath(idx_curr_path).src)
            return true;
        else
            return false;
    }
    else
    {
        // 차선변경해야하는 부분에서 차선변경
        if (idx_curr_path.link > getPath(idx_curr_path).dst)
            return true;
        else
            return false;
    }
    // //현재 차선이 목표 차선보다 크면 (현재 2차선, 목표 1차선) 왼쪽으로 차선변경
    // if (getPath(idx_curr_path).src < idx_curr_path.link)
    //     return true;
    // else
    //     return false;
}
bool LocalPath::tickCheckRightChange()
{
        // 차선변경하는 곳이 아닐 때
    if (getPath(idx_curr_path).dst == -1)
    {
        // src와 현재 차선이 다르면 차선 복귀.
        if (idx_curr_path.link < getPath(idx_curr_path).src)
            return true;
        else
            return false;
    }
    else
    {
        // 차선변경해야하는 부분에서 차선변경
        if (idx_curr_path.link < getPath(idx_curr_path).dst)
            return true;
        else
            return false;
    }
}
bool LocalPath::tickCheckEndLaneChange()
{
    // 차선변경 끝나는 지점의 path와 link가 같고, 차선변경 종료 지점과 3m이하 가까울때.
    if (idx_curr_path.path == idx_lane_change_end.path && idx_curr_path.link == idx_lane_change_end.link)
    {
        core_map::waypoint wp = getWayPoint(idx_lane_change_end);
        double dis = hypot(curr_pose.position.x - wp.x, curr_pose.position.y - wp.y);
        if (dis < 3.0)
            return true;
    }
    return false;
}
bool LocalPath::checkFarPath()
{
    PC_XYZI *tmp_path;
    switch (state_path)
    {
    case StatePath::lane_keeping:
        tmp_path = &path_lane_keeping;
        break;
    case StatePath::left_changing:
        tmp_path = &path_lanechange;
        break;
    case StatePath::right_changing:
        tmp_path = &path_lanechange;
        break;
    default:
        return false;
        break;
    }

    double min_dis = DBL_MAX;
    for (size_t i = 0; i < tmp_path->size(); i++)
    {
        double dis = hypot(curr_pose.position.x - tmp_path->at(i).x, curr_pose.position.y - tmp_path->at(i).y);
        min_dis = dis < min_dis ? dis : min_dis;
    }
    // 2.5미터 이상 차이나면 false
    if (min_dis > 2.5)
        return true;
    else
        return false;
}

// 미완성
bool LocalPath::checkCollisionBehindLeft()
{
    // 옆차선 직진링크(겹치는 부분)
    double dis_first = 0;
    double dis_marjin_lanechange = 15;
    // (현재속도 * 4(차선변경 총길이)) + (40(최대속도)-현재속도) * 4(차선변경 시간)
    double dis_check = (vehicle_speed * time_lane_change + (40 - vehicle_speed) * time_lane_change) / 3.6;
    // 차선변경 전 생성경로
    dis_check += 3.0 + 0.25 * vehicle_speed / 3.6;
    // 안전거리 6m
    dis_check += dis_marjin_lanechange;
    for (size_t iter_path = 0; iter_path < path_behind.size(); iter_path++)
    {
        if (path_behind.at(iter_path).waypointAry.size() < 2)
            continue;
        // 옆차선 이전링크(교차로, 또는 직진링크)
        double dis_seccond = dis_first;
        nav_msgs::Path nav_path = toNavPath(path_behind.at(iter_path));
        for (size_t i = 1; i < nav_path.poses.size(); i++)
        {
            // 첫번 째 링크는 옆차선 직진링크
            if (iter_path == 0)
                dis_first += calcDistance(nav_path.poses.at(i - 1).pose.position, nav_path.poses.at(i).pose.position);
            else
                dis_seccond += calcDistance(nav_path.poses.at(i - 1).pose.position, nav_path.poses.at(i).pose.position);
            pcl::PointXYZI p;
            p.x = nav_path.poses.at(i).pose.position.x;
            p.y = nav_path.poses.at(i).pose.position.y;
            path_debug.push_back(p);
            if (dis_seccond > dis_check || dis_first > dis_check)
                break;
            if (checkCollision(nav_path.poses.at(i).pose))
            {
                // gps to map tf 가져와서 역변환
                geometry_msgs::TransformStamped transform;
                transform.transform.translation.x = curr_pose.position.x;
                transform.transform.translation.y = curr_pose.position.y;
                transform.transform.rotation = curr_pose.orientation;
                tf2::Transform tfGpsToWorld;
                tf2::fromMsg(transform.transform, tfGpsToWorld);
                tfGpsToWorld = tfGpsToWorld.inverse();
                geometry_msgs::TransformStamped gpsToWorldTransform;
                gpsToWorldTransform.transform = tf2::toMsg(tfGpsToWorld);
                double max_x = -DBL_MAX;
                for (int i = 0; i < msg_object.polygon.at(index_object).markers.points.size(); i++)
                {
                    geometry_msgs::Pose pose_object;
                    pose_object.position.x = msg_object.polygon.at(index_object).markers.points.at(i).x;
                    pose_object.position.y = msg_object.polygon.at(index_object).markers.points.at(i).y;
                    pose_object.orientation.w = 1;
                    tf2::doTransform(pose_object, pose_object, gpsToWorldTransform);
                    max_x = max_x > pose_object.position.x ? max_x : pose_object.position.x;
                }

                if (msg_object.polygon.at(index_object).class_name == "irregular")
                {
                    if (max_x > -0.5)
                        return true;
                }
                else
                {
                    if (msg_object.polygon.at(index_object).speed > 2.0)
                    {
                        if (max_x > -5.0)
                            return true;
                        else
                        {
                            if (vehicle_speed - msg_object.polygon.at(index_object).speed * 3.6 > 10.0)
                                return false;
                            else
                                return true;
                        }
                    }
                }
            }
        }
    }
    return false;
}
// 미완성
bool LocalPath::checkCollisionBehindRight()
{
    // 옆차선 직진링크(겹치는 부분)
    double dis_first = 0;
    double dis_marjin_lanechange = 15;
    // (현재속도 * 4(차선변경 총길이)) + (40(최대속도)-현재속도) * 4(차선변경 시간)
    double dis_check = (vehicle_speed * time_lane_change + (40 - vehicle_speed) * time_lane_change) / 3.6;
    // 차선변경 전 생성경로
    dis_check += 3.0 + 0.25 * vehicle_speed / 3.6;
    // 안전거리 6m
    dis_check += dis_marjin_lanechange;
    for (size_t iter_path = 0; iter_path < path_behind.size(); iter_path++)
    {
        // 옆차선 이전링크(교차로, 또는 직진링크)
        double dis_seccond = dis_first;
        nav_msgs::Path nav_path = toNavPath(path_behind.at(iter_path));
        for (size_t i = 1; i < nav_path.poses.size(); i++)
        {
            // 첫번 째 링크는 옆차선 직진링크
            if (iter_path == 0)
                dis_first += calcDistance(nav_path.poses.at(i - 1).pose.position, nav_path.poses.at(i).pose.position);
            else
                dis_seccond += calcDistance(nav_path.poses.at(i - 1).pose.position, nav_path.poses.at(i).pose.position);
            pcl::PointXYZI p;
            p.x = nav_path.poses.at(i).pose.position.x;
            p.y = nav_path.poses.at(i).pose.position.y;
            path_debug.push_back(p);
            if (dis_seccond > dis_check || dis_first > dis_check)
                break;
            if (checkCollision(nav_path.poses.at(i).pose))
            {
                // gps to map tf 가져와서 역변환
                geometry_msgs::TransformStamped transform;
                transform.transform.translation.x = curr_pose.position.x;
                transform.transform.translation.y = curr_pose.position.y;
                transform.transform.rotation = curr_pose.orientation;
                tf2::Transform tfGpsToWorld;
                tf2::fromMsg(transform.transform, tfGpsToWorld);
                tfGpsToWorld = tfGpsToWorld.inverse();
                geometry_msgs::TransformStamped gpsToWorldTransform;
                gpsToWorldTransform.transform = tf2::toMsg(tfGpsToWorld);
                double max_x = -DBL_MAX;
                for (int i = 0; i < msg_object.polygon.at(index_object).markers.points.size(); i++)
                {
                    geometry_msgs::Pose pose_object;
                    pose_object.position.x = msg_object.polygon.at(index_object).markers.points.at(i).x;
                    pose_object.position.y = msg_object.polygon.at(index_object).markers.points.at(i).y;
                    pose_object.orientation.w = 1;
                    tf2::doTransform(pose_object, pose_object, gpsToWorldTransform);
                    max_x = max_x > pose_object.position.x ? max_x : pose_object.position.x;
                }
                if (msg_object.polygon.at(index_object).class_name == "irregular")
                {
                    if (max_x > -0.5)
                        return true;
                }
                else
                {
                    if (msg_object.polygon.at(index_object).speed > 2.0)
                    {
                        if (max_x > -5.0)
                            return true;
                        else
                        {
                            if (vehicle_speed - msg_object.polygon.at(index_object).speed * 3.6 > 10.0)
                                return false;
                            else
                                return true;
                        }
                    }
                }
            }
        }
    }
    return false;
}

bool LocalPath::genPathLaneChangeLeft()
{
    is_exist_obstacle = false;
    PC_XYZI path_changing;
    path_changing = genPathLaneChange(true);
    if (!path_changing.points.empty())
    {
        state_path = StatePath::left_changing;
        path_lanechange_base = path_changing;
        path_lanechange = path_lanechange_base;
        return true;
    }
    else
    {
        fprintf(stderr, "can't generate left lane change paht\n");
        lane_change_state = LaneChangeState::DEFAULT;
        return false;
    }
}
bool LocalPath::genPathLaneChangeRight()
{
    is_exist_obstacle = false;
    PC_XYZI path_changing;
    path_changing = genPathLaneChange(false);
    if (!path_changing.points.empty())
    {
        state_path = StatePath::right_changing;
        path_lanechange_base = path_changing;
        path_lanechange = path_lanechange_base;
        return true;
    }
    else
    {
        fprintf(stderr, "can't generate right lane change paht\n");
        lane_change_state = LaneChangeState::DEFAULT;
        return false;
    }
}
bool LocalPath::setLaneChangeLeft()
{
    lane_change_state = LaneChangeState::LEFT_CHANGING;
    return true;
}
bool LocalPath::setLaneChangeRight()
{
    lane_change_state = LaneChangeState::RIGHT_CHANGING;
    return true;
}
bool LocalPath::genPathLaneKeeping()
{
    path_lane_keeping = genGlobalFollowPath(120.0);
    state_path = StatePath::lane_keeping;
    if (!path_lane_keeping.empty())
        return true;
    else
        return false;
}
bool LocalPath::genSpeedProfileStop(const double dis_stop)
{
    PC_XYZI path_tmp;
    if (state_path == StatePath::lane_keeping)
        path_tmp = path_lane_keeping;
    else
        path_tmp = path_lanechange;

    Trajectory trj;
    double min_speed = 1.0;
    double max_speed = 40.0;
    double tmp_speed = vehicle_speed > min_speed ? vehicle_speed : min_speed;
    if (trj.speedProfileDesign(max_speed / 3.6, tmp_speed / 3.6, 0, dis_stop))
    {
        trj.getTrajectory(path_tmp);
        // 경로중에 낮은속도 선택
        if (state_path == StatePath::lane_keeping)
        {
            for (size_t iter = 0; iter < path_lane_keeping.size(); iter++)
                path_lane_keeping.at(iter).intensity = path_lane_keeping.at(iter).intensity < path_tmp.at(iter).intensity ? path_lane_keeping.at(iter).intensity : path_tmp.at(iter).intensity;
        }
        else
        {
            for (size_t iter = 0; iter < path_lanechange.size(); iter++)
                path_lanechange.at(iter).intensity = path_lanechange.at(iter).intensity < path_tmp.at(iter).intensity ? path_lanechange.at(iter).intensity : path_tmp.at(iter).intensity;
        }
    }
    else
    {
        if (state_path == StatePath::lane_keeping)
        {
            for (size_t iter = 0; iter < path_lane_keeping.size(); iter++)
                path_lane_keeping.at(iter).intensity = 0;
        }
        else
        {
            for (size_t iter = 0; iter < path_lanechange.size(); iter++)
                path_lanechange.at(iter).intensity = 0;
        }
    }
    return true;
}

PC_XYZI LocalPath::genPathLaneChange(const bool &is_left_lane)
{
    PC_XYZI path_changing;
    WayPointIndex next_point_idx_daegue = index_lane_change;
    // 차선변경시 제어점이 한번에 틀어지지 않게 하기 위해 look ahead distance만큼 직진경로 생성
    double lad = 3.0 + 0.25 * max_speed_lane_change;
    double dis_accum = 0.0;
    // 최소거리만큼 점 추가 룩어헤드 거리보다 크면 경로 추가 그만.
    while (dis_accum < lad)
    {
        WayPointIndex curr_idx = next_point_idx_daegue;
        next_point_idx_daegue = findNextPointIndex(next_point_idx_daegue);
        if (isValidIndex(next_point_idx_daegue))
        {
            dis_accum += calcDistance(curr_idx, next_point_idx_daegue);
            pcl::PointXYZI p = convertXYZI(getWayPoint(next_point_idx_daegue));
            if (!checkCollisionIdx(next_point_idx_daegue))
                path_changing.points.push_back(p);
            else
            {
                // gps to map tf 가져와서 역변환
                geometry_msgs::TransformStamped transform;
                transform.transform.translation.x = curr_pose.position.x;
                transform.transform.translation.y = curr_pose.position.y;
                transform.transform.rotation = curr_pose.orientation;
                geometry_msgs::Pose pose_object;
                pose_object.position.x = msg_object.polygon[index_object].global_x;
                pose_object.position.y = msg_object.polygon[index_object].global_y;
                pose_object.orientation.w = 1;
                tf2::Transform tfGpsToWorld;
                tf2::fromMsg(transform.transform, tfGpsToWorld);
                tfGpsToWorld = tfGpsToWorld.inverse();
                geometry_msgs::TransformStamped gpsToWorldTransform;
                gpsToWorldTransform.transform = tf2::toMsg(tfGpsToWorld);

                tf2::doTransform(pose_object, pose_object, gpsToWorldTransform);
                if (pose_object.position.x < 0)
                    return PC_XYZI();
                else
                    path_changing.points.push_back(p);
            }
        }
        else
        {
            //최소 경로 추가 불가능하면 빈 경로 보내주고 함수종료.
            return PC_XYZI();
        }
    }

    //m/s
    double v = max_speed_lane_change < 5.0 ? 5.0 / 3.6 : max_speed_lane_change / 3.6;
    geometry_msgs::Pose p[4];
    std::vector<geometry_msgs::Pose> p_2D_bezier;
    p_2D_bezier.resize(5);
    // 차선변경 시작되는 index
    p[0].position.x = getWayPoint(idx_lane_change_start).x;
    p[0].position.y = getWayPoint(idx_lane_change_start).y;
    p_2D_bezier[0].position.x = getWayPoint(idx_lane_change_start).x;
    p_2D_bezier[0].position.y = getWayPoint(idx_lane_change_start).y;

    if (isValidIndex(idx_lane_change_end))
    {
        // 차선변경시 차선변경시작과 끝 위치의 각도가 5도 이상이고, 마지막 링크가 아니면 안함.
        if (fabs(calcTangent(idx_lane_change_start) - calcTangent(idx_lane_change_end)) > (7.5 * kDegToRad))
        {
            if (idx_lane_change_start.path + 1 != msg_path_global.pathAry.size())
                return PC_XYZI();
        }
        p[3].position.x = getWayPoint(idx_lane_change_end).x;
        p[3].position.y = getWayPoint(idx_lane_change_end).y;
        p_2D_bezier[4].position.x = getWayPoint(idx_lane_change_end).x;
        p_2D_bezier[4].position.y = getWayPoint(idx_lane_change_end).y;
        p_2D_bezier[2].position.x = (p_2D_bezier[0].position.x + p_2D_bezier[4].position.x) / 2.0;
        p_2D_bezier[2].position.y = (p_2D_bezier[0].position.y + p_2D_bezier[4].position.y) / 2.0;

        double dis = hypot(p[0].position.x - p[3].position.x, p[0].position.y - p[3].position.y) / 2 * 0.618; //0.618 예전에 0.5씀
        // double dis = hypot(p_2D_bezier[0].position.x - p_2D_bezier[4].position.x, p_2D_bezier[0].position.y - p_2D_bezier[4].position.y) / 2.0 * 0.618; //0.618 예전에 0.5씀
        // double dis = time_lane_change * v;
        // 현재 자세가 차선과 평행하지 않으면 차선변경 경로 안만듬.?? 어떻게든 만들어야 되나?
        if (fabs(tf2::getYaw(curr_pose.orientation) - calcTangent(idx_lane_change_end)) > RAD2DEG(10))
            return PC_XYZI();
        double cos_start = cos(tf2::getYaw(curr_pose.orientation));
        double sin_start = sin(tf2::getYaw(curr_pose.orientation));
        geometry_msgs::Pose pt_end;
        pt_end.position.x = getWayPoint(idx_lane_change_end).x;
        pt_end.position.y = getWayPoint(idx_lane_change_end).y;
        tf2::Quaternion q;
        double cos_end = cos(calcTangent(idx_lane_change_end));
        double sin_end = sin(calcTangent(idx_lane_change_end));

        p[1].position.x = cos_start * dis + p[0].position.x;
        p[1].position.y = sin_start * dis + p[0].position.y;
        // WayPointIndex p1 = findLookAheadPoint(next_point_idx_daegue, dis);
        // if(!isValidIndex(p1))
        //     return PC_XYZI();
        // p[1].position.x = getWayPoint(p1).x;
        // p[1].position.y = getWayPoint(p1).y;
        // p_2D_bezier[1].position.x = getWayPoint(p1).x;
        // p_2D_bezier[1].position.y = getWayPoint(p1).y;

        p[2].position.x = p[3].position.x - cos_end * dis;
        p[2].position.y = p[3].position.y - sin_end * dis;
        // WayPointIndex p2 = findLookBehindPoint(idx_lane_change_end, dis);
        // if(!isValidIndex(p2))
        //     return PC_XYZI();
        // p[2].position.x = getWayPoint(p2).x;
        // p[2].position.y = getWayPoint(p2).y;
        // p_2D_bezier[3].position.x = getWayPoint(p2).x;
        // p_2D_bezier[3].position.y = getWayPoint(p2).y;

        //약 0.5m간격이 되게 커브 포인트 갯수 조절.
        int point_num = 5 * lad;
        //베지에 커브 생성
        std::vector<geometry_msgs::Pose> curve = genBezierCurve(p[0], p[1], p[2], p[3], point_num);
        // std::vector<geometry_msgs::Pose> curve = genBezierCurve(p_2D_bezier, point_num);
        v = v < speed_min_lanechange ? speed_min_lanechange : v;
        for (size_t iter = 0; iter < curve.size(); iter++)
        {
            if (iter > 0)
                dis_accum += hypot(curve.at(iter).position.x - curve.at(iter - 1).position.x, curve.at(iter).position.y - curve.at(iter - 1).position.y);
            if (!checkCollision(curve[iter]))
            {
                pcl::PointXYZI pp;
                pp.x = curve[iter].position.x;
                pp.y = curve[iter].position.y;
                pp.intensity = v * 3.6;
                path_changing.points.push_back(pp);
            }
            else
                return PC_XYZI();
        }

        //차선변경 후 따라갈 포인트 생성
        next_point_idx_daegue = idx_lane_change_end;
        //최소 10미터
        bool is_collision = false;
        // 경로의 길이는 차선변경 끝난시점으로부터 + 80m
        double dis_max_path_length = dis_accum + 80;
        while (dis_accum < dis_max_path_length)
        {
            WayPointIndex curr_idx = next_point_idx_daegue;
            next_point_idx_daegue = findNextPointIndex(next_point_idx_daegue);
            if (isValidIndex(next_point_idx_daegue))
            {
                double dis = calcDistance(curr_idx, next_point_idx_daegue);
                dis_accum += dis;
                pcl::PointXYZI p = convertXYZI(getWayPoint(next_point_idx_daegue));
                path_changing.points.push_back(p);
                // if(is_collision == true)
                // {
                //     p.intensity = 0;
                //     path_changing.points.push_back(p);
                // }
                // else
                // {
                //     // 장애물 체크
                //     if(!checkCollisionIdx(next_point_idx_daegue))
                //         path_changing.points.push_back(p);
                //     else
                //     {
                //         // gps to map tf 가져와서 역변환
                //         geometry_msgs::TransformStamped transform;
                //         transform.transform.translation.x = curr_pose.position.x;
                //         transform.transform.translation.y = curr_pose.position.y;
                //         transform.transform.rotation = curr_pose.orientation;
                //         geometry_msgs::Pose pose_object;
                //         pose_object.position.x = msg_object.polygon[index_object].global_x;
                //         pose_object.position.y = msg_object.polygon[index_object].global_y;
                //         pose_object.orientation.w = 1;
                //         tf2::Transform tfGpsToWorld;
                //         tf2::fromMsg(transform.transform, tfGpsToWorld);
                //         tfGpsToWorld = tfGpsToWorld.inverse();
                //         geometry_msgs::TransformStamped gpsToWorldTransform;
                //         gpsToWorldTransform.transform = tf2::toMsg(tfGpsToWorld);

                //         tf2::doTransform(pose_object, pose_object, gpsToWorldTransform);
                //         if (pose_object.position.x > 0)
                //         {
                //             p.intensity = 0;
                //             path_changing.points.push_back(p);
                //             is_collision = true;
                //             PC_XYZI path_tmp;
                //             path_tmp = path_changing;
                //             Trajectory trj;
                //             double min_speed = 5.0;
                //             double tmp_speed = max_speed_lane_change > min_speed ? max_speed_lane_change : min_speed;
                //             double dis_safety = 5.0;
                //             if (trj.speedProfileDesign(getLink(curr_idx).speed /3.6, tmp_speed / 3.6, 0, dis_accum - dis_safety))
                //             {
                //                 trj.getTrajectory(path_tmp);
                //                 // 경로중에 낮은속도 선택
                //                 for (size_t iter = 0; iter < path_changing.size(); iter++)
                //                     path_changing.at(iter).intensity = path_changing.at(iter).intensity < path_tmp.at(iter).intensity ? path_changing.at(iter).intensity : path_tmp.at(iter).intensity;
                //                 // return true;
                //             }
                //             else
                //             {
                //                 for (size_t iter = 0; iter < path_changing.size(); iter++)
                //                     path_changing.at(iter).intensity = 0;
                //             }
                //         }
                //         else
                //             path_changing.points.push_back(p);
                //     }
                // }
            }
            // 글로벌 경로 끝남
            else
            {
                Trajectory trj;
                double min_speed = 1.0;
                double curr_speed = vehicle_speed < min_speed ? min_speed : vehicle_speed;
                double max_speed = getLink(curr_idx).speed;
                double end_speed = 0;
                double dis_safety = 0.5;
                if (trj.speedProfileDesign(max_speed / 3.6, curr_speed / 3.6, 0, dis_accum - dis_safety))
                {
                    PC_XYZI path_trj = path_changing;
                    trj.getTrajectory(path_trj);
                    for (size_t iter_trj = 0; iter_trj < path_changing.size(); iter_trj++)
                    {
                        double v_raw = path_changing.at(iter_trj).intensity;
                        double v_trj = path_trj.at(iter_trj).intensity;
                        path_changing.at(iter_trj).intensity = v_raw < v_trj ? v_raw : v_trj;
                    }
                }
                else
                {
                    fprintf(stderr, "to fast to lanechange\n");
                    return PC_XYZI();
                }
                break;
            }
        }
        Trajectory trj;
        double curr_speed = max_speed_lane_change > speed_min_lanechange ? max_speed_lane_change : speed_min_lanechange;
        double max_speed = max_speed_lane_change > speed_min_lanechange ? max_speed_lane_change : speed_min_lanechange;
        max_speed_lane_change = max_speed_lane_change > speed_min_lanechange ? max_speed_lane_change : speed_min_lanechange;
        max_speed = max_speed < max_speed_lane_change ? max_speed : max_speed_lane_change;
        double end_speed = getLink(idx_lane_change_end).speed < max_speed ? getLink(idx_lane_change_end).speed : max_speed;
        if (trj.speedProfileDesign(max_speed / 3.6, curr_speed / 3.6, end_speed / 3.6, dis_accum))
        {
            PC_XYZI path_trj = path_changing;
            trj.getTrajectory(path_trj);
            for (size_t iter_trj = 0; iter_trj < path_changing.size(); iter_trj++)
            {
                double v_raw = path_changing.at(iter_trj).intensity;
                double v_trj = path_trj.at(iter_trj).intensity;
                path_changing.at(iter_trj).intensity = v_raw < v_trj ? v_raw : v_trj;
            }
        }
        else
        {
            fprintf(stderr, "to fast to lanechange\n");
            return PC_XYZI();
        }
        return path_changing;
    }
    else
    {
        fprintf(stderr, "Look Ahead Index Error\n");
        return PC_XYZI();
    }
}

bool LocalPath::checkEndPath()
{
    if (idx_curr_path.path + 1 == msg_path_global.pathAry.size())
        return true;
    else
        return false;
}

bool LocalPath::tickCheckFailLaneChange()
{
    if (!isValidIndex(idx_curr_path))
        return false;
    // // 차선변경하는곳인데 차선변경 거리가 안나와서 못하는경우..
    // if(getPath(idx_curr_path).changeLane)
    // {
    //     double dis_accum = 0;
    //     for (int iter_point = idx_curr_path.point; iter_point + 1 < getLink(idx_curr_path).waypointAry.size(); iter_point++)
    //     {
    //         WayPointIndex idx_curr = idx_curr_path;
    //         idx_curr.point = iter_point;
    //         WayPointIndex idx_next = idx_curr;
    //         idx_next.point++;
    //         dis_accum += calcDistance(idx_curr, idx_next);
    //     }

    //     if(dis_accum < 3.0 + 0.25 * vehicle_speed / 3.6 + vehicle_speed * time_lane_change / 3.6)
    //     {
    //         return true;
    //     }
    // }
    // else
    // {
    //     // 차선변경 하는곳이 아니고 교차로가 아니면 차선이 다르면 경로 새로요청
    //     if(!getPath(idx_curr_path).changeLane && getLink(idx_curr_path).linkid[8] != 'I')
    //     {
    //         if(getPath(idx_curr_path).src != idx_curr_path.link)
    //             return true;
    //     }
    // }
    // 차선변경 하는곳이 아니고 교차로가 아니면 차선이 다르면 경로 새로요청
    if (!getPath(idx_curr_path).changeLane)
    {
        if (!getPath(idx_curr_path).changeLane && getLink(idx_curr_path).linkid[8] != 'I')
        {
            if (getPath(idx_curr_path).src != idx_curr_path.link)
                return true;
        }
    }
    return false;
}
bool LocalPath::checkUnprotected()
{
    unprotected_link = UnprotectedLink();
    if (checkUnprotectedNext())
    {
        unprotected_link.dis_check = 10.0;
        return true;
    }
    else
        return (checkUnprotectedStraight() || checkUnprotectedTurnLeft() || checkUnprotectedTurnRight());
}
bool LocalPath::checkUnprotectedNext()
{
    unprotected_link = UnprotectedLink();
    WayPointIndex idx_next_path = findNextPathIndex(idx_curr_path);
    if (!isValidIndex(idx_next_path))
        return false;
    for (int iter_linkid = 0; iter_linkid < unprotected_straight.size(); iter_linkid++)
    {
        if (getLink(idx_next_path).linkid == unprotected_straight.at(iter_linkid).link_id)
        {
            unprotected_link = unprotected_straight.at(iter_linkid);
            return true;
        }
    }
    for (int iter_linkid = 0; iter_linkid < unprotected_left.size(); iter_linkid++)
    {
        if (getLink(idx_next_path).linkid == unprotected_left.at(iter_linkid).link_id)
        {
            unprotected_link = unprotected_left.at(iter_linkid);
            return true;
        }
    }
    for (int iter_linkid = 0; iter_linkid < unprotected_right.size(); iter_linkid++)
    {
        if (getLink(idx_next_path).linkid == unprotected_right.at(iter_linkid).link_id)
        {
            unprotected_link = unprotected_right.at(iter_linkid);
            return true;
        }
    }
    return false;
}
bool LocalPath::checkUnprotectedStraight()
{
    for (int iter_linkid = 0; iter_linkid < unprotected_straight.size(); iter_linkid++)
    {
        if (getLink(idx_curr_path).linkid == unprotected_straight.at(iter_linkid).link_id)
        {
            unprotected_link = unprotected_straight.at(iter_linkid);
            return true;
        }
    }
    return false;
}
bool LocalPath::checkUnprotectedTurnLeft()
{
    for (int iter_linkid = 0; iter_linkid < unprotected_left.size(); iter_linkid++)
    {
        if (getLink(idx_curr_path).linkid == unprotected_left.at(iter_linkid).link_id)
        {
            unprotected_link = unprotected_left.at(iter_linkid);
            return true;
        }
    }
    return false;
}
bool LocalPath::checkUnprotectedTurnRight()
{
    for (int iter_linkid = 0; iter_linkid < unprotected_right.size(); iter_linkid++)
    {
        if (getLink(idx_curr_path).linkid == unprotected_right.at(iter_linkid).link_id)
        {
            unprotected_link = unprotected_right.at(iter_linkid);
            return true;
        }
    }
    return false;
}
bool LocalPath::checkCollisionUnprotectedTurn()
{
    // 옆차선 직진링크(겹치는 부분)
    double dis_first = 0;
    // (현재속도 * 4(차선변경 총길이)) + (40(최대속도)-현재속도) * 4(차선변경 시간)
    double dis_check = 40;
    for (size_t iter_path = 0; iter_path < path_behind.size(); iter_path++)
    {
        if (path_behind.at(iter_path).waypointAry.size() < 2)
            continue;
        // 옆차선 이전링크(교차로, 또는 직진링크)
        double dis_seccond = dis_first;
        nav_msgs::Path nav_path = toNavPath(path_behind.at(iter_path));
        for (size_t i = 1; i < nav_path.poses.size(); i++)
        {
            // 첫번 째 링크는 옆차선 직진링크
            if (iter_path == 0)
                dis_first += calcDistance(nav_path.poses.at(i - 1).pose.position, nav_path.poses.at(i).pose.position);
            else
                dis_seccond += calcDistance(nav_path.poses.at(i - 1).pose.position, nav_path.poses.at(i).pose.position);
            if (dis_seccond > dis_check || dis_first > dis_check)
                break;
            pcl::PointXYZI p;
            p.x = nav_path.poses.at(i).pose.position.x;
            p.y = nav_path.poses.at(i).pose.position.y;
            path_debug.push_back(p);
            if (checkCollision(nav_path.poses.at(i).pose))
            {
                double dis_obj = hypot(path_behind.front().waypointAry.front().x - msg_object.polygon.at(index_object).global_x, path_behind.front().waypointAry.front().y - msg_object.polygon.at(index_object).global_y);
                double speed_min_obj = (msg_object.polygon.at(index_object).speed > 0.01) ? msg_object.polygon.at(index_object).speed : 0.01;
                double time_obj = dis_obj / msg_object.polygon.at(index_object).speed;
                double dis_ego = -1.0;
                double time_ego = -1.0;
                if (getLink(idx_curr_path).linkid.at(8) == 'I')
                {
                    WayPointIndex idx_half = idx_curr_path;
                    idx_half.point = getLink(idx_curr_path).waypointAry.size() / 2.0;
                    if (isValidIndex(idx_half))
                    {
                        dis_ego = hypot(getWayPoint(idx_half).x - curr_pose.position.x, getWayPoint(idx_half).y - curr_pose.position.y);
                        double speed_min = (vehicle_speed < 3.6) ? 3.6 : vehicle_speed;
                        time_ego = dis_ego / (speed_min / 3.6);
                    }
                }
                else
                {
                    WayPointIndex idx_half = idx_curr_path;
                    idx_half = findNextPathIndex(idx_half);
                    if (isValidIndex(idx_half))
                    {
                        idx_half.point = getLink(idx_curr_path).waypointAry.size() / 2.0;
                        if (isValidIndex(idx_half))
                        {
                            dis_ego = hypot(getWayPoint(idx_half).x - curr_pose.position.x, getWayPoint(idx_half).y - curr_pose.position.y);
                            double speed_min = (vehicle_speed < 3.6) ? 3.6 : vehicle_speed;
                            time_ego = dis_ego / (speed_min / 3.6);
                        }
                    }
                }
                if (time_ego > 0.0 && dis_ego > 0.0)
                {
                    // 내가 장애물보다 2초이상 늦게 도착하면 정지
                    if (time_ego > time_obj - 2)
                        return true;
                }
                // msg_object.polygon.at(index_object).global_x;
                // msg_object.polygon.at(index_object).global_x;
                // // double dis_object = calcDistance(path_behind.at(iter_path).waypointAry.at(i), path_behind.at(iter_path).waypointAry.front());
                // double dis_object = (dis_seccond > 0.01) ? dis_seccond : dis_first;
                // if ( dis_object < msg_object.polygon.at(index_object).speed * 10.0)
                //     return true;
                // else
                //     return false;
            }
        }
    }
    return false;
}
UnprotectedLink const LocalPath::getUnprotectedLink()
{
    return unprotected_link;
}

// 미완성
bool LocalPath::checkMustLaneChange()
{
    if ((getPath(idx_curr_path).changeLane && getPath(idx_curr_path).dst != idx_curr_path.link))
    {
        // 마지막 링크면
        if (idx_curr_path.path + 1 >= msg_path_global.pathAry.size())
        {
            genSpeedProfileMustLaneChange();
            return true;
        }
        // 마지막링크가 아니고 다음 path에서 좌회전이나 우회전을 해야할때.
        else
        {
            WayPointIndex idx_tmp = idx_curr_path;
            idx_tmp.path++;
            WayPointIndex idx_next_path = idx_tmp;
            idx_next_path.link = getPath(idx_tmp).src;
            if (getLink(idx_next_path).linkid.substr(8, 2) == "IR" || getLink(idx_next_path).linkid.substr(8, 2) == "IL")
            {
                int diff_lane = abs(getPath(idx_curr_path).dst - idx_curr_path.link);
                genSpeedProfileMustLaneChange(diff_lane);
                return true;
            }
        }
    }
    return false;
}

PC_XYZI LocalPath::genSpeedProfileLaneChange()
{
    double min_dis = DBL_MAX;
    int index = 0;
    PC_XYZI path_cut = path_lanechange_base;
    for (int i = 0; i + 1 < path_cut.size(); i++)
    {
        double dis = hypot(curr_pose.position.x - path_cut.at(i).x, curr_pose.position.y - path_cut.at(i).y);
        if (min_dis > dis)
        {
            min_dis = dis;
            index = i;
        }
    }
    PC_XYZI path_changing;
    path_cut.erase(path_cut.begin(), path_cut.begin() + index);
    bool is_exist_object = false;
    double dis_accum = 0;
    bool is_obstacle = false;
    for (int i = 1; i < path_cut.size(); i++)
    {
        dis_accum += hypot(path_cut.at(i).x - path_cut.at(i - 1).x, path_cut.at(i).y - path_cut.at(i - 1).y);
        if (is_obstacle)
        {
            path_cut.at(i).intensity = 0;
            continue;
        }
        // 충돌검사
        double tangent = calcTangent(path_cut, i);
        geometry_msgs::Pose pose_tmp;
        pose_tmp.position.x = path_cut.at(i).x;
        pose_tmp.position.y = path_cut.at(i).y;
        if (tangent < M_PI && tangent > -M_PI)
        {
            tf2::Quaternion q;
            q.setRPY(0, 0, tangent);
            pose_tmp.orientation = tf2::toMsg(q);
        }
        else
        {
            tf2::Quaternion q;
            q.setRPY(0, 0, 0);
            pose_tmp.orientation = tf2::toMsg(q);
        }
        if (!checkCollision(pose_tmp))
            continue;
        else
        {
            is_obstacle = true;
            path_cut.at(i).intensity = 0;
            PC_XYZI path_tmp = path_cut;
            Trajectory trj;
            // double start_speed = vehicle_speed > min_speed ? vehicle_speed : min_speed;
            double start_speed = vehicle_speed;
            double dis_safety = 5.0;
            // double max_speed = getLink(idx_lane_change_end).speed < 7.0 ? getLink(idx_lane_change_end).speed : 7.0;
            double max_speed = getLink(idx_lane_change_end).speed;
            if (trj.speedProfileDesign(max_speed / 3.6, start_speed / 3.6, 0, dis_accum - dis_safety))
            {
                trj.getTrajectory(path_tmp);
                // 경로중에 낮은속도 선택
                for (size_t iter = 0; iter < path_cut.size(); iter++)
                    path_cut.at(iter).intensity = path_cut.at(iter).intensity < path_tmp.at(iter).intensity ? path_cut.at(iter).intensity : path_tmp.at(iter).intensity;
            }
            else
            {
                for (size_t iter = 0; iter < path_cut.size(); iter++)
                    path_cut.at(iter).intensity = 0;
            }
        }
    }

    Trajectory trj;
    double start_speed = vehicle_speed;
    double max_speed = start_speed > speed_min_lanechange ? start_speed : speed_min_lanechange;
    double end_speed = getWayPoint(idx_lane_change_mid).v * 3.6 < start_speed ? getWayPoint(idx_lane_change_mid).v * 3.6 : start_speed;
    if (trj.speedProfileDesign(max_speed / 3.6, start_speed / 3.6, end_speed / 3.6, dis_accum))
    {
        PC_XYZI path_trj = path_cut;
        trj.getTrajectory(path_trj);
        for (size_t iter_trj = 0; iter_trj < path_cut.size(); iter_trj++)
        {
            if (is_exist_obstacle)
                path_cut.at(iter_trj).intensity = path_cut.at(iter_trj).intensity < path_trj.at(iter_trj).intensity ? path_cut.at(iter_trj).intensity : path_trj.at(iter_trj).intensity;
            else
                path_cut.at(iter_trj).intensity = path_cut.at(iter_trj).intensity;
        }
    }
    else
    {
        for (size_t iter_trj = 0; iter_trj < path_cut.size(); iter_trj++)
            path_cut.at(iter_trj).intensity = 0;
    }
    path_lanechange = path_cut;
    return path_cut;
}

bool LocalPath::checkObjectStaticLeftLane()
{
    WayPointIndex idx_start_check = findSideLanePoint(idx_curr_path, true);
    if (!isValidIndex(idx_start_check))
        return false;
    WayPointIndex idx_next = findNextPointIndex(idx_start_check);
    double speed_object_min = DBL_MAX;
    std::vector<int> indice_object_tmp;
    for (int iter_point = idx_start_check.point; iter_point + 1 < getLink(idx_start_check).waypointAry.size(); iter_point++)
    {
        idx_next = findNextPointIndex(idx_next);
        if (isValidIndex(idx_next))
        {
            if (checkCollisionIdx(idx_next))
            {
                if (!indice_object_tmp.empty())
                {
                    if (indice_object_tmp.back() == index_object)
                        continue;
                }
                indice_object_tmp.push_back(index_object);
                double speed_object = msg_object.polygon.at(index_object).speed;
                speed_object_min = (speed_object_min < speed_object) ? speed_object_min : speed_object;
            }
        }
        else
            break;
    }
    if (speed_object_min < 3.0)
    {
        // 다음 링크가
        WayPointIndex idx_next_path = findNextPathIndex(idx_curr_path);
        if (isValidIndex(idx_next_path))
        {
            // 교차로이고
            if (getLink(idx_next_path).linkid[8] == 'I')
            {
                // 비보호가 아니고
                if (!isUnprotectedIntersection(getLink(idx_next_path).linkid))
                {
                    // 차선변경을 해야할때
                    if (getPath(idx_curr_path).changeLane && getPath(idx_curr_path).dst != idx_curr_path.link)
                    {
                        // 비정형 장애물이 아니면
                        double dis_max = 0.0;
                        for (int iter_object = 0; iter_object < indice_object_tmp.size(); iter_object++)
                        {
                            if (msg_object.polygon.at(indice_object_tmp.at(iter_object)).class_name == "irregular")
                                return true;
                            else
                            {
                                double dis = hypot(getWayPoint(idx_next_path).x - msg_object.polygon.at(iter_object).global_x, getWayPoint(idx_next_path).y - msg_object.polygon.at(iter_object).global_y);
                                dis_max = dis_max < dis ? dis : dis_max;
                            }
                        }
                        // 정지선과 10이내의 장애물이라면 정지장애물 뒤로 차선변경 가능
                        if (dis_max < 10)
                            return false;
                    }
                }
            }
        }
        // 앞에 장애물이 있으면
        if (index_object_front != -1 && index_object_front < msg_object.polygon.size())
        {
            // 옆 장애물의 속도가 3m/s 이상 빠르지않으면 차선변경 안함
            if (speed_object_min < msg_object.polygon.at(index_object_front).speed + 3.0)
                return true;
        }
        else
            return true;
    }
    return false;
}
bool LocalPath::checkObjectStaticRightLane()
{
    WayPointIndex idx_start_check = findSideLanePoint(idx_curr_path, false);
    if (!isValidIndex(idx_start_check))
        return false;
    WayPointIndex idx_next = findNextPointIndex(idx_start_check);
    double speed_object_min = DBL_MAX;
    std::vector<int> indice_object_tmp;
    for (int iter_point = idx_start_check.point; iter_point + 1 < getLink(idx_start_check).waypointAry.size(); iter_point++)
    {
        idx_next = findNextPointIndex(idx_next);
        if (isValidIndex(idx_next))
        {
            if (checkCollisionIdx(idx_next))
            {
                if (!indice_object_tmp.empty())
                {
                    if (indice_object_tmp.back() == index_object)
                        continue;
                }
                indice_object_tmp.push_back(index_object);
                double speed_object = msg_object.polygon.at(index_object).speed;
                speed_object_min = (speed_object_min < speed_object) ? speed_object_min : speed_object;
            }
        }
        else
            break;
    }
    if (speed_object_min < 3.0)
    {
        // 다음 링크가
        WayPointIndex idx_next_path = findNextPathIndex(idx_curr_path);
        if (isValidIndex(idx_next_path))
        {
            // 교차로이고
            if (getLink(idx_next_path).linkid[8] == 'I')
            {
                // 비보호가 아니고
                if (!isUnprotectedIntersection(getLink(idx_next_path).linkid))
                {
                    // 차선변경을 해야할때
                    if (getPath(idx_curr_path).changeLane && getPath(idx_curr_path).dst != idx_curr_path.link)
                    {
                        // 비정형 장애물이 아니면
                        double dis_max = 0.0;
                        for (int iter_object = 0; iter_object < indice_object_tmp.size(); iter_object++)
                        {
                            if (msg_object.polygon.at(indice_object_tmp.at(iter_object)).class_name == "irregular")
                                return true;
                            else
                            {
                                double dis = hypot(getWayPoint(idx_next_path).x - msg_object.polygon.at(iter_object).global_x, getWayPoint(idx_next_path).y - msg_object.polygon.at(iter_object).global_y);
                                dis_max = dis_max < dis ? dis : dis_max;
                            }
                        }
                        // 정지선과 10이내의 장애물이라면 정지장애물 뒤로 차선변경 가능
                        if (dis_max < 10)
                            return false;
                    }
                }
            }
        }
        // 앞에 장애물이 있으면
        if (index_object_front != -1 && index_object_front < msg_object.polygon.size())
        {
            // 옆 장애물의 속도가 3m/s 이상 빠르지않으면 차선변경 안함
            if (speed_object_min < msg_object.polygon.at(index_object_front).speed + 3.0)
                return true;
        }
        else
            return true;
    }
    return false;
}

bool LocalPath::checkObjectnearGoal()
{
    if (path_lane_keeping.empty())
        return false;
    for (int iter_point = path_lane_keeping.size() - 1; iter_point >= 0; iter_point--)
    {
        geometry_msgs::Pose pose;
        pose.position.x = path_lane_keeping.at(iter_point).x;
        pose.position.y = path_lane_keeping.at(iter_point).y;
        tf2::Quaternion q;
        q.setRPY(0, 0, calcTangent(path_lane_keeping, iter_point));
        pose.orientation = tf2::toMsg(q);
        if (checkCollision(pose))
        {
            geometry_msgs::Pose pose_obj;
            pose_obj.position.x = msg_object.polygon.at(index_object).global_x;
            pose_obj.position.y = msg_object.polygon.at(index_object).global_y;
            double dis = calcDistance(pose_obj.position, goal_pose.position);
            if (dis < 20 && msg_object.polygon.at(index_object).class_name != "irregular")
                return true;
        }
    }
    return false;
}

core_map::waypoint const LocalPath::getPointLeftLane()
{
    WayPointIndex idx_left_lane = findSideLanePoint(idx_curr_path, true);
    if (isValidIndex(idx_left_lane))
        return getWayPoint(idx_left_lane);
    else
    {
        core_map::waypoint not_found_point;
        not_found_point.x = -1;
        not_found_point.y = -1;
        return not_found_point;
    }
}

core_map::waypoint const LocalPath::getPointRightLane()
{
    WayPointIndex idx_right_lane = findSideLanePoint(idx_curr_path, false);
    if (isValidIndex(idx_right_lane))
        return getWayPoint(idx_right_lane);
    else
    {
        core_map::waypoint not_found_point;
        not_found_point.x = -1;
        not_found_point.y = -1;
        return not_found_point;
    }
}

void LocalPath::genSpeedProfileMustLaneChange(const int scale)
{
    double accum_dis = 0.01;
    for (int iter_point = idx_curr_path.point + 1; iter_point < getLink(idx_curr_path).waypointAry.size(); iter_point++)
    {
        WayPointIndex idx_pre_point = idx_curr_path;
        WayPointIndex idx_cur_point = idx_curr_path;
        idx_pre_point.point = iter_point - 1;
        idx_cur_point.point = iter_point;
        accum_dis += calcDistance(idx_cur_point, idx_pre_point);
    }

    double min_speed = 1.0;
    Trajectory trj;
    if (!path_lane_keeping.empty())
    {
        PC_XYZI path_trj;
        path_trj = path_lane_keeping;
        double tmp_speed = vehicle_speed > min_speed ? vehicle_speed : min_speed;
        double dis_safety = 16.0 * scale;
        bool bool_slow_down = false;
        // 속도를 줄여야하면 최대 가속도를 낮춰서 목적지에 천천히 도달하게함.
        if (bool_slow_down)
            trj.setMaxAcc(0.5);
        if (trj.speedProfileDesign(getLink(idx_curr_path).speed / 3.6, tmp_speed / 3.6, 0, accum_dis - dis_safety))
        {
            trj.getTrajectory(path_trj);
            // 경로중에 낮은속도 선택
            for (size_t iter = 0; iter < path_lane_keeping.size(); iter++)
            {
                path_lane_keeping.at(iter).intensity = path_lane_keeping.at(iter).intensity < path_trj.at(iter).intensity ? path_lane_keeping.at(iter).intensity : path_trj.at(iter).intensity;
            }
        }
        else
        {
            for (size_t iter = 0; iter < path_lane_keeping.size(); iter++)
                path_lane_keeping.at(iter).intensity = 0;
        }
    }
}

bool LocalPath::checkIrregularFront()
{
    return is_irr_same_link;
}

bool LocalPath::checkMeNearGoal()
{
    WayPointIndex tmp = idx_curr_path;
    double dis_accum = 0.0;
    while (true)
    {
        for (int iter_point = tmp.point; iter_point < getLink(tmp).waypointAry.size(); iter_point++)
        {
            if (iter_point == 0)
                continue;
            dis_accum += calcDistance(WayPointIndex(tmp.path, tmp.link, iter_point - 1), WayPointIndex(tmp.path, tmp.link, iter_point));
        }
        tmp = findNextPathIndex(tmp);
        if (!isValidIndex(tmp))
            break;
    }
    if (dis_accum < 100)
        return true;
    else
        return false;
}

void LocalPath::genBsdPoints()
{
    UnprotectedLink unprotect;
    std::vector<std::string> straight;
    core_map::point a;
    double dis = 0.0;
    double time = 0.0;
    // 화장실앞 1차선
    unprotect.link_id = "155M0039IZ0101";
    unprotect.bsd_points.clear();
    a.x = 24.0879;
    a.y = 175.945;
    unprotect.bsd_points.push_back(a);
    a.x = 23.9345;
    a.y = 178.019;
    unprotect.bsd_points.push_back(a);
    unprotected_straight.push_back(unprotect);

    // 화장실앞 2차선
    unprotect.link_id = "155M0039IZ0202";
    unprotect.bsd_points.clear();
    a.x = 24.0879;
    a.y = 175.945;
    unprotect.bsd_points.push_back(a);
    a.x = 23.9345;
    a.y = 178.019;
    unprotect.bsd_points.push_back(a);
    unprotected_straight.push_back(unprotect);

    // 화장실 건너편 1차선
    unprotect.link_id = "155M0058IZ0101";
    unprotect.bsd_points.clear();
    // 오른쪽에서 좌회전하는 1차선
    a.x = 27.8442;
    a.y = 178.118;
    unprotect.bsd_points.push_back(a);
    // 오른쪽에서 좌회전하는 2차선
    a.x = 27.6509;
    a.y = 179.068;
    unprotect.bsd_points.push_back(a);
    // 오른쪽에서 우회전하는 차량 검사
    a.x = 28.7484;
    a.y = 185.249;
    unprotect.bsd_points.push_back(a);
    // 건너편에서 좌회전하는 차
    a.x = 24.7626;
    a.y = 183.018;
    unprotect.bsd_points.push_back(a);
    unprotected_straight.push_back(unprotect);

    // 화장실 건너편 2차선
    unprotect.link_id = "155M0058IZ0202";
    unprotect.bsd_points.clear();
    // 오른쪽에서 좌회전하는 차량 검사
    a.x = 29.4748;
    a.y = 179.212;
    unprotect.bsd_points.push_back(a);
    // 오른쪽에서 우회전하는 차량 검사
    a.x = 30.4219;
    a.y = 181.899;
    unprotect.bsd_points.push_back(a);
    // 건너편에서 좌회전하는 차
    a.x = 26.7784;
    a.y = 180.412;
    unprotect.bsd_points.push_back(a);
    unprotected_straight.push_back(unprotect);

    // 터널 후 우회전 골목 -> 직진
    unprotect.link_id = "155M0071IZ0101";
    unprotect.bsd_points.clear();
    // 오른쪽에서 좌회전하는 차량 검사
    a.x = 168.07;
    a.y = 154.23;
    unprotect.bsd_points.push_back(a);
    unprotected_straight.push_back(unprotect);

    // 직진 -> 터널 후 우회전 골목
    unprotect.link_id = "155M0074IZ0101";
    unprotect.bsd_points.clear();
    // 오른쪽에서 좌회전하는 차량 검사
    a.x = 163.25;
    a.y = 152.62;
    unprotect.bsd_points.push_back(a);
    // 오른쪽에서 우회전하는 차량 검사
    a.x = 162.71;
    a.y = 151.36;
    unprotect.bsd_points.push_back(a);
    // 반대편에서 좌회전하는 차량 검사
    a.x = 168.01;
    a.y = 151.34;
    unprotect.bsd_points.push_back(a);
    unprotected_straight.push_back(unprotect);

    // HD없는 쪽 -> 직진 큰길
    unprotect.link_id = "155M0405IZ0101";
    unprotect.bsd_points.clear();
    // 왼쪽에서 좌회전하는 1차선
    a.x = 582.02;
    a.y = -60.49;
    unprotect.bsd_points.push_back(a);
    unprotected_straight.push_back(unprotect);

    // 큰길 - >직진 HD없는 쪽
    unprotect.link_id = "155M0121IZ0101";
    unprotect.bsd_points.clear();
    // 오른쪽에서 좌회전하는 1차선
    a.x = 577.20;
    a.y = -63.66;
    unprotect.bsd_points.push_back(a);
    // 오른쪽에서 우회전하는 1차선
    a.x = 577.71;
    a.y = -66.99;
    unprotect.bsd_points.push_back(a);
    // 반대편에서 좌회전하는 1차선
    a.x = 582.23;
    a.y = -69.0283;
    unprotect.bsd_points.push_back(a);
    unprotected_straight.push_back(unprotect);

    // 가드레일 전 1차선
    unprotect.link_id = "155M0102IZ0101";
    unprotect.bsd_points.clear();
    // 오른쪽에서 우회전하는 1차선
    a.x = 721.705;
    a.y = -10.17;
    unprotect.bsd_points.push_back(a);
    unprotected_straight.push_back(unprotect);

    // 가드레일 전 2차선
    unprotect.link_id = "155M0102IZ0202";
    unprotect.bsd_points.clear();
    // 오른쪽에서 우회전하는 1차선
    a.x = 718.01;
    a.y = -13.43;
    unprotect.bsd_points.push_back(a);
    // 오른쪽에서 우회전하는 2차선
    a.x = 719.68;
    a.y = -13.00;
    unprotect.bsd_points.push_back(a);
    unprotected_straight.push_back(unprotect);

    // 화장실에서 -> 좌회전 DIP골목
    unprotect.link_id = "155M0039IL0101";
    unprotect.bsd_points.clear();
    // 반대차선 1차선
    a.x = 24.6393;
    a.y = 173.8;
    unprotect.bsd_points.push_back(a);
    // 반대차선 2차선
    a.x = 27.67;
    a.y = 173.62;
    unprotect.bsd_points.push_back(a);
    // 왼쪽에서 좌회전하는 차량 검사
    a.x = 29.4748;
    a.y = 179.212;
    unprotect.bsd_points.push_back(a);
    unprotected_left.push_back(unprotect);

    // DIP골목 -> 화장실 앞 1차선
    unprotect.link_id = "155M0086IL0101";
    unprotect.bsd_points.clear();
    // 오른쪽에서 직진하는 1차선
    a.x = 22.104;
    a.y = 179.252;
    unprotect.bsd_points.push_back(a);
    // 오른쪽에서 좌회전하는 차
    a.x = 25.4041;
    a.y = 181.994;
    unprotect.bsd_points.push_back(a);
    // 왼쪽에서 직진하는 1차선
    a.x = 24.6393;
    a.y = 173.8;
    unprotect.bsd_points.push_back(a);
    // 왼쪽에서 직진하는 2차선
    a.x = 27.67;
    a.y = 173.62;
    unprotect.bsd_points.push_back(a);
    unprotected_left.push_back(unprotect);

    // 화장실 앞 2차선
    unprotect.link_id = "155M0086IL0102";
    unprotect.bsd_points.clear();
    // 오른쪽에서 직진하는 2차선
    a.x = 19.01;
    a.y = 179.21;
    unprotect.bsd_points.push_back(a);
    // 오른쪽에서 직진하는 1차선
    a.x = 22.104;
    a.y = 179.252;
    unprotect.bsd_points.push_back(a);
    // 오른쪽에서 좌회전하는 차
    a.x = 25.4041;
    a.y = 181.994;
    unprotect.bsd_points.push_back(a);
    // 왼쪽에서 직진하는 1차선
    a.x = 24.6393;
    a.y = 173.8;
    unprotect.bsd_points.push_back(a);
    // 왼쪽에서 직진하는 2차선
    a.x = 27.67;
    a.y = 173.62;
    unprotect.bsd_points.push_back(a);
    unprotected_left.push_back(unprotect);

    // DIP, 공터주자창 사이 -> 좌회전
    unprotect.link_id = "155M0085IL0101";
    unprotect.bsd_points.clear();
    // 오른쪽에서 좌회전하는 차
    a.x = 167.863;
    a.y = 151.654;
    unprotect.bsd_points.push_back(a);
    // 오른쪽서 직진하는 1차선
    a.x = 169.7;
    a.y = 153.507;
    unprotect.bsd_points.push_back(a);
    // 왼쪽에서 직진하는 1차선
    a.x = 167.38;
    a.y = 157.37;
    unprotect.bsd_points.push_back(a);
    unprotected_left.push_back(unprotect);

    // 터널 후 우회전 골목 -> 좌회전 DIP 골목
    unprotect.link_id = "155M0071IL0101";
    unprotect.bsd_points.clear();
    // 반대편에서 우회전하는 1차선
    a.x = 165.42;
    a.y = 157.87;
    unprotect.bsd_points.push_back(a);
    // 반대편에서 직진하는 1차선
    a.x = 167.38;
    a.y = 157.37;
    unprotect.bsd_points.push_back(a);
    // 왼쪽에서 좌회전하는 1차선
    a.x = 163.25;
    a.y = 152.62;
    unprotect.bsd_points.push_back(a);
    unprotected_left.push_back(unprotect);

    // astar 에서 큰길로 좌회전
    unprotect.link_id = "155M0118IL0101";
    unprotect.bsd_points.clear();
    // 오른쪽에서 직진하는 1차선
    a.x = 583.63;
    a.y = -61.24;
    unprotect.bsd_points.push_back(a);
    // 오른쪽에서 좌회전하는 1차선
    a.x = 580.48;
    a.y = -65.86;
    unprotect.bsd_points.push_back(a);
    // 왼쪽에서 직진하는 1차선
    a.x = 580.92;
    a.y = -58.49;
    unprotect.bsd_points.push_back(a);
    unprotected_left.push_back(unprotect);

    // HD없는곳에서 나오는 좌회전
    unprotect.link_id = "155M0405IL0101";
    unprotect.bsd_points.clear();
    // 반대편에서 우회전하는 1차선
    a.x = 577.23;
    a.y = -60.68;
    unprotect.bsd_points.push_back(a);
    // 반대편에서 직진하는 1차선
    a.x = 580.46;
    a.y = -62.29;
    unprotect.bsd_points.push_back(a);
    // 왼쪽에서 좌회전하는 1차선
    a.x = 574.28;
    a.y = -64.24;
    unprotect.bsd_points.push_back(a);
    unprotected_left.push_back(unprotect);

    // DIP 골목 -> 우회전 1차선
    unprotect.link_id = "155M0086IR0101";
    unprotect.bsd_points.clear();
    // 오른쪽에서 직진하는 1차선
    a.x = 26.32;
    a.y = 182.98;
    unprotect.bsd_points.push_back(a);
    // 오른쪽에서 직진하는 2차선
    a.x = 28.89;
    a.y = 180.35;
    unprotect.bsd_points.push_back(a);
    unprotected_right.push_back(unprotect);

    // DIP 골목 -> 우회전2차선
    unprotect.link_id = "155M0086IR0102";
    unprotect.bsd_points.clear();
    // 오른쪽에서 직진하는 2차선
    a.x = 28.89;
    a.y = 180.35;
    unprotect.bsd_points.push_back(a);
    unprotected_right.push_back(unprotect);

    // DIP, 공터주자창 사이 -> 우회전
    unprotect.link_id = "155M0085IR0101";
    unprotect.bsd_points.clear();
    // 왼쪽에서 직진하는 1차선
    a.x = 165.96;
    a.y = 149.62;
    unprotect.bsd_points.push_back(a);
    unprotected_right.push_back(unprotect);
    // 우회전 -> DIP, 공터주자창 사이
    unprotect.link_id = "155M0074IR0101";
    unprotect.bsd_points.clear();
    // 반대편에서 좌회전 1차선
    a.x = 164.92;
    a.y = 154.77;
    unprotect.bsd_points.push_back(a);
    unprotected_right.push_back(unprotect);

    // 큰길에서 ASTAR 하는곳으로 들어가는 우회전
    unprotect.link_id = "155M0121IR0101";
    unprotect.bsd_points.clear();
    // 반대편에서 좌회전하는 1차선
    a.x = 577.26;
    a.y = -63.14;
    unprotect.bsd_points.push_back(a);
    unprotected_right.push_back(unprotect);

    // ASTAR -> HD맵 없는쪽으로 가는 우회전
    unprotect.link_id = "155M0118IR0101";
    unprotect.bsd_points.clear();
    // 왼쪽에서 직진하는 1차선
    a.x = 579.86;
    a.y = -67.02;
    unprotect.bsd_points.push_back(a);
    unprotected_right.push_back(unprotect);

    // HD맵에서 -> 우회전 가드레일쪽 1차선
    unprotect.link_id = "155M0412IR0101";
    unprotect.bsd_points.clear();
    // 왼쪽에서 직진하는 1차선
    a.x = 721.08;
    a.y = -7.97;
    unprotect.bsd_points.push_back(a);
    // 왼쪽에서 직진하는 2차선
    a.x = 717.76;
    a.y = -10.46;
    unprotect.bsd_points.push_back(a);
    unprotected_right.push_back(unprotect);

    // HD맵에서 -> 우회전 가드레일쪽 2차선
    unprotect.link_id = "155M0412IR0102";
    unprotect.bsd_points.clear();
    // 왼쪽에서 직진하는 2차선
    a.x = 719.63;
    a.y = -10.73;
    unprotect.bsd_points.push_back(a);
    unprotected_right.push_back(unprotect);
}

bool LocalPath::isUnprotectedIntersection(std::string linkid)
{
    if (linkid.substr(0, 9) == "155M0058I")
        return true;
    else if (linkid.substr(0, 9) == "155M0039I")
        return true;
    else if (linkid.substr(0, 9) == "155M0102I")
        return true;
    else
        return false;
}

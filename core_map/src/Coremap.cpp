#include "Coremap.h"
#include "crc16.h"

enum Mouse
{
    FIRST_CLICK,
    SECOND_CLICK
};

enum
{
    WAVE_ONLY,
    BAG_TEST,
    TEST_V2X,
    REAL_POS,
    MAKE_TABLE,
    SIMULATE
};

bool compareMission(Mission a, Mission b)
{
    if(a.missionID <= b.missionID)
        return true;
    else
        return false;
}

Coremap::Coremap(ros::NodeHandle _nh) : tf_listener(tf_buffer)
{
    nh = _nh;
    nh.param<int>("/run_type", runType, BAG_TEST);
    nh.param<bool>("/run_opt", runOptimizer, false);
    mouse_flag = Mouse::FIRST_CLICK;

    m_fsmState = state::MISSION_INIT;
    m_penalty = false;
    m_sequence = 0;
    m_latitude = 0.0;
    m_longitude = 0.0;
    m_elevation = 0.0;
    fprintf(stderr, "- State Change : MISSION_INIT\n\n");

    m_v2xMAP.resize(22);
    m_v2xSPAT.resize(22);
    offset_x = 302533.174487;
    offset_y = 4124215.34631;
    m_globalPos.x = 0.0;
    m_globalPos.y = 0.0;
    initColor();

    clicked_point_sub = nh.subscribe("/move_base_simple/goal", 1, &Coremap::clickedPointCallback, this);
    v2x_clear_sub = nh.subscribe("/core/control/mission_state", 1, &Coremap::missionStateCallback, this);
    
    if (runType == BAG_TEST)
    {
       wave_sub = nh.subscribe("/core/v2x/wave_packet", 1, &Coremap::waveCallback, this);
    }
    else if(runType == SIMULATE)
    {
        // SIMULATE POS
        //  m_globalPos.x = 680.033;
        //  m_globalPos.y = 3.7508;
        //
        lte_sub = nh.subscribe("/simulate/lte_packet", 1, &Coremap::lteCallback, this);
        getting_on_sub = nh.subscribe("/simulate/getting_on", 1, &Coremap::gettingOnCallback, this);
        getting_off_sub = nh.subscribe("/simulate/getting_off", 1, &Coremap::gettingOffCallback, this);
        sim_call_req_srv = nh.serviceClient<core_map::SimCallReq>("/simulate/call_request");

        std::unique_lock<std::mutex> lock_tcp(m_mutex);
        m_fsmState = state::WAIT_ORDER;
        fprintf(stderr, "- State Change : WAIT_ORDER\n\n");
        lock_tcp.unlock();
    }
    bestpos_sub = nh.subscribe("/bestpos", 1, &Coremap::bestposCallback, this);
    inspvax_sub = nh.subscribe("/inspvax", 1, &Coremap::inspvaxCallback, this);
    avante_sub = nh.subscribe("/core/control/avante_data", 1, &Coremap::avanteCallback, this);

    //V2X
    rviz_v2x_call_pub = nh.advertise<Markerarray>("/core/v2x/call_array", 1);
    rviz_v2x_map_pub = nh.advertise<Markerarray>("/core/v2x/map", 1);
    irr_pub = nh.advertise<Markerarray>("/core/v2x/irregular_loc", 1);
    mission_pub = nh.advertise<std_msgs::String>("/core/v2x/lte_packet", 1); // V2X LTE 디버깅 bag 취득용
    v2x_pub = nh.advertise<std_msgs::String>("/core/v2x/wave_packet", 1);          // V2X Wave 디버깅 bag 취득용
    close_traffic_light_pub = nh.advertise<core_map::traffic_light_info>("/core/v2x/traffic_signal", 1);
    //map
    rviz_map_pub = nh.advertise<Markerarray>("/core/map/hdmap_array", 10);
    path_pub = nh.advertise<core_map::global_path>("/core/map/global_path", 1); //Clicked Point 경로를 위한 publish
    real_path_pub = nh.advertise<sensor_msgs::PointCloud2>("/core/map/real_path", 1);
    fsm_pub = nh.advertise<std_msgs::Int8>("/core/map/mission_fsm", 1);
    //srv
    bsdSrv = nh.advertiseService("/core/map/bsd_srv", &Coremap::srvBsdLink, this);
    emergencyPathSrv = nh.advertiseService("/core/map/emergency_path_srv", &Coremap::srvEmergencyPath, this);
    checkProfitSrv = nh.advertiseService("/core/map/check_profit_srv", &Coremap::srvCheckProfit, this);
    intersectionSrv = nh.advertiseService("/core/map/intersection_srv", &Coremap::srvIntersection, this);
    frontIrrSrv = nh.advertiseService("/core/map/front_irr_srv", &Coremap::srvFrontIrr, this);

    std::string mapPath;
    nh.param<std::string>("/hdmap_path", mapPath, "/home/a/k_city_ws/src/core_map/HDMAP");

    std::string daguePath = mapPath + std::string("/K-City");
    hdmap.readHDMap(daguePath);
    std::vector<c1_node> node;
    std::vector<a3_link> link;
    std::vector<a1_lane> lane;
    std::vector<a2_stop> stop;

    hdmap.getHDMap(node, link, lane, stop);
    visualizeMap(node, link, lane, stop);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
//    pcl_map = nh.advertise<sensor_msgs::PointCloud2>("/pcl_map", 1);
//    pcl::PointCloud<pcl::PointXYZI> laneChange;
//    for(int i = 0; i < link.size(); i++)
//    {
//        if(link[i].linkid[8] != 'C')
//        {
//            for(int j = 0; j < link[i].waypoint.size(); j++)
//            {
//                pcl::PointXYZI p;
//                p.x = link[i].waypoint[j].x;
//                p.y = link[i].waypoint[j].y;
//                p.z = 0;

//                bool cl = false;
//                if(link[i].waypoint[j].change_left[0] != '1')
//                {
//                    if(link[i].waypoint[j].change_left[2] == '2')
//                        cl = true;
//                }
//                bool cr = false;
//                if(link[i].waypoint[j].change_right[0] != '1')
//                {
//                    if(link[i].waypoint[j].change_right[2] == '2')
//                        cr = true;
//                }

//                if(cl)
//                {
//                    if(cr)
//                        p.intensity = 255;
//                    else
//                        p.intensity = 64;
//                }
//                else
//                {
//                    if(cr)
//                        p.intensity = 128;
//                    else
//                        p.intensity = 0;
//                }

//                laneChange.push_back(p);
//            }
//        }
//    }
//    sensor_msgs::PointCloud2 out;
//    pcl::toROSMsg(laneChange, out);
//    out.header.frame_id = "world";
//    pcl_map.publish(out);

//    ros::Rate loop_rate(15);
//    while (ros::ok())
//    {
//        if (pcl_map.getNumSubscribers() > 0)
//        {
//            pcl_map.publish(out);
//            break;
//        }
//        else
//            loop_rate.sleep();
//    }
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
//    pcl_map = nh.advertise<sensor_msgs::PointCloud2>("/pcl_map", 1);
//    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
//    if (pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/a/ss_full_0917.pcd", *cloud) != -1) //* load the file
//    {
//        sensor_msgs::PointCloud2 out;
//        pcl::toROSMsg(*cloud, out);
//        out.header.frame_id = "world";
//        pcl_map.publish(out);
//    }
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////

    std::string tablePath = mapPath + std::string("/Table");
    if (runType == MAKE_TABLE)
    {
        dijkstra.initDijkstra(&hdmap, &table);
        makeTable(tablePath);
    }
    else
    {
        initTable(tablePath);
        dijkstra.initDijkstra(&hdmap, &table);
        multiCall.initMulticall(&hdmap, &table);
//        m_roadInfo_th = std::thread(&Coremap::roadInfo_thread, this);
        if (runType != BAG_TEST)
        {
            initWAVE();
            if (runType != WAVE_ONLY && runType != SIMULATE)
                initLTE();
        }
    }
}

void Coremap::initColor()
{
    std_msgs::ColorRGBA temp0;
    temp0.a = 0.8f;
    temp0.r = 1.0;
    std_msgs::ColorRGBA temp1;
    temp1.a = 0.8f;
    temp1.g = 1.0;
    std_msgs::ColorRGBA temp2;
    temp2.a = 0.8f;
    temp2.b = 1.0;
    std_msgs::ColorRGBA temp3;
    temp3.a = 0.8f;
    temp3.g = 0.5;
    temp3.b = 0.5;
    std_msgs::ColorRGBA temp4;
    temp4.a = 0.8f;
    temp4.r = 0.5;
    temp4.b = 0.5;
    std_msgs::ColorRGBA temp5;
    temp5.a = 0.8f;
    temp5.r = 0.5;
    temp5.g = 0.5;
    colors.push_back(temp5);
    colors.push_back(temp4);
    colors.push_back(temp3);
    colors.push_back(temp2);
    colors.push_back(temp1);
    colors.push_back(temp0);
    colorIndex = 0;
}
void Coremap::initWAVE()
{
    //WAVE Connection
    bool flagWave = false;
    std::string obu_ip = "192.168.10.10";
    uint32_t obu_port = 15130;
    memset(&addrWAVE, 0, sizeof(addrWAVE));
    addrWAVE.sin_family = AF_INET;
    addrWAVE.sin_port = htons(obu_port);
    // String IP 주소 변환
    if (inet_pton(AF_INET, obu_ip.c_str(), &addrWAVE.sin_addr) <= 0)
    {
        fprintf(stderr, "WAVE Error, inet_pton\n");
    }
    // 클라이언트 Socket FD 생성
    if ((sockWAVE = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
    {
        fprintf(stderr, "WAVE Error, socket create failed\n");
    }

    // 클라이언트, 서버 TCP 연결
    int connected = -1;
    if ((connected = connect(sockWAVE, (const sockaddr *)&addrWAVE, sizeof addrWAVE)) < 0)
    {
        fprintf(stderr, "WAVE Error, socket connect failed : %d\n", connected);
    }
    else
    {
        fprintf(stderr, "WAVE Connected!!\n");
        flagWave = true;
    }
    if (flagWave)
        m_wave_th = std::thread(&Coremap::wave_thread, this);
}

void Coremap::initLTE()
{
    //LTE Connection
    bool flagLTE = false;
    sockLTE = socket(PF_INET, SOCK_STREAM, 0);
    if (sockLTE == -1)
    {
        fprintf(stderr, "LTE Soket Error!!\n");
    }
    else
    {
        memset(&addrLTE, 0, sizeof(addrLTE));
        addrLTE.sin_family = AF_INET;
        addrLTE.sin_addr.s_addr = inet_addr("172.20.0.1");
        addrLTE.sin_port = htons(17001);
        if (connect(sockLTE, (struct sockaddr *)&addrLTE, sizeof(addrLTE)) == -1)
        {
            fprintf(stderr, "LTE Connection Error!!\n");
        }
        else
        {
            fprintf(stderr, "LTE Connected!!\n");
            flagLTE = true;
        }
    }
    if (flagLTE)
    {
        if(runType == REAL_POS)
        {
            while (1)
            {
                if (m_latitude == 0.0)
                {
                    fprintf(stderr, "GPS is not initialized!\n");
                    ros::spinOnce();
                    ros::Duration(0.5).sleep();
                }
                else
                    break;
            }
        }

        access_mission_server();
        m_lte_recv_th = std::thread(&Coremap::lte_recv_thread, this);
    }
}

void Coremap::resetMarker(Markerarray &marker, ros::Publisher pub)
{
    marker.markers.clear();
    visualization_msgs::Marker temp;
    temp.header.frame_id = "world";
    temp.action = visualization_msgs::Marker::DELETEALL;
    marker.markers.push_back(temp);

    pub.publish(marker);
    marker.markers.clear();
}

void Coremap::cvtMsg(a3_link src, core_map::a3_link &msg)
{
    msg.linkid = src.linkid;
    msg.fromnode = src.fromnode;
    msg.tonode = src.tonode;
    msg.roadtype = src.roadtype;
    msg.speed = src.speed;
    msg.lane = src.lane;
    msg.code = src.code;
    msg.cost = src.cost;

    for (int i = 0; i < src.waypoint.size(); i++)
    {
        core_map::waypoint temp;
        temp.x = src.waypoint[i].x;
        temp.y = src.waypoint[i].y;
        temp.change_left = src.waypoint[i].change_left;
        temp.change_right = src.waypoint[i].change_right;
        msg.waypointAry.push_back(temp);
    }
}

void Coremap::makeTable(std::string tablePath)
{
    std::ofstream out;
    tablePath += "/MultiCall_Table.csv";
    out.open(tablePath);
    std::cout << "Make Table Start!" << std::endl;
    for (size_t i = 0; i < hdmap.m_node.size(); i++)
    {
        size_t startNode = hdmap.m_node[i].thisIndex;
        for (size_t j = 0; j < hdmap.m_node.size(); j++)
        {
            std::vector<a3_link> global_path;
            std::vector<Motion> motion;

            size_t endNode = hdmap.m_node[j].thisIndex;

            if (dijkstra.findPath(startNode, endNode, global_path, motion))
            {
                double totalTime;
                dijkstra.getPathTime(totalTime);
                out << totalTime;
            }
            else
            {
                if (i == j)
                    out << "0";
                else
                    out << std::to_string(TIME_MAX);
            }

            if (j < hdmap.m_node.size() - 1)
                out << ",";
        }
        out << std::endl;
        double percent = double(i + 1) / double(hdmap.m_node.size()) * 100.0;
        std::cout << percent << "% complete" << std::endl;
    }
    out.close();
}

void Coremap::initTable(std::string tablePath)
{
    std::ifstream read;
    std::vector<std::string> buf;
    tablePath += "/MultiCall_Table.csv";
    read.open(tablePath);

    if (read.is_open())
    {
        while (!read.eof())
        {
            std::string str;
            std::getline(read, str);
            buf.push_back(str);
        }
    }
    if (buf.back() == "")
        buf.pop_back();

    for (size_t i = 0; i < buf.size(); i++)
    {
        std::string temp;
        std::vector<double> rowData;

        for (size_t j = 0; j < buf[i].size(); j++)
        {
            if (buf[i][j] != ',')
                temp += buf[i][j];
            else
            {
                rowData.push_back(stod(temp));
                temp = "";
            }
        }
        rowData.push_back(stod(temp));
        temp = "";

        table.push_back(rowData);
        rowData.clear();
    }
    read.close();
}

void Coremap::transGps(double lat, double lon, double &east, double &north)
{
    double Lat = lat;
    double Lon = lon;
    double Easting;
    double Northing;
    int Zone;

    Zone = (int)floor(Lon / 6 + 31);

    Easting = 0.5 * log((1 + cos(Lat * M_PI / 180) * sin(Lon * M_PI / 180 - (6 * Zone - 183) * M_PI / 180)) / (1 - cos(Lat * M_PI / 180) * sin(Lon * M_PI / 180 - (6 * Zone - 183) * M_PI / 180))) * 0.9996 * 6399593.62 / pow((1 + pow(0.0820944379, 2) * pow(cos(Lat * M_PI / 180), 2)), 0.5) * (1 + pow(0.0820944379, 2) / 2 * pow((0.5 * log((1 + cos(Lat * M_PI / 180) * sin(Lon * M_PI / 180 - (6 * Zone - 183) * M_PI / 180)) / (1 - cos(Lat * M_PI / 180) * sin(Lon * M_PI / 180 - (6 * Zone - 183) * M_PI / 180)))), 2) * pow(cos(Lat * M_PI / 180), 2) / 3) + 500000;
    Easting = round(Easting * 100) * 0.01;
    Northing = (atan(tan(Lat * M_PI / 180) / cos((Lon * M_PI / 180 - (6 * Zone - 183) * M_PI / 180))) - Lat * M_PI / 180) * 0.9996 * 6399593.625 / sqrt(1 + 0.006739496742 * pow(cos(Lat * M_PI / 180), 2)) * (1 + 0.006739496742 / 2 * pow(0.5 * log((1 + cos(Lat * M_PI / 180) * sin((Lon * M_PI / 180 - (6 * Zone - 183) * M_PI / 180))) / (1 - cos(Lat * M_PI / 180) * sin((Lon * M_PI / 180 - (6 * Zone - 183) * M_PI / 180)))), 2) * pow(cos(Lat * M_PI / 180), 2)) + 0.9996 * 6399593.625 * (Lat * M_PI / 180 - 0.005054622556 * (Lat * M_PI / 180 + sin(2 * Lat * M_PI / 180) / 2) + 4.258201531e-05 * (3 * (Lat * M_PI / 180 + sin(2 * Lat * M_PI / 180) / 2) + sin(2 * Lat * M_PI / 180) * pow(cos(Lat * M_PI / 180), 2)) / 4 - 1.674057895e-07 * (5 * (3 * (Lat * M_PI / 180 + sin(2 * Lat * M_PI / 180) / 2) + sin(2 * Lat * M_PI / 180) * pow(cos(Lat * M_PI / 180), 2)) / 4 + sin(2 * Lat * M_PI / 180) * pow(cos(Lat * M_PI / 180), 2) * pow(cos(Lat * M_PI / 180), 2)) / 3);

    Northing = round(Northing * 100) * 0.01;

    double offset_x = 471194.505;
    double offset_y = 3965610.5720000002;

    east = Easting - offset_x;
    north = Northing - offset_y;
}

void Coremap::getTotalDist(pcl::PointCloud<pcl::PointXY> points, double &dist)
{
    if(points.size() > 1)
    {
        pcl::PointXY prev = points.front();
        dist = 0;
        for(int i = 1; i < points.size(); i++)
        {
            double tempDist = hypot(prev.x - points[i].x, prev.y - points[i].y);
            dist += tempDist;
            prev = points[i];
        }
    }
    else
    {
        dist = 0;
    }
}

void Coremap::getBsdLink(pcl::PointXY p, core_map::links& tempBsd)
{
    int linkIndex, linkPointIndex;
    hdmap.getLinkInfo(p, linkIndex, linkPointIndex);

    if(linkIndex == -1 || linkPointIndex == -1)
    {
        // std::cout << "SrvBsdLink Error : 가까운 링크를 찾을 수 없음!" << std::endl;
        return;
    }

    a3_link targetLink = hdmap.m_link[linkIndex];
    targetLink.waypoint.clear();

    for (int i = 0; i < linkPointIndex; i++)
    {
        Waypoint temp;
        temp.x = hdmap.m_link[linkIndex].waypoint[i].x;
        temp.y = hdmap.m_link[linkIndex].waypoint[i].y;
        temp.change_left = hdmap.m_link[linkIndex].waypoint[i].change_left;
        temp.change_right = hdmap.m_link[linkIndex].waypoint[i].change_right;

        targetLink.waypoint.push_back(temp);
    }
    core_map::a3_link cvtTargetLink;
    cvtMsg(targetLink, cvtTargetLink);

    std::reverse(cvtTargetLink.waypointAry.begin(), cvtTargetLink.waypointAry.end());
    tempBsd.link.push_back(cvtTargetLink);

    int fromNode = hdmap.m_link[linkIndex].from_index;
    std::vector<a3_link> links;
    hdmap.getConnectedLink(fromNode, links);
    if (links.size() > 0)
    {
        // 교차로인 경우
        if (links[0].linkid[8] == 'I')
        {
            for (int i = 0; i < links.size(); i++)
            {
                // 직진이 아닌경우 한칸만 보내면 됨
                if (links[i].linkid.substr(8, 2) != "IZ")
                {
                    core_map::a3_link temp;
                    cvtMsg(links[i], temp);
                    std::reverse(temp.waypointAry.begin(), temp.waypointAry.end());
                    tempBsd.link.push_back(temp);
                }
                // 직진인 경우 그 이전 링크까지 보내야함
                else
                {
                    // 일단 현재 링크 추가
                    core_map::a3_link temp;
                    cvtMsg(links[i], temp);
                    std::reverse(temp.waypointAry.begin(), temp.waypointAry.end());

                    // 현재 링크랑 연결된 직진차선 검사
                    int nodeIndex = links[i].from_index;
                    std::vector<a3_link> tempConnect;
                    hdmap.getConnectedLink(nodeIndex, tempConnect);

                    if (tempConnect.size() > 0)
                    {
                        for (int j = 0; j < tempConnect.size(); j++)
                        {
                            if (!tempConnect[0].change_lane)
                            {
                                core_map::a3_link temp2;
                                cvtMsg(tempConnect[j], temp2);
                                std::reverse(temp2.waypointAry.begin(), temp2.waypointAry.end());
                                temp.waypointAry.insert(temp.waypointAry.end(), temp2.waypointAry.begin(), temp2.waypointAry.end());
                                break;
                            }
                        }
                    }
                    tempBsd.link.push_back(temp);
                }
            }
        }
        // 교차로가 아닌경우
        else
        {
            for (int i = 0; i < links.size(); i++)
            {
                if (!links[i].change_lane)
                {
                    // 일단 현재 링크 추가
                    core_map::a3_link temp;
                    cvtMsg(links[i], temp);
                    std::reverse(temp.waypointAry.begin(), temp.waypointAry.end());

                    // 현재 링크랑 연결된 직진차선 검사
                    int nodeIndex = links[i].from_index;
                    std::vector<a3_link> tempConnect;
                    hdmap.getConnectedLink(nodeIndex, tempConnect);

                    if (tempConnect.size() > 0)
                    {
                        // 교차로인 경우
                        if (tempConnect[0].linkid[8] == 'I')
                        {
                            for (int j = 0; j < tempConnect.size(); j++)
                            {
                                if (tempConnect[j].linkid.substr(8, 2) == "IZ")
                                {
                                    core_map::a3_link temp2;
                                    cvtMsg(tempConnect[j], temp2);
                                    std::reverse(temp2.waypointAry.begin(), temp2.waypointAry.end());
                                    temp.waypointAry.insert(temp.waypointAry.end(), temp2.waypointAry.begin(), temp2.waypointAry.end());
                                    break;
                                }
                            }
                        }
                        // 교차로가 아닌 경우
                        else
                        {
                            for (int j = 0; j < tempConnect.size(); j++)
                            {
                                if (!tempConnect[j].change_lane)
                                {
                                    core_map::a3_link temp2;
                                    cvtMsg(tempConnect[j], temp2);
                                    std::reverse(temp2.waypointAry.begin(), temp2.waypointAry.end());
                                    temp.waypointAry.insert(temp.waypointAry.end(), temp2.waypointAry.begin(), temp2.waypointAry.end());
                                    break;
                                }
                            }
                        }
                    }
                    tempBsd.link.push_back(temp);
                }
            }
        }
    }
}

bool Coremap::optimizeRoute(pcl::PointXY currPos, int remainSeat)
{
    pcl::PointCloud<pcl::PointXY> sol;
    std::vector<int> selected;
    order.clear();
    multiCallPath.clear();
    multiCall.optimizeRoute(missionList, currPos, remainSeat, sol, order, selected);
    visualizeSelectedMission(order);
    std::vector<std::vector<a3_link>> links;
    // Optimize 된 순서대로 Dijkstra
    bool isImpossible = false;
    for (size_t i = 0; i < sol.size() - 1; i++)
    {
        std::vector<a3_link> global_path;
        std::vector<Motion> motion;
        std::vector<a3_link> link;
        if (dijkstra.findPath(sol[i], sol[i + 1], 0, 0, global_path, motion))
        {
            core_map::global_path tempPath;
            setGlobalPath(motion, global_path, tempPath, false);
            multiCallPath.push_back(tempPath);
            links.push_back(global_path);
        }
        else
        {
            isImpossible = true;
            break;
        }
    }

    if (isImpossible)
    {
        for (int i = 0; i < selected.size(); i++)
        {
            missionList[selected[i]].status = MISSION_STATE_IMPOSSIBLE;
        }
        return false;
    }

    std::unique_lock<std::mutex> lock_tcp(m_mutex);
    m_finalPath = multiCallPath.front();
    m_finalLink = links.front();
    m_fsmState = state::MISSION_CHECK;
    fprintf(stderr, "- State Change : MISSION_CHECK\n\n");
    lock_tcp.unlock();

   if(runType != SIMULATE)
   {
       for (int i = 0; i < selected.size(); i++)
       {
           m_currentMissionIndex = selected[i];
           callRequest(missionList[selected[i]].missionID);
       }
   }
   else
   {
        for (int i = 0; i < selected.size(); i++)
        {
            m_currentMissionIndex = selected[i];
            core_map::SimCallReq request;
            request.request.callID =  missionList[selected[i]].missionID;
            if(sim_call_req_srv.call(request))
            {
                if(request.response.flag)
                {
                    fprintf(stderr, "Call ID(%d) is Matched!\n", missionList[selected[i]].missionID);
                    fprintf(stderr, "Estimated Time : %f\n", m_finalPath.totalTime);
                    missionList[selected[i]].status = MISSION_STATE_CLEAR;
                    path_pub.publish(m_finalPath);
                    visualizePath(m_finalPath);
                    std::unique_lock<std::mutex> lock_tcp(m_mutex);
                    m_fsmState = state::WAIT_GETTING_ON;
                    fprintf(stderr, "- State Change : WAIT_GETTING_ON\n\n");
                    lock_tcp.unlock();
                }
                else
                {
                    fprintf(stderr, "Call Selecting Fail\n");
                    missionList[selected[i]].status  = MISSION_STATE_CLEAR;
                    std::unique_lock<std::mutex> lock_tcp(m_mutex);
                    m_fsmState = state::WAIT_ORDER;
                    fprintf(stderr, "- State Change : WAIT_ORDER\n\n");
                    lock_tcp.unlock();
                }
            }
        }
   }
    return true;
}

void Coremap::clickedPointCallback(const geometry_msgs::PoseStamped &msg)
{
    if (mouse_flag == Mouse::FIRST_CLICK)
    {
        mouseSrc.x = msg.pose.position.x;
        mouseSrc.y = msg.pose.position.y;
        mouse_flag = Mouse::SECOND_CLICK;
        std::cout << "Mouse src point : (" << mouseSrc.x << ", " << mouseSrc.y << ")" << std::endl;
    }
    else
    {
        mouseDst.x = msg.pose.position.x;
        mouseDst.y = msg.pose.position.y;
        mouse_flag = Mouse::FIRST_CLICK;
        std::cout << "Mouse dst point : (" << mouseDst.x << ", " << mouseDst.y << ")" << std::endl;
        mousePathReq(mouseSrc, mouseDst);
    }
}

void Coremap::missionStateCallback(const std_msgs::Int8 &msg)
{
    std::unique_lock<std::mutex> lock_tcp(m_mutex);

    // SIMULATE POS
    //  int srcLane = multiCallPath.front().pathAry.back().src;
    //  m_globalPos.x = multiCallPath.front().pathAry.back().links[srcLane].waypointAry.back().x;
    //  m_globalPos.y = multiCallPath.front().pathAry.back().links[srcLane].waypointAry.back().y;
    //

    pcl::PointXY globalPos = m_globalPos;
    lock_tcp.unlock();
    if (msg.data == 1)
    {
        // 승차지점 도착
        if (order.size() > 1 && multiCallPath.size() > 1)
        {
            order.erase(order.begin());
            multiCallPath.erase(multiCallPath.begin());
            int finalLane;
            if(multiCallPath.front().pathAry.back().changeLane)
            {
                finalLane = multiCallPath.front().pathAry.back().dst;
            }
            else
            {
                finalLane = multiCallPath.front().pathAry.back().src;
            }

            pcl::PointXY finalPoint;
            finalPoint.x = multiCallPath.back().pathAry.back().links[finalLane].waypointAry.back().x;
            finalPoint.y = multiCallPath.back().pathAry.back().links[finalLane].waypointAry.back().y;

            std::vector<a3_link> global_path;
            std::vector<Motion> motion;
            double initalSpeed = 0;
            int ChangeLane = 0; // Not Change Lane
            std::vector<a3_link> link;
            if (dijkstra.findPath(globalPos, finalPoint, initalSpeed, ChangeLane, global_path, motion))
            {
                core_map::global_path tempPath;
                setGlobalPath(motion, global_path, tempPath, false);
                multiCallPath.front() = tempPath;
                link = global_path;
            }

            std::unique_lock<std::mutex> lock_tcp(m_mutex);
            m_finalPath = multiCallPath.front();
            m_finalLink = link;
            lock_tcp.unlock();
            if(runType != SIMULATE)
            {
                startingPointArrive();
            }
            else
            {
                fprintf(stderr, "Send Starting Point Arrive Message!\n");
            }
        }
        else
        {
            fprintf(stderr, "유효하지 않은 Starting Point Arrive Callback\n");
        }
    }
    else if (msg.data == 2)
    {
        // 하차지점 도착
        if (order.size() > 0 && multiCallPath.size() > 0 && order.size() < 2 && multiCallPath.size() < 2)
        {
            order.erase(order.begin());
            multiCallPath.erase(multiCallPath.begin());
            if(runType != SIMULATE)
            {
                endPointArrive();
            }
            else
            {
                fprintf(stderr, "Send End Point Arrive Message!\n");
            }
        }
        else
        {
            fprintf(stderr, "유효하지 않은 End Point Arrive Callback\n");
        }
    }
}

void Coremap::bestposCallback(const novatel_gps_msgs::NovatelPosition::ConstPtr &msg)
{
    std::unique_lock<std::mutex> lock_tcp(m_mutex);
    //위도, 경도, 높이
    m_latitude = msg->lat;
    m_longitude = msg->lon;
    m_elevation = msg->height;
    lock_tcp.unlock();
}

void Coremap::inspvaxCallback(const novatel_gps_msgs::Inspvax::ConstPtr &msg)
{
    std::unique_lock<std::mutex> lock_tcp(m_mutex);
    //북 기준 시계방향으로 0~359
    m_heading = msg->azimuth;
    lock_tcp.unlock();
}

void Coremap::avanteCallback(const core_control::AvanteData::ConstPtr &msg)
{
    std::unique_lock<std::mutex> lock_tcp(m_mutex);
    m_speed = static_cast<unsigned char>(msg->acc.vehicle_speed);
    lock_tcp.unlock();
}

void Coremap::waveCallback(const std_msgs::String &msg)
{
    std::string msgs = msg.data;
    std::string payload;
    obu_tcp_header_t *header = (obu_tcp_header_t *)&msgs[0];
    payload.append(msgs, sizeof(obu_tcp_header_t), header->payload_size);

//    fprintf(stderr,"Header(Message Type) : 0x%x\n",header->packet_type);
//    fprintf(stderr,"Header(Sequence) : %d\n",header->current_sequence);
//    fprintf(stderr,"Header(Payload Size) : %d\n",header->payload_size);
//    fprintf(stderr,"Header(Device Type) : 0x%X\n",header->device_type);
//    fprintf(stderr,"Header(Device ID) : %02X-%02X-%02X\n",header->device_id[2],header->device_id[1],header->device_id[0]);
//    fprintf(stderr,"Payload : 0x%s\n\n",array_to_hex_str((char*)payload.c_str(),payload.size()).c_str());

    //Decoding Example
    MessageFrame_t *msgFrame = nullptr;
    std::vector<uint8_t> mypayload(payload.begin(), payload.end());
    uint8_t *buffer = &mypayload[0];
    auto res = uper_decode(nullptr, &asn_DEF_MessageFrame, (void **)&msgFrame, buffer, mypayload.size(), 0, 0);

    switch (res.code)
    {
    case asn_dec_rval_code_e::RC_OK:
//        fprintf(stderr,"\n[RC_OK]\n");
        switch (msgFrame->messageId)
        {
        case dsrc_msg_id::BASIC_SAFETY_MESSAGE:
//            fprintf(stderr, "[recv bsm]\n");
            break;
        case dsrc_msg_id::PROBE_VEHICLE_DATA:
            fprintf(stderr, "[recv pvd]\n");
            break;
        case dsrc_msg_id::SIGNAL_PHASE_AND_TIMING_MESSAGE:
//            fprintf(stderr,"[recv spat]\n");
            recvSPAT(msgFrame);
            break;
        case dsrc_msg_id::MAP_DATA:
//            fprintf(stderr,"[recv map]\n");
            recvMAP(msgFrame);
            break;
        case dsrc_msg_id::TRAVELER_INFORMATION:
            fprintf(stderr, "[recv tim]\n");
            break;
        case dsrc_msg_id::ROAD_SIDE_ALERT:
            fprintf(stderr, "[recv tim]\n");
            break;
        case dsrc_msg_id::RTCM_CORRECTIONS:
            fprintf(stderr, "[recv rtcm]\n");
            break;
        default:
            break;
        }
//        asn_fprint(stderr, &asn_DEF_MessageFrame, msgFrame);
        break;
    case asn_dec_rval_code_e::RC_WMORE:
        fprintf(stderr, "\n[RC_WMORE]\n");
        break;
    case asn_dec_rval_code_e::RC_FAIL:
        fprintf(stderr, "\n[RC_FAIL]\n");
        break;
    }
}

void Coremap::lteCallback(const std_msgs::String &msg)
{
    std::string msgs = msg.data;
    obu_tcp_header_t *header = (obu_tcp_header_t *)&msgs[0];

    int payloadSize = header->payload_size;
    if(msgs.size() == payloadSize + sizeof(obu_tcp_header_t))
    {
        uint16_t checksum;
        memcpy(&checksum, &msgs[sizeof(obu_tcp_header_t) + payloadSize - 2], 2);
        unsigned char payload[payloadSize - 2];
        memcpy(payload, &msgs[sizeof(obu_tcp_header_t)], payloadSize - 2);
        if (checksum == crc_16(payload, payloadSize - 2))
        {
            std_msgs::String msg;
            msg.data = msgs;
            mission_pub.publish(msg);

            switch (header->packet_type)
            {
            case lte_packet_id::ACCESS_RESTRICTION_NOTIFICATION:
                checkAccessRestriction(payload);
                break;
            case lte_packet_id::ACCESS_PERMISSION_RESPONSE:
                checkAccessPermission(payload);
                break;
            case lte_packet_id::LOCATION_MESSAGE_ERROR:
                checkLocationMessage(payload);
                break;
            case lte_packet_id::TAXI_CALL_LIST:
                checkCallListForSim(payload);
                break;
            case lte_packet_id::CALL_RESPONSE:
                checkCallResponse(payload);
                break;
            case lte_packet_id::GETTING_ON_CONFIRM:
                checkGettingOn(payload);
                break;
            case lte_packet_id::GETTING_OFF_CONFIRM:
                checkGettingOff(payload);
                break;
            }
        }
    }
}

void Coremap::gettingOnCallback(const std_msgs::Int8 &msg)
{
    if(msg.data == 0)
    {
        fprintf(stderr, "Getting On Complete!\n");
        path_pub.publish(m_finalPath);
        visualizePath(m_finalPath);
        std::unique_lock<std::mutex> lock_tcp(m_mutex);
        m_fsmState = state::WAIT_GETTING_OFF;
        fprintf(stderr, "- State Change : WAIT_GETTING_OFF\n\n");
        lock_tcp.unlock();
    }
    else
    {
        fprintf(stderr, "Fail to Gettion On\n");
        std::unique_lock<std::mutex> lock_tcp(m_mutex);
        m_fsmState = state::WAIT_ORDER;
        fprintf(stderr, "- State Change : WAIT_ORDER\n\n");
        lock_tcp.unlock();
    }
}

void Coremap::gettingOffCallback(const std_msgs::Int8 &msg)
{
    if(msg.data == 0)
    {
        fprintf(stderr, "Getting Off Complete!\n");
        std::unique_lock<std::mutex> lock_tcp(m_mutex);
        m_fsmState = state::WAIT_ORDER;
        fprintf(stderr, "- State Change : WAIT_ORDER\n\n");
        lock_tcp.unlock();
    }
    else
    {
        fprintf(stderr, "Fail to Gettion On\n");
        std::unique_lock<std::mutex> lock_tcp(m_mutex);
        m_fsmState = state::WAIT_ORDER;
        fprintf(stderr, "- State Change : WAIT_ORDER\n\n");
        lock_tcp.unlock();
    }
}

void Coremap::visualizeMap(std::vector<c1_node> node, std::vector<a3_link> link, std::vector<a1_lane> lane, std::vector<a2_stop> stop)
{
    Markerarray nodeArr, linkArr, laneArr, stopArr, grid;

    //Visualize Node Array
    nodeArr.markers.resize(1);
    nodeArr.markers[0].header.frame_id = "world";
    nodeArr.markers[0].header.stamp = ros::Time::now();
    nodeArr.markers[0].ns = "nodes";
    nodeArr.markers[0].action = visualization_msgs::Marker::ADD;
    nodeArr.markers[0].pose.orientation.w = 1.0;
    nodeArr.markers[0].id = 0;
    nodeArr.markers[0].type = visualization_msgs::Marker::POINTS;
    nodeArr.markers[0].scale.x = 0.5;
    nodeArr.markers[0].scale.y = 0.5;

    nodeArr.markers[0].color.b = 1.0f;
    nodeArr.markers[0].color.a = 1.0;

    for (size_t i = 0; i < node.size(); i++)
    {
        geometry_msgs::Point p;
        p.x = static_cast<double>(node[i].xy.x);
        p.y = static_cast<double>(node[i].xy.y);
        p.z = 0;

        nodeArr.markers[0].points.push_back(p);
    }

    //Visualize Link Array
    for (size_t i = 0; i < link.size(); i++)
    {
        visualization_msgs::Marker temp;
        temp.header.frame_id = "world";
        temp.header.stamp = ros::Time::now();
        temp.ns = "links";
        temp.action = visualization_msgs::Marker::ADD;
        temp.pose.orientation.w = 1.0;
        temp.type = visualization_msgs::Marker::LINE_STRIP;
        temp.scale.x = 0.1;

        temp.color.g = 0.5f;
        temp.color.r = 0.8f;
        temp.color.a = 0.5f;
        temp.id = static_cast<int>(i);

        for (size_t j = 0; j < link[i].waypoint.size(); j++)
        {
            geometry_msgs::Point p;
            p.x = static_cast<double>(link[i].waypoint[j].x);
            p.y = static_cast<double>(link[i].waypoint[j].y);
            p.z = 0;

            temp.points.push_back(p);
        }
        linkArr.markers.push_back(temp);
    }

    //Visualize Lane Array
    for (size_t i = 0; i < lane.size(); i++)
    {
        visualization_msgs::Marker temp;
        temp.header.frame_id = "world";
        temp.header.stamp = ros::Time::now();
        temp.ns = "lanes";
        temp.action = visualization_msgs::Marker::ADD;
        temp.pose.orientation.w = 1.0;
        temp.type = visualization_msgs::Marker::LINE_STRIP;
        temp.scale.x = 0.2;

        temp.color.g = 1.0;
        temp.color.a = 1.0;
        temp.id = static_cast<int>(i);
        for (size_t j = 0; j < lane[i].xy.size(); j++)
        {
            geometry_msgs::Point p;
            p.x = static_cast<double>(lane[i].xy[j].x);
            p.y = static_cast<double>(lane[i].xy[j].y);
            p.z = 0;

            temp.points.push_back(p);
        }
        laneArr.markers.push_back(temp);
    }

    //Visualize Stop Array
    for (size_t i = 0; i < stop.size(); i++)
    {
        visualization_msgs::Marker temp;
        temp.header.frame_id = "world";
        temp.header.stamp = ros::Time::now();
        temp.ns = "stops";
        temp.action = visualization_msgs::Marker::ADD;
        temp.pose.orientation.w = 1.0;
        temp.type = visualization_msgs::Marker::LINE_STRIP;
        temp.scale.x = 0.2;

        temp.color.r = 1.0f;
        temp.color.a = 1.0;
        temp.id = static_cast<int>(i);
        for (size_t j = 0; j < stop[i].xy.size(); j++)
        {
            geometry_msgs::Point p;
            p.x = static_cast<double>(stop[i].xy[j].x);
            p.y = static_cast<double>(stop[i].xy[j].y);
            p.z = 0;

            temp.points.push_back(p);
        }
        stopArr.markers.push_back(temp);
    }

    //Visualize Grid
    pcl::PointXY offset;
    double tileSize = hdmap.tilemap.tileSize;
    offset.x = hdmap.tilemap.offset.x;
    offset.y = hdmap.tilemap.offset.y;

    int cell_x = hdmap.tilemap.cell_x + 1;
    int cell_y = hdmap.tilemap.cell_y + 1;
    for (size_t i = 0; i < cell_x; i++)
    {
        visualization_msgs::Marker temp;
        temp.header.frame_id = "world";
        temp.header.stamp = ros::Time::now();
        temp.ns = "grid";
        temp.action = visualization_msgs::Marker::ADD;
        temp.pose.orientation.w = 1.0;
        temp.type = visualization_msgs::Marker::LINE_STRIP;
        if (i % 5 == 0)
        {
            temp.scale.x = 1;
        }
        else
        {
            temp.scale.x = 0.2;
        }
        temp.color.r = 1.0;
        temp.color.g = 1.0;
        temp.color.b = 1.0;
        temp.color.a = 0.5;
        temp.id = static_cast<int>(i);

        geometry_msgs::Point p1;
        p1.x = tileSize * i - static_cast<double>(offset.x);
        p1.y = static_cast<double>(-offset.y);
        geometry_msgs::Point p2;
        p2.x = tileSize * i - static_cast<double>(offset.x);
        p2.y = tileSize * (cell_y - 1) - static_cast<double>(offset.y);
        temp.points.push_back(p1);
        temp.points.push_back(p2);

        grid.markers.push_back(temp);
    }
    for (size_t i = 0; i < cell_y; i++)
    {
        visualization_msgs::Marker temp;
        temp.header.frame_id = "world";
        temp.header.stamp = ros::Time::now();
        temp.ns = "grid";
        temp.action = visualization_msgs::Marker::ADD;
        temp.pose.orientation.w = 1.0;
        temp.type = visualization_msgs::Marker::LINE_STRIP;
        if (i % 5 == 0)
        {
            temp.scale.x = 1.5;
        }
        else
        {
            temp.scale.x = 0.2;
        }
        temp.color.r = 1.0;
        temp.color.g = 1.0;
        temp.color.b = 1.0;
        temp.color.a = 0.5;
        temp.id = static_cast<int>(i) + cell_x;

        geometry_msgs::Point p1;
        p1.x = static_cast<double>(-offset.x);
        p1.y = tileSize * i - static_cast<double>(offset.y);
        geometry_msgs::Point p2;
        p2.x = tileSize * (cell_x - 1) - static_cast<double>(offset.x);
        p2.y = tileSize * i - static_cast<double>(offset.y);
        temp.points.push_back(p1);
        temp.points.push_back(p2);

        grid.markers.push_back(temp);
    }

    while (ros::ok())
    {
        if (rviz_map_pub.getNumSubscribers() > 0)
        {
            rviz_map_pub.publish(nodeArr);
            rviz_map_pub.publish(linkArr);
            rviz_map_pub.publish(laneArr);
            rviz_map_pub.publish(stopArr);
            rviz_map_pub.publish(grid);
            break;
        }
        else
        {
            std::cout << "waiting for visualize map..." << std::endl;
            ros::Duration(1.0).sleep();
        }
    }

    std::cout << "visualization HDMap!" << std::endl
              << std::endl;
}

void Coremap::visualizeMission(pcl::PointCloud<pcl::PointXY> srcPoints, pcl::PointCloud<pcl::PointXY> dstPoints)
{
    Markerarray src, dst;

    //Visualize src Array
    for (size_t i = 0; i < srcPoints.size(); i++)
    {
        visualization_msgs::Marker temp;
        temp.header.frame_id = "world";
        temp.header.stamp = ros::Time::now();
        temp.ns = "srcPoints";
        temp.id = i;
        temp.type = visualization_msgs::Marker::CYLINDER;
        temp.action = visualization_msgs::Marker::ADD;
        temp.pose.position.x = static_cast<double>(srcPoints[i].x);
        temp.pose.position.y = static_cast<double>(srcPoints[i].y);
        temp.pose.position.z = 5.0;
        temp.pose.orientation.x = 0.0;
        temp.pose.orientation.y = 0.0;
        temp.pose.orientation.z = 0.0;
        temp.pose.orientation.w = 1.0;
        temp.scale.x = 15.0;
        temp.scale.y = 15.0;
        temp.scale.z = 10.0;
        temp.color.a = 0.3f;
        temp.color.r = 0.0;
        temp.color.g = 1.0;
        temp.color.b = 0.0;
        src.markers.push_back(temp);
    }

    for (size_t i = 0; i < dstPoints.size(); i++)
    {
        visualization_msgs::Marker temp;
        temp.header.frame_id = "world";
        temp.header.stamp = ros::Time::now();
        temp.ns = "dstPoints";
        temp.id = i;
        temp.type = visualization_msgs::Marker::CUBE;
        temp.action = visualization_msgs::Marker::ADD;
        temp.pose.position.x = static_cast<double>(dstPoints[i].x);
        temp.pose.position.y = static_cast<double>(dstPoints[i].y);
        temp.pose.position.z = 5.0;
        temp.pose.orientation.x = 0.0;
        temp.pose.orientation.y = 0.0;
        temp.pose.orientation.z = 0.0;
        temp.pose.orientation.w = 1.0;
        temp.scale.x = 15.0;
        temp.scale.y = 15.0;
        temp.scale.z = 10.0;
        temp.color.a = 0.3f;
        temp.color.r = 1.0;
        temp.color.g = 0.0;
        temp.color.b = 0.0;
        dst.markers.push_back(temp);
    }

    rviz_v2x_call_pub.publish(src);
    rviz_v2x_call_pub.publish(dst);
}

void Coremap::visualizeSelectedMission(std::vector<MissionPoint> order)
{
    Markerarray sol;
    resetMarker(sol, rviz_v2x_call_pub);

    std::vector<int> id;
    for (size_t i = 0; i < order.size(); i++)
    {
        bool exist = false;
        for (size_t j = 0; j < id.size(); j++)
        {
            if (order[i].id == id[j])
            {
                exist = true;
                break;
            }
        }

        if (!exist)
            id.push_back(order[i].id);
    }

    for (size_t i = 0; i < order.size(); i++)
    {
        visualization_msgs::Marker temp;
        temp.header.frame_id = "world";
        temp.header.stamp = ros::Time::now();
        temp.ns = "solution";
        temp.id = i;
        temp.action = visualization_msgs::Marker::ADD;
        temp.pose.position.x = static_cast<double>(order[i].location.x);
        temp.pose.position.y = static_cast<double>(order[i].location.y);
        temp.pose.position.z = 10.0;
        temp.pose.orientation.x = 0.0;
        temp.pose.orientation.y = 0.0;
        temp.pose.orientation.z = 0.0;
        temp.pose.orientation.w = 1.0;
        temp.scale.x = 15.0;
        temp.scale.y = 15.0;
        temp.scale.z = 20.0;

        if (order[i].src)
        {
            temp.type = visualization_msgs::Marker::CYLINDER;
            for (size_t j = 0; j < id.size(); j++)
            {
                if (order[i].id == id[j])
                {
                    int tempIndex = colorIndex;
                    if (colorIndex + j > 5)
                    {
                        tempIndex = colorIndex + j - 5;
                    }
                    temp.color = colors[tempIndex];
                }
            }
        }
        else
        {
            temp.type = visualization_msgs::Marker::CUBE;
            for (size_t j = 0; j < id.size(); j++)
            {
                if (order[i].id == id[j])
                {
                    int tempIndex = colorIndex;
                    if (colorIndex + j > 5)
                    {
                        tempIndex = colorIndex + j - 5;
                    }
                    temp.color = colors[tempIndex];
                }
            }
        }
        sol.markers.push_back(temp);
    }
    if (colorIndex + id.size() > 5)
    {
        colorIndex = colorIndex + id.size() - 5;
    }
    else
    {
        colorIndex = colorIndex + id.size();
    }
    rviz_v2x_call_pub.publish(sol);
}

void Coremap::visualizePath(core_map::global_path path)
{
    pcl::PointCloud<pcl::PointXYZI> real;
    for (size_t i = 0; i < path.pathAry.size(); i++)
    {
        size_t srcIndex = static_cast<size_t>(path.pathAry[i].src);
        for (size_t j = 0; j < path.pathAry[i].links[srcIndex].waypointAry.size(); j++)
        {
            pcl::PointXYZI p;
            p.x = static_cast<float>(path.pathAry[i].links[srcIndex].waypointAry[j].x);
            p.y = static_cast<float>(path.pathAry[i].links[srcIndex].waypointAry[j].y);
            p.z = 0;
            p.intensity = static_cast<float>(path.pathAry[i].links[srcIndex].waypointAry[j].v);
            real.push_back(p);
        }
        if (path.pathAry[i].changeLane)
        {
            size_t dstIndex = static_cast<size_t>(path.pathAry[i].dst);
            for (size_t j = 0; j < path.pathAry[i].links[dstIndex].waypointAry.size(); j++)
            {
                pcl::PointXYZI p;
                p.x = static_cast<float>(path.pathAry[i].links[dstIndex].waypointAry[j].x);
                p.y = static_cast<float>(path.pathAry[i].links[dstIndex].waypointAry[j].y);
                p.z = 0;
                p.intensity = static_cast<float>(path.pathAry[i].links[dstIndex].waypointAry[j].v);
                real.push_back(p);
            }
        }
    }

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(real, output);
    output.header.frame_id = "world";
    real_path_pub.publish(output);
}

bool Coremap::mousePathReq(pcl::PointXY src, pcl::PointXY dst)
{
    std::cout << "\n▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼" << std::endl;
    std::cout << "Mouse Path Requested!" << std::endl;
    ros::Time time_start = ros::Time::now();
    std::cout << "(" << src.x << ", " << src.y << ") -> (" << dst.x << ", " << dst.y << ")" << std::endl;

    std::vector<a3_link> global_path;
    std::vector<Motion> motion;
    core_map::global_path path_msg;
    double currentSpeed = 0;
    if (dijkstra.findPath(src, dst, currentSpeed, 0, global_path, motion))
    {
        setGlobalPath(motion, global_path, path_msg, true);
        std::cout << "globalPath.totalCost : " << path_msg.totalCost << "m" << std::endl;
        std::cout << "globalPath.totalTime : " << path_msg.totalTime << "s" << std::endl;
        path_pub.publish(path_msg);
        std::vector<a3_link> link;
        link = global_path;
        std::unique_lock<std::mutex> lock_tcp(m_mutex);
        m_finalPath = path_msg;
        m_finalLink = link;
        lock_tcp.unlock();

        ros::Time time_end = ros::Time::now();
        double time_diff = (time_end - time_start).toSec();
        std::cout << "Emergency Total time : " << time_diff * 1000.0 << " ms" << std::endl;
        std::cout << "▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲" << std::endl;
        return true;
    }
    else
    {
        std::cout << "Can't find path!!" << std::endl;
        ros::Time time_end = ros::Time::now();
        double time_diff = (time_end - time_start).toSec();
        std::cout << "Emergency Total time : " << time_diff * 1000.0 << " ms" << std::endl;
        std::cout << "▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲" << std::endl;
        return false;
    }
}

bool Coremap::srvBsdLink(Bsd::Request &req, Bsd::Response &res)
{
    fprintf(stderr, "\n▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼\n");
    fprintf(stderr, "Bsd Requested!\n");
    ros::Time time_start = ros::Time::now();

    for(int i = 0; i < req.p.size(); i++)
    {
        pcl::PointXY p;
        p.x = req.p[i].x;
        p.y = req.p[i].y;
        fprintf(stderr, "req.p[%d] : (%f, %f)\n", i, p.x, p.y);

        core_map::links tempBsd;
        getBsdLink(p, tempBsd);
        res.bsdLink.push_back(tempBsd);
    }
    ros::Time time_end = ros::Time::now();
    double time_diff = (time_end - time_start).toSec();
    fprintf(stderr, "Bsd Total time : %f ms\n", time_diff * 1000.0);
    fprintf(stderr, "▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲\n");
    return true;
}

bool Coremap::srvEmergencyPath(RequestPath::Request &req, RequestPath::Response &res)
{
    fprintf(stderr, "\n▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼\n");
    fprintf(stderr, "Emergency Path Requested!\n");
    ros::Time time_start = ros::Time::now();

    pcl::PointXY src, dst;
    src.x = static_cast<float>(req.src.x);
    src.y = static_cast<float>(req.src.y);
    dst.x = static_cast<float>(req.dst.x);
    dst.y = static_cast<float>(req.dst.y);
    fprintf(stderr, "ChangeLane : %d, CurrSpeed : %f\n", req.changeLane, req.currSpeed);
    fprintf(stderr, "(%f, %f) -> (%f, %f)\n", src.x, src.y, dst.x, dst.y);

    std::vector<a3_link> global_path;
    std::vector<Motion> motion;

    if (dijkstra.findPath(src, dst, req.currSpeed, req.changeLane, global_path, motion))
    {
        setGlobalPath(motion, global_path, res.path, true);
        fprintf(stderr, "globalPath.totalCost : %fm\n", res.path.totalCost);
        fprintf(stderr, "globalPath.totalTime : %fs\n", res.path.totalTime);
        path_pub.publish(res.path);

        std::unique_lock<std::mutex> lock_tcp(m_mutex);
        m_finalPath = res.path;
        lock_tcp.unlock();

        ros::Time time_end = ros::Time::now();
        double time_diff = (time_end - time_start).toSec();
        fprintf(stderr, "Emergency Total time : %f ms\n", time_diff * 1000.0);
        fprintf(stderr, "▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲\n");
        return true;
    }
    else
    {
        ros::Time time_end = ros::Time::now();
        double time_diff = (time_end - time_start).toSec();
        fprintf(stderr, "Can't find path!!\n");
        fprintf(stderr, "Emergency Total time : %f ms\n", time_diff * 1000.0);
        fprintf(stderr, "▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲\n");
        return false;
    }
}

bool Coremap::srvCheckProfit(CheckProfit::Request &req, CheckProfit::Response &res)
{
    ros::Time time_start = ros::Time::now();
    fprintf(stderr, "\n▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼\n");
    fprintf(stderr, "CheckProfit Request\n");
    if(m_finalPath.pathAry.size() > 0)
    {
        for(int i = 0; i < req.tonode.size(); i++)
        {
            a3_link lastLink;
            std::unique_lock<std::mutex> lock_tcp(m_mutex);
            lastLink = m_finalLink.back();
            lock_tcp.unlock();

            std::string srcNode = req.tonode[i];
            std::vector<int> dstIndex;
            if(lastLink.cost > LANECHANGE_END_THRESH)
            {
                int bundleIndex = lastLink.bundle_index;
                for(int i = 0; i < hdmap.bundle[bundleIndex].size(); i++)
                {
                    if(hdmap.bundle[bundleIndex][i] == nullptr)
                        continue;

                    if(hdmap.bundle[bundleIndex][i]->tonode == lastLink.tonode)
                    {
                        dstIndex.push_back(hdmap.bundle[bundleIndex][i]->from_index);
                    }
                }
            }
            else
                dstIndex.push_back(lastLink.from_index);

            int srcIndex = -1;
            for (int i = 0; i < hdmap.m_node.size(); i++)
            {
                if (hdmap.m_node[i].nodeid == srcNode)
                {
                    srcIndex = i;
                    break;
                }
            }

            double minTime = DBL_MAX;
            if (srcIndex != -1 && dstIndex.size() > 0)
            {
                for(int i = 0; i < dstIndex.size(); i++)
                {
                    if(table[srcIndex][dstIndex[i]] < minTime)
                    {
                        minTime = table[srcIndex][dstIndex[i]];
                    }
                }
                res.totalTime.push_back(minTime);
                fprintf(stderr, "req.tonode[%d] : %s  ->  res.totalTime[%d] : %f\n", i, req.tonode[i].c_str(), i, res.totalTime[i]);
            }
            else
            {
                res.totalTime.push_back(TIME_MAX);
                fprintf(stderr, "req.tonode[%d] : %s  ->  res.totalTime[%d] : INF\n", i, req.tonode[i].c_str(), i);
//                fprintf(stderr, "srvCheckProfit Error : 시작노드나 도착노드를 찾지못함\n");
            }
        }
        ros::Time time_end = ros::Time::now();
        double time_diff = (time_end - time_start).toSec();
        fprintf(stderr, "CheckProfit Total time : %f ms\n", time_diff * 1000.0);
        fprintf(stderr, "▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲\n");
        return true;
    }
    else
    {
        ros::Time time_end = ros::Time::now();
        double time_diff = (time_end - time_start).toSec();
        fprintf(stderr, "srvCheckProfit Error : 설정된 전역경로가 없음\n");
        fprintf(stderr, "CheckProfit Total time : %f ms\n", time_diff * 1000.0);
        fprintf(stderr, "▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲\n");
        return false;
    }
}

bool Coremap::srvIntersection(core_map::Intersection::Request &req, core_map::Intersection::Response &res)
{
    ros::Time time_start = ros::Time::now();
    fprintf(stderr, "\n▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼\n");
    fprintf(stderr, "Intersection Request\n");
    fprintf(stderr, "req.p : (%f, %f)\n", req.p.x, req.p.y);
    pcl::PointXY p;
    p.x = req.p.x;
    p.y = req.p.y;
    int linkIndex, linkPointIndex;
    hdmap.getLinkInfo(p, linkIndex, linkPointIndex);

    if (linkIndex != -1 && linkPointIndex != -1)
    {
        if (hdmap.m_link[linkIndex].linkid[8] == 'I')
            res.dist = 0;
        else
        {
            if (hdmap.m_link[linkIndex].isStop)
            {
                res.dist = 0;
                pcl::PointXY prev;
                prev.x = hdmap.m_link[linkIndex].waypoint[linkPointIndex].x;
                prev.y = hdmap.m_link[linkIndex].waypoint[linkPointIndex].y;

                for (int i = linkPointIndex + 1; i < hdmap.m_link[linkIndex].waypoint.size(); i++)
                {
                    double diff_x = hdmap.m_link[linkIndex].waypoint[i].x - prev.x;
                    double diff_y = hdmap.m_link[linkIndex].waypoint[i].y - prev.y;
                    double diff = hypot(diff_x, diff_y);

                    res.dist += diff;
                    prev.x = hdmap.m_link[linkIndex].waypoint[i].x;
                    prev.y = hdmap.m_link[linkIndex].waypoint[i].y;
                }
            }
            else
                res.dist = DBL_MAX;
        }


        ros::Time time_end = ros::Time::now();
        double time_diff = (time_end - time_start).toSec();
        fprintf(stderr, "res.dist : %f\n", res.dist);
        fprintf(stderr, "Intersection Total time : %f ms\n", time_diff * 1000.0);
        fprintf(stderr, "▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲\n");


        return true;
    }
    else
    {
        ros::Time time_end = ros::Time::now();
        double time_diff = (time_end - time_start).toSec();
        fprintf(stderr, "SrvIntersection Error : 가까운 링크를 찾을 수 없음!\n");
        fprintf(stderr, "Intersection Total time : %f ms\n", time_diff * 1000.0);
        fprintf(stderr, "▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲\n");
        return false;
    }
}

bool Coremap::srvFrontIrr(FrontIrr::Request &req, FrontIrr::Response &res)
{
    ros::Time time_start = ros::Time::now();
    fprintf(stderr, "\n▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼\n");
    fprintf(stderr, "FrontIrr Request\n");
    fprintf(stderr, "req.p : (%f, %f)\n", req.p.x, req.p.y);
    std::vector<a3_link> final_link;
    std::unique_lock<std::mutex> lock_tcp(m_mutex);
    final_link = m_finalLink;
    lock_tcp.unlock();


    pcl::PointXY p;
    p.x = req.p.x;
    p.y = req.p.y;
    int linkIndex, linkPointIndex;
    hdmap.getLinkInfo(p, linkIndex, linkPointIndex);

    if (linkIndex != -1 && linkPointIndex != -1)
    {
        int thisBundle = hdmap.m_link[linkIndex].bundle_index;
        int index1 = -1;
        for(int i = 0; i < final_link.size(); i++)
        {
            if(final_link[i].bundle_index == thisBundle)
            {
                index1 = i;
                break;
            }
        }

        if(index1 != -1)
        {
            int index2 = -1;
            for(int i = index1; i < final_link.size(); i++)
            {
                if(final_link[i].linkid.substr(8, 2) != "IR" && final_link[i].linkid.substr(8, 2) != "IL")
                {
                    index2 = i;
                }
                else
                {
                    break;
                }
            }

            std::vector<a3_link> connectLink;
            int prevNode = hdmap.m_link[linkIndex].to_index;
            connectLink.push_back(hdmap.m_link[linkIndex]);
            for(int i = index1 + 1; i <= index2; i++)
            {
                bool find = false;
                int bundleIndex = final_link[i].bundle_index;
                for(int j = 0; j < hdmap.bundle[bundleIndex].size(); j++)
                {
                    if(hdmap.bundle[bundleIndex][j] == nullptr)
                        continue;

                    if(!hdmap.bundle[bundleIndex][j]->change_lane && hdmap.bundle[bundleIndex][j]->from_index == prevNode)
                    {
                        if(hdmap.bundle[bundleIndex][j]->linkid.substr(8, 2) != "IR" && hdmap.bundle[bundleIndex][j]->linkid.substr(8, 2) != "IL")
                        {
                            connectLink.push_back(*hdmap.bundle[bundleIndex][j]);
                            prevNode = hdmap.bundle[bundleIndex][j]->to_index;
                            find = true;
                            break;
                        }
                    }
                }

                if(!find)
                    break;
            }

            pcl::PointCloud<pcl::PointXY> points;
            if(connectLink.front().isIrr)
            {
                int startIrr = connectLink.front().startIrr;
                int endIrr = connectLink.front().endIrr;
                if(linkPointIndex < startIrr)
                {
                    for(int i = linkPointIndex; i <= startIrr; i++)
                    {
                        pcl::PointXY p;
                        p.x = connectLink.front().waypoint[i].x;
                        p.y = connectLink.front().waypoint[i].y;
                        points.push_back(p);
                    }
                    getTotalDist(points, res.dist);
                    return true;
                }
                else if(linkPointIndex > startIrr && linkPointIndex <= endIrr)
                {
                    res.dist = 0;
                    return true;
                }
                else
                {
                    for(int i = linkPointIndex; i < connectLink.front().waypoint.size(); i++)
                    {
                        pcl::PointXY p;
                        p.x = connectLink.front().waypoint[i].x;
                        p.y = connectLink.front().waypoint[i].y;
                        points.push_back(p);
                    }
                }
            }
            else
            {
                if(connectLink.size() < 2)
                {
                    res.dist = -1;
                    return true;
                }
                else
                {
                    for(int i = linkPointIndex; i < connectLink.front().waypoint.size(); i++)
                    {
                        pcl::PointXY p;
                        p.x = connectLink.front().waypoint[i].x;
                        p.y = connectLink.front().waypoint[i].y;
                        points.push_back(p);
                    }
                }
            }

            int irrLink = -1;
            for(int i = 1; i < connectLink.size(); i++)
            {
                if(connectLink[i].isIrr)
                {
                    irrLink = i;
                    break;
                }
            }

            if(irrLink == -1)
            {
                res.dist = -1;
                return true;
            }
            else
            {
                for(int i = 1; i < irrLink; i++)
                {
                    for(int j = 0; j < connectLink[i].waypoint.size(); j++)
                    {
                        pcl::PointXY p;
                        p.x = connectLink[i].waypoint[j].x;
                        p.y = connectLink[i].waypoint[j].y;
                        points.push_back(p);
                    }
                }

                for(int i = 0; i < connectLink[irrLink].startIrr; i++)
                {
                    pcl::PointXY p;
                    p.x = connectLink[irrLink].waypoint[i].x;
                    p.y = connectLink[irrLink].waypoint[i].y;
                    points.push_back(p);
                }

                getTotalDist(points, res.dist);
                return true;
            }

        }
        else
        {
            ros::Time time_end = ros::Time::now();
            double time_diff = (time_end - time_start).toSec();
            fprintf(stderr, "SrvFrontIrr Error : 해당 포인트가 경로상에 존재하지 않음!\n");
            fprintf(stderr, "FrontIrr Total time : %f ms\n", time_diff * 1000.0);
            fprintf(stderr, "▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲\n");
            return false;
        }
    }
    else
    {
        ros::Time time_end = ros::Time::now();
        double time_diff = (time_end - time_start).toSec();
        fprintf(stderr, "SrvFrontIrr Error : 가까운 링크를 찾을 수 없음!\n");
        fprintf(stderr, "FrontIrr Total time : %f ms\n", time_diff * 1000.0);
        fprintf(stderr, "▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲\n");
        return false;
    }
}

void Coremap::setGlobalPath(std::vector<Motion> motion, std::vector<a3_link> path, core_map::global_path &res, bool visualize)
{
    pcl::PointXY prevPos;
    prevPos.x = path[0].waypoint[0].x;
    prevPos.y = path[0].waypoint[0].y;
    double dist = 0;
    int motionIndex = 0;

    pcl::PointXY nextPos;
    double nextDist = 0;
    int nextMotionIndex = 0;

    double totalCost = 0;
    for (size_t i = 0; i < path.size(); i++)
    {
        totalCost += path[i].cost;
        core_map::path temp_path;
        temp_path.links.resize(1);
        temp_path.src = -1;
        temp_path.dst = -1;
        std::string src, dst;
        if (path[i].change_lane)
        {
            temp_path.changeLane = true;
            src = path[i].linkid.substr(10, 2);
            dst = path[i].linkid.substr(12, 2);
        }
        if (path[i].isStop)
            temp_path.isStop = true;

        size_t bundleIndex = path[i].bundle_index;
        for (size_t j = 0; j < hdmap.bundle[bundleIndex].size(); j++)
        {
            if (j == 0 && hdmap.bundle[bundleIndex][0] == nullptr)
                continue;
            if (j != path[i].my_index)
            {
                std::string bundleID = hdmap.bundle[bundleIndex][j]->linkid;
                if (bundleID.substr(8, 2) == "IL" || bundleID.substr(8, 2) == "IR" || bundleID.substr(8, 2) == "CR" || bundleID.substr(8, 2) == "CL")
                    continue;

                core_map::a3_link tempLink;
                std::vector<core_map::waypoint> bundleWaypoint;

                pcl::PointXY tempPrevPos;
                if (i == 0)
                {
                    tempPrevPos = prevPos;
                }
                else
                {
                    tempPrevPos.x = hdmap.bundle[bundleIndex][j]->waypoint[0].x;
                    tempPrevPos.y = hdmap.bundle[bundleIndex][j]->waypoint[0].y;
                }

                int tempMotionIndex = motionIndex;
                double temp_dist = dist;
                bool start1 = false;
                bool start2 = false;
                size_t xySize = hdmap.bundle[bundleIndex][j]->waypoint.size();
                for (size_t k = 0; k < xySize; k++)
                {
                    core_map::waypoint temp;
                    double min = DBL_MAX;
                    double bundleX = static_cast<double>(hdmap.bundle[bundleIndex][j]->waypoint[k].x);
                    double bundleY = static_cast<double>(hdmap.bundle[bundleIndex][j]->waypoint[k].y);
                    // 시작링크가 차선변경 할 경우 가장 가까운 포인트를 찾아서 해당 포인트부터 visualize
                    if (i == 0 && !start1)
                    {
                        double minDiff = sqrt((bundleX - tempPrevPos.x) * (bundleX - tempPrevPos.x) + (bundleY - tempPrevPos.y) * (bundleY - tempPrevPos.y));
                        size_t minIndex = 0;
                        while (1)
                        {
                            bundleX = static_cast<double>(hdmap.bundle[bundleIndex][j]->waypoint[k].x);
                            bundleY = static_cast<double>(hdmap.bundle[bundleIndex][j]->waypoint[k].y);
                            double diff = sqrt((bundleX - tempPrevPos.x) * (bundleX - tempPrevPos.x) + (bundleY - tempPrevPos.y) * (bundleY - tempPrevPos.y));
                            if (minDiff >= diff)
                            {
                                minDiff = diff;
                                minIndex = k;
                                if (k < hdmap.bundle[bundleIndex][j]->waypoint.size() - 1)
                                    k++;
                                else
                                {
                                    start1 = true;
                                    k = minIndex;
                                    bundleX = static_cast<double>(hdmap.bundle[bundleIndex][j]->waypoint[k].x);
                                    bundleY = static_cast<double>(hdmap.bundle[bundleIndex][j]->waypoint[k].y);
                                    tempPrevPos.x = bundleX;
                                    tempPrevPos.y = bundleY;
                                    break;
                                }
                            }
                            else
                            {
                                start1 = true;
                                k = minIndex;
                                bundleX = static_cast<double>(hdmap.bundle[bundleIndex][j]->waypoint[k].x);
                                bundleY = static_cast<double>(hdmap.bundle[bundleIndex][j]->waypoint[k].y);
                                tempPrevPos.x = bundleX;
                                tempPrevPos.y = bundleY;
                                break;
                            }
                        }
                    }
                    // 도착링크가 차선변경 할 경우 가장 가까운 포인트를 찾아서 해당 포인트까지 visualize
                    if (i == path.size() - 1 && !start2)
                    {
                        double tempX = static_cast<double>(hdmap.bundle[bundleIndex][j]->waypoint[k].x);
                        double tempY = static_cast<double>(hdmap.bundle[bundleIndex][j]->waypoint[k].y);
                        pcl::PointXY lastPos;
                        lastPos.x = path[i].waypoint.back().x;
                        lastPos.y = path[i].waypoint.back().y;
                        size_t minIndex = 0;
                        double minDiff = DBL_MAX;
                        xySize = k;
                        while (1)
                        {
                            tempX = static_cast<double>(hdmap.bundle[bundleIndex][j]->waypoint[xySize].x);
                            tempY = static_cast<double>(hdmap.bundle[bundleIndex][j]->waypoint[xySize].y);
                            double diff = sqrt((tempX - lastPos.x) * (tempX - lastPos.x) + (tempY - lastPos.y) * (tempY - lastPos.y));
                            if (minDiff >= diff)
                            {
                                minDiff = diff;
                                minIndex = xySize;
                                if (xySize < hdmap.bundle[bundleIndex][j]->waypoint.size() - 1)
                                    xySize++;
                                else
                                {
                                    start2 = true;
                                    xySize = minIndex;
                                    break;
                                }
                            }
                            else
                            {
                                start2 = true;
                                xySize = minIndex;
                                break;
                            }
                        }
                    }

                    temp_dist += sqrt((bundleX - tempPrevPos.x) * (bundleX - tempPrevPos.x) + (bundleY - tempPrevPos.y) * (bundleY - tempPrevPos.y));
                    for (size_t l = static_cast<size_t>(tempMotionIndex); l < motion.size(); l++)
                    {
                        double diff = fabs(motion[l].p - temp_dist);
                        if (diff < min)
                        {
                            min = diff;
                            temp.v = motion[l].v;
                            tempMotionIndex = static_cast<int>(l) + 1;
                        }
                        else
                            break;
                    }
                    temp.x = static_cast<double>(hdmap.bundle[bundleIndex][j]->waypoint[k].x);
                    temp.y = static_cast<double>(hdmap.bundle[bundleIndex][j]->waypoint[k].y);
                    temp.change_left = hdmap.bundle[bundleIndex][j]->waypoint[k].change_left;
                    temp.change_right = hdmap.bundle[bundleIndex][j]->waypoint[k].change_right;
                    bundleWaypoint.push_back(temp);
                    tempPrevPos.x = hdmap.bundle[bundleIndex][j]->waypoint[k].x;
                    tempPrevPos.y = hdmap.bundle[bundleIndex][j]->waypoint[k].y;
                }
                cvtMsg(*hdmap.bundle[bundleIndex][j], tempLink);
                tempLink.waypointAry = bundleWaypoint;
                if (tempLink.linkid[8] != 'C')
                {
                    if (tempLink.linkid[8] != 'I')
                    {
                        if (temp_path.changeLane)
                        {
                            if (tempLink.linkid.substr(10, 2) == src)
                            {
                                if (tempLink.lane != 91)
                                {
                                    temp_path.src = tempLink.lane;
                                    temp_path.links.push_back(tempLink);
                                }
                                else
                                {
                                    temp_path.src = 0;
                                    temp_path.links[0] = tempLink;
                                }
                            }
                            else if (tempLink.linkid.substr(10, 2) == dst)
                            {
                                if (tempLink.lane != 91)
                                {
                                    temp_path.dst = tempLink.lane;
                                    temp_path.links.push_back(tempLink);
                                }
                                else
                                {
                                    temp_path.dst = 0;
                                    temp_path.links[0] = tempLink;
                                }
                            }
                            else
                            {
                                if (tempLink.lane != 91)
                                    temp_path.links.push_back(tempLink);
                                else
                                    temp_path.links[0] = tempLink;
                            }
                        }
                        else
                        {
                            if (tempLink.lane != 91)
                                temp_path.links.push_back(tempLink);
                            else
                                temp_path.links[0] = tempLink;
                        }
                    }
                    else
                    {
                        temp_path.links.push_back(tempLink);
                    }
                }
            }
            else
            {
                core_map::a3_link tempLink;
                std::vector<core_map::waypoint> refWaypoint;
                nextPos = prevPos;
                nextMotionIndex = motionIndex;
                nextDist = dist;
                for (size_t j = 0; j < path[i].waypoint.size(); j++)
                {
                    core_map::waypoint temp;
                    double min = DBL_MAX;
                    double refX = static_cast<double>(path[i].waypoint[j].x);
                    double refY = static_cast<double>(path[i].waypoint[j].y);
                    nextDist += sqrt((refX - nextPos.x) * (refX - nextPos.x) + (refY - nextPos.y) * (refY - nextPos.y));
                    for (size_t l = static_cast<size_t>(nextMotionIndex); l < motion.size(); l++)
                    {
                        double diff = fabs(motion[l].p - nextDist);
                        if (diff < min)
                        {
                            min = diff;
                            temp.v = motion[l].v;
                            nextMotionIndex = static_cast<int>(l) + 1;
                        }
                        else
                            break;
                    }
                    temp.x = static_cast<double>(path[i].waypoint[j].x);
                    temp.y = static_cast<double>(path[i].waypoint[j].y);
                    temp.change_left = path[i].waypoint[j].change_left;
                    temp.change_right = path[i].waypoint[j].change_right;
                    refWaypoint.push_back(temp);
                    nextPos.x = path[i].waypoint[j].x;
                    nextPos.y = path[i].waypoint[j].y;
                }
                cvtMsg(*hdmap.bundle[bundleIndex][j], tempLink);
                tempLink.waypointAry = refWaypoint;
                if (!temp_path.changeLane)
                {
                    if (tempLink.linkid[8] != 'I')
                    {
                        if (tempLink.lane != 91)
                        {
                            temp_path.src = tempLink.lane;
                            temp_path.links.push_back(tempLink);
                        }
                        else
                        {
                            temp_path.src = 0;
                            temp_path.links[0] = tempLink;
                        }
                    }
                    else
                    {
                        temp_path.src = temp_path.links.size();
                        temp_path.links.push_back(tempLink);
                    }
                }
            }
        }
        res.pathAry.push_back(temp_path);
        prevPos = nextPos;
        motionIndex = nextMotionIndex;
        dist = nextDist;
    }
    res.header.frame_id = "world";
    res.totalCost = totalCost;
    dijkstra.getPathTime(res.totalTime);
    if (visualize)
    {
        visualizePath(res);
    }
}

void Coremap::roadInfo_thread()
{
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        geometry_msgs::TransformStamped tfStamped;
        try
        {
            tfStamped = tf_buffer.lookupTransform("world", "pos", ros::Time(0));
        }
        catch (tf2::TransformException &ex)
        {
            fprintf(stderr, "%s\n", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        std::unique_lock<std::mutex> lock_tcp(m_mutex);
        m_globalPos.x = tfStamped.transform.translation.x;
        m_globalPos.y = tfStamped.transform.translation.y;
        std::vector<Intersection> v2xMAP = m_v2xMAP;

        std::vector<Signal> v2xSPAT = m_v2xSPAT;

        core_map::global_path finalPath = m_finalPath;
        int fsmState = m_fsmState;
        lock_tcp.unlock();

        int linkIndex, linkPointIndex, leftLaneIndex, rightLaneIndex;
        hdmap.getRoadInfo(m_globalPos, linkIndex, linkPointIndex, leftLaneIndex, rightLaneIndex);

        if (linkIndex == -1)
            continue;

        core_map::a3_link link;
        //Set Link
        std::vector<core_map::waypoint> waypointAry;
        for (size_t i = 0; i < hdmap.m_link[linkIndex].waypoint.size(); i++)
        {
            core_map::waypoint temp;
            temp.x = hdmap.m_link[linkIndex].waypoint[i].x;
            temp.y = hdmap.m_link[linkIndex].waypoint[i].y;
            temp.change_left = hdmap.m_link[linkIndex].waypoint[i].change_left;
            temp.change_right = hdmap.m_link[linkIndex].waypoint[i].change_right;
            temp.v = 0;
            waypointAry.push_back(temp);
        }
        link.waypointAry = waypointAry;
        link.linkid = hdmap.m_link[linkIndex].linkid;
        link.fromnode = hdmap.m_link[linkIndex].fromnode;
        link.tonode = hdmap.m_link[linkIndex].tonode;
        link.roadtype = hdmap.m_link[linkIndex].roadtype;
        link.speed = hdmap.m_link[linkIndex].speed;
        link.lane = hdmap.m_link[linkIndex].lane;
        link.code = hdmap.m_link[linkIndex].code;
        link.cost = hdmap.m_link[linkIndex].cost;

        // 현재 globalPath에서의 몇번째 링크를 가는중인지 탐색
        int currentIndex = -1;
        bool find = false;
        for (int i = 0; i < finalPath.pathAry.size(); i++)
        {
            for(int j = 0; finalPath.pathAry[i].links.size(); j++)
            {
                if(finalPath.pathAry[i].links[j].linkid == link.linkid)
                {
                    currentIndex = i;
                    find = true;
                    break;
                }
            }
            if(find)
            {
                break;
            }
        }

        if (currentIndex != -1)
        {
            core_map::traffic_light_info trafficLightInfo;
            for (int i = currentIndex + 1; i < finalPath.pathAry.size(); i++)
            {
                int idIndex = -1;
                int signalGroup = -1;
                bool findTrafficLight = false;
                int src = finalPath.pathAry[i].src;
                // 동일한 교차로 Link 탐색
                if (finalPath.pathAry[i].links[src].linkid[8] == 'I')
                {
                    for (int j = 0; j < v2xMAP.size(); j++)
                    {
                        for (int k = 0; k < v2xMAP[j].data.size(); k++)
                        {
                            if (finalPath.pathAry[i].links[src].linkid == v2xMAP[j].data[k].linkID)
                            {
                                idIndex = j;
                                signalGroup = v2xMAP[j].data[k].signalGroup;
                                findTrafficLight = true;
                                j = v2xMAP.size();
                                break;
                            }
                        }
                    }
                }

                // 동일한 Link가 없을경우 이웃한 Link 탐색
                if ((idIndex == -1 || signalGroup == -1))
                {
                    for (int j = 0; j < v2xMAP.size(); j++)
                    {
                        for (int k = 0; k < v2xMAP[j].data.size(); k++)
                        {
                            if (finalPath.pathAry[i].links[src].linkid.substr(0, 10) == v2xMAP[j].data[k].linkID.substr(0, 10))
                            {
                                idIndex = j;
                                signalGroup = v2xMAP[j].data[k].signalGroup;
                                findTrafficLight = true;
                                j = v2xMAP.size();
                                break;
                            }
                        }
                    }
                }

                // 3번, 4번 교차로 우회전신호 대응
                // Intersection 3번 우회전 구간
                if (finalPath.pathAry[i].links[src].linkid == "155M0117IR0102")
                {
                    for (int j = 0; j < v2xMAP[2].data.size(); j++)
                    {
                        if (v2xMAP[2].data[j].linkID == "155M0117IL0101")
                        {
                            idIndex = 2;
                            signalGroup = v2xMAP[2].data[j].signalGroup;
                            findTrafficLight = true;
                            break;
                        }
                    }
                }
                // Intersection 4번 우회전 구간
                else if (finalPath.pathAry[i].links[src].linkid == "155M0120IR0102")
                {
                    for (int j = 0; j < v2xMAP[3].data.size(); j++)
                    {
                        if (v2xMAP[3].data[j].linkID == "155M0120IL0101")
                        {
                            idIndex = 3;
                            signalGroup = v2xMAP[3].data[j].signalGroup;
                            findTrafficLight = true;
                            break;
                        }
                    }
                }

                if (findTrafficLight)
                {
                    // 가장 가까운 wayPointIndex 탐색
                    int src = finalPath.pathAry[currentIndex].src;
                    double minDist = DBL_MAX;
                    int closeIndex = 0;
                    for (int j = 0; j < finalPath.pathAry[currentIndex].links[src].waypointAry.size(); j++)
                    {
                        double temp_x = finalPath.pathAry[currentIndex].links[src].waypointAry[j].x;
                        double temp_y = finalPath.pathAry[currentIndex].links[src].waypointAry[j].y;

                        double dist = sqrt((m_globalPos.x - temp_x) * (m_globalPos.x - temp_x) + (m_globalPos.y - temp_y) * (m_globalPos.y - temp_y));
                        if (minDist >= dist)
                        {
                            minDist = dist;
                            closeIndex = j;
                        }
                        else
                            break;
                    }

                    // 가장 가까운 신호등까지의 거리 계산
                    double dist = 0;
                    double prev_x = finalPath.pathAry[currentIndex].links[src].waypointAry[closeIndex].x;
                    double prev_y = finalPath.pathAry[currentIndex].links[src].waypointAry[closeIndex].y;
                    for (int j = closeIndex; j < finalPath.pathAry[currentIndex].links[src].waypointAry.size(); j++)
                    {
                        double temp_x = finalPath.pathAry[currentIndex].links[src].waypointAry[j].x;
                        double temp_y = finalPath.pathAry[currentIndex].links[src].waypointAry[j].y;
                        dist += sqrt((temp_x - prev_x) * (temp_x - prev_x) + (temp_y - prev_y) * (temp_y - prev_y));
                        prev_x = temp_x;
                        prev_y = temp_y;
                    }

                    for (int j = currentIndex + 1; j < i; j++)
                    {
                        src = finalPath.pathAry[j].src;
                        for (int k = 0; k < finalPath.pathAry[j].links[src].waypointAry.size(); k++)
                        {
                            double temp_x = finalPath.pathAry[j].links[src].waypointAry[k].x;
                            double temp_y = finalPath.pathAry[j].links[src].waypointAry[k].y;
                            dist += sqrt((temp_x - prev_x) * (temp_x - prev_x) + (temp_y - prev_y) * (temp_y - prev_y));
                            prev_x = temp_x;
                            prev_y = temp_y;
                        }
                    }

                    core_map::traffic_light tempInfo;
                    float validTime = 2.0;
                    src = finalPath.pathAry[i].src;
                    for (int j = 0; j < v2xSPAT[idIndex].data.size(); j++)
                    {
                        if (v2xSPAT[idIndex].data[j].signalGroup == signalGroup)
                        {
                            ros::Time currentTime = ros::Time::now();
                            double timeDiff = (currentTime - v2xSPAT[idIndex].recvTime).toSec();
                            if (timeDiff < validTime)
                            {
                                tempInfo.linkid = finalPath.pathAry[i].links[src].linkid;
                                tempInfo.dist = dist;
                                tempInfo.status = v2xSPAT[idIndex].data[j].status;
                                tempInfo.minEndTime = v2xSPAT[idIndex].data[j].minEndTime;
                                trafficLightInfo.infoAry.push_back(tempInfo);
                                break;
                            }
                            // validTime을 초과한 경우
                            else
                            {
                                if (trafficLightInfo.infoAry.size() < 1)
                                {
                                    if (v2xSPAT[idIndex].data[j].minEndTime - (timeDiff * 10.0) > 0)
                                    {
                                        tempInfo.linkid = finalPath.pathAry[i].links[src].linkid;
                                        tempInfo.dist = dist;
                                        tempInfo.status = v2xSPAT[idIndex].data[j].status;
                                        tempInfo.minEndTime = v2xSPAT[idIndex].data[j].minEndTime - (timeDiff * 10.0);
                                        trafficLightInfo.infoAry.push_back(tempInfo);
                                    }
                                    else
                                    {
                                        tempInfo.linkid = finalPath.pathAry[i].links[src].linkid;
                                        tempInfo.dist = dist;
                                        tempInfo.status = 0;
                                        tempInfo.minEndTime = v2xSPAT[idIndex].data[j].minEndTime;
                                    }
                                }
                            }
                        }
                    }
                }
            }
            close_traffic_light_pub.publish(trafficLightInfo);
        }
        else
        {
            core_map::traffic_light_info trafficLightInfo;
            close_traffic_light_pub.publish(trafficLightInfo);
        }

        std_msgs::Int8 fsm;
        fsm.data = fsmState;
        fsm_pub.publish(fsm);
        loop_rate.sleep();
    }
}

void Coremap::wave_thread()
{
    int size = -1;
    char buff[10000];
    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        std::string msgs;
        if ((size = recv(sockWAVE, buff, 1500, MSG_NOSIGNAL)) > 0)
        {
            msgs.append((char *)buff, size);

            std::string payload;
            obu_tcp_header_t *header = (obu_tcp_header_t *)&msgs[0];
            payload.append(msgs, sizeof(obu_tcp_header_t), header->payload_size);
//            fprintf(stderr,"Header(Message Type) : 0x%x\n",header->packet_type);
//            fprintf(stderr,"Header(Sequence) : %d\n",header->current_sequence);
//            fprintf(stderr,"Header(Payload Size) : %d\n",header->payload_size);
//            fprintf(stderr,"Header(Device Type) : 0x%X\n",header->device_type);
//            fprintf(stderr,"Header(Device ID) : %02X-%02X-%02X\n",header->device_id[2],header->device_id[1],header->device_id[0]);
//            fprintf(stderr,"Payload : 0x%s\n\n",array_to_hex_str((char*)payload.c_str(),payload.size()).c_str());

            std_msgs::String msg;
            msg.data = msgs;
            v2x_pub.publish(msg);

            MessageFrame_t *msgFrame = nullptr;
            std::vector<uint8_t> mypayload(payload.begin(), payload.end());
            uint8_t *buffer = &mypayload[0];
            auto res = uper_decode(nullptr, &asn_DEF_MessageFrame, (void **)&msgFrame, buffer, mypayload.size(), 0, 0);

            switch (res.code)
            {
            case asn_dec_rval_code_e::RC_OK:
                switch (msgFrame->messageId)
                {
                case dsrc_msg_id::BASIC_SAFETY_MESSAGE:
//                    fprintf(stderr, "[recv bsm]\n");
                    break;
                case dsrc_msg_id::PROBE_VEHICLE_DATA:
                    fprintf(stderr, "[recv pvd]\n");
                    break;
                case dsrc_msg_id::SIGNAL_PHASE_AND_TIMING_MESSAGE:
//                    fprintf(stderr,"[recv spat]\n");
                    recvSPAT(msgFrame);
                    break;
                case dsrc_msg_id::MAP_DATA:
//                    fprintf(stderr,"[recv map]\n");
                    recvMAP(msgFrame);
                    break;
                case dsrc_msg_id::TRAVELER_INFORMATION:
                    fprintf(stderr, "[recv tim]\n");
                    break;
                case dsrc_msg_id::ROAD_SIDE_ALERT:
                    fprintf(stderr, "[recv tim]\n");
                    break;
                case dsrc_msg_id::RTCM_CORRECTIONS:
                    fprintf(stderr, "[recv rtcm]\n");
                    break;
                default:
                    break;
                }
//                asn_fprint(stderr, &asn_DEF_MessageFrame, msgFrame);
                break;
            case asn_dec_rval_code_e::RC_WMORE:
                fprintf(stderr, "\n[RC_WMORE]\n");
                break;
            case asn_dec_rval_code_e::RC_FAIL:
                fprintf(stderr, "\n[RC_FAIL]\n");
                break;
            }
        }
        loop_rate.sleep();
    }
}

void Coremap::lte_recv_thread()
{
    ros::Rate loop_rate(10);

    bool receiveStart = true;
    int payloadSize = 0;
    std::string totalMsg = "";
    char buff[10000];

    while (ros::ok())
    {
        std::unique_lock<std::mutex> lock_tcp(m_mutex);
        int fsmState = m_fsmState;
        lock_tcp.unlock();

        switch (fsmState)
        {
        // 서버 접속 요청 1초 이후에도 응답이 없을경우
        case state::MISSION_INIT:
            if ((ros::Time::now() - m_reqTime).toSec() > 1.1)
                access_mission_server();
            break;
        // 미션 매칭 요청 3초 이후에도 응답이 없을경우
        case state::MISSION_CHECK:
            if ((ros::Time::now() - m_reqTime).toSec() > 3.1)
                callRequest(m_currentMissionIndex);
            break;
        }

        // 미션 실패로 인한 패널티
        if(m_penalty)
        {
            double timeDiff = (ros::Time::now() - m_penaltyTime).toSec();
            if(timeDiff < 11.0)
            {
                fprintf(stderr, "Mission Fail Penalty %f sec remain...", 11.0 - timeDiff);
            }
            else
            {
                m_penalty = false;
                std::unique_lock<std::mutex> lock_tcp(m_mutex);
                m_fsmState = state::WAIT_ORDER;
                fprintf(stderr, "- State Change : WAIT_ORDER\n\n");
                lock_tcp.unlock();
            }
        }

        int size = -1;
        std::string currentPacket;
        if ((size = recv(sockLTE, buff, 2000, MSG_NOSIGNAL)) > 0)
        {
            currentPacket.append((char *)buff, size);
            totalMsg += currentPacket;
        }

        if (receiveStart)
        {
            obu_tcp_header_t *header = (obu_tcp_header_t *)&currentPacket[0];
            payloadSize = header->payload_size;
            receiveStart = false;
        }

        if (totalMsg.size() == payloadSize + sizeof(obu_tcp_header_t))
        {
            uint16_t checksum;
            memcpy(&checksum, &totalMsg[sizeof(obu_tcp_header_t) + payloadSize - 2], 2);
            unsigned char payload[payloadSize - 2];
            memcpy(payload, &totalMsg[sizeof(obu_tcp_header_t)], payloadSize - 2);
            if (checksum == crc_16(payload, payloadSize - 2))
            {
                std_msgs::String msg;
                msg.data = totalMsg;
                mission_pub.publish(msg);

                obu_tcp_header_t *header = (obu_tcp_header_t *)&totalMsg[0];
//                fprintf(stderr, "-------------------------------------------------\n");
//                fprintf(stderr, "Header(Message Type) : 0x%x\n", header->packet_type);
//                fprintf(stderr, "Header(Sequence) : %d\n", header->current_sequence);
//                fprintf(stderr, "Header(Payload Size) : %d\n", header->payload_size);
//                fprintf(stderr, "Header(Device Type) : 0x%X\n", header->device_type);
//                fprintf(stderr, "Header(Device ID) : %02X-%02X-%02X\n", header->device_id[2], header->device_id[1], header->device_id[0]);
//                fprintf(stderr, "totalMsg Size : %d\n", totalMsg.size());
                switch (header->packet_type)
                {
                case lte_packet_id::ACCESS_RESTRICTION_NOTIFICATION:
                    checkAccessRestriction(payload);
                    break;
                case lte_packet_id::ACCESS_PERMISSION_RESPONSE:
                    checkAccessPermission(payload);
                    break;
                case lte_packet_id::LOCATION_MESSAGE_ERROR:
                    checkLocationMessage(payload);
                    break;
                case lte_packet_id::TAXI_CALL_LIST:
                    checkCallList(payload);
                    break;
                case lte_packet_id::CALL_RESPONSE:
                    checkCallResponse(payload);
                    break;
                case lte_packet_id::GETTING_ON_CONFIRM:
                    checkGettingOn(payload);
                    break;
                case lte_packet_id::GETTING_OFF_CONFIRM:
                    checkGettingOff(payload);
                    break;
                case lte_packet_id::MISSION_GIVING_UP:
                    checkMissionGivingUp(payload);
                    break;
                }
            }
            else
            {
                fprintf(stderr, "(LTE Recv) Checksum ERROR!!\n");
            }
            totalMsg = "";
            receiveStart = true;
        }
        loop_rate.sleep();
    }
    close(sockLTE);
}

void Coremap::lte_send_thread()
{
    ros::Rate loop_rate(10);
    ros::Time ref = ros::Time::now();
    while (ros::ok())
    {
        std::unique_lock<std::mutex> lock_tcp(m_mutex);
        unsigned char sequence = m_sequence;
        m_sequence++;
        lock_tcp.unlock();

        obu_tcp_header_t header;
        header.packet_type = lte_packet_id::VEHICLE_LOCATION_INFORMATION;
        header.current_sequence = sequence;
        header.payload_size = 0x19;
        header.device_type = obu_tcp_header_t::D_CLIENT;
        header.device_id[0] = cbnu_id[0];
        header.device_id[1] = cbnu_id[1];
        header.device_id[2] = cbnu_id[2];

        vehicle_location loc;
        if (runType == TEST_V2X)
        {
            loc.lat = 35.880143;
            loc.lon = 128.626841;
            loc.elev = 100;
            loc.heading = 0;
            loc.speed = 0;
        }
        else if (runType == REAL_POS)
        {
            lock_tcp.lock();
            double lat = m_latitude;
            double lon = m_longitude;
            float elev = m_elevation;
            unsigned short heading = m_heading;
            unsigned char speed = m_speed;
            lock_tcp.unlock();

            loc.lat = lat;
            loc.lon = lon;
            loc.elev = elev;
            loc.heading = heading;
            loc.speed = speed;
        }
        unsigned char msg[34];
        memcpy(&msg[0], &header, sizeof(obu_tcp_header_t));
        memcpy(&msg[9], &loc, sizeof(vehicle_location));
        uint16_t checksum = crc_16(&msg[9], sizeof(vehicle_location));
        memcpy(&msg[32], &checksum, 2);

//        int remainBuffer = -1;
//        ioctl(sockLTE, SIOCOUTQ, &remainBuffer);
//        fprintf(stderr, "Remain Buffer : %d\n", remainBuffer);

        int size = -1;

        lock_tcp.lock();
        if ((size = ::send(sockLTE, msg, 34, MSG_NOSIGNAL)) > 0)
        {
            ros::Time start = ros::Time::now();
//            fprintf(stderr, "(seq %d) Send Vehicle Location Information!\n\n", header.current_sequence);
//            fprintf(stderr, "diff : %fms\n", (start - ref).toSec() * 1000.0);
            ref = start;
        }
        lock_tcp.unlock();

        loop_rate.sleep();
    }
}

void Coremap::recvSPAT(MessageFrame_t *msgFrame)
{
    int id = (msgFrame->value.choice.SPAT.intersections.list.array[0]->id.id - 10000) / 10;

    Signal tempSignal;
    tempSignal.recvTime = ros::Time::now();
    tempSignal.intersectionID = id;

    int stateCount = msgFrame->value.choice.SPAT.intersections.list.array[0]->states.list.count;
    for (int i = 0; i < stateCount; i++)
    {
        SignalData tempData;
        tempData.signalGroup = msgFrame->value.choice.SPAT.intersections.list.array[0]->states.list.array[i]->signalGroup;
        tempData.status = msgFrame->value.choice.SPAT.intersections.list.array[0]->states.list.array[i]->state_time_speed.list.array[0]->eventState;
        tempData.minEndTime = msgFrame->value.choice.SPAT.intersections.list.array[0]->states.list.array[i]->state_time_speed.list.array[0]->timing->minEndTime;

        tempSignal.data.push_back(tempData);
    }

    std::unique_lock<std::mutex> lock_tcp(m_mutex);
    if (id > 0 && id < 23)
    {
        m_v2xSPAT[id - 1] = tempSignal;
    }
    lock_tcp.unlock();
}

void Coremap::recvMAP(MessageFrame_t *msgFrame)
{
    Markerarray nodeList;
    int id = (msgFrame->value.choice.MapData.intersections->list.array[0]->id.id - 10000) / 10;

    int count = 0;
    double ref_x;
    double ref_y;
    double currentLat = msgFrame->value.choice.MapData.intersections->list.array[0]->refPoint.lat / 10000000.0;
    double currentLon = msgFrame->value.choice.MapData.intersections->list.array[0]->refPoint.Long / 10000000.0;
    transGps(currentLat, currentLon, ref_x, ref_y);

    visualization_msgs::Marker temp;
    temp.header.frame_id = "world";
    temp.header.stamp = ros::Time::now();
    temp.ns = std::string("Alphacity-") + std::to_string(id);
    temp.id = count;
    temp.type = visualization_msgs::Marker::CYLINDER;
    temp.action = visualization_msgs::Marker::ADD;
    temp.pose.position.x = static_cast<double>(ref_x);
    temp.pose.position.y = static_cast<double>(ref_y);
    temp.pose.position.z = 5.0;
    temp.pose.orientation.x = 0.0;
    temp.pose.orientation.y = 0.0;
    temp.pose.orientation.z = 0.0;
    temp.pose.orientation.w = 1.0;
    temp.scale.x = 1.0;
    temp.scale.y = 1.0;
    temp.scale.z = 10.0;
    temp.color.a = 0.8f;
    temp.color.r = 0.0;
    temp.color.g = 1.0;
    temp.color.b = 0.0;
    nodeList.markers.push_back(temp);
    count++;

    Intersection tempIntersection;
    tempIntersection.intersectionID = id;

    int laneSize = msgFrame->value.choice.MapData.intersections->list.array[0]->laneSet.list.count;
    for (int i = 0; i < laneSize; i++)
    {
        double temp_x = msgFrame->value.choice.MapData.intersections->list.array[0]->laneSet.list.array[i]->nodeList.choice.nodes.list.array[0]->delta.choice.node_XY6.x / 100.0;
        double temp_y = msgFrame->value.choice.MapData.intersections->list.array[0]->laneSet.list.array[i]->nodeList.choice.nodes.list.array[0]->delta.choice.node_XY6.y / 100.0;

        double final_x = ref_x + temp_x;
        double final_y = ref_y + temp_y;
        visualization_msgs::Marker temp1;
        temp1.header.frame_id = "world";
        temp1.header.stamp = ros::Time::now();
        temp1.ns = std::string("Alphacity-") + std::to_string(id);
        temp1.id = count;
        temp1.type = visualization_msgs::Marker::CYLINDER;
        temp1.action = visualization_msgs::Marker::ADD;
        temp1.pose.position.x = static_cast<double>(final_x);
        temp1.pose.position.y = static_cast<double>(final_y);
        temp1.pose.position.z = 5.0;
        temp1.pose.orientation.x = 0.0;
        temp1.pose.orientation.y = 0.0;
        temp1.pose.orientation.z = 0.0;
        temp1.pose.orientation.w = 1.0;
        temp1.scale.x = 1.0;
        temp1.scale.y = 1.0;
        temp1.scale.z = 10.0;
        temp1.color.a = 0.8f;
        temp1.color.r = 1.0;
        temp1.color.g = 1.0;
        temp1.color.b = 1.0;
        nodeList.markers.push_back(temp1);
        count++;

        pcl::PointXY p;
        p.x = final_x;
        p.y = final_y;
        std::vector<a3_link> intersectionLink;
        hdmap.getIntersection(p, intersectionLink);

        if (intersectionLink.empty() || msgFrame->value.choice.MapData.intersections->list.array[0]->laneSet.list.array[i]->connectsTo == nullptr)
            continue;

        int connectToSize = msgFrame->value.choice.MapData.intersections->list.array[0]->laneSet.list.array[i]->connectsTo->list.count;
        for (int j = 0; j < connectToSize; j++)
        {
            int lane = msgFrame->value.choice.MapData.intersections->list.array[0]->laneSet.list.array[i]->connectsTo->list.array[j]->connectingLane.lane;

            int realSize = msgFrame->value.choice.MapData.intersections->list.array[0]->laneSet.list.count;

            if (lane > realSize)
                continue;

            pcl::PointXY nodeXY;
            nodeXY.x = ref_x + msgFrame->value.choice.MapData.intersections->list.array[0]->laneSet.list.array[lane - 1]->nodeList.choice.nodes.list.array[0]->delta.choice.node_XY6.x / 100.0;
            nodeXY.y = ref_y + msgFrame->value.choice.MapData.intersections->list.array[0]->laneSet.list.array[lane - 1]->nodeList.choice.nodes.list.array[0]->delta.choice.node_XY6.y / 100.0;

            int toIndex;
            hdmap.getNodeInfo(nodeXY, toIndex);

            if (toIndex != -1)
            {
                for (int k = 0; k < intersectionLink.size(); k++)
                {
                    if (intersectionLink[k].to_index == toIndex)
                    {
                        IntersectionData tempData;
                        tempData.signalGroup = *msgFrame->value.choice.MapData.intersections->list.array[0]->laneSet.list.array[i]->connectsTo->list.array[j]->signalGroup;
                        tempData.linkID = intersectionLink[k].linkid;
                        tempIntersection.data.push_back(tempData);
                    }
                }
            }
        }

        std::unique_lock<std::mutex> lock_tcp(m_mutex);
        if (id > 0 && id < 23)
        {
            m_v2xMAP[id - 1] = tempIntersection;
        }
        lock_tcp.unlock();
    }
    rviz_v2x_map_pub.publish(nodeList);
}

void Coremap::access_mission_server()
{
    std::unique_lock<std::mutex> lock_tcp(m_mutex);
    unsigned char sequence = m_sequence;
    m_sequence++;
    lock_tcp.unlock();

    obu_tcp_header_t header;
    header.packet_type = lte_packet_id::ACCESS_PERMISSION_REQUEST;
    header.current_sequence = sequence;
    header.payload_size = 0x16;
    header.device_type = obu_tcp_header_t::D_CLIENT;
    header.device_id[0] = cbnu_id[0];
    header.device_id[1] = cbnu_id[1];
    header.device_id[2] = cbnu_id[2];

    access_permission access;
    if (runType == TEST_V2X)
    {
        access.lat = 35.880143;
        access.lon = 128.626841;
        access.elev = 100;
    }
    else if (runType == REAL_POS)
    {
        std::unique_lock<std::mutex> lock_tcp(m_mutex);
        double lat = m_latitude;
        double lon = m_longitude;
        float elev = m_elevation;
        lock_tcp.unlock();

        access.lat = lat;
        access.lon = lon;
        access.elev = elev;
    }

    unsigned char msg[31];
    memcpy(&msg[0], &header, sizeof(obu_tcp_header_t));
    memcpy(&msg[9], &access, sizeof(access_permission));
    uint16_t checksum = crc_16(&msg[9], sizeof(access_permission));
    memcpy(&msg[29], &checksum, 2);

    int size = -1;
    lock_tcp.lock();
    if ((size = ::send(sockLTE, msg, 31, MSG_NOSIGNAL)) > 0)
    {
        m_reqTime = ros::Time::now();
        fprintf(stderr, "(seq %d) Send Access Permission Message!\n", header.current_sequence);
    }
    lock_tcp.unlock();
}

void Coremap::callRequest(int missionID)
{
    std::unique_lock<std::mutex> lock_tcp(m_mutex);
    unsigned char sequence = m_sequence;
    m_sequence++;
    lock_tcp.unlock();

    obu_tcp_header_t header;
    header.packet_type = lte_packet_id::CALL_REQUEST;
    header.current_sequence = sequence;
    header.payload_size = 0x03;
    header.device_type = obu_tcp_header_t::D_CLIENT;
    header.device_id[0] = cbnu_id[0];
    header.device_id[1] = cbnu_id[1];
    header.device_id[2] = cbnu_id[2];

    call_request req;
    req.requestCallID = static_cast<unsigned char>(missionID);

    unsigned char msg[12];
    memcpy(&msg[0], &header, sizeof(obu_tcp_header_t));
    memcpy(&msg[9], &req, sizeof(call_request));
    uint16_t checksum = crc_16(&msg[9], sizeof(call_request));
    memcpy(&msg[10], &checksum, 2);

    int size = -1;

    lock_tcp.lock();
    if ((size = ::send(sockLTE, msg, 12, MSG_NOSIGNAL)) > 0)
    {
        m_reqTime = ros::Time::now();
        fprintf(stderr, "(seq %d) Send Call Request Message!\n", header.current_sequence);
    }
    lock_tcp.unlock();
}

void Coremap::checkAccessRestriction(unsigned char *payload)
{
    fprintf(stderr, "Access Restriction Error!\n");
    switch (*payload)
    {
    case 0x01:
        fprintf(stderr, "정의되지 않은 Device ID 사용\n\n");
        break;
    case 0x02:
        fprintf(stderr, "대회용으로 제공하지 않은 LTE 장비 사용\n\n");
        break;
    case 0x03:
        fprintf(stderr, "정의되지 않은 packetType 사용\n\n");
        break;
    case 0x04:
        fprintf(stderr, "서비스 시나리오 순서에 맞지 않는 packetType 사용\n\n");
        break;
    case 0x05:
        fprintf(stderr, "packetType에 정의된 payloadLength가 아님\n");
        break;
    case 0x06:
        fprintf(stderr, "deviceType이 '0XCE'가 아님\n\n");
        break;
    case 0x07:
        fprintf(stderr, "동일한 sequence 3회 이상 전송\n\n");
        break;
    case 0x08:
        fprintf(stderr, "동일한 IP로 접속되어있음\n\n");
        break;
    case 0x09:
        fprintf(stderr, "TCP Connection 이후 10초 이내에 장치접속 요청 미실시\n\n");
        break;
    case 0x0A:
        fprintf(stderr, "다른팀의 ID 사용\n\n");
        break;
    case 0x0B:
        fprintf(stderr, "checksum 오류\n\n");
        break;
    case 0x0C:
        fprintf(stderr, "10회 이상 동일한 오류 발생(시스템 담당자에게 해제요청)\n\n");
        break;
    }
}

void Coremap::checkAccessPermission(unsigned char *payload)
{
    fprintf(stderr, "Access Permission Check\n");
    access_permission_response res;
    memcpy(&res, payload, sizeof(access_permission_response));
    switch (res.response)
    {
    case 0x01:
    {
        fprintf(stderr, "V2X 서버 접속 완료\n\n");
        std::unique_lock<std::mutex> lock_tcp(m_mutex);
        m_fsmState = state::WAIT_ORDER;
        fprintf(stderr, "- State Change : WAIT_ORDER\n\n");
        lock_tcp.unlock();
        m_lte_send_th = std::thread(&Coremap::lte_send_thread, this);
        break;
    }
    case 0x02:
        switch (res.errorCode)
        {
        case 0x00:
            fprintf(stderr, "오류 없음\n\n");
            break;
        case 0x01:
            fprintf(stderr, "latitude 오류\n\n");
            break;
        case 0x02:
            fprintf(stderr, "longitude 오류\n\n");
            break;
        case 0x03:
            fprintf(stderr, "elevation 오류\n\n");
            break;
        case 0x04:
            fprintf(stderr, "접속 허가 응답 이후 재접속 요청\n\n");
            break;
        case 0x05:
            fprintf(stderr, "V2X서버 응답을 받기 전 반복해서 접속요청\n\n");
            break;
        }
        break;
    }
}

void Coremap::checkLocationMessage(unsigned char *payload)
{
    // 비트단위 확인 필요!!
    fprintf(stderr, "Location Message Check\n");
    std::vector<int> errors;
    for (int i = 0; i < 8; i++)
    {
        if (*payload & nBit[i])
            errors.push_back(i);
    }

    for (int i = 0; i < errors.size(); i++)
    {
        switch (errors[i])
        {
        case 0:
            fprintf(stderr, "latitude 오류\n");
            break;
        case 1:
            fprintf(stderr, "longitude 오류\n");
            break;
        case 2:
            fprintf(stderr, "elevation 오류\n");
            break;
        case 3:
            fprintf(stderr, "heading 오류\n");
            break;
        case 4:
            fprintf(stderr, "접속 승인 후 5초 이내에 위치정보가 수신되지 않음\n");
            break;
        case 5:
            fprintf(stderr, "최근 5초 이내에 위치정보가 수신되지 않음\n");
            break;
        case 6:
            fprintf(stderr, "위치 정보 전송주기가 50ms 이하임\n");
            break;
        case 7:
            fprintf(stderr, "위치 정보 전송주기가 200ms 이상임\n");
            break;
        }
    }
    fprintf(stderr, "\n");
}

void Coremap::checkCallList(unsigned char *payload)
{
    call_list *tempList = (call_list *)&payload[0];
//    fprintf(stderr, "Call List Message\n");
//    fprintf(stderr, "Call_List(number of call) : %d\n", tempList->numberCall);
//    fprintf(stderr, "Call_List(available call) : %d\n", tempList->numberAvailableCall);
//    fprintf(stderr, "Call_List(number of irr ) : %d\n\n", tempList->numberIrregular);
    int numberCall = tempList->numberCall;
    call_data *call[numberCall];
    std::vector<Mission> tempMissionList;
//    tempMissionList.resize(tempList->numberCall);
    for (int i = 0; i < numberCall; i++)
    {
        call[i] = (call_data *)&payload[sizeof(call_list) + sizeof(call_data) * i];

        std::vector<int> includeMission;
        for (int j = 0; j < 4; j++)
        {
            if (call[i]->includeMission & nBit[j])
                includeMission.push_back(j);
        }

        Mission tempMission;
        tempMission.missionID = call[i]->id;
        tempMission.point = call[i]->point;
        tempMission.distance = call[i]->distance;
        tempMission.irrID = call[i]->irregularId;
        double src_x, src_y;
        transGps(call[i]->sLatitude, call[i]->sLongitude, src_x, src_y);
        tempMission.src.x = src_x;
        tempMission.src.y = src_y;
        double dst_x, dst_y;
        transGps(call[i]->eLatitude, call[i]->eLongitude, dst_x, dst_y);
        tempMission.dst.x = dst_x;
        tempMission.dst.y = dst_y;
        tempMission.includeMission = includeMission;

        switch (call[i]->status)
        {
        case 0x00:
            tempMission.status = MISSION_STATE_AVAILABLE;
            break;
        case 0x01:
            tempMission.status = MISSION_STATE_UNAVAILABLE;
            break;
        default:
            break;
        }
        tempMissionList.push_back(tempMission);
//        tempMissionList[tempMission.missionID - 1] = tempMission;
    }
    std::sort(tempMissionList.begin(), tempMissionList.end(), compareMission);

    std::unique_lock<std::mutex> lock_tcp(m_mutex);
    int numberIrr = tempList->numberIrregular;
    Markerarray irrLoc;
    std::vector<Irregular> tempIrregular;
    irregularLoc *irr[numberIrr];
    for (int i = 0; i < numberIrr; i++)
    {
        irr[i] = (irregularLoc *)&payload[sizeof(call_list) + sizeof(call_data) * numberCall + sizeof(irregularLoc) * i];

        double px[4], py[4];
        transGps(irr[i]->lat1, irr[i]->lon1, px[0], py[0]);
        transGps(irr[i]->lat2, irr[i]->lon2, px[1], py[1]);
        transGps(irr[i]->lat3, irr[i]->lon3, px[2], py[2]);
        transGps(irr[i]->lat4, irr[i]->lon4, px[3], py[3]);

        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = "world";
        std::string irrID = "irr" + std::to_string(irr[i]->irregularId);
        marker.ns = irrID;
        marker.id = i;
        marker.type = visualization_msgs::Marker::LINE_STRIP;

        Irregular temp;
        temp.irrID = irr[i]->irregularId;
        geometry_msgs::Point p[4];
        for(int j = 0; j < 4; j++)
        {
            temp.p[j].x = px[j];
            temp.p[j].y = py[j];
            p[j].x = px[j];
            p[j].y = py[j];
        }
        tempIrregular.push_back(temp);
        marker.points.push_back(p[0]);
        marker.points.push_back(p[1]);
        marker.points.push_back(p[2]);
        marker.points.push_back(p[3]);
        marker.points.push_back(p[0]);

        marker.scale.x = 0.5;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.lifetime = ros::Duration(3.0);
        irrLoc.markers.push_back(marker);
    }
    irr_pub.publish(irrLoc);

    // 미션리스트 초기화
    if (missionList.size() != numberCall)
    {
        // 비정형 구간 HDMAP에 등록
        for(int i = 0; i < tempIrregular.size(); i++)
        {
            std::vector<std::pair<double, int>> diff;
            for(int j = 1; j < 4; j++)
            {
                std::pair<double, int> tempDiff;
                tempDiff.first = hypot(tempIrregular[i].p[0].x - tempIrregular[i].p[j].x, tempIrregular[i].p[0].y - tempIrregular[i].p[j].y);
                tempDiff.second = j;
                diff.push_back(tempDiff);
            }

            std::sort(diff.begin(), diff.end());

            pcl::PointXY center[2];
            center[0].x = (tempIrregular[i].p[0].x + tempIrregular[i].p[diff[0].second].x) / 2.0;
            center[0].y = (tempIrregular[i].p[0].y + tempIrregular[i].p[diff[0].second].y) / 2.0;
            center[1].x = (tempIrregular[i].p[diff[1].second].x + tempIrregular[i].p[diff[2].second].x) / 2.0;
            center[1].y = (tempIrregular[i].p[diff[1].second].y + tempIrregular[i].p[diff[2].second].y) / 2.0;

            int linkIndex[2], linkPointIndex[2];
            hdmap.getLinkInfo(center[0], linkIndex[0], linkPointIndex[0]);
            hdmap.getLinkInfo(center[1], linkIndex[1], linkPointIndex[1]);

            if(linkIndex[0] == -1 || linkIndex[1] == -1)
                continue;

            if(linkIndex[0] == linkIndex[1])
            {
                hdmap.m_link[linkIndex[0]].isIrr = true;
                hdmap.m_link[linkIndex[0]].timecost += 30;

                int bundleIndex = hdmap.m_link[linkIndex[0]].bundle_index;
                for(int j = 0; j < hdmap.bundle[bundleIndex].size(); j++)
                {
                    if(hdmap.bundle[bundleIndex][j] == nullptr)
                        continue;

                    if(hdmap.bundle[bundleIndex][j]->from_index == hdmap.m_link[linkIndex[0]].from_index)
                    {
                        hdmap.bundle[bundleIndex][j]->timecost += 20;
                    }
                }

                if(linkPointIndex[0] > linkPointIndex[1])
                {
                    hdmap.m_link[linkIndex[0]].startIrr = linkPointIndex[1];
                    hdmap.m_link[linkIndex[0]].endIrr = linkPointIndex[0];
                }
                else
                {
                    hdmap.m_link[linkIndex[0]].startIrr = linkPointIndex[0];
                    hdmap.m_link[linkIndex[0]].endIrr = linkPointIndex[1];
                }
            }
        }

        missionList.resize(tempList->numberCall);
        missionList = tempMissionList;

        FILE *fp = fopen("/home/a/Mission_save.txt", "w");
        for (int i = 0; i < tempMissionList.size(); i++)
        {
            fprintf(fp, "Mission[%d] : (%f, %f) -> (%f, %f), score(%d), irrID(%d), dist(%d), include(", tempMissionList[i].missionID, tempMissionList[i].src.x, tempMissionList[i].src.y, tempMissionList[i].dst.x, tempMissionList[i].dst.y, tempMissionList[i].point, tempMissionList[i].irrID, tempMissionList[i].distance);
            for (int j = 0; j < tempMissionList[i].includeMission.size(); j++)
            {
                fprintf(fp, "%d", tempMissionList[i].includeMission[j]);
                if(j != tempMissionList[i].includeMission.size() -1)
                    fprintf(fp, ", ");
            }
            fprintf(fp, ")\n");
        }

        fprintf(fp, "\n");
        fprintf(fp, "Irr\n");
        for (int i = 0; i < numberIrr; i++)
        {
            irr[i] = (irregularLoc *)&payload[sizeof(call_list) + sizeof(call_data) * numberCall + sizeof(irregularLoc) * i];
            fprintf(fp, "Irr[%d] : (%f, %f), (%f, %f), (%f, %f), (%f, %f)\n", i, tempIrregular[i].p[0].x, tempIrregular[i].p[0].y, tempIrregular[i].p[1].x, tempIrregular[i].p[1].y, tempIrregular[i].p[2].x, tempIrregular[i].p[2].y, tempIrregular[i].p[3].x, tempIrregular[i].p[3].y);
        }

        fclose(fp);
    }

    pcl::PointCloud<pcl::PointXY> srcPoints, dstPoints;
    for (int i = 0; i < tempMissionList.size(); i++)
    {
        if (missionList[i].status != MISSION_STATE_CLEAR && missionList[i].status != MISSION_STATE_IMPOSSIBLE)
        {
            missionList[i].status = tempMissionList[i].status;
        }

        if (missionList[i].status == MISSION_STATE_AVAILABLE)
        {
            srcPoints.push_back(missionList[i].src);
            dstPoints.push_back(missionList[i].dst);
        }
    }
    int fsmState = m_fsmState;
    lock_tcp.unlock();

    if (fsmState == state::WAIT_ORDER)
    {
        std::unique_lock<std::mutex> lock_tcp(m_mutex);
        pcl::PointXY globalPos = m_globalPos;
        lock_tcp.unlock();

        visualizeMission(srcPoints, dstPoints);

        if (runOptimizer)
        {
            while (1)
            {
                if (optimizeRoute(globalPos, 1))
                    break;
            }
        }
    }
}

void Coremap::checkCallListForSim(unsigned char *payload)
{
    call_list *tempList = (call_list *)&payload[0];
    int numberCall = tempList->numberCall;
    call_data *call[numberCall];
    std::vector<Mission> tempMissionList;
//    tempMissionList.resize(tempList->numberCall);
    for (int i = 0; i < numberCall; i++)
    {
        call[i] = (call_data *)&payload[sizeof(call_list) + sizeof(call_data) * i];

        std::vector<int> includeMission;
        for (int j = 0; j < 4; j++)
        {
            if (call[i]->includeMission & nBit[j])
                includeMission.push_back(j);
        }

        Mission tempMission;
        tempMission.missionID = call[i]->id;
        tempMission.point = call[i]->point;
        tempMission.distance = call[i]->distance;
        tempMission.irrID = call[i]->irregularId;
        tempMission.src.x = call[i]->sLatitude;
        tempMission.src.y = call[i]->sLongitude;
        tempMission.dst.x = call[i]->eLatitude;
        tempMission.dst.y = call[i]->eLongitude;
        tempMission.includeMission = includeMission;

        switch (call[i]->status)
        {
        case 0x00:
            tempMission.status = MISSION_STATE_AVAILABLE;
            break;
        case 0x01:
            tempMission.status = MISSION_STATE_UNAVAILABLE;
            break;
        default:
            break;
        }
        tempMissionList.push_back(tempMission);
//        tempMissionList[tempMission.missionID - 1] = tempMission;
    }
    std::sort(tempMissionList.begin(), tempMissionList.end(), compareMission);

    std::unique_lock<std::mutex> lock_tcp(m_mutex);
    int numberIrr = tempList->numberIrregular;
    Markerarray irrLoc;
    std::vector<Irregular> tempIrregular;
    irregularLoc *irr[numberIrr];
    for (int i = 0; i < numberIrr; i++)
    {
        irr[i] = (irregularLoc *)&payload[sizeof(call_list) + sizeof(call_data) * numberCall + sizeof(irregularLoc) * i];

        double px[4], py[4];
        px[0] = irr[i]->lat1;
        py[0] = irr[i]->lon1;
        px[1] = irr[i]->lat2;
        py[1] = irr[i]->lon2;
        px[2] = irr[i]->lat3;
        py[2] = irr[i]->lon3;
        px[3] = irr[i]->lat4;
        py[3] = irr[i]->lon4;

        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = "world";
        std::string irrID = "irr" + std::to_string(irr[i]->irregularId);
        marker.ns = irrID;
        marker.id = i;
        marker.type = visualization_msgs::Marker::LINE_STRIP;

        Irregular temp;
        temp.irrID = irr[i]->irregularId;
        geometry_msgs::Point p[4];
        for(int j = 0; j < 4; j++)
        {
            temp.p[j].x = px[j];
            temp.p[j].y = py[j];
            p[j].x = px[j];
            p[j].y = py[j];
        }
        tempIrregular.push_back(temp);
        marker.points.push_back(p[0]);
        marker.points.push_back(p[1]);
        marker.points.push_back(p[2]);
        marker.points.push_back(p[3]);
        marker.points.push_back(p[0]);

        marker.scale.x = 0.5;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.lifetime = ros::Duration(3.0);
        irrLoc.markers.push_back(marker);
    }
    irr_pub.publish(irrLoc);

    // 미션리스트 초기화
    if (missionList.size() != numberCall)
    {
        // 비정형 구간 HDMAP에 등록
        for(int i = 0; i < tempIrregular.size(); i++)
        {
            std::vector<std::pair<double, int>> diff;
            for(int j = 1; j < 4; j++)
            {
                std::pair<double, int> tempDiff;
                tempDiff.first = hypot(tempIrregular[i].p[0].x - tempIrregular[i].p[j].x, tempIrregular[i].p[0].y - tempIrregular[i].p[j].y);
                tempDiff.second = j;
                diff.push_back(tempDiff);
            }

            std::sort(diff.begin(), diff.end());

            pcl::PointXY center[2];
            center[0].x = (tempIrregular[i].p[0].x + tempIrregular[i].p[diff[0].second].x) / 2.0;
            center[0].y = (tempIrregular[i].p[0].y + tempIrregular[i].p[diff[0].second].y) / 2.0;
            center[1].x = (tempIrregular[i].p[diff[1].second].x + tempIrregular[i].p[diff[2].second].x) / 2.0;
            center[1].y = (tempIrregular[i].p[diff[1].second].y + tempIrregular[i].p[diff[2].second].y) / 2.0;

            int linkIndex[2], linkPointIndex[2];
            hdmap.getLinkInfo(center[0], linkIndex[0], linkPointIndex[0]);
            hdmap.getLinkInfo(center[1], linkIndex[1], linkPointIndex[1]);

            if(linkIndex[0] == -1 || linkIndex[1] == -1)
                continue;

            if(linkIndex[0] == linkIndex[1])
            {
                hdmap.m_link[linkIndex[0]].isIrr = true;
                hdmap.m_link[linkIndex[0]].timecost += 30;

                int bundleIndex = hdmap.m_link[linkIndex[0]].bundle_index;
                for(int j = 0; j < hdmap.bundle[bundleIndex].size(); j++)
                {
                    if(hdmap.bundle[bundleIndex][j] == nullptr)
                        continue;

                    if(hdmap.bundle[bundleIndex][j]->from_index == hdmap.m_link[linkIndex[0]].from_index)
                    {
                        hdmap.bundle[bundleIndex][j]->timecost += 20;
                    }
                }

                if(linkPointIndex[0] > linkPointIndex[1])
                {
                    hdmap.m_link[linkIndex[0]].startIrr = linkPointIndex[1];
                    hdmap.m_link[linkIndex[0]].endIrr = linkPointIndex[0];
                }
                else
                {
                    hdmap.m_link[linkIndex[0]].startIrr = linkPointIndex[0];
                    hdmap.m_link[linkIndex[0]].endIrr = linkPointIndex[1];
                }
            }
        }

        missionList.resize(tempList->numberCall);
        missionList = tempMissionList;
    }

    pcl::PointCloud<pcl::PointXY> srcPoints, dstPoints;
    for (int i = 0; i < tempMissionList.size(); i++)
    {
        if (missionList[i].status != MISSION_STATE_CLEAR && missionList[i].status != MISSION_STATE_IMPOSSIBLE)
        {
            missionList[i].status = tempMissionList[i].status;
        }

        if (missionList[i].status == MISSION_STATE_AVAILABLE)
        {
            srcPoints.push_back(missionList[i].src);
            dstPoints.push_back(missionList[i].dst);
        }
    }
    int fsmState = m_fsmState;
    lock_tcp.unlock();

    if (fsmState == state::WAIT_ORDER)
    {
        std::unique_lock<std::mutex> lock_tcp(m_mutex);
        pcl::PointXY globalPos = m_globalPos;
        lock_tcp.unlock();

        visualizeMission(srcPoints, dstPoints);

        if (runOptimizer)
        {
            while (1)
            {
                if (optimizeRoute(globalPos, 1))
                    break;
            }
        }
    }
}

void Coremap::checkCallResponse(unsigned char *payload)
{
    call_response *res = (call_response *)&payload[0];

    if (res->errorCode == 0x00)
    {
        fprintf(stderr, "Call ID(%d) is Matched!\n", (int)res->matchingCallID);
        fprintf(stderr, "Estimated Time : %f\n", m_finalPath.totalTime);
        missionList[res->matchingCallID - 1].status = MISSION_STATE_CLEAR;
        path_pub.publish(m_finalPath);
        visualizePath(m_finalPath);
        std::unique_lock<std::mutex> lock_tcp(m_mutex);
        m_fsmState = state::WAIT_GETTING_ON;
        fprintf(stderr, "- State Change : WAIT_GETTING_ON\n\n");
        lock_tcp.unlock();
    }
    else if (res->errorCode == 0x10)
    {
        fprintf(stderr, "Call Selecting Fail\n");
        switch (res->responseStatus)
        {
        case 0x01:
            // 10초간 미션선택 못함
            fprintf(stderr, "Penalty-time 중 요청\n");
            break;
        case 0x02:
            fprintf(stderr, "대회 일시 정지중\n");
            break;
        case 0x03:
            fprintf(stderr, "수행중인 미션이 있음\n");
            break;
        case 0x04:
        {
            fprintf(stderr, "이미 수행한 미션임\n");
            std::unique_lock<std::mutex> lock_tcp(m_mutex);
            missionList[res->matchingCallID - 1].status = MISSION_STATE_CLEAR;
            lock_tcp.unlock();
            break;
        }
        case 0x05:
            fprintf(stderr, "현재 수행중인 미션임\n");
            break;
        case 0x06:
        {
            fprintf(stderr, "다른팀에서 수행중인 미션임\n");
            std::unique_lock<std::mutex> lock_tcp(m_mutex);
            missionList[res->matchingCallID - 1].status = MISSION_STATE_UNAVAILABLE;
            lock_tcp.unlock();
            break;
        }
        case 0x07:
            fprintf(stderr, "현재 해당 차량이 정비중임\n");
            break;
        }
        std::unique_lock<std::mutex> lock_tcp(m_mutex);
        m_fsmState = state::WAIT_ORDER;
        fprintf(stderr, "- State Change : WAIT_ORDER\n\n");
        lock_tcp.unlock();
    }
    else
    {
        switch (res->errorCode)
        {
        case 0x11:
            fprintf(stderr, "V2X 서버에서 콜 리스트를 송출하기 이전에 미션을 요청함\n");
            break;
        case 0x12:
            fprintf(stderr, "요청한 콜 아이디가 0이거나 60초과\n");
            break;
        }
        fprintf(stderr, "TCP Disconnected!\n\n");
        std::unique_lock<std::mutex> lock_tcp(m_mutex);
        m_fsmState = state::MISSION_INIT;
        fprintf(stderr, "- State Change : MISSION_INIT\n\n");
        lock_tcp.unlock();
        access_mission_server();
    }
}

void Coremap::checkGettingOn(unsigned char *payload)
{
    getting_on_confirm *confirm = (getting_on_confirm *)&payload[0];
    switch (confirm->confirmResult)
    {
    case 0x00:
    {
        fprintf(stderr, "Getting On Complete!\n");
        path_pub.publish(m_finalPath);
        visualizePath(m_finalPath);
        std::unique_lock<std::mutex> lock_tcp(m_mutex);
        m_fsmState = state::WAIT_GETTING_OFF;
        fprintf(stderr, "- State Change : WAIT_GETTING_OFF\n\n");
        lock_tcp.unlock();
        break;
    }
    case 0x01:
    {
        fprintf(stderr, "Fail to Gettion On!\n");
        m_penalty = true;
        m_penaltyTime = ros::Time::now();
        switch (confirm->errorCode)
        {
        case 0x01:
            fprintf(stderr, "평가자의 판단에 의한 승차실패\n");
            break;
        case 0x21:
            fprintf(stderr, "콜 아이디가 0이거나 60초과\n");
            break;
        case 0x22:
            fprintf(stderr, "latitude 오류\n");
            break;
        case 0x23:
            fprintf(stderr, "longitude 오류\n");
            break;
        case 0x24:
            fprintf(stderr, "매칭되지 않은 ID 전송\n");
            break;
        case 0x25:
            fprintf(stderr, "미션요청, 목적지 도착, 미션결과 확인 단계에서 출발지 도착 메세지 전송\n");
            break;
        }
        break;
    }
    }
}

void Coremap::checkGettingOff(unsigned char *payload)
{
    getting_off_confirm *confirm = (getting_off_confirm *)&payload[0];
    switch (confirm->confirmResult)
    {
    case 0x00:
    {
        fprintf(stderr, "Getting Off Complete!\n");
        std::unique_lock<std::mutex> lock_tcp(m_mutex);
        m_fsmState = state::WAIT_ORDER;
        fprintf(stderr, "- State Change : WAIT_ORDER\n\n");
        lock_tcp.unlock();
        break;
    }
    case 0x01:
    {
        fprintf(stderr, "Fail to Getting Off!\n");
        m_penalty = true;
        m_penaltyTime = ros::Time::now();
        switch (confirm->errorCode)
        {
        case 0x01:
            fprintf(stderr, "평가자의 판단에 의한 하차실패\n");
            break;
        case 0x31:
            fprintf(stderr, "콜 아이디가 0이거나 60초과\n");
            break;
        case 0x32:
            fprintf(stderr, "latitude 오류\n");
            break;
        case 0x33:
            fprintf(stderr, "longitude 오류\n");
            break;
        case 0x34:
            fprintf(stderr, "매칭되지 않은 ID 전송\n");
            break;
        case 0x35:
            fprintf(stderr, "미션요청, 출발지 도착, 미션결과 확인 단계에서 목적지 도착 메세지 전송\n");
            break;
        }
        break;
    }
    }
}

void Coremap::checkMissionComplete(unsigned char *payload)
{
    mission_complete_result *confirm = (mission_complete_result *)&payload[0];
    if (confirm->confirmResult == 0x00)
        fprintf(stderr, "Mission Complete!!\n");
    else
        fprintf(stderr, "Mission Fail!!\n");

    std::unique_lock<std::mutex> lock_tcp(m_mutex);
    m_fsmState = state::WAIT_ORDER;
    fprintf(stderr, "- State Change : WAIT_ORDER\n\n");
    lock_tcp.unlock();
}

void Coremap::checkMissionGivingUp(unsigned char *payload)
{
    mission_giving_up *msg = (mission_giving_up *)&payload[0];
    fprintf(stderr, "Error : Mission[%d] Giving Up!\n", msg->matchingCallID);
}

void Coremap::startingPointArrive()
{
    std::unique_lock<std::mutex> lock_tcp(m_mutex);
    unsigned char sequence = m_sequence;
    m_sequence++;
    lock_tcp.unlock();

    obu_tcp_header_t header;
    header.packet_type = lte_packet_id::STARTING_POINT_ARRIVE;
    header.current_sequence = sequence;
    header.payload_size = 0x14;
    header.device_type = obu_tcp_header_t::D_CLIENT;
    header.device_id[0] = cbnu_id[0];
    header.device_id[1] = cbnu_id[1];
    header.device_id[2] = cbnu_id[2];

    lock_tcp.lock();
    double lat = m_latitude;
    double lon = m_longitude;
    unsigned char speed = m_speed;
    lock_tcp.unlock();

    start_point_arrive arrive;
    arrive.matchingCallID = static_cast<unsigned char>(missionList[m_currentMissionIndex].missionID);
    arrive.lat = lat;
    arrive.lon = lon;
    arrive.speed = speed;

    unsigned char msg[29];
    memcpy(&msg[0], &header, sizeof(obu_tcp_header_t));
    memcpy(&msg[9], &arrive, sizeof(start_point_arrive));
    uint16_t checksum = crc_16(&msg[9], sizeof(start_point_arrive));
    memcpy(&msg[27], &checksum, 2);

    int size = -1;
    lock_tcp.lock();
    if ((size = ::send(sockLTE, msg, 29, MSG_NOSIGNAL)) > 0)
    {
        fprintf(stderr, "(seq %d) Mission[%d] Send Starting Point Arrive Message!\n", header.current_sequence, missionList[m_currentMissionIndex].missionID);
    }
    lock_tcp.unlock();
}

void Coremap::endPointArrive()
{
    std::unique_lock<std::mutex> lock_tcp(m_mutex);
    unsigned char sequence = m_sequence;
    m_sequence++;
    lock_tcp.unlock();

    obu_tcp_header_t header;
    header.packet_type = lte_packet_id::END_POINT_ARRIVE;
    header.current_sequence = sequence;
    header.payload_size = 0x14;
    header.device_type = obu_tcp_header_t::D_CLIENT;
    header.device_id[0] = cbnu_id[0];
    header.device_id[1] = cbnu_id[1];
    header.device_id[2] = cbnu_id[2];

    lock_tcp.lock();
    double lat = m_latitude;
    double lon = m_longitude;
    unsigned char speed = m_speed;
    lock_tcp.unlock();

    end_point_arrive arrive;
    arrive.matchingCallID = static_cast<unsigned char>(missionList[m_currentMissionIndex].missionID);
    arrive.lat = lat;
    arrive.lon = lon;
    arrive.speed = speed;

    unsigned char msg[29];
    memcpy(&msg[0], &header, sizeof(obu_tcp_header_t));
    memcpy(&msg[9], &arrive, sizeof(end_point_arrive));
    uint16_t checksum = crc_16(&msg[9], sizeof(end_point_arrive));
    memcpy(&msg[27], &checksum, 2);

    int size = -1;
    lock_tcp.lock();
    if ((size = ::send(sockLTE, msg, 29, MSG_NOSIGNAL)) > 0)
    {
        fprintf(stderr, "(seq %d) Mission[%d] Send End Point Arrive Message!\n", header.current_sequence, missionList[m_currentMissionIndex].missionID);
    }
    lock_tcp.unlock();
}

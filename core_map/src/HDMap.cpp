#include "HDMap.h"

Tilemap::Tilemap()
{
    tileSize = 25;
    // please initialize with your map size
    offset.x = 138;
    offset.y = 623;

    cell_x = (284 / tileSize) + 1;
    cell_y = (963 / tileSize) + 1;

    std::cout << "tileSize_x : " << cell_x << std::endl;
    std::cout << "tileSize_y : " << cell_y << std::endl;

    tileMapNode.resize(cell_x);
    tileMapLink.resize(cell_x);
    tileMapLane.resize(cell_x);

    for(int i = 0; i < cell_x; i++)
    {
        tileMapNode[i].resize(cell_y);
        tileMapLink[i].resize(cell_y);
        tileMapLane[i].resize(cell_y);
    }
}

HDMap::HDMap()
{
    offset.x = static_cast<float>(302533.174487);
    offset.y = static_cast<float>(4124215.34631);

    min_x = min_y = DBL_MAX;
    max_x = max_y = DBL_MIN;
}

void HDMap::readC1Node(std::string path)
{
    // A1_LANE 경로
    path = path + "/C1_NODE.csv";
    std::vector<std::string> buf;

    std::ifstream in(path);
    std::string temp;
    while(getline(in, temp))
    {
        // csv 파일 한 줄씩 buf에 저장
        buf.push_back(temp);
    }

    //0번째 줄은 형식 변수명 이므로 1번줄 부터 시작
    for(size_t i = 1; i < buf.size(); i++)
    {
        //데이터 초기화
        std::string _xy = "";
        std::string _nodeid = "";
        std::string _nodetype = "";
        std::string _date = "";
        std::string _remark = "";
        std::string _its_nodeid = "";
        std::string _hdufid = "";
        int index = 0;

        //XY 데이터 분리
        for (size_t j = 14; j < buf[i].size(); j++)
        {
            if (buf[i][j] != ')')
            {
                _xy += buf[i][j];
            }
            else
            {
                index = static_cast<int>(j) + 4;
                break;
            }
        }


        // fid 제거
        for (size_t j = index; j < buf[i].size(); j++)
        {
            if (buf[i][j] == ',')
            {
                index = static_cast<int>(j) + 1;
                break;
            }
        }

        //NODEID 데이터 분리
        for (size_t j = index; j < buf[i].size(); j++)
        {
            if (buf[i][j] != ',')
            {
                _nodeid += buf[i][j];
            }
            else
            {
                index = static_cast<int>(j) + 1;
                break;
            }
        }

        //NODETYPE 데이터 분리
        for (size_t j = index; j < buf[i].size(); j++)
        {
            if (buf[i][j] != ',')
            {
                _nodetype += buf[i][j];
            }
            else
            {
                index = static_cast<int>(j) + 1;
                break;
            }
        }

        //DATE 데이터 분리
        for (size_t j = index; j < buf[i].size(); j++)
        {
            if (buf[i][j] != ',')
            {
                _date += buf[i][j];
            }
            else
            {
                index = static_cast<int>(j) + 1;
                break;
            }
        }

        //REMARK 데이터 분리
        for (size_t j = index; j < buf[i].size(); j++)
        {
            if (buf[i][j] != ',')
            {
                _remark += buf[i][j];
            }
            else
            {
                index = static_cast<int>(j) + 1;
                break;
            }
        }

        //ITS_NODEID 데이터 분리
        for (size_t j = index; j < buf[i].size(); j++)
        {
            if (buf[i][j] != ',')
            {
                _its_nodeid += buf[i][j];
            }
            else
            {
                index = static_cast<int>(j) + 1;
                break;
            }
        }

        //HDUFID 데이터 분리
        for (size_t j = index; j < buf[i].size(); j++)
        {
            if (buf[i][j] != ',')
            {
                _hdufid += buf[i][j];
            }
            else
            {
                index = static_cast<int>(j) + 1;
                break;
            }
        }

        //위에서 분리한 _xy 데이터 콤마 단위로 분할
        char* cp_str = new char[_xy.size() + 1];
        strcpy(cp_str, _xy.c_str());

        char* tok;
        tok = strtok(cp_str, ",");
        std::vector<std::string> split_comma;
        while (tok != NULL)
        {
            temp = tok;
            split_comma.push_back(temp);
            tok = strtok(NULL, ",");
        }
        delete[] cp_str;

        //콤마 단위로 분할된 데이터 x, y분리
        std::string split_x;
        std::string split_y;
        c1_node temp_node;
        for(size_t j = 0; j < split_comma.size(); j++)
        {
            cp_str = new char[split_comma[j].size() + 1];
            strcpy(cp_str, split_comma[j].c_str());

            //X값 분리
            tok = strtok(cp_str, " ");
            split_x = tok;
            double x;
            x = stod(split_x);
            //Y값 분리
            tok = strtok(NULL," ");
            split_y = tok;
            double y;
            y = stod(split_y);

            x = x - offset.x;
            y = y - offset.y;

            if(x > max_x)
            {
                max_x = x;
            }
            if(y > max_y)
            {
                max_y = y;
            }
            if(x < min_x)
            {
                min_x = x;
            }
            if(y < min_y)
            {
                min_y = y;
            }

            pcl::PointXY point;
            point.x = static_cast<float>(x);
            point.y = static_cast<float>(y);
            temp_node.xy = point;

            delete[] cp_str;
        }
        temp_node.nodeid = _nodeid;
        temp_node.nodetype = stoi(_nodetype);
        temp_node.date = stoi(_date);
        temp_node.remark = _remark;
        temp_node.its_nodeid = _its_nodeid;
        temp_node.hdufid = _hdufid;
        temp_node.thisIndex = m_node.size();
        m_node.push_back(temp_node);

        int index_x = static_cast<int>((temp_node.xy.x + tilemap.offset.x) / tilemap.tileSize);
        int index_y = static_cast<int>((temp_node.xy.y + tilemap.offset.y) / tilemap.tileSize);
        tilemap.tileMapNode[index_x][index_y].push_back(static_cast<int>(i) - 1);
    }
    if(m_node.size() > 0)
    {
        std::cout << "Reading C1_NODE success!" << std::endl;
        std::cout << "node count : " << m_node.size() << std::endl << std::endl;
    }
    else
    {
        std::cout << "ERROR : Fail to read C1_NODE!" << std::endl;
    }
}

void HDMap::readA3Link(std::string path)
{
    std::vector<std::string> linkBundle;
    // A1_LANE 경로
    path = path + "/A3_LINK.csv";
    std::vector<std::string> buf;
    std::ifstream in(path);
    std::string temp;
    while(getline(in, temp))
    {
        // csv 파일 한 줄씩 buf에 저장
        buf.push_back(temp);
    }

    //0번째 줄은 형식 변수명 이므로 1번줄 부터 시작
    int point_count = 0; //포인트 갯수 카운트
    for(size_t i = 1; i < buf.size(); i++)
    {
        //데이터 초기화
        std::string _xy = "";
        std::string _linkid = "";
        std::string _fromnode = "";
        std::string _tonode = "";
        std::string _length = "";
        std::string _roadtype = "";
        std::string _roadno = "";
        std::string _speed = "";
        std::string _lane = "";
        std::string _code = "";
        std::string _gid = "";
        std::string _date = "";
        std::string _remark = "";
        std::string _its_linkid = "";
        std::string _hdufid = "";
        int index = 0;

        //XY 데이터 분리
        for (size_t j = 19; j < buf[i].size(); j++)
        {
            if (buf[i][j] != ')')
            {
                _xy += buf[i][j];
            }
            else
            {
                index = static_cast<int>(j) + 4;
                break;
            }
        }

        //LINKID 데이터 분리
        for (size_t j = index; j < buf[i].size(); j++)
        {
            if (buf[i][j] != ',')
            {
                _linkid += buf[i][j];
            }
            else
            {
                index = static_cast<int>(j) + 1;
                break;
            }
        }

        //FROMNODE 데이터 분리
        for (size_t j = index; j < buf[i].size(); j++)
        {
            if (buf[i][j] != ',')
            {
                _fromnode += buf[i][j];
            }
            else
            {
                index = static_cast<int>(j) + 1;
                break;
            }
        }

        //TONODE 데이터 분리
        for (size_t j = index; j < buf[i].size(); j++)
        {
            if (buf[i][j] != ',')
            {
                _tonode += buf[i][j];
            }
            else
            {
                index = static_cast<int>(j) + 1;
                break;
            }
        }

        //LENGTH 데이터 분리
        for (size_t j = index; j < buf[i].size(); j++)
        {
            if (buf[i][j] != ',')
            {
                _length += buf[i][j];
            }
            else
            {
                index = static_cast<int>(j) + 1;
                break;
            }
        }

        //ROADTYPE 데이터 분리
        for (size_t j = index; j < buf[i].size(); j++)
        {
            if (buf[i][j] != ',')
            {
                _roadtype += buf[i][j];
            }
            else
            {
                index = static_cast<int>(j) + 1;
                break;
            }
        }

        //ROADNO 데이터 분리
        for (size_t j = index; j < buf[i].size(); j++)
        {
            if (buf[i][j] != ',')
            {
                _roadno += buf[i][j];
            }
            else
            {
                index = static_cast<int>(j) + 1;
                break;
            }
        }

        //SPEED 데이터 분리
        for (size_t j = index; j < buf[i].size(); j++)
        {
            if (buf[i][j] != ',')
            {
                _speed += buf[i][j];
            }
            else
            {
                index = static_cast<int>(j) + 1;
                break;
            }
        }

        //LANE 데이터 분리
        for (size_t j = index; j < buf[i].size(); j++)
        {
            if (buf[i][j] != ',')
            {
                _lane += buf[i][j];
            }
            else
            {
                index = static_cast<int>(j) + 1;
                break;
            }
        }

        //CODE 데이터 분리
        for (size_t j = index; j < buf[i].size(); j++)
        {
            if (buf[i][j] != ',')
            {
                _code += buf[i][j];
            }
            else
            {
                index = static_cast<int>(j) + 1;
                break;
            }
        }

        //GID 데이터 분리
        for (size_t j = index; j < buf[i].size(); j++)
        {
            if (buf[i][j] != ',')
            {
                _gid += buf[i][j];
            }
            else
            {
                index = static_cast<int>(j) + 1;
                break;
            }
        }

        //DATE 데이터 분리
        for (size_t j = index; j < buf[i].size(); j++)
        {
            if (buf[i][j] != ',')
            {
                _date += buf[i][j];
            }
            else
            {
                index = static_cast<int>(j) + 1;
                break;
            }
        }

        //REMARK 데이터 분리
        for (size_t j = index; j < buf[i].size(); j++)
        {
            if (buf[i][j] != ',')
            {
                _remark += buf[i][j];
            }
            else
            {
                index = static_cast<int>(j) + 1;
                break;
            }
        }

        //ITS_LINKID 데이터 분리
        for (size_t j = index; j < buf[i].size(); j++)
        {
            if (buf[i][j] != ',')
            {
                _its_linkid += buf[i][j];
            }
            else
            {
                index = static_cast<int>(j) + 1;
                break;
            }
        }

        //HDUFID 데이터 분리
        for (size_t j = index; j < buf[i].size(); j++)
        {
            if (buf[i][j] != ',')
            {
                _hdufid += buf[i][j];
            }
            else
            {
                index = static_cast<int>(j) + 1;
                break;
            }
        }

        //위에서 분리한 _xy 데이터 콤마 단위로 분할
        char* cp_str = new char[_xy.size() + 1];
        strcpy(cp_str, _xy.c_str());

        char* tok;
        tok = strtok(cp_str, ",");
        std::vector<std::string> split_comma;
        while (tok != NULL)
        {
            temp = tok;
            split_comma.push_back(temp);
            tok = strtok(NULL, ",");
        }
        delete[] cp_str;

        //콤마 단위로 분할된 데이터 x, y분리
        std::string split_x;
        std::string split_y;
        a3_link temp_link;
        std::vector<pcl::PointXY> index_xy;
        for(size_t j = 0; j < split_comma.size(); j++)
        {
            cp_str = new char[split_comma[j].size() + 1];
            strcpy(cp_str, split_comma[j].c_str());

            //X값 분리
            tok = strtok(cp_str, " ");
            split_x = tok;
            double x;
            x = stod(split_x);
            //Y값 분리
            tok = strtok(NULL," ");
            split_y = tok;
            double y;
            y = stod(split_y);

            x = x - offset.x;
            y = y - offset.y;

            if(x > max_x)
            {
                max_x = x;
            }
            if(y > max_y)
            {
                max_y = y;
            }
            if(x < min_x)
            {
                min_x = x;
            }
            if(y < min_y)
            {
                min_y = y;
            }

            Waypoint point;
            point.x = static_cast<float>(x);
            point.y = static_cast<float>(y);

            double resolutionWaypoint = 0.5;
            if(temp_link.waypoint.size() > 0)
            {
                double x = temp_link.waypoint.back().x;
                double y = temp_link.waypoint.back().y;
                double dist = sqrt((point.x - x)*(point.x - x) + (point.y - y)*(point.y - y));

                if(dist > resolutionWaypoint)
                {
                    double prev_x = x;
                    double prev_y = y;
                    double newDist = sqrt((point.x - prev_x)*(point.x - prev_x) + (point.y - prev_y)*(point.y - prev_y));
                    while(1)
                    {
                        double rate = resolutionWaypoint / newDist;
                        double dx = point.x - prev_x;
                        double dy = point.y - prev_y;

                        double new_x = prev_x + (dx * rate);
                        double new_y = prev_y + (dy * rate);
                        Waypoint newPoint;
                        newPoint.x = static_cast<float>(new_x);
                        newPoint.y = static_cast<float>(new_y);
                        temp_link.waypoint.push_back(newPoint);

                        //Save Into Tile Map
                        bool isStore = false;
                        int index_x = static_cast<int>((newPoint.x + tilemap.offset.x) / tilemap.tileSize);
                        int index_y = static_cast<int>((newPoint.y + tilemap.offset.y) / tilemap.tileSize);
                        for(size_t j = 0; j < index_xy.size(); j++)
                        {
                            if(index_x == index_xy[j].x && index_y == index_xy[j].y)
                            {
                                isStore = true;
                            }
                        }
                        if(!isStore)
                        {
                            pcl::PointXY tempIndex;
                            tempIndex.x = index_x;
                            tempIndex.y = index_y;

                            index_xy.push_back(tempIndex);
                        }

                        point_count++;
                        newDist = sqrt((point.x - new_x)*(point.x - new_x) + (point.y - new_y)*(point.y - new_y));
                        if(newDist < resolutionWaypoint)
                            break;
                        else
                        {
                            prev_x = new_x;
                            prev_y = new_y;
                        }
                    }
                }
            }
            temp_link.waypoint.push_back(point);
            point_count++;

            //Save into Tile Map
            bool isStore = false;
            int index_x = static_cast<int>((point.x + tilemap.offset.x) / tilemap.tileSize);
            int index_y = static_cast<int>((point.y + tilemap.offset.y) / tilemap.tileSize);
            for(size_t j = 0; j < index_xy.size(); j++)
            {
                if(index_x == index_xy[j].x && index_y == index_xy[j].y)
                {
                    isStore = true;
                }
            }
            if(!isStore)
            {
                pcl::PointXY tempIndex;
                tempIndex.x = index_x;
                tempIndex.y = index_y;

                index_xy.push_back(tempIndex);
            }

            delete[] cp_str;
        }
        temp_link.linkid = _linkid;
        temp_link.fromnode = _fromnode;
        temp_link.tonode = _tonode;
        temp_link.length = _length;
        temp_link.roadtype = stoi(_roadtype);
        temp_link.roadno = _roadno;
        temp_link.speed = stoi(_speed);
        temp_link.lane = stoi(_lane);
        temp_link.code = stoi(_code);
        temp_link.gid = _gid;
        temp_link.date = _date;
        temp_link.remark = _remark;
        temp_link.its_linkid = _its_linkid;
        temp_link.hdufid = _hdufid;
        temp_link.isStop = false;
        temp_link.isIrr = false;
        temp_link.startIrr = false;
        temp_link.endIrr = false;

        //Link의 총 길이(cost) 계산
        temp_link.cost = 0;
        for(size_t j = 0; j < temp_link.waypoint.size() - 1; j++)
        {
            double temp_cost = sqrt((temp_link.waypoint[j + 1].x - temp_link.waypoint[j].x) * (temp_link.waypoint[j + 1].x - temp_link.waypoint[j].x) + (temp_link.waypoint[j + 1].y - temp_link.waypoint[j].y) * (temp_link.waypoint[j + 1].y - temp_link.waypoint[j].y));
            temp_link.cost += temp_cost;
        }
        temp_link.timecost = temp_link.cost / (temp_link.speed * 1000.0 / 3600.0);
        //fromnode, tonode 인덱스 탐색
        bool from_flag = false;
        bool to_flag = false;
        for(size_t j = 0; j < m_node.size(); j++)
        {
            if(m_node[j].nodeid == _fromnode)
            {
                temp_link.from_index = static_cast<int>(j);
                from_flag = true;
            }
            if(m_node[j].nodeid == _tonode)
            {
                temp_link.to_index = static_cast<int>(j);
                to_flag = true;
            }

            if(from_flag && to_flag)
                break;
        }

        //링크 묶음 생성
        bool exist = false;
        size_t bundleIndex = -1;
        if(_linkid[8] == 'I' || _linkid[8] == 'O')
        {
            for(size_t j = 0; j < linkBundle.size(); j++)
            {
                if(_linkid.substr(0, 9) == linkBundle[j])
                {
                    exist = true;
                    bundleIndex = j;
                    break;
                }
            }
        }
        else
        {
            for(size_t j = 0; j < linkBundle.size(); j++)
            {
                if(_linkid.substr(0, 8) == linkBundle[j])
                {
                    exist = true;
                    bundleIndex = j;
                    break;
                }
            }
        }
        //이미 존재하는 묶음이면 인덱스 저장
        if(exist)
        {
            temp_link.bundle_index = bundleIndex;
        }
        //처음 만들어지는 묶음이면 묶음 생성 후 저장
        else
        {
            if(_linkid[8] == 'I')
                linkBundle.push_back(_linkid.substr(0, 9));
            else
                linkBundle.push_back(_linkid.substr(0, 8));

            temp_link.bundle_index = linkBundle.size() - 1;
        }
        m_link.push_back(temp_link);

        //타일맵에 저장
        for(size_t j = 0; j < index_xy.size(); j++)
        {
            tilemap.tileMapLink[index_xy[j].x][index_xy[j].y].push_back(i - 1);
        }
    }

    //차선변경 링크 정보 추가
    for(size_t i = 0; i < m_link.size(); i++)
    {
        if(m_link[i].linkid[8] == 'C')
        {
            m_link[i].change_lane = true;
            std::string lane_name = m_link[i].linkid.substr(0, 8);
            std::string src_lane = m_link[i].linkid.substr(10, 2);
            std::string dst_lane = m_link[i].linkid.substr(12, 2);

            for(size_t j = 0; j < m_link.size(); j++)
            {
                if(m_link[j].linkid.substr(0, 8) == lane_name && m_link[j].linkid[8] != 'C')
                {
                    std::string laneCheck = m_link[j].linkid.substr(10, 4);
                    if(laneCheck.substr(0, 2) == src_lane)
                    {
                        m_link[i].src_index = j;
                    }
                    else if(laneCheck.substr(0, 2) == dst_lane)
                    {
                        m_link[i].dst_index = j;
                    }
                }
            }
        }
        else
        {
            m_link[i].change_lane = false;
        }
    }

    // Create Bundle
    bundle.resize(linkBundle.size());
    for(size_t i = 0; i < bundle.size(); i++)
    {
        bundle[i].push_back(nullptr);
//        bundle[i].resize(1);
//        bundle[i][0] = nullptr;
    }

    std::vector<a3_link*> changeLane;
    for(size_t i = 0; i < m_link.size(); i++)
    {
        size_t bundleIndex = m_link[i].bundle_index;

        if(!m_link[i].change_lane)
        {
            if(m_link[i].linkid[8] == 'I')
            {
                m_link[i].my_index = bundle[bundleIndex].size();
                bundle[bundleIndex].push_back(&m_link[i]);
            }
            else
            {
                if(m_link[i].lane == 91)
                {
                    bundle[bundleIndex][0] = &m_link[i];
                    m_link[i].my_index = 0;
                }
                else
                {
                    if(bundle[bundleIndex].size() - 1 < m_link[i].lane)
                    {
                        bundle[bundleIndex].resize(m_link[i].lane + 1);
                    }
                    m_link[i].my_index = m_link[i].lane;
                    bundle[bundleIndex][m_link[i].lane] = &m_link[i];
                }
            }
        }
        else
        {
            changeLane.push_back(&m_link[i]);
        }
    }

    for(size_t i = 0; i < changeLane.size(); i++)
    {
        size_t bundleIndex = changeLane[i]->bundle_index;
        changeLane[i]->timecost += 3.0;
        changeLane[i]->my_index = bundle[bundleIndex].size();
        bundle[bundleIndex].push_back(changeLane[i]);
    }

    if(m_link.size() > 0)
    {
        std::cout << "Reading A3_LINK success!" << std::endl;
        std::cout << "link count : " << m_link.size() << "\tpoint_count : " << point_count << std::endl << std::endl;
    }
    else
    {
        std::cout << "ERROR : Fail to read A3_LINK!" << std::endl;
    }
}

void HDMap::readA1Lane(std::string path)
{
    // A1_LANE 경로
    path = path + "/A1_LANE.csv";
    std::vector<std::string> buf;

    std::ifstream in(path);
    std::string temp;
    while(getline(in, temp))
    {
        // csv 파일 한 줄씩 buf에 저장
        buf.push_back(temp);
    }

    //0번째 줄은 형식 변수명 이므로 1번줄 부터 시작
    for(size_t i = 1; i< buf.size(); i++)
    {
        //데이터 초기화
        std::string _xy = "";
        std::string _r_link = "";
        std::string _l_link = "";
        std::string _lanetype = "";
        std::string _lanecode = "";
        std::string _barrier = "";
        std::string _lno = "";
        std::string _code = "";
        std::string _date = "";
        std::string _remark = "";
        std::string _hdufid = "";
        int index = 0;

        //XY 데이터 분리
        for (size_t j = 19; j < buf[i].size(); j++)
        {
            if (buf[i][j] != ')')
            {
                _xy += buf[i][j];
            }
            else
            {
                index = static_cast<int>(j) + 4;
                break;
            }
        }

        //R_LINK 데이터 분리
        for (size_t j = index; j < buf[i].size(); j++)
        {
            if (buf[i][j] != ',')
            {
                _r_link += buf[i][j];
            }
            else
            {
                index = static_cast<int>(j) + 1;
                break;
            }
        }

        //L_LINK 데이터 분리
        for (size_t j = index; j < buf[i].size(); j++)
        {
            if (buf[i][j] != ',')
            {
                _l_link += buf[i][j];
            }
            else
            {
                index = static_cast<int>(j) + 1;
                break;
            }
        }

        //LANETYPE 데이터 분리
        for (size_t j = index; j < buf[i].size(); j++)
        {
            if (buf[i][j] != ',')
            {
                _lanetype += buf[i][j];
            }
            else
            {
                index = static_cast<int>(j) + 1;
                break;
            }
        }

        //LANECODE 데이터 분리
        for (size_t j = index; j < buf[i].size(); j++)
        {
            if (buf[i][j] != ',')
            {
                _lanecode += buf[i][j];
            }
            else
            {
                index = static_cast<int>(j) + 1;
                break;
            }
        }

        //BARRIER 데이터 분리
        for (size_t j = index; j < buf[i].size(); j++)
        {
            if (buf[i][j] != ',')
            {
                _barrier += buf[i][j];
            }
            else
            {
                index = static_cast<int>(j) + 1;
                break;
            }
        }

        //LNO 데이터 분리
        for (size_t j = index; j < buf[i].size(); j++)
        {
            if (buf[i][j] != ',')
            {
                _lno += buf[i][j];
            }
            else
            {
                index = static_cast<int>(j) + 1;
                break;
            }
        }

        //CODE 데이터 분리
        for (size_t j = index; j < buf[i].size(); j++)
        {
            if (buf[i][j] != ',')
            {
                _code += buf[i][j];
            }
            else
            {
                index = static_cast<int>(j) + 1;
                break;
            }
        }

        //DATE 데이터 분리
        for (size_t j = index; j < buf[i].size(); j++)
        {
            if (buf[i][j] != ',')
            {
                _date += buf[i][j];
            }
            else
            {
                index = static_cast<int>(j) + 1;
                break;
            }
        }

        //REMARK 데이터 분리
        for (size_t j = index; j < buf[i].size(); j++)
        {
            if (buf[i][j] != ',')
            {
                _remark += buf[i][j];
            }
            else
            {
                index = static_cast<int>(j) + 1;
                break;
            }
        }

        //HDUFID 데이터 분리
        for (size_t j = index; j < buf[i].size(); j++)
        {
            if (buf[i][j] != ',')
            {
                _hdufid += buf[i][j];
            }
            else
            {
                index = static_cast<int>(j) + 1;
                break;
            }
        }

        //위에서 분리한 _xy 데이터 콤마 단위로 분할
        char* cp_str = new char[_xy.size() + 1];
        strcpy(cp_str, _xy.c_str());

        char* tok;
        tok = strtok(cp_str, ",");
        std::vector<std::string> split_comma;
        while (tok != NULL)
        {
            temp = tok;
            split_comma.push_back(temp);
            tok = strtok(NULL, ",");
        }
        delete[] cp_str;

        //콤마 단위로 분할된 데이터 x, y분리
        std::string split_x;
        std::string split_y;
        a1_lane temp_lane;
        std::vector<pcl::PointXY> index_xy;
        for(size_t j = 0; j < split_comma.size(); j++)
        {
            cp_str = new char[split_comma[j].size() + 1];
            strcpy(cp_str, split_comma[j].c_str());

            //X값 분리
            tok = strtok(cp_str, " ");
            split_x = tok;
            double x;
            x = stod(split_x);
            //Y값 분리
            tok = strtok(NULL," ");
            split_y = tok;
            double y;
            y = stod(split_y);

            x = x - offset.x;
            y = y - offset.y;

            if(x > max_x)
            {
                max_x = x;
            }
            if(y > max_y)
            {
                max_y = y;
            }
            if(x < min_x)
            {
                min_x = x;
            }
            if(y < min_y)
            {
                min_y = y;
            }

            pcl::PointXY point;
            point.x = static_cast<float>(x);
            point.y = static_cast<float>(y);
            temp_lane.xy.push_back(point);

            bool isStore = false;
            int index_x = static_cast<int>((point.x + tilemap.offset.x) / tilemap.tileSize);
            int index_y = static_cast<int>((point.y + tilemap.offset.y) / tilemap.tileSize);
            for(size_t j = 0; j < index_xy.size(); j++)
            {
                if(index_x == index_xy[j].x && index_y == index_xy[j].y)
                {
                    isStore = true;
                }
            }
            if(!isStore)
            {
                pcl::PointXY tempIndex;
                tempIndex.x = index_x;
                tempIndex.y = index_y;

                index_xy.push_back(tempIndex);
            }

            delete[] cp_str;
        }

        temp_lane.r_link = _r_link;
        temp_lane.l_link = _l_link;
        temp_lane.lanetype = _lanetype;
        temp_lane.lanecode = _lanecode;
        temp_lane.barrier = _barrier;
        temp_lane.lno = _lno;
        temp_lane.code = _code;
        temp_lane.date = _date;
        temp_lane.remark = _remark;
        temp_lane.hdufid = _hdufid;

        //r_link, l_link 인덱스 탐색
        bool r_flag = false;
        bool l_flag = false;
        for(size_t j = 0; j < m_link.size(); j++)
        {
            if(m_link[j].linkid == _r_link)
            {
                temp_lane.r_index = static_cast<int>(j);
                r_flag = true;
            }
            if(m_link[j].linkid == _l_link)
            {
                temp_lane.l_index = static_cast<int>(j);
                l_flag = true;
            }

            if(r_flag && l_flag)
                break;
        }
        m_lane.push_back(temp_lane);

        //타일맵에 저장
        for(size_t j = 0; j < index_xy.size(); j++)
        {
            tilemap.tileMapLane[index_xy[j].x][index_xy[j].y].push_back(i - 1);
        }
    }
    if(m_lane.size() > 0)
    {
        std::cout << "Reading A1_LANE success!" << std::endl;
        std::cout << "lane count : " << m_lane.size() << std::endl
                  << std::endl;
    }
    else
    {
        std::cout << "ERROR : Fail to read A1_LANE!" << std::endl;
    }
}

void HDMap::readA2Stop(std::string path)
{
    // A1_LANE 경로
    path = path + "/A2_STOP.csv";
    std::vector<std::string> buf;

    std::ifstream in(path);
    std::string temp;
    while(getline(in, temp))
    {
        // csv 파일 한 줄씩 buf에 저장
        buf.push_back(temp);
    }

    //0번째 줄은 형식 변수명 이므로 1번줄 부터 시작
    for(size_t i = 1; i< buf.size(); i++)
    {
        //데이터 초기화
        std::string _xy = "";
        std::string _linkid = "";
        std::string _code = "";
        std::string _date = "";
        std::string _remark = "";
        std::string _hdufid = "";
        int index = 0;

        //XY 데이터 분리
        for (size_t j = 19; j < buf[i].size(); j++)
        {
            if (buf[i][j] != ')')
            {
                _xy += buf[i][j];
            }
            else
            {
                index = static_cast<int>(j) + 4;
                break;
            }
        }

        //LINKID 데이터 분리
        for (size_t j = index; j < buf[i].size(); j++)
        {
            if (buf[i][j] != ',')
            {
                _linkid += buf[i][j];
            }
            else
            {
                index = static_cast<int>(j) + 1;
                break;
            }
        }

        //CODE 데이터 분리
        for (size_t j = index; j < buf[i].size(); j++)
        {
            if (buf[i][j] != ',')
            {
                _code += buf[i][j];
            }
            else
            {
                index = static_cast<int>(j) + 1;
                break;
            }
        }

        //DATE 데이터 분리
        for (size_t j = index; j < buf[i].size(); j++)
        {
            if (buf[i][j] != ',')
            {
                _date += buf[i][j];
            }
            else
            {
                index = static_cast<int>(j) + 1;
                break;
            }
        }

        //REMARK 데이터 분리
        for (size_t j = index; j < buf[i].size(); j++)
        {
            if (buf[i][j] != ',')
            {
                _remark += buf[i][j];
            }
            else
            {
                index = static_cast<int>(j) + 1;
                break;
            }
        }

        //HDUFID 데이터 분리
        for (size_t j = index; j < buf[i].size(); j++)
        {
            if (buf[i][j] != ',')
            {
                _hdufid += buf[i][j];
            }
            else
            {
                index = static_cast<int>(j) + 1;
                break;
            }
        }

        //위에서 분리한 _xy 데이터 콤마 단위로 분할
        char* cp_str = new char[_xy.size() + 1];
        strcpy(cp_str, _xy.c_str());

        char* tok;
        tok = strtok(cp_str, ",");
        std::vector<std::string> split_comma;
        while (tok != NULL)
        {
            temp = tok;
            split_comma.push_back(temp);
            tok = strtok(NULL, ",");
        }
        delete[] cp_str;

        //콤마 단위로 분할된 데이터 x, y분리
        std::string split_x;
        std::string split_y;
        a2_stop temp_stop;
        std::vector<pcl::PointXY> index_xy;
        for(size_t j = 0; j < split_comma.size(); j++)
        {
            cp_str = new char[split_comma[j].size() + 1];
            strcpy(cp_str, split_comma[j].c_str());

            //X값 분리
            tok = strtok(cp_str, " ");
            split_x = tok;
            double x;
            x = stod(split_x);
            //Y값 분리
            tok = strtok(NULL," ");
            split_y = tok;
            double y;
            y = stod(split_y);

            x = x - offset.x;
            y = y - offset.y;

            if(x > max_x)
            {
                max_x = x;
            }
            if(y > max_y)
            {
                max_y = y;
            }
            if(x < min_x)
            {
                min_x = x;
            }
            if(y < min_y)
            {
                min_y = y;
            }

            pcl::PointXY point;
            point.x = static_cast<float>(x);
            point.y = static_cast<float>(y);
            temp_stop.xy.push_back(point);
            delete[] cp_str;
        }

        temp_stop.linkid = _linkid;
        temp_stop.code = stoi(_code);
        temp_stop.date = stoi(_date);
        temp_stop.hdufid = _hdufid;

        for(size_t j = 0; j < m_link.size(); j++)
        {
            if(m_link[j].linkid.substr(0, 8) == temp_stop.linkid.substr(0, 8) && m_link[j].linkid[8]!= 'I')
            {
                m_link[j].isStop = true;
            }
        }
        m_stop.push_back(temp_stop);
    }
    if(m_stop.size() > 0)
    {
        std::cout << "Reading A2_STOP success!" << std::endl;
        std::cout << "stop count : " << m_stop.size() << std::endl;
    }
    else
    {
        std::cout << "ERROR : Fail to read A2_STOP!" << std::endl;
    }
}

void HDMap::wayPointCheck()
{
    for (int i = 0; i < m_link.size(); i++)
    {
        for (int j = 0; j < m_link[i].waypoint.size(); j++)
        {
            pcl::PointXY p;
            p.x = m_link[i].waypoint[j].x;
            p.y = m_link[i].waypoint[j].y;

            int leftLaneIndex;
            int rightLaneIndex;
            getLaneInfo(p, i, leftLaneIndex, rightLaneIndex);

            if (leftLaneIndex != -1)
                m_link[i].waypoint[j].change_left = m_lane[leftLaneIndex].lanetype;
            else
                m_link[i].waypoint[j].change_left = "000";

            if (rightLaneIndex != -1)
                m_link[i].waypoint[j].change_right = m_lane[rightLaneIndex].lanetype;
            else
                m_link[i].waypoint[j].change_right = "000";
        }
    }
}

void HDMap::readHDMap(std::string path)
{
    std::cout << "▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼" << std::endl;
    readC1Node(path);
    readA3Link(path);
    readA1Lane(path);
    readA2Stop(path);
    wayPointCheck();
    std::cout << "min_x : " << min_x << ", min_y : " << min_y << std::endl;
    std::cout << "max_x : " << max_x << ", max_y : " << max_y << std::endl;
    std::cout << "▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲" << std::endl;
}

void HDMap::getHDMap(std::vector<c1_node> &node, std::vector<a3_link> &link, std::vector<a1_lane> &lane, std::vector<a2_stop> &stop)
{
    for (size_t i = 0; i < m_node.size(); i++)
    {
        node.push_back(m_node[i]);
    }

    for (size_t i = 0; i < m_link.size(); i++)
    {
        link.push_back(m_link[i]);
    }

    for (size_t i = 0; i < m_lane.size(); i++)
    {
        lane.push_back(m_lane[i]);
    }

    for (size_t i = 0; i < m_stop.size(); i++)
    {
        stop.push_back(m_stop[i]);
    }
}

void HDMap::getTilemapIndex(pcl::PointXY p, int &index_x, int &index_y)
{
    index_x = static_cast<int>((p.x + tilemap.offset.x) / tilemap.tileSize);
    index_y = static_cast<int>((p.y + tilemap.offset.y) / tilemap.tileSize);
}

void HDMap::getLinkInfo(pcl::PointXY p, int &linkIndex, int &linkPointIndex)
{
    int index_x, index_y;
    getTilemapIndex(p, index_x, index_y);

    double minDist = DBL_MAX;
    linkIndex = -1;
    linkPointIndex = -1;

    for (int i = 0; i < 9; i++)
    {
        int x = index_x - 1 + (i % 3);
        int y = index_y - 1 + (i / 3);

        if (x >= 0 && y >= 0 && x < tilemap.cell_x && y < tilemap.cell_y)
        {
            for (size_t j = 0; j < tilemap.tileMapLink[x][y].size(); j++)
            {
                int index = tilemap.tileMapLink[x][y][j];
                if (m_link[index].linkid[8] != 'C')
                {
                    for (size_t k = 0; k < m_link[index].waypoint.size() - 1; k++)
                    {
                        double dist = sqrt((p.x - m_link[index].waypoint[k].x) * (p.x - m_link[index].waypoint[k].x) + (p.y - m_link[index].waypoint[k].y) * (p.y - m_link[index].waypoint[k].y));

                        if (dist < minDist && dist < 10)
                        {
                            minDist = dist;
                            linkIndex = index;
                            linkPointIndex = static_cast<int>(k);
                        }
                    }
                }
            }
        }
    }
}

void HDMap::getChangeLinkInfo(int bundleIndex, int from_index, std::vector<a3_link> &result)
{
    for(int i = 0; i < bundle[bundleIndex].size(); i++)
    {
        if(bundle[bundleIndex][i] == nullptr)
            continue;

        if(bundle[bundleIndex][i]->change_lane && bundle[bundleIndex][i]->from_index == from_index)
            result.push_back(*bundle[bundleIndex][i]);
    }
}

void HDMap::getStartChangeLinkInfo(std::string linkid, pcl::PointXY p, std::vector<int> &linkIndex, std::vector<int> &linkPointIndex)
{
    int index_x, index_y;
    getTilemapIndex(p, index_x, index_y);
    for (int i = 0; i < 9; i++)
    {
        int x = index_x - 1 + (i % 3);
        int y = index_y - 1 + (i / 3);

        if (x >= 0 && y >= 0 && x < tilemap.cell_x && y < tilemap.cell_y)
        {
            for (size_t j = 0; j < tilemap.tileMapLink[x][y].size(); j++)
            {
                int index = tilemap.tileMapLink[x][y][j];
                std::string thisLinkid = m_link[index].linkid;

                // Check if is already exist
                bool isStore = false;
                for (size_t k = 0; k < linkIndex.size(); k++)
                {
                    if (linkIndex[k] == index)
                        isStore = true;
                }

                if (thisLinkid.substr(0, 8) == linkid.substr(0, 8) && thisLinkid[8] == 'C' && thisLinkid.substr(10, 2) == linkid.substr(10, 2) && !isStore)
                {
                    double diff = DBL_MAX;
                    double minK = 0;
                    for (size_t k = 0; k < m_link[index].waypoint.size(); k++)
                    {
                        double tempDist = sqrt((p.x - m_link[index].waypoint[k].x) * (p.x - m_link[index].waypoint[k].x) + (p.y - m_link[index].waypoint[k].y) * (p.y - m_link[index].waypoint[k].y));
                        if (diff > tempDist)
                        {
                            diff = tempDist;
                            minK = k;
                        }
                        else
                            break;
                    }
                    linkIndex.push_back(index);
                    linkPointIndex.push_back(minK);
                }
            }
        }
    }
}

void HDMap::getEndChangeLinkInfo(std::string linkid, pcl::PointXY p, std::vector<int> &linkIndex, std::vector<int> &linkPointIndex)
{
    int index_x, index_y;
    getTilemapIndex(p, index_x, index_y);
    for (int i = 0; i < 9; i++)
    {
        int x = index_x - 1 + (i % 3);
        int y = index_y - 1 + (i / 3);

        if (x >= 0 && y >= 0 && x < tilemap.cell_x && y < tilemap.cell_y)
        {
            for (size_t j = 0; j < tilemap.tileMapLink[x][y].size(); j++)
            {
                int index = tilemap.tileMapLink[x][y][j];
                std::string thisLinkid = m_link[index].linkid;

                // Check if is already exist
                bool isStore = false;
                for (size_t k = 0; k < linkIndex.size(); k++)
                {
                    if (linkIndex[k] == index)
                        isStore = true;
                }

                if (thisLinkid.substr(0, 8) == linkid.substr(0, 8) && thisLinkid[8] == 'C' && thisLinkid.substr(12, 2) == linkid.substr(10, 2) && !isStore)
                {
                    double diff = DBL_MAX;
                    double minK = 0;
                    for (size_t k = 0; k < m_link[index].waypoint.size(); k++)
                    {
                        double tempDist = sqrt((p.x - m_link[index].waypoint[k].x) * (p.x - m_link[index].waypoint[k].x) + (p.y - m_link[index].waypoint[k].y) * (p.y - m_link[index].waypoint[k].y));
                        if (diff > tempDist)
                        {
                            diff = tempDist;
                            minK = k;
                        }
                        else
                            break;
                    }
                    linkIndex.push_back(index);
                    linkPointIndex.push_back(minK);
                }
            }
        }
    }
}

void HDMap::getLaneInfo(pcl::PointXY p, int linkIndex, int &leftLaneIndex, int &rightLaneIndex)
{
    int index_x, index_y;
    getTilemapIndex(p, index_x, index_y);
    double leftMin = DBL_MAX;
    double rightMin = DBL_MAX;
    leftLaneIndex = -1;
    rightLaneIndex = -1;
    for (int i = 0; i < 9; i++)
    {
        int x = index_x - 1 + (i % 3);
        int y = index_y - 1 + (i / 3);

        if (x >= 0 && y >= 0 && x < tilemap.cell_x && y < tilemap.cell_y)
        {
            for (size_t j = 0; j < tilemap.tileMapLane[x][y].size(); j++)
            {
                int index = tilemap.tileMapLane[x][y][j];

                if (m_lane[index].r_link == m_link[linkIndex].linkid)
                {
                    for (size_t k = 0; k < m_lane[index].xy.size(); k++)
                    {
                        double dist = sqrt((p.x - m_lane[index].xy[k].x) * (p.x - m_lane[index].xy[k].x) + (p.y - m_lane[index].xy[k].y) * (p.y - m_lane[index].xy[k].y));

                        if (dist < leftMin)
                        {
                            leftMin = dist;
                            leftLaneIndex = index;
                        }
                    }
                }
                else if (m_lane[index].l_link == m_link[linkIndex].linkid)
                {
                    for (size_t k = 0; k < m_lane[index].xy.size(); k++)
                    {
                        double dist = sqrt((p.x - m_lane[index].xy[k].x) * (p.x - m_lane[index].xy[k].x) + (p.y - m_lane[index].xy[k].y) * (p.y - m_lane[index].xy[k].y));

                        if (dist < rightMin)
                        {
                            rightMin = dist;
                            rightLaneIndex = index;
                        }
                    }
                }
            }
        }
    }
}

void HDMap::getRoadInfo(pcl::PointXY p, int &linkIndex, int &linkPointIndex, int &leftLaneIndex, int &rightLaneIndex)
{
    getLinkInfo(p, linkIndex, linkPointIndex);
    getLaneInfo(p, linkIndex, leftLaneIndex, rightLaneIndex);
}

void HDMap::getNodeInfo(pcl::PointXY p, int &nodeIndex)
{
    nodeIndex = -1;
    int index_x, index_y;
    getTilemapIndex(p, index_x, index_y);
    double minDist = DBL_MAX;
    for (int i = 0; i < 9; i++)
    {
        int x = index_x - 1 + (i % 3);
        int y = index_y - 1 + (i / 3);

        if (x >= 0 && y >= 0 && x < tilemap.cell_x && y < tilemap.cell_y)
        {
            for (size_t j = 0; j < tilemap.tileMapNode[x][y].size(); j++)
            {
                int index = tilemap.tileMapNode[x][y][j];
                double tempDist = sqrt((p.x - m_node[index].xy.x) * (p.x - m_node[index].xy.x) + (p.y - m_node[index].xy.y) * (p.y - m_node[index].xy.y));
                if (tempDist < minDist && tempDist <= 1.0)
                {
                    minDist = tempDist;
                    nodeIndex = index;
                }
            }
        }
    }
}

void HDMap::getIntersection(pcl::PointXY p, std::vector<a3_link> &intersectionLink)
{
    int fromIndex;
    getNodeInfo(p, fromIndex);

    if (fromIndex != -1)
    {
        p.x = m_node[fromIndex].xy.x;
        p.y = m_node[fromIndex].xy.y;

        int index_x, index_y;
        getTilemapIndex(p, index_x, index_y);
        for (int i = 0; i < tilemap.tileMapLink[index_x][index_y].size(); i++)
        {
            int index = tilemap.tileMapLink[index_x][index_y][i];
            if (m_link[index].from_index == fromIndex && m_link[index].linkid[8] == 'I')
            {
                intersectionLink.push_back(m_link[index]);
            }
        }
    }
}

void HDMap::getConnectedLink(int frmoNode, std::vector<a3_link> &links)
{
    int index_x, index_y;
    getTilemapIndex(m_node[frmoNode].xy, index_x, index_y);

    for (int i = 0; i < tilemap.tileMapLink[index_x][index_y].size(); i++)
    {
        int linkIndex = tilemap.tileMapLink[index_x][index_y][i];
        a3_link temp = m_link[linkIndex];
        if (temp.to_index == frmoNode)
        {
            links.push_back(temp);
        }
    }
}

bool HDMap::isIrrBundle(int bundleIndex)
{
    for(int i = 0; i < bundle[bundleIndex].size(); i++)
    {
        if(bundle[bundleIndex][i] == nullptr)
            continue;

        if(bundle[bundleIndex][i]->isIrr)
            return true;
    }
    return false;
}

double HDMap::checkSignalIntersection(std::string linkid)
{
    // Intersection 1
    if(linkid == "155M0040IL0101" || linkid == "155M0040IL0102")
    {
        return 30.0;
    }
    // Intersection 2
    else if(linkid == "155M0052IL0101")
    {
        return 30.0;
    }
    else if(linkid == "155M0052IZ0101")
    {
        return 30.0;
    }
    else if(linkid == "155M0052IZ0202")
    {
        return 30.0;
    }
    else if(linkid == "155M0054IR0201")
    {
        return 30.0;
    }
    else if(linkid == "155M0054IZ0101")
    {
        return 30.0;
    }
    else if(linkid == "155M0054IZ0202")
    {
        return 30.0;
    }
    else if(linkid == "155M0072IL0101" || linkid == "155M0072IL0102")
    {
        return 30.0;
    }
    else if(linkid == "155M0072IR0102" || linkid == "155M0072IR0101")
    {
        return 30.0;
    }
    // Intersection 3
    else if(linkid == "155M0053IR0201")
    {
        return 30.0;
    }
    else if(linkid == "155M0053IZ0101")
    {
        return 30.0;
    }
    else if(linkid == "155M0053IZ0202")
    {
        return 30.0;
    }
    else if(linkid == "155M0115IL0101")
    {
        return 30.0;
    }
    else if(linkid == "155M0115IZ0101")
    {
        return 30.0;
    }
    else if(linkid == "155M0115IZ0202")
    {
        return 30.0;
    }
    else if(linkid == "155M0117IL0101" || linkid == "155M0117IL0102")
    {
        return 30.0;
    }
    else if(linkid == "155M0117IR0102" || linkid == "155M0117IR0101")
    {
        return 30.0;
    }
    // Intersection 4
    else if(linkid == "155M0101IR0201")
    {
        return 30.0;
    }
    else if(linkid == "155M0101IZ0101")
    {
        return 30.0;
    }
    else if(linkid == "155M0101IZ0202")
    {
        return 30.0;
    }
    else if(linkid == "155M0114IL0101")
    {
        return 30.0;
    }
    else if(linkid == "155M0114IZ0101")
    {
        return 30.0;
    }
    else if(linkid == "155M0114IZ0202")
    {
        return 30.0;
    }
    else if(linkid == "155M0120IL0101" || linkid == "155M0120IL0102")
    {
        return 30.0;
    }
    else if(linkid == "155M0120IR0102" || linkid == "155M0120IR0101")
    {
        return 30.0;
    }
    // Intersection 5
    else if(linkid == "155M0106IZ0101")
    {
        return 30.0;
    }
    else if(linkid == "155M0106IZ0202")
    {
        return 30.0;
    }
    else if(linkid == "155M0113IZ0101")
    {
        return 30.0;
    }
    else if(linkid == "155M0113IZ0202")
    {
        return 30.0;
    }
    // Intersection 14
    else if(linkid == "155M0122IL0101" || linkid == "155M0122IL0102")
    {
        return 30.0;
    }
    else if(linkid == "155M0147IR0202")
    {
        return 30.0;
    }
    // Intersection 16
    else if(linkid == "155M0109IZ0101")
    {
        return 30.0;
    }
    else if(linkid == "155M0109IZ0202")
    {
        return 30.0;
    }
    else if(linkid == "155M0109IZ9101")
    {
        return 30.0;
    }
    else if(linkid == "155M0135IZ0101")
    {
        return 30.0;
    }
    else if(linkid == "155M0135IZ0202")
    {
        return 30.0;
    }
    // Intersection 17
    else if(linkid == "155M0138IZ0101")
    {
        return 30.0;
    }
    else if(linkid == "155M0138IZ0202")
    {
        return 30.0;
    }
    else if(linkid == "155M0142IZ0101")
    {
        return 30.0;
    }
    else if(linkid == "155M0142IZ0202")
    {
        return 30.0;
    }
    else if(linkid == "155M0142IZ0303")
    {
        return 30.0;
    }
    // Intersection 18
    else if(linkid == "155M0139IZ0101")
    {
        return 30.0;
    }
    else if(linkid == "155M0139IZ0202")
    {
        return 30.0;
    }
    else if(linkid == "155M0049IZ0101")
    {
        return 30.0;
    }
    else if(linkid == "155M0049IZ0202")
    {
        return 30.0;
    }
    // Intersection 19
    else if(linkid == "155M0048IZ0101")
    {
        return 30.0;
    }
    else if(linkid == "155M0048IZ0202")
    {
        return 30.0;
    }
    else if(linkid == "155M0045IZ0101")
    {
        return 30.0;
    }
    else if(linkid == "155M0045IZ0202")
    {
        return 30.0;
    }
    // Intersection 20
    else if(linkid == "155M0046IZ0101")
    {
        return 30.0;
    }
    else if(linkid == "155M0046IZ0202")
    {
        return 30.0;
    }
    else if(linkid == "155M0044IZ0101")
    {
        return 30.0;
    }
    else if(linkid == "155M0044IZ0202")
    {
        return 30.0;
    }
    // Intersection 21
    else if(linkid == "155M0047IZ0101")
    {
        return 30.0;
    }
    else if(linkid == "155M0047IZ0202")
    {
        return 30.0;
    }
    else if(linkid == "155M0043IZ0101")
    {
        return 30.0;
    }
    else if(linkid == "155M0043IZ0202")
    {
        return 30.0;
    }
    // Intersection 22
    else if(linkid == "155M0038IL0101")
    {
        return 30.0;
    }
    else if(linkid == "155M0038IZ0101")
    {
        return 30.0;
    }
    else if(linkid == "155M0038IZ0202")
    {
        return 30.0;
    }
    else if(linkid == "155M0042IR0201")
    {
        return 30.0;
    }
    else if(linkid == "155M0042IZ0101")
    {
        return 30.0;
    }
    else if(linkid == "155M0042IZ0202")
    {
        return 30.0;
    }
    else if(linkid == "155M0079IL0101" || linkid == "155M0079IL0102")
    {
        return 30.0;
    }
    else if(linkid == "155M0079IR0102" || linkid == "155M0079IR0101")
    {
        return 30.0;
    }
    else
        return 0;
}

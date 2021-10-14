#include "Dijkstra.h"

void Adjlist::makeGraph(std::vector<c1_node> &node, std::vector<a3_link> &link)
{
    for (size_t i = 0; i < node.size(); i++)
    {
        Node temp_node;
        temp_node.thisNodeInfo = node[i];
        temp_node.cost = DBL_MAX;
        temp_node.fromNode = nullptr;
        head.push_back(temp_node);
    }

    for (size_t i = 0; i < link.size(); i++)
    {
        bool flag1 = false;
        bool flag2 = false;
        int index = -1;

        Node temp_node;
        temp_node.thisLinkInfo = &link[i];
        for (size_t j = 0; j < head.size(); j++)
        {
            if (link[i].fromnode == head[j].thisNodeInfo.nodeid)
            {
                index = static_cast<int>(j);
                flag1 = true;
            }

            if (link[i].tonode == head[j].thisNodeInfo.nodeid)
            {
                temp_node.thisNode = &head[j];
                flag2 = true;
            }

            if (flag1 && flag2)
                break;
        }
        if (index != -1)
        {
            head[index].adjNode.push_back(temp_node);
        }
    }
}

void Adjlist::clearGraph()
{
    for (size_t i = 0; i < head.size(); i++)
    {
        head[i].cost = DBL_MAX;
        head[i].fromNode = nullptr;
    }
}

void Dijkstra::initDijkstra(HDMap *map, std::vector<std::vector<double>> *table)
{
    this->map = map;
    graph.makeGraph(map->m_node, map->m_link);
    this->table = table;
}

void Dijkstra::clearDijkstra()
{
    graph.clearGraph(); //이전 그래프 초기화
}

void Dijkstra::findCloseLink(pcl::PointXY start, pcl::PointXY end, int& startLinkIndex, int& endLinkIndex, int& startPointIndex, int& endPointIndex)
{
    map->getLinkInfo(start, startLinkIndex, startPointIndex);
    map->getLinkInfo(end, endLinkIndex, endPointIndex);

    // 도착지까지 포인트를 더 주기 위한 코드
    // if(endLinkIndex != -1)
    // {
    //     const double marginDist = 1.5; // 도착지까지 더 주는 포인트 최대 거리
    //     if(endPointIndex < map->m_link[endLinkIndex].waypoint.size() - 1)
    //     {
    //         int newPointIndex = endPointIndex;
    //         double dist = 0;
    //         pcl::PointXY prev;
    //         prev.x = map->m_link[endLinkIndex].waypoint[endPointIndex].x;
    //         prev.y = map->m_link[endLinkIndex].waypoint[endPointIndex].y;
    //         for(int i = endPointIndex; i < map->m_link[endLinkIndex].waypoint.size(); i++)
    //         {
    //             pcl::PointXY curr;
    //             curr.x = map->m_link[endLinkIndex].waypoint[i].x;
    //             curr.y = map->m_link[endLinkIndex].waypoint[i].y;
    //             dist += hypot(prev.x - curr.x, prev.y - curr.y);
    //             prev = curr;
    //             if(dist < marginDist)
    //             {
    //                 newPointIndex = i;
    //             }
    //             else
    //                 break;
    //         }

    //         endPointIndex = newPointIndex;
    //     }
    // }
}

void Dijkstra::findStartChangeLink(std::string linkid, pcl::PointXY p, std::vector<int> &linkIndex, std::vector<int> &linkPointIndex)
{
    map->getStartChangeLinkInfo(linkid, p, linkIndex, linkPointIndex);
}

void Dijkstra::findEndChangeLink(std::string linkid, pcl::PointXY p, std::vector<int> &linkIndex, std::vector<int> &linkPointIndex)
{
    map->getEndChangeLinkInfo(linkid, p, linkIndex, linkPointIndex);
}

bool Dijkstra::findPath(size_t startNode, size_t endNode, std::vector<a3_link> &globalPath, double &totalDist, double &totalTime)
{
    clearDijkstra();
    totalDist = totalTime = 0;
    Node *nodePointer1 = &graph.head[startNode];
    Node *nodePointer2 = &graph.head[endNode];
    nodePointer1->cost = 0;
    std::priority_queue<Node *> pq;
    pq.push(nodePointer1);

    //Dijkstra 알고리즘
    while (!pq.empty())
    {
        Node *nowNode = pq.top();
        pq.pop();

        for (size_t i = 0; i < nowNode->adjNode.size(); i++)
        {
            double newVal = nowNode->cost + nowNode->adjNode[i].thisLinkInfo->timecost;
            double beforeVal = nowNode->adjNode[i].thisNode->cost;

            if (newVal < beforeVal)
            {
                nowNode->adjNode[i].thisNode->cost = newVal;
                pq.push(nowNode->adjNode[i].thisNode);
                nowNode->adjNode[i].thisNode->thisLinkInfo = nowNode->adjNode[i].thisLinkInfo;
                nowNode->adjNode[i].thisNode->fromNode = nowNode;
            }
        }
    }

    std::vector<a3_link> shortpath;
    Node *tempnode = nodePointer2;
    while (tempnode->fromNode != nullptr)
    {
        shortpath.push_back(*tempnode->thisLinkInfo);
        tempnode = tempnode->fromNode;
    }
    if (shortpath.size() == 0)
    {
        return false;
    }
    //거꾸로 저장되어 있으므로 뒤집음
    for (size_t i = 0; i < shortpath.size(); i++)
    {
        globalPath.push_back(shortpath[shortpath.size() - 1 - i]);
        totalDist += shortpath[shortpath.size() - 1 - i].cost;
        totalTime += shortpath[shortpath.size() - 1 - i].timecost;
    }

    return true;
}

bool Dijkstra::findPath(pcl::PointXY p1, pcl::PointXY p2, int changeLane, std::vector<a3_link> &globalPath)
{
    clearDijkstra();

    // 시작, 도착 링크 인덱스
    int startLinkIndex;
    int endLinkIndex;
    int startPointIndex;
    int endPointIndex;
    findCloseLink(p1, p2, startLinkIndex, endLinkIndex, startPointIndex, endPointIndex);

    if (startLinkIndex < 0 || endLinkIndex < 0)
    {
        if(endLinkIndex == -2)
        {
            std::cout << "목적지가 비정형으로부터 마진이 부족함" << std::endl;
        }
        else
        {
            std::cout << "가까운 포인트를 찾을 수 없음 (가까운 포인트가 10m 이상임)" << std::endl;
        }
        return false;
    }
    else
    {
        if(map->m_link[startLinkIndex].isIrr)
        {
            int startIrr = map->m_link[startLinkIndex].startIrr;
            int endIrr = map->m_link[startLinkIndex].endIrr;
            if(startPointIndex >= startIrr && startPointIndex <= endIrr)
            {
                std::cout << "Error : 출발지가 비정형 구간 내부임" << std::endl;
                return false;
            }
        }

        if(map->m_link[endLinkIndex].isIrr)
        {
            int startIrr = map->m_link[endLinkIndex].startIrr;
            int endIrr = map->m_link[endLinkIndex].endIrr;
            if(endPointIndex >= startIrr && endPointIndex <= endIrr)
            {
                std::cout << "Error : 목적지가 비정형 구간 내부임" << std::endl;
                return false;
            }
        }
    }

    if (changeLane != 0)
    {
        if (map->m_link[startLinkIndex].linkid[8] == 'I')
        {
            std::cout << "교차로에서는 차선변경이 불가능함" << std::endl;
            return false;
        }
        std::vector<int> startChangeIndex;
        std::vector<int> startChangePointIndex;
        findStartChangeLink(map->m_link[startLinkIndex].linkid, p1, startChangeIndex, startChangePointIndex);

        //Change Left
        bool find = false;
        if (changeLane == 1)
        {
            for (int i = 0; i < startChangeIndex.size(); i++)
            {
                int index = startChangeIndex[i];
                if (map->m_link[index].linkid.substr(8, 2) == "CL")
                {
                    startLinkIndex = startChangeIndex[i];
                    startPointIndex = startChangePointIndex[i];
                    find = true;
                    break;
                }
            }
        }
        //Change Right
        else if (changeLane == 2)
        {
            for (int i = 0; i < startChangeIndex.size(); i++)
            {
                int index = startChangeIndex[i];
                if (map->m_link[index].linkid.substr(8, 2) == "CR")
                {
                    startLinkIndex = startChangeIndex[i];
                    startPointIndex = startChangePointIndex[i];
                    find = true;
                    break;
                }
            }
        }

        if (!find)
        {
            std::cout << "차선변경이 불가능함" << std::endl;
            return false;
        }
    }

    if (startLinkIndex == -1 || endLinkIndex == -1)
    {
        std::cout << "가까운 링크를 찾을 수 없음" << std::endl;
        return false;
    }

    size_t startNode = map->m_link[startLinkIndex].to_index;
    size_t endNode = map->m_link[endLinkIndex].from_index;

    bool start_flag = true;
    //같은 링크 내에서 경로를 찾는경우
    if (startLinkIndex == endLinkIndex && startPointIndex <= endPointIndex)
    {
        start_flag = false;
        a3_link temp = map->m_link[startLinkIndex];
        temp.waypoint.clear();
        temp.cost = 0;
        Waypoint prev = map->m_link[startLinkIndex].waypoint[startPointIndex];

        for (int i = startPointIndex; i <= endPointIndex; i++)
        {
            temp.waypoint.push_back(map->m_link[startLinkIndex].waypoint[i]);

            float x1 = prev.x;
            float y1 = prev.y;
            float x2 = map->m_link[startLinkIndex].waypoint[i].x;
            float y2 = map->m_link[startLinkIndex].waypoint[i].y;

            temp.cost += static_cast<double>(sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)));
            prev.x = x2;
            prev.y = y2;
        }
        globalPath.push_back(temp);
        return true;
    }
    //바로 다음 링크에 해당하는 경로를 찾는경우
    if (startNode == endNode)
    {
        start_flag = false;

        //시작링크
        a3_link temp1 = map->m_link[startLinkIndex];
        temp1.waypoint.clear();
        temp1.cost = 0;
        Waypoint prev = map->m_link[startLinkIndex].waypoint[startPointIndex];

        for (size_t i = startPointIndex; i < map->m_link[startLinkIndex].waypoint.size(); i++)
        {
            temp1.waypoint.push_back(map->m_link[startLinkIndex].waypoint[i]);

            float x1 = prev.x;
            float y1 = prev.y;
            float x2 = map->m_link[startLinkIndex].waypoint[i].x;
            float y2 = map->m_link[startLinkIndex].waypoint[i].y;

            temp1.cost += static_cast<double>(sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)));
            prev.x = x2;
            prev.y = y2;
        }

        //종료링크
        a3_link temp2 = map->m_link[endLinkIndex];
        temp2.waypoint.clear();
        temp2.cost = 0;
        prev = map->m_link[endLinkIndex].waypoint[0];

        for (int i = 0; i < endPointIndex + 1; i++)
        {
            temp2.waypoint.push_back(map->m_link[endLinkIndex].waypoint[i]);

            float x1 = prev.x;
            float y1 = prev.y;
            float x2 = map->m_link[endLinkIndex].waypoint[i].x;
            float y2 = map->m_link[endLinkIndex].waypoint[i].y;

            temp2.cost += static_cast<double>(sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)));
            prev.x = x2;
            prev.y = y2;
        }
        globalPath.push_back(temp1);
        globalPath.push_back(temp2);
        return true;
    }
    std::string startLinkid = map->m_link[startLinkIndex].linkid;
    std::string endLinkid = map->m_link[endLinkIndex].linkid;
    // 바로 옆 링크가 도착 링크인 경우 바로 차선변경
    if (startLinkid != endLinkid && startLinkid.substr(0, 8) == endLinkid.substr(0, 8))
    {
        int index = map->m_link[startLinkIndex].bundle_index;
        for (int i = 0; i < map->bundle[index].size(); i++)
        {
            if (map->bundle[index][i] == nullptr)
                continue;

            if (map->bundle[index][i]->fromnode == map->m_link[startLinkIndex].fromnode && map->bundle[index][i]->tonode == map->m_link[endLinkIndex].tonode)
            {
                pcl::PointXY startPoint;
                startPoint.x = map->m_link[startLinkIndex].waypoint[startPointIndex].x;
                startPoint.y = map->m_link[startLinkIndex].waypoint[startPointIndex].y;

                double minStart = DBL_MAX;
                int startIndex = 0;
                for (int j = 0; j < map->bundle[index][i]->waypoint.size(); j++)
                {
                    pcl::PointXY tempStart;
                    tempStart.x = map->bundle[index][i]->waypoint[j].x;
                    tempStart.y = map->bundle[index][i]->waypoint[j].y;

                    double dist = sqrt((tempStart.x - startPoint.x) * (tempStart.x - startPoint.x) + (tempStart.y - startPoint.y) * (tempStart.y - startPoint.y));
                    if (minStart >= dist)
                    {
                        minStart = dist;
                        startIndex = j;
                    }
                    else
                        break;
                }

                pcl::PointXY endPoint;
                endPoint.x = map->m_link[endLinkIndex].waypoint[endPointIndex].x;
                endPoint.y = map->m_link[endLinkIndex].waypoint[endPointIndex].y;

                double minEnd = DBL_MAX;
                double endIndex = 0;
                for (int j = 0; j < map->bundle[index][i]->waypoint.size(); j++)
                {
                    pcl::PointXY tempEnd;
                    tempEnd.x = map->bundle[index][i]->waypoint[j].x;
                    tempEnd.y = map->bundle[index][i]->waypoint[j].y;

                    double dist = sqrt((tempEnd.x - endPoint.x) * (tempEnd.x - endPoint.x) + (tempEnd.y - endPoint.y) * (tempEnd.y - endPoint.y));
                    if (minEnd >= dist)
                    {
                        minEnd = dist;
                        endIndex = j;
                    }
                    else
                        break;
                }

                a3_link tempLink = *map->bundle[index][i];
                tempLink.cost = 0;
                tempLink.waypoint.clear();

                if (startIndex < endIndex)
                {
                    pcl::PointXY prev;
                    prev.x = map->bundle[index][i]->waypoint[startIndex].x;
                    prev.y = map->bundle[index][i]->waypoint[startIndex].y;

                    for (int j = startIndex; j < endIndex + 1; j++)
                    {
                        tempLink.waypoint.push_back(map->bundle[index][i]->waypoint[j]);
                        double dist = sqrt((prev.x - tempLink.waypoint.back().x) * (prev.x - tempLink.waypoint.back().x) + (prev.y - tempLink.waypoint.back().y) * (prev.y - tempLink.waypoint.back().y));
                        prev.x = tempLink.waypoint.back().x;
                        prev.y = tempLink.waypoint.back().y;
                        tempLink.cost += dist;
                    }
                    if (tempLink.cost > LANECHANGE_END_THRESH)
                    {
                        start_flag = false;
                        globalPath.push_back(tempLink);
                        return true;
                    }
                }
            }
        }
    }
    //Dijkstra 알고리즘
    double totalDist = 0;
    double totalTime = 0;
    bool makePath = false;
    if (start_flag)
    {
        if (findPath(startNode, endNode, globalPath, totalDist, totalTime))
            makePath = true;
        else
        {
            makePath = false;
            totalTime = DBL_MAX;
        }

        //첫번째 링크 앞에 Start Call Point링크를 추가
        a3_link temp = map->m_link[startLinkIndex];
        std::vector<Waypoint> xy = temp.waypoint;
        temp.waypoint.clear();
        temp.cost = 0;
        Waypoint prev = xy[startPointIndex];

        for (size_t i = startPointIndex; i < xy.size(); i++)
        {
            temp.waypoint.push_back(xy[i]);

            float x1 = prev.x;
            float y1 = prev.y;
            float x2 = xy[i].x;
            float y2 = xy[i].y;

            temp.cost += static_cast<double>(sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)));
            prev.x = x2;
            prev.y = y2;
        }
        globalPath.insert(globalPath.begin(), temp);

        //마지막 링크 뒤에 End Call Point링크 추가
        temp = map->m_link[endLinkIndex];
        xy = temp.waypoint;
        temp.waypoint.clear();
        temp.cost = 0;
        prev = xy[0];

        for (int i = 0; i < endPointIndex + 1; i++)
        {
            temp.waypoint.push_back(xy[i]);

            float x1 = prev.x;
            float y1 = prev.y;
            float x2 = xy[i].x;
            float y2 = xy[i].y;

            temp.cost += static_cast<double>(sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)));
            prev.x = x2;
            prev.y = y2;
        }
        globalPath.push_back(temp);
    }

    if (changeLane == 0)
    {
        bool startChange = false;
        bool endChange = false;
        // 시작차선에서의 차선변경 링크 인덱스
        std::vector<int> startChangeIndex;
        std::vector<int> startChangePointIndex;
        // 도착차선에서의 차선변경 링크 인덱스
        std::vector<int> endChangeIndex;
        std::vector<int> endChangePointIndex;

        if (map->m_link[startLinkIndex].linkid[8] != 'I' && !map->m_link[startLinkIndex].change_lane && globalPath.front().cost > LANECHANGE_START_THRESH)
        {
            findStartChangeLink(map->m_link[startLinkIndex].linkid, p1, startChangeIndex, startChangePointIndex);
            if (startChangeIndex.size() > 0)
            {
                startChangeIndex.push_back(startLinkIndex);
                startChangePointIndex.push_back(startPointIndex);
                startChange = true;
            }
        }
        if (map->m_link[endLinkIndex].linkid[8] != 'I' && !map->m_link[endLinkIndex].change_lane && globalPath.back().cost > LANECHANGE_END_THRESH)
        {
            findEndChangeLink(map->m_link[endLinkIndex].linkid, p2, endChangeIndex, endChangePointIndex);
            if (endChangeIndex.size() > 0)
            {
                endChangeIndex.push_back(endLinkIndex);
                endChangePointIndex.push_back(endPointIndex);
                endChange = true;
            }
        }

        // Dijkstra에서도 경로가 만들어지지 않고 시작, 도착 차선 모두 차선변경이 불가능하면 return false
        if (!makePath && !startChange && !endChange)
        {
            return false;
        }

        bool upgrade = false;
        int newStartLinkIndex = 0;
        int newEndLinkIndex = 0;
        int newStartPointIndex = 0;
        int newEndPointIndex = 0;
        if (startChange)
        {
            // 시작차선과 도착차선 모두 차선변경
            if (endChange)
            {
                for (int i = 0; i < startChangeIndex.size(); i++)
                {
                    int newStartNode = map->m_link[startChangeIndex[i]].to_index;

                    for (int j = 0; j < endChangeIndex.size(); j++)
                    {
                        int newEndNode = map->m_link[endChangeIndex[j]].from_index;
                        double newTime = table->at(newStartNode)[newEndNode];
                        if (totalTime > newTime)
                        {
                            totalTime = newTime;
                            upgrade = true;

                            newStartLinkIndex = startChangeIndex[i];
                            newStartPointIndex = startChangePointIndex[i];
                            newEndLinkIndex = endChangeIndex[j];
                            newEndPointIndex = endChangePointIndex[j];
                        }
                    }
                }
            }
            // 시작차선만 차선변경
            else
            {
                for (int i = 0; i < startChangeIndex.size(); i++)
                {
                    int newStartNode = map->m_link[startChangeIndex[i]].to_index;
                    double newTime = table->at(newStartNode)[endNode];
                    if (totalTime > newTime)
                    {
                        totalTime = newTime;
                        upgrade = true;

                        newStartLinkIndex = startChangeIndex[i];
                        newStartPointIndex = startChangePointIndex[i];
                        newEndLinkIndex = endLinkIndex;
                        newEndPointIndex = endPointIndex;
                    }
                }
            }
        }
        else
        {
            // 도착차선만 차선변경
            if (endChange)
            {
                for (int i = 0; i < endChangeIndex.size(); i++)
                {
                    int newEndNode = map->m_link[endChangeIndex[i]].from_index;
                    double newTime = table->at(startNode)[newEndNode];
                    if (totalTime > newTime)
                    {
                        totalTime = newTime;
                        upgrade = true;

                        newStartLinkIndex = startLinkIndex;
                        newStartPointIndex = startPointIndex;
                        newEndLinkIndex = endChangeIndex[i];
                        newEndPointIndex = endChangePointIndex[i];
                    }
                }
            }
        }

        if (upgrade && totalTime < TIME_MAX)
        {
            std::vector<a3_link> diffPath;
            if (findPath(newStartLinkIndex, newStartPointIndex, newEndLinkIndex, newEndPointIndex, diffPath))
                globalPath = diffPath;
        }
        else
        {
            if (!makePath)
                return false;
        }
    }
    else
    {
        bool endChange = false;
        // 도착차선에서의 차선변경 링크 인덱스
        std::vector<int> endChangeIndex;
        std::vector<int> endChangePointIndex;

        if (map->m_link[endLinkIndex].linkid[8] != 'I' && !map->m_link[endLinkIndex].change_lane && globalPath.back().cost > LANECHANGE_END_THRESH)
        {
            findEndChangeLink(map->m_link[endLinkIndex].linkid, p2, endChangeIndex, endChangePointIndex);
            if (endChangeIndex.size() > 0)
                endChange = true;
        }

        // Dijkstra에서도 경로가 만들어지지 않고 시작, 도착 차선 모두 차선변경이 불가능하면 return false
        if (!makePath && !endChange)
        {
            return false;
        }

        bool upgrade = false;
        int newEndLinkIndex = 0;
        int newEndPointIndex = 0;

        // 도착차선만 차선변경
        if (endChange)
        {
            for (int i = 0; i < endChangeIndex.size(); i++)
            {
                int newEndNode = map->m_link[endChangeIndex[i]].from_index;
                double newTime = table->at(startNode)[newEndNode];
                if (totalTime > newTime)
                {
                    totalTime = newTime;
                    upgrade = true;

                    newEndLinkIndex = endChangeIndex[i];
                    newEndPointIndex = endChangePointIndex[i];
                }
            }
        }

        if (upgrade)
        {
            std::vector<a3_link> diffPath;
            if (findPath(startLinkIndex, startPointIndex, newEndLinkIndex, newEndPointIndex, diffPath))
                globalPath = diffPath;
        }

        else
        {
            if (!makePath)
                return false;
        }
    }
    return true;
}

bool Dijkstra::findPath(pcl::PointXY p1, pcl::PointXY p2, double initalSpeed, int changeLane, std::vector<a3_link> &globalPath, std::vector<Motion> &motion)
{
    if (findPath(p1, p2, changeLane, globalPath))
    {
        reshapePath(globalPath, changeLane);
        avoidObstacle(globalPath, changeLane);
        trajectoryPlanning(globalPath, initalSpeed);
        m_globalPath = globalPath;
        getTrajectory(motion);
        return true;
//        if(avoidObstacle(globalPath, changeLane))
//        {
//            trajectoryPlanning(globalPath, initalSpeed);
//            getTrajectory(motion);
//            return true;
//        }
//        else
//        {
//            return false;
//        }
    }
    else
        return false;
}

bool Dijkstra::findPath(size_t startNode, size_t endNode, std::vector<a3_link> &globalPath, std::vector<Motion> &motion)
{
    double totalDist, totalTime;
    if (findPath(startNode, endNode, globalPath, totalDist, totalTime))
    {
        trajectoryPlanning(globalPath, 0);
        getTrajectory(motion);
        m_globalPath = globalPath;
        return true;
    }
    else
        return false;
}

bool Dijkstra::findPath(size_t startLinkIndex, size_t startPointIndex, size_t endLinkIndex, size_t endPointIndex, std::vector<a3_link> &globalPath)
{
    clearDijkstra();
    // 시작, 도착 링크 인덱스
    size_t startNode = map->m_link[startLinkIndex].to_index;
    size_t endNode = map->m_link[endLinkIndex].from_index;

    bool start_flag = true;
    //같은 링크 내에서 경로를 찾는경우
    if (startLinkIndex == endLinkIndex && startPointIndex < endPointIndex)
    {
        start_flag = false;
        a3_link temp = map->m_link[startLinkIndex];
        temp.waypoint.clear();
        temp.cost = 0;
        Waypoint prev = map->m_link[startLinkIndex].waypoint[startPointIndex];

        for (int i = startPointIndex; i <= endPointIndex; i++)
        {
            temp.waypoint.push_back(map->m_link[startLinkIndex].waypoint[i]);

            float x1 = prev.x;
            float y1 = prev.y;
            float x2 = map->m_link[startLinkIndex].waypoint[i].x;
            float y2 = map->m_link[startLinkIndex].waypoint[i].y;

            temp.cost += static_cast<double>(sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)));
            prev.x = x2;
            prev.y = y2;
        }
        globalPath.push_back(temp);
        return true;
    }

    //바로 다음 링크에 해당하는 경로를 찾는경우
    if (startNode == endNode)
    {
        start_flag = false;

        //시작링크
        a3_link temp1 = map->m_link[startLinkIndex];
        temp1.waypoint.clear();
        temp1.cost = 0;
        Waypoint prev = map->m_link[startLinkIndex].waypoint[startPointIndex];

        for (size_t i = startPointIndex; i < map->m_link[startLinkIndex].waypoint.size(); i++)
        {
            temp1.waypoint.push_back(map->m_link[startLinkIndex].waypoint[i]);

            float x1 = prev.x;
            float y1 = prev.y;
            float x2 = map->m_link[startLinkIndex].waypoint[i].x;
            float y2 = map->m_link[startLinkIndex].waypoint[i].y;

            temp1.cost += static_cast<double>(sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)));
            prev.x = x2;
            prev.y = y2;
        }
        //종료링크
        a3_link temp2 = map->m_link[endLinkIndex];
        temp2.waypoint.clear();
        temp2.cost = 0;
        prev = map->m_link[endLinkIndex].waypoint[0];

        for (size_t i = 0; i < endPointIndex + 1; i++)
        {
            temp2.waypoint.push_back(map->m_link[endLinkIndex].waypoint[i]);

            float x1 = prev.x;
            float y1 = prev.y;
            float x2 = map->m_link[endLinkIndex].waypoint[i].x;
            float y2 = map->m_link[endLinkIndex].waypoint[i].y;

            temp2.cost += static_cast<double>(sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)));
            prev.x = x2;
            prev.y = y2;
        }
        globalPath.push_back(temp1);
        globalPath.push_back(temp2);
    }

    //Dijkstra 알고리즘
    double totalTime;
    double totalDist;
    if (start_flag)
    {
        if (findPath(startNode, endNode, globalPath, totalDist, totalTime))
        {
            //첫번째 링크 앞에 Start Call Point링크를 추가
            a3_link temp = map->m_link[startLinkIndex];
            std::vector<Waypoint> xy = temp.waypoint;
            temp.waypoint.clear();
            temp.cost = 0;
            Waypoint prev = xy[startPointIndex];

            for (size_t i = startPointIndex; i < xy.size(); i++)
            {
                temp.waypoint.push_back(xy[i]);

                float x1 = prev.x;
                float y1 = prev.y;
                float x2 = xy[i].x;
                float y2 = xy[i].y;

                temp.cost += static_cast<double>(sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)));
                prev.x = x2;
                prev.y = y2;
            }
            globalPath.insert(globalPath.begin(), temp);

            //마지막 링크 뒤에 End Call Point링크 추가
            temp = map->m_link[endLinkIndex];
            xy = temp.waypoint;
            temp.waypoint.clear();
            temp.cost = 0;
            prev = xy[0];

            for (int i = 0; i < endPointIndex + 1; i++)
            {
                temp.waypoint.push_back(xy[i]);

                float x1 = prev.x;
                float y1 = prev.y;
                float x2 = xy[i].x;
                float y2 = xy[i].y;

                temp.cost += static_cast<double>(sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)));
                prev.x = x2;
                prev.y = y2;
            }
            globalPath.push_back(temp);
        }
        else
            return false;
    }
    return true;
}

void Dijkstra::editStartLink(a3_link &origin, a3_link target)
{
    // 가장 가까운 wayPointIndex 탐색
    double ref_x = origin.waypoint.front().x;
    double ref_y = origin.waypoint.front().y;

    double minDist = DBL_MAX;
    int closeIndex = 0;
    for (int i = 0; i < target.waypoint.size(); i++)
    {
        double temp_x = target.waypoint[i].x;
        double temp_y = target.waypoint[i].y;

        double dist = sqrt((ref_x - temp_x) * (ref_x - temp_x) + (ref_y - temp_y) * (ref_y - temp_y));

        if (minDist >= dist)
        {
            minDist = dist;
            closeIndex = i;
        }
        else
            break;
    }

    // 새로운 wayPoint들의 총 거리 계산
    target.cost = 0;
    std::vector<Waypoint> xy;
    Waypoint prev;
    prev.x = target.waypoint[closeIndex].x;
    prev.y = target.waypoint[closeIndex].y;

    for (int i = closeIndex; i < target.waypoint.size(); i++)
    {
        xy.push_back(target.waypoint[i]);

        float x1 = prev.x;
        float y1 = prev.y;
        float x2 = xy.back().x;
        float y2 = xy.back().y;

        target.cost += static_cast<double>(sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)));
        prev.x = x2;
        prev.y = y2;
    }
    target.waypoint = xy;
    origin = target;
}

void Dijkstra::editEndLink(a3_link &origin, a3_link target)
{
    // 가장 가까운 wayPointIndex 탐색
    double ref_x = origin.waypoint.back().x;
    double ref_y = origin.waypoint.back().y;

    double minDist = DBL_MAX;
    int closeIndex = 0;
    for (int i = 0; i < target.waypoint.size(); i++)
    {
        double temp_x = target.waypoint[i].x;
        double temp_y = target.waypoint[i].y;

        double dist = sqrt((ref_x - temp_x) * (ref_x - temp_x) + (ref_y - temp_y) * (ref_y - temp_y));

        if (minDist >= dist)
        {
            minDist = dist;
            closeIndex = i;
        }
        else
            break;
    }

    // 새로운 wayPoint들의 총 거리 계산
    target.cost = 0;
    std::vector<Waypoint> xy;
    Waypoint prev;
    prev.x = target.waypoint[0].x;
    prev.y = target.waypoint[0].y;

    for (int i = 0; i < closeIndex + 1; i++)
    {
        xy.push_back(target.waypoint[i]);

        float x1 = prev.x;
        float y1 = prev.y;
        float x2 = xy.back().x;
        float y2 = xy.back().y;

        target.cost += static_cast<double>(sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)));
        prev.x = x2;
        prev.y = y2;
    }
    target.waypoint = xy;
    origin = target;
}

bool Dijkstra::rewiring(std::vector<a3_link> &globalPath, int currentIndex, int nextNode, int changeLane)
{
    if(currentIndex < 0 || currentIndex >= globalPath.size())
        return false;

    if (currentIndex > 0)
    {
        int bundleIndex = globalPath[currentIndex].bundle_index;
        if(globalPath[currentIndex].change_lane && map->isIrrBundle(bundleIndex))
        {
            int dst = globalPath[currentIndex].dst_index;
            if(currentIndex == globalPath.size() - 1)
            {
                if(map->m_link[dst].isIrr)
                {
                    double startDist, endDist;
                    getIrrDist(map->m_link[dst], startDist, endDist);
                    if(globalPath.back().cost > endDist)
                        return false;
                    // else
                    // 목적지가 비정형 시작 전이면 rewiring 해도 됨 
                }
            }
            else
            {
                bool pass = false;
                if(map->m_link[dst].isIrr)
                {
                    if(map->m_link[dst].to_index != nextNode)
                        pass = true;
                }
                if(!pass)
                    return false;
            }
        }
        // 교차로가 아닌경우
        if (globalPath[currentIndex].linkid[8] != 'I')
        {
            int wiredLinkIndex = -1;

            for (int i = 0; i < map->bundle[bundleIndex].size(); i++)
            {
                if (map->bundle[bundleIndex][i] == nullptr)
                    continue;

                if(globalPath[currentIndex].change_lane)
                {
                    if (map->bundle[bundleIndex][i]->to_index == nextNode && !map->bundle[bundleIndex][i]->change_lane && map->bundle[bundleIndex][i]->from_index == globalPath[currentIndex].from_index)
                        wiredLinkIndex = i;
                }
                else
                {
                    if (map->bundle[bundleIndex][i]->to_index == nextNode && !map->bundle[bundleIndex][i]->change_lane)
                        wiredLinkIndex = i;
                }

                // 현재 번들중에 차선변경링크가 있을때는 return
                if (map->bundle[bundleIndex][i]->change_lane && map->bundle[bundleIndex][i]->to_index == nextNode)
                {
                    int dstIndex = map->bundle[bundleIndex][i]->dst_index;
                    int newNextNode = map->m_link[dstIndex].from_index;
                    int newCurrentIndex = currentIndex - 1;

                    //globalPath를 dstLink로 변경
                    if (rewiring(globalPath, newCurrentIndex, newNextNode, changeLane))
                    {
                        // 마지막 링크가 아닐경우 그낭 globalPath 수정
                        if (currentIndex != globalPath.size() - 1)
                            globalPath[currentIndex] = map->m_link[dstIndex];
                        // 마지막 링크일경우
                        else
                        {
                            int dstIndex = globalPath[currentIndex].dst_index;
                            a3_link target = map->m_link[dstIndex];
                            editEndLink(globalPath[currentIndex], target);
                        }
                    }
                    else
                    {
                        // 마지막 링크가 아닐경우 그낭 globalPath 수정
                        if (currentIndex != globalPath.size() - 1)
                        {
                            if(globalPath[currentIndex].from_index == map->bundle[bundleIndex][i]->from_index)
                                globalPath[currentIndex] = *map->bundle[bundleIndex][i];
                            else
                            {
                                if(wiredLinkIndex != -1)
                                    globalPath[currentIndex] = *map->bundle[bundleIndex][wiredLinkIndex];
                                else
                                    return false;
                            }
                        }
                        // 마지막 링크인 경우 수정안하고 그대로 두면 됨
//                        else
//                        {
//                        }
                    }
                    return true;
                }
            }

            // for문이 끝났는데도 return하지 못한경우
            // 터널과 같이 차선변경이 불가능한 번들
            if (wiredLinkIndex != -1)
            {
                //                int newNextNode = map->m_link[wiredLinkIndex].from_index;
                int newNextNode = map->bundle[bundleIndex][wiredLinkIndex]->from_index;
                int newCurrentIndex = currentIndex - 1;
                if (rewiring(globalPath, newCurrentIndex, newNextNode, changeLane))
                {
                    // globalPath를 nextNode와 연결되어있는 Link로 변경
                    //                    globalPath[currentIndex] = map->m_link[wiredLinkIndex];
                    globalPath[currentIndex] = *map->bundle[bundleIndex][wiredLinkIndex];
                    return true;
                }
                // 변경 불가능
                else
                    return false;
            }
            // nextNode로 연결되어있는 Link가 없음
            else
                return false;
        }

        // 교차로인 경우
        else
        {
            for (int i = 0; i < map->bundle[bundleIndex].size(); i++)
            {
                if (map->bundle[bundleIndex][i] == nullptr)
                    continue;

                if (map->bundle[bundleIndex][i]->to_index == nextNode)
                {
                    int newNextNode = map->bundle[bundleIndex][i]->from_index;
                    int newCurrentIndex = currentIndex - 1;

                    if (rewiring(globalPath, newCurrentIndex, newNextNode, changeLane))
                    {
                        // globalPath를 nextNode와 연결되어있는 Link로 변경
                        globalPath[currentIndex] = *map->bundle[bundleIndex][i];
                        return true;
                    }
                    // 변경 불가능
                    else
                        return false;
                }
            }

            // nextNode로 연결이 안되어있는 교차로임
            return false;
        }
    }
    // globalPath 0번인 경우
    else
    {
        if (changeLane == 0)
        {
            if (globalPath[0].cost > LANECHANGE_START_THRESH && globalPath[0].linkid[8] != 'I')
            {
                int bundleIndex = globalPath[0].bundle_index;

                int currentLane;
                if (globalPath[0].change_lane)
                    currentLane = stoi(globalPath[0].linkid.substr(10, 2));
                else
                    currentLane = globalPath[0].lane;

                int dstLane = stoi(map->m_node[nextNode].nodeid.substr(8, 2));

                // 현재 차선이 목적 차선이랑 다르면 차선변경 링크로 변경
                if (currentLane != dstLane)
                {
                    for (int i = 0; i < map->bundle[bundleIndex].size(); i++)
                    {
                        if (map->bundle[bundleIndex][i] == nullptr)
                            continue;

                        if (map->bundle[bundleIndex][i]->change_lane && map->bundle[bundleIndex][i]->to_index == nextNode && map->bundle[bundleIndex][i]->from_index == globalPath[0].from_index)
                        {
                            //globalPath를 차선변경Link로 변경
                            editStartLink(globalPath[0], *map->bundle[bundleIndex][i]);
                            return true;
                        }
                    }
                }
                // 현재 차선이 목적 차선이랑 같으면 직진 링크로 변경
                else
                {
                    for (int i = 0; i < map->bundle[bundleIndex].size(); i++)
                    {
                        if (map->bundle[bundleIndex][i] == nullptr)
                            continue;

                        if (!map->bundle[bundleIndex][i]->change_lane && map->bundle[bundleIndex][i]->to_index == nextNode)
                        {
                            //globalPath를 차선변경Link로 변경
                            editStartLink(globalPath[0], *map->bundle[bundleIndex][i]);
                            return true;
                        }
                    }
                }

                // 현재 번들중에 차선변경링크가 있을때는 return
                return false;
            }
            else
            {
                if(nextNode == globalPath.front().to_index)
                {
                    return true;
                }
                else
                {
                    return false;
                }
            }
        }
        //changeLane flag로 차선변경이 고정되어 있는 경우
        else
            return false;
    }
}

bool Dijkstra::reverseWiring(std::vector<a3_link> &globalPath, int currentIndex, int prevNode)
{
    if(currentIndex < 0 || currentIndex >= globalPath.size())
        return false;

    if (currentIndex > 0)
    {
        int bundleIndex = globalPath[currentIndex].bundle_index;
        // 교차로가 아닌경우
        if (globalPath[currentIndex].linkid[8] != 'I')
        {
            int wiredLinkIndex = -1;

            for (int i = 0; i < map->bundle[bundleIndex].size(); i++)
            {
                if (map->bundle[bundleIndex][i] == nullptr)
                    continue;

                if (map->bundle[bundleIndex][i]->from_index == prevNode)
                    wiredLinkIndex = i;

                // 현재 번들중에 차선변경링크가 있을때는 return
                if (map->bundle[bundleIndex][i]->change_lane && map->bundle[bundleIndex][i]->from_index == prevNode)
                {
                    // 마지막 링크가 아닐경우 그낭 globalPath 수정
                    if (currentIndex != globalPath.size() - 1)
                        globalPath[currentIndex] = *map->bundle[bundleIndex][i];
                    // 마지막 링크일경우
                    else
                    {
//                        int dstIndex = globalPath[currentIndex].dst_index;
//                        a3_link target = map->m_link[dstIndex];
                        a3_link target = *map->bundle[bundleIndex][i];
                        if(globalPath.back().cost > LANECHANGE_END_THRESH)
                            editEndLink(globalPath[currentIndex], target);
                    }
                    return true;
                }
            }

            // for문이 끝났는데도 return하지 못한경우
            // 터널과 같이 차선변경이 불가능한 번들
            if (wiredLinkIndex != -1)
            {
                int newPrevNode = map->bundle[bundleIndex][wiredLinkIndex]->to_index;
                int newCurrentIndex = currentIndex + 1;
                if (reverseWiring(globalPath, newCurrentIndex, newPrevNode))
                {
                    // globalPath를 prevNode와 연결되어있는 Link로 변경
                    globalPath[currentIndex] = *map->bundle[bundleIndex][wiredLinkIndex];
                    return true;
                }
                // 변경 불가능
                else
                    return false;
            }
            // prevNode로 연결되어있는 Link가 없음
            else
                return false;
        }

        // 교차로인 경우
        else
        {
            for (int i = 0; i < map->bundle[bundleIndex].size(); i++)
            {
                if (map->bundle[bundleIndex][i] == nullptr)
                    continue;

                if (map->bundle[bundleIndex][i]->from_index == prevNode)
                {
                    int newPrevNode = map->bundle[bundleIndex][i]->to_index;
                    int newCurrentIndex = currentIndex + 1;

                    if (reverseWiring(globalPath, newCurrentIndex, newPrevNode))
                    {
                        // globalPath를 nextNode와 연결되어있는 Link로 변경
                        globalPath[currentIndex] = *map->bundle[bundleIndex][i];
                        return true;
                    }
                    // 변경 불가능
                    else
                        return false;
                }
            }

            // prevNode로 연결이 안되어있는 교차로임
            return false;
        }
    }
}

void Dijkstra::reshapePath(std::vector<a3_link> &globalPath, int changeLane)
{
    if (globalPath.size() > 2)
    {
        for (int i = 1; i < globalPath.size(); i++)
        {
            if (globalPath[i].change_lane)
            {
                int dstIndex = globalPath[i].dst_index;
                int nextNode = map->m_link[dstIndex].to_index;
                rewiring(globalPath, i, nextNode, changeLane);
            }
        }
    }
}

bool Dijkstra::avoidObstacle(std::vector<a3_link> &globalPath, int changeLane)
{
    if(globalPath.size() > 2)
    {
        for(int i = 1; i < globalPath.size(); i++)
        {
            if(globalPath[i].isIrr)
            {
                //마지막링크가 아니거나 마지막링크여도 목적지가 비정형 다음이면 path를 다듬어야함
                if(i < globalPath.size() - 1 || (i == globalPath.size() - 1 && globalPath[i].waypoint.size() > globalPath[i].endIrr))
                {
                    int bundleIndex = globalPath[i].bundle_index;
                    int toIndex = globalPath[i].to_index;
                    for(int j = 0; j < map->bundle[bundleIndex].size(); j++)
                    {
                        if(map->bundle[bundleIndex][j] == nullptr)
                            continue;

                        if(map->bundle[bundleIndex][j]->change_lane && map->bundle[bundleIndex][j]->to_index == toIndex)
                        {
                            int nextNode = map->bundle[bundleIndex][j]->from_index;
                            if(rewiring(globalPath, i - 1, nextNode, changeLane))
                            {
                                if(i == globalPath.size() - 1)
                                {
                                    editEndLink(globalPath[i], *map->bundle[bundleIndex][j]);
                                }
                                else
                                {
                                    globalPath[i] = *map->bundle[bundleIndex][j];
                                }
                            }
                            else
                            {
                                int fromIndex = globalPath[i].from_index;
                                std::vector<a3_link> result;
                                map->getChangeLinkInfo(bundleIndex, fromIndex, result);

                                for(int k = 0; k < result.size(); k++)
                                {
                                    int prevNode = result[k].to_index;
                                    if(reverseWiring(globalPath, i + 1, prevNode))
                                    {
                                        globalPath[i] = result[k];
                                        return true;
                                    }
                                }
                                return false;
                            }
                            break;
                        }
                    }
                }
                else
                {
                    return true;
                }
            }
        }
    }
    return true;
}

void Dijkstra::trajectoryPlanning(std::vector<a3_link> globalPath, double initalSpeed)
{
    trajectory.setPath(globalPath);
    trajectory.speedProfileDesign(initalSpeed);
}

void Dijkstra::getTrajectory(std::vector<Motion> &motion)
{
    trajectory.getMotion(motion);
}

void Dijkstra::getPathTime(double &time)
{
    trajectory.getTime(time);
    for(int i = 0; i < m_globalPath.size(); i++)
    {
        if(m_globalPath[i].change_lane)
            time += 3.0;

        if(m_globalPath[i].linkid[8] == 'I')
            time += map->checkSignalIntersection(m_globalPath[i].linkid);
    }
}

void Dijkstra::getIrrDist(a3_link irrLink, double& startDist, double& endDist)
{
    int startIrr = irrLink.startIrr;
    int endIrr = irrLink.endIrr;

    pcl::PointXY prev;
    prev.x = irrLink.waypoint.front().x;
    prev.y = irrLink.waypoint.front().y;

    for(int i = 0; i < startIrr; i++)
    {
        pcl::PointXY curr;
        curr.x = irrLink.waypoint[i].x;
        curr.y = irrLink.waypoint[i].y;

        startDist += hypot(prev.x - curr.x, prev.y - curr.y);
        prev = curr;
    }

    endDist = startDist;
    prev.x = irrLink.waypoint[startIrr].x;
    prev.y = irrLink.waypoint[startIrr].y;

    for(int i = startIrr; i < endIrr; i++)
    {
        pcl::PointXY curr;
        curr.x = irrLink.waypoint[i].x;
        curr.y = irrLink.waypoint[i].y;

        endDist += hypot(prev.x - curr.x, prev.y - curr.y);
        prev = curr;
    }
}

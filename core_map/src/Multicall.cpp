#include "Multicall.h"

Multicall::Multicall()
{
    id = 0;
}

void Multicall::initMulticall(HDMap *map, std::vector<std::vector<double>> *table)
{
    this->map = map;
    this->table = table;
    dijkstra.initDijkstra(map, table);
}

double Multicall::makeCombination(size_t currNode, std::vector<size_t> nodes, std::vector<int> &result)
{
    size_t n0 = nodes[0];
    size_t n1 = nodes[1];
    size_t n2 = nodes[2];
    size_t n3 = nodes[3];

    std::vector<double> solutions;
    //    solutions.push_back(table[currNode][n0] + table[n0][n1] + table[n1][n2] + table[n2][n3]);
    //    solutions.push_back(table[currNode][n0] + table[n0][n2] + table[n2][n3] + table[n3][n1]);
    //    solutions.push_back(table[currNode][n0] + table[n0][n2] + table[n2][n1] + table[n1][n3]);
    //    solutions.push_back(table[currNode][n2] + table[n2][n3] + table[n3][n0] + table[n0][n1]);
    //    solutions.push_back(table[currNode][n2] + table[n2][n0] + table[n0][n3] + table[n3][n1]);
    //    solutions.push_back(table[currNode][n2] + table[n2][n0] + table[n0][n1] + table[n1][n3]);
    solutions.push_back(table->at(currNode)[n0] + table->at(n0)[n1] + table->at(n1)[n2] + table->at(n2)[n3]);
    solutions.push_back(table->at(currNode)[n0] + table->at(n0)[n2] + table->at(n2)[n3] + table->at(n3)[n1]);
    solutions.push_back(table->at(currNode)[n0] + table->at(n0)[n2] + table->at(n2)[n1] + table->at(n1)[n3]);
    solutions.push_back(table->at(currNode)[n2] + table->at(n2)[n3] + table->at(n3)[n0] + table->at(n0)[n1]);
    solutions.push_back(table->at(currNode)[n2] + table->at(n2)[n0] + table->at(n0)[n3] + table->at(n3)[n1]);
    solutions.push_back(table->at(currNode)[n2] + table->at(n2)[n0] + table->at(n0)[n1] + table->at(n1)[n3]);

    double bestSol = DBL_MAX;
    size_t bestIndex = 0;
    for (size_t i = 0; i < 6; i++)
    {
        if (solutions[i] < bestSol)
        {
            bestSol = solutions[i];
            bestIndex = i;
        }
    }

    switch (bestIndex)
    {
    case 0:
        result.push_back(0);
        result.push_back(1);
        result.push_back(2);
        result.push_back(3);
        break;
    case 1:
        result.push_back(0);
        result.push_back(2);
        result.push_back(3);
        result.push_back(1);
        break;
    case 2:
        result.push_back(0);
        result.push_back(2);
        result.push_back(1);
        result.push_back(3);
        break;
    case 3:
        result.push_back(2);
        result.push_back(3);
        result.push_back(0);
        result.push_back(1);
        break;
    case 4:
        result.push_back(2);
        result.push_back(0);
        result.push_back(3);
        result.push_back(1);
        break;
    case 5:
        result.push_back(2);
        result.push_back(0);
        result.push_back(1);
        result.push_back(3);
        break;
    }

    return bestSol;
}

void Multicall::sortCombination(std::vector<size_t> links, std::vector<size_t> linkPoint, std::vector<int> &combination)
{
    // for(int i = 0; i < combination.size(); i++)
    // {
    //     int ref = links[combination[i]];
    //     int start = i;

    //     for(int j = i + 1; j < combination.size(); j++)
    //     {
    //         if(ref == links[combination[j]])
    //         {
    //             for(int k = j; k > start - 1; k--)
    //             {
    //                 if(linkPoint[start] > linkPoint[combination[k]])
    //                 {
    //                     combination[start] =
    //                 }

    //             }
    //         }
    //         else
    //             break;
    //     }
    // }
}

void Multicall::optimizeRoute(std::vector<Mission> missionList, pcl::PointXY currentPos, int remainSeat, pcl::PointCloud<pcl::PointXY> &solution, std::vector<MissionPoint> &order, std::vector<int> &selectedMission)
{
    static int count = 0;
    count++;
    std::string path = "/home/a/Mission_Log/optimize_" + std::to_string(count) + ".txt";
    FILE *fp = fopen(path.c_str(), "w");
    fprintf(fp, "OptimizeRoute#%d : currPos(%lf, %lf)\n", count, currentPos.x, currentPos.y);
    int currentLinkIndex, currentPointIndex;
    map->getLinkInfo(currentPos, currentLinkIndex, currentPointIndex);
    size_t currNode = map->m_link[currentLinkIndex].to_index;

    int best_i = 0;
    int best_j = 0;
    double bestSol = -1;
    pcl::PointCloud<pcl::PointXY> bestCombination;
    size_t callSize = missionList.size();

    // 사용하려면 시간당 점수이득(point/sec)로 코드 수정해야함
    if (remainSeat == 2)
    {
        for (size_t i = 0; i < callSize; i++)
        {
            if (missionList[i].status == MISSION_STATE_AVAILABLE)
            {
                for (size_t j = i + 1; j < callSize; j++)
                {
                    if (missionList[j].status == MISSION_STATE_AVAILABLE)
                    {
                        int startLinkIndex1, endLinkIndex1, startPointIndex1, endPointIndex1;
                        int startLinkIndex2, endLinkIndex2, startPointIndex2, endPointIndex2;
                        dijkstra.findCloseLink(missionList[i].src, missionList[i].dst, startLinkIndex1, endLinkIndex1, startPointIndex1, endPointIndex1);
                        dijkstra.findCloseLink(missionList[j].src, missionList[j].dst, startLinkIndex2, endLinkIndex2, startPointIndex2, endPointIndex2);

                        pcl::PointCloud<pcl::PointXY> points;
                        points.push_back(missionList[i].src);
                        points.push_back(missionList[i].dst);
                        points.push_back(missionList[j].src);
                        points.push_back(missionList[j].dst);

                        std::vector<size_t> nodes;
                        nodes.push_back(map->m_link[startLinkIndex1].to_index);
                        nodes.push_back(map->m_link[endLinkIndex1].from_index);
                        nodes.push_back(map->m_link[startLinkIndex2].to_index);
                        nodes.push_back(map->m_link[endLinkIndex2].from_index);

                        std::vector<size_t> links;
                        links.push_back(startLinkIndex1);
                        links.push_back(endLinkIndex1);
                        links.push_back(startLinkIndex2);
                        links.push_back(endLinkIndex2);

                        std::vector<size_t> linkPoint;
                        linkPoint.push_back(startPointIndex1);
                        linkPoint.push_back(endPointIndex1);
                        linkPoint.push_back(startPointIndex2);
                        linkPoint.push_back(endPointIndex2);

                        std::vector<int> tempCombination;
                        double sol = makeCombination(currNode, nodes, tempCombination);
                        if (sol < bestSol)
                        {
                            sortCombination(links, linkPoint, tempCombination);

                            if (currentLinkIndex == links[tempCombination[0]])
                            {
                                if (currentPointIndex > linkPoint[tempCombination[0]])
                                    continue;
                            }
                            pcl::PointCloud<pcl::PointXY> temp;
                            temp.push_back(currentPos);
                            temp.push_back(points[tempCombination[0]]);
                            temp.push_back(points[tempCombination[1]]);
                            temp.push_back(points[tempCombination[2]]);
                            temp.push_back(points[tempCombination[3]]);
                            bestCombination = temp;
                            bestSol = sol;
                            best_i = i;
                            best_j = j;

                            std::vector<MissionPoint> tempOrder;
                            for (size_t k = 0; k < tempCombination.size(); k++)
                            {
                                int num = tempCombination[k];
                                MissionPoint temp;
                                switch (num)
                                {
                                case 0:
                                    temp.id = id;
                                    temp.src = true;
                                    temp.dst = false;
                                    break;
                                case 1:
                                    temp.id = id;
                                    temp.src = false;
                                    temp.dst = true;
                                    break;
                                case 2:
                                    temp.id = id + 1;
                                    temp.src = true;
                                    temp.dst = false;
                                    break;
                                case 3:
                                    temp.id = id + 1;
                                    temp.src = false;
                                    temp.dst = true;
                                    break;
                                }
                                temp.location = points[num];
                                tempOrder.push_back(temp);
                            }
                            order = tempOrder;
                        }
                    }
                }
            }
        }
        id = id + 2;

        solution = bestCombination;
        selectedMission.push_back(best_i);
        selectedMission.push_back(best_j);
        std::cout << "Best Solution : call(" << best_i << ", " << best_j << ") -> " << bestSol << "sec" << std::endl;
        std::cout << "(" << solution[0].x << ", " << solution[0].y << ")" << std::endl;
        std::cout << "(" << solution[1].x << ", " << solution[1].y << ")" << std::endl;
        std::cout << "(" << solution[2].x << ", " << solution[2].y << ")" << std::endl;
        std::cout << "(" << solution[3].x << ", " << solution[3].y << ")" << std::endl;
        std::cout << "(" << solution[4].x << ", " << solution[4].y << ")" << std::endl;
    }
    else
    {
        for (size_t i = 0; i < callSize; i++)
        {
            if (missionList[i].status == MISSION_STATE_AVAILABLE)
            {
                int currLinkIndex = currentLinkIndex;
                int currPointIndex = currentPointIndex;
                int startLinkIndex1, endLinkIndex, startPointIndex, endPointIndex;
                dijkstra.findCloseLink(missionList[i].src, missionList[i].dst, startLinkIndex1, endLinkIndex, startPointIndex, endPointIndex);

                if (startLinkIndex1 < 0 || endLinkIndex < 0)
                {
                    missionList[i].status = MISSION_STATE_IMPOSSIBLE;
                    fprintf(fp, " Mission[%d] : IMPOSSIBLE\n", missionList[i].missionID);
                    continue;
                }
                int startLinkIndex2 = startLinkIndex1;
                double currLength, startLength1, startLength2, endLength;

                bool noChange1 = false;
                bool noChange2 = false;
                if(currLinkIndex == startLinkIndex1 && currPointIndex <= startPointIndex)
                {
                    noChange1 = true;
                }
                else
                {
                    laneChangeCheck(currLinkIndex, startLinkIndex1, currPointIndex, startPointIndex, currLength, startLength1);
                }
                if(startLinkIndex2 == endLinkIndex && startPointIndex <= endPointIndex)
                {
                    noChange2 = true;
                }
                else
                {
                    laneChangeCheck(startLinkIndex2, endLinkIndex, startPointIndex, endPointIndex, startLength2, endLength);
                }

                pcl::PointCloud<pcl::PointXY> points;
                points.push_back(missionList[i].src);
                points.push_back(missionList[i].dst);

                std::vector<size_t> nodes;
                nodes.push_back(map->m_link[currLinkIndex].to_index);
                nodes.push_back(map->m_link[startLinkIndex1].from_index);
                nodes.push_back(map->m_link[startLinkIndex2].to_index);
                nodes.push_back(map->m_link[endLinkIndex].from_index);

                std::vector<size_t> links;
                links.push_back(startLinkIndex1);
                links.push_back(endLinkIndex);

                std::vector<size_t> linkPoint;
                linkPoint.push_back(startPointIndex);
                linkPoint.push_back(endPointIndex);                

                double curr2start = table->at(nodes[0])[nodes[1]];
                double start2end = table->at(nodes[2])[nodes[3]];
                if ((curr2start >= TIME_MAX && !noChange1) || (start2end >= TIME_MAX && !noChange2))
                {
                    fprintf(fp, " Mission[%d] : IMPOSSIBLE\n", missionList[i].missionID);
                    continue;
                }

                if(noChange1)
                {
                    curr2start = 0;
                }
                else
                {
                    if(map->m_link[currLinkIndex].bundle_index == map->m_link[startLinkIndex1].bundle_index && currLinkIndex != startLinkIndex1)
                    {
                        if(checkSameBundle(currLinkIndex, startLinkIndex1))
                        {
                            double diff = (currLength + startLength1) - map->m_link[currLinkIndex].cost;
                            if(diff > LANECHANGE_START_THRESH)
                            {
                                curr2start = 0;
                            }
                        }
                    }
                }

                if(noChange2)
                {
                    start2end = 0;
                }
                else
                {
                    if(map->m_link[startLinkIndex2].bundle_index == map->m_link[endLinkIndex].bundle_index && startLinkIndex2 != endLinkIndex)
                    {
                        if(checkSameBundle(startLinkIndex2, endLinkIndex))
                        {
                            double diff = (startLength2 + endLength) - map->m_link[startLinkIndex2].cost;
                            if(diff > LANECHANGE_START_THRESH)
                            {
                                start2end = 0;
                            }
                        }
                    }
                }

                double sol;
                if(curr2start <= DBL_MIN && start2end <= DBL_MIN)
                {
                    sol = DBL_MAX;
                }
                else
                {
                    sol = missionList[i].point / (curr2start + start2end);
                }

                fprintf(fp, " Mission[%d] : %f\n", missionList[i].missionID, sol);

                if (sol > bestSol)
                {
                    std::vector<MissionPoint> tempOrder;
                    MissionPoint temp1;
                    temp1.id = id;
                    temp1.src = true;
                    temp1.dst = false;
                    temp1.location = points[0];
                    tempOrder.push_back(temp1);
                    MissionPoint temp2;
                    temp2.id = id;
                    temp2.src = false;
                    temp2.dst = true;
                    temp2.location = points[1];
                    tempOrder.push_back(temp2);
                    order = tempOrder;

                    pcl::PointCloud<pcl::PointXY> temp;
                    temp.push_back(currentPos);
                    temp.push_back(points[0]);
                    temp.push_back(points[1]);

                    bestCombination = temp;
                    bestSol = sol;
                    best_i = i;
                }
            }
            else
            {
                fprintf(fp, " Mission[%d] : UNAVAILABLE\n", missionList[i].missionID);
            }
        }

        if(bestSol != -1)
        {
            solution = bestCombination;
            selectedMission.push_back(best_i);
            std::cout << "Best Solution : call[" << missionList[best_i].missionID << "] -> " << "Total " << missionList[best_i].point << " point, " << bestSol << " point/sec" << std::endl;
            fprintf(fp, "Best Solution : call[%d] -> Total %d point,  %f point/sec\n",missionList[best_i].missionID, missionList[best_i].point, bestSol);
            fprintf(fp, "(%lf, %lf) -> (%lf, %lf) -> (%lf, %lf)\n", solution[0].x, solution[0].y, solution[1].x, solution[1].y, solution[2].x, solution[2].y);
            std::cout << "(" << solution[0].x << ", " << solution[0].y << ")" << std::endl;
            std::cout << "(" << solution[1].x << ", " << solution[1].y << ")" << std::endl;
            std::cout << "(" << solution[2].x << ", " << solution[2].y << ")" << std::endl;
        }
        else
        {
            std::cout << "Optimize Error : Can't Find Solution!!" << std::endl;
            fprintf(fp, "Can't Find Solution\n");
        }
    }
    fclose(fp);
}

void Multicall::laneChangeCheck(int& startLinkIndex, int& endLinkIndex, int startPointIndex, int endPointIndex, double& startLength, double& endLength)
{
    // 출발 링크 길이 계산
    double startLinkLength = 0;
    pcl::PointXY prev1;
    prev1.x = map->m_link[startLinkIndex].waypoint[startPointIndex].x;
    prev1.y = map->m_link[startLinkIndex].waypoint[startPointIndex].y;
    for(int i = startPointIndex; i < map->m_link[startLinkIndex].waypoint.size(); i++)
    {
        pcl::PointXY curr;
        curr.x = map->m_link[startLinkIndex].waypoint[i].x;
        curr.y = map->m_link[startLinkIndex].waypoint[i].y;

        double diff = hypot(curr.x - prev1.x, curr.y - prev1.y);
        startLinkLength += diff;
        prev1 = curr;
    }
    startLength = startLinkLength;
    // 도착 링크 길이 계산
    pcl::PointXY prev2;
    prev2.x = map->m_link[endLinkIndex].waypoint[0].x;
    prev2.y = map->m_link[endLinkIndex].waypoint[0].y;
    double endLinkLength = 0;
    for(int i = 0; i < endPointIndex + 1; i++)
    {
        pcl::PointXY curr;
        curr.x = map->m_link[endLinkIndex].waypoint[i].x;
        curr.y = map->m_link[endLinkIndex].waypoint[i].y;

        double diff = hypot(curr.x - prev2.x, curr.y - prev2.y);
        endLinkLength += diff;
        prev2 = curr;
    }
    endLength = endLinkLength;

    bool startChange = false;
    bool endChange = false;
    // 시작차선에서의 차선변경 링크 인덱스
    std::vector<int> startChangeIndex;
    std::vector<int> startChangePointIndex;
    // 도착차선에서의 차선변경 링크 인덱스
    std::vector<int> endChangeIndex;
    std::vector<int> endChangePointIndex;

    if (map->m_link[startLinkIndex].linkid[8] != 'I' && startLinkLength > LANECHANGE_START_THRESH)
    {
        pcl::PointXY p1;
        p1.x = map->m_link[startLinkIndex].waypoint[startPointIndex].x;
        p1.y = map->m_link[startLinkIndex].waypoint[startPointIndex].y;
        dijkstra.findStartChangeLink(map->m_link[startLinkIndex].linkid, p1, startChangeIndex, startChangePointIndex);
        if (startChangeIndex.size() > 0)
        {
            startChangeIndex.push_back(startLinkIndex);
            startChangePointIndex.push_back(startPointIndex);
            startChange = true;
        }
    }
    if (map->m_link[endLinkIndex].linkid[8] != 'I' && endLinkLength > LANECHANGE_END_THRESH)
    {
        pcl::PointXY p2;
        p2.x = map->m_link[endLinkIndex].waypoint[endPointIndex].x;
        p2.y = map->m_link[endLinkIndex].waypoint[endPointIndex].y;
        dijkstra.findEndChangeLink(map->m_link[endLinkIndex].linkid, p2, endChangeIndex, endChangePointIndex);
        if (endChangeIndex.size() > 0)
        {
            endChangeIndex.push_back(endLinkIndex);
            endChangePointIndex.push_back(endPointIndex);
            endChange = true;
        }
    }

    int startNode = map->m_link[startLinkIndex].to_index;
    int endNode = map->m_link[endLinkIndex].from_index;

    int newStartLinkIndex = 0;
    int newEndLinkIndex = 0;
    bool upgrade = false;
    double currTime = TIME_MAX;
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
                    if (currTime > newTime)
                    {
                        currTime = newTime;

                        newStartLinkIndex = startChangeIndex[i];
                        newEndLinkIndex = endChangeIndex[j];
                        upgrade = true;
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
                if (currTime > newTime)
                {
                    currTime = newTime;

                    newStartLinkIndex = startChangeIndex[i];
                    newEndLinkIndex = endLinkIndex;
                    upgrade = true;
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
                if (currTime > newTime)
                {
                    currTime = newTime;

                    newStartLinkIndex = startLinkIndex;
                    newEndLinkIndex = endChangeIndex[i];
                    upgrade = true;
                }
            }
        }
    }

    if(upgrade)
    {
        startLinkIndex = newStartLinkIndex;
        endLinkIndex= newEndLinkIndex;
    }
}

bool Multicall::checkSameBundle(int startLinkIndex, int endLinkIndex)
{
    std::string srcLane, dstLane;
    srcLane = map->m_link[startLinkIndex].linkid.substr(10, 2);

    if(!map->m_link[endLinkIndex].change_lane)
    {
        dstLane = map->m_link[endLinkIndex].linkid.substr(10, 2);
    }
    else
    {
        dstLane = map->m_link[endLinkIndex].linkid.substr(12, 2);
    }

    std::string lane = srcLane + dstLane;
    int bundleIndex = map->m_link[startLinkIndex].bundle_index;

    for(int i = 0; i < map->bundle[bundleIndex].size(); i++)
    {
        if(map->bundle[bundleIndex][i] != nullptr)
        {
            if(map->bundle[bundleIndex][i]->change_lane)
            {
                if(map->bundle[bundleIndex][i]->linkid.substr(10, 4) == lane)
                    return true;
            }
        }
    }

    return false;
}

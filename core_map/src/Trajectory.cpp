#include "Trajectory.h"
const double maxAcceleration = 0.8; // (max Acceleration)
void Trajectory::setPath(std::vector<a3_link> path)
{
    clearMotion();
    this->path = path;
    dt = 0.01;
    ta = 0;       //현재링크 등가속시간
    tc = 0;       //현재링크 등속시간
    tb = 0;       //현재링크 감속시간
    linktime = 0; //링크 하나를 지나는 예상시간
}

void Trajectory::coeff(double vm, double v1, double v2, double dist)
{
    double num = (2.0 * vm * vm) - (v1 * v1) - (v2 * v2);
    double den = 2.0 * maxAcceleration;

    //최고속도에 도달하지 않는경우
    if (dist < (num / den))
    {
        double fullT = ((-2.0 * v1) + sqrt(4.0 * v1 * v1 + 8.0 * maxAcceleration * dist)) / (2.0 * maxAcceleration); //계속 최대로 가속했을 때 도착하는 시간
        double fullSpeed = v1 + maxAcceleration * fullT;                                                             //계속 최대로 가속했을 때 도달하는 속도
        if (fullSpeed > v2)
        {
            double vx = sqrt((2 * maxAcceleration * dist + v1 * v1 + v2 * v2) / 2.0); //로컬 최대속도
            ta = (vx - v1) / maxAcceleration;
            tc = 0;
            tb = (vx - v2) / maxAcceleration;
        }
        else
        {
            ta = fullT;
            tc = 0;
            tb = 0;
        }
    }
    //최고속도에 도달하고 바로 떨어지는 경우
    else if (dist == (num / den))
    {
        ta = (vm - v1) / maxAcceleration;
        tc = 0;
        tb = (vm - v2) / maxAcceleration;
    }
    //등속구간이 있는 경우
    else
    {
        ta = (vm - v1) / maxAcceleration;
        tc = (dist - (num / den)) / vm;
        tb = (vm - v2) / maxAcceleration;
    }
}

void Trajectory::trapezoidal(double localtime, double vm, double v1, double v2, double prevpos, double lastpos)
{
    Motion m;
    if(ta <= 0 && tc <= 0 && tb <= 0)
    {
        m.a = 0;
        m.p = 0;
        m.v = 0;
    }
    //가속구간
    else if (localtime <= ta && ta != 0.0)
    {
        m.a = maxAcceleration;
        m.v = maxAcceleration * localtime + v1;
        m.p = (2.0 * v1 * localtime + maxAcceleration * localtime * localtime) / 2.0 + prevpos;
    }
    //등속구간
    else if (localtime <= tc + ta)
    {
        m.a = 0;
        m.v = vm;
        m.p = ((v1 + vm) * ta) / 2.0 + vm * (localtime - ta) + prevpos;
    }
    //감속구간
    else if (localtime < linktime)
    {
        m.a = -maxAcceleration;
        m.v = v2 + maxAcceleration * (linktime - localtime);
        m.p = lastpos - (2.0 * v2 + maxAcceleration * (linktime - localtime)) * (linktime - localtime) / 2.0;
    }
    //정지구간
    else
    {
        m.a = 0;
        m.v = 0;
        m.p = lastpos;
    }

    motion.push_back(m);
}

void Trajectory::remakePrevProfile(size_t i, double vm, double v1, double v2, double dist, double lastpos)
{
    motionSize.pop_back();
    lastpos = lastpos - path[i + 1].cost;
    double newDist = dist + path[i].cost;
    double prevV1;
    if (motionSize.size() == 0)
    {
        prevV1 = motion[0].v;
    }
    else
    {
        prevV1 = motion[motionSize[motionSize.size() - 1]].v;
    }

    double bound = (prevV1 + v2) * (fabs(prevV1 - v2) / maxAcceleration) / 2;
    if (bound > newDist)
    {
        if(i != 0)
            remakePrevProfile(i - 1, vm, v1, v2, newDist, lastpos);
    }
    vm = (double)path[i].speed * 1000.0 / 3600.0; // km/h ---> m/s 단위 변경

    //이전링크의 v2까지 풀감속해서 도달할 수 있는 속도를 현재링크의 v2로 만듬
    double newV2 = sqrt(2 * maxAcceleration * dist + v2 * v2);
    v2 = newV2;

    //마지막 링크가 아니면 motionSize 불러오기
    if(i != 0)
    {
        motion.resize(motionSize.back());
        v1 = motion.back().v;
    }
    // 마지막링크일 경우 경로 속도프로파일 불가능
    //
    else
    {
        v1 = sqrt(2 * maxAcceleration * newDist + v2 * v2);
        if(prevV1 < v1)
            v1 = prevV1;
        else
            std::cout << "Trajectory Error : 현재속도가 너무 빨라서 강제로 줄임" << std::endl;

        motion.resize(0);
    }

    dist = path[i].cost;
    coeff(vm, v1, v2, dist);
    linktime = ta + tc + tb;
    double t = 0;
    double prevpos = motion.back().p;
    while (t <= linktime)
    {
        trapezoidal(t, vm, v1, v2, prevpos, lastpos);
        t += dt;
    }
    motionSize.push_back(motion.size());
}

void Trajectory::speedProfileDesign(double initalSpeed)
{
    ta = 0;             //현재링크 등가속시간
    tc = 0;             //현재링크 등속시간
    tb = 0;             //현재링크 감속시간
    linktime = 0;       //링크 하나를 지나는 예상시간
    double lastpos = 0; //최종위치

    double prevpos = 0;
    double currentSpeed = initalSpeed;
    int lastSize = 0;
    for (size_t i = 0; i < path.size(); i++)
    {
        //        std::cout << path[i].linkid << std::endl;
        lastpos += path[i].cost;
        double vm = (double)path[i].speed * 1000.0 / 3600.0; // km/h ---> m/s 단위 변경
        double v1 = currentSpeed;
        double v2;
        if (i < path.size() - 1)
        {
            v2 = (double)path[i + 1].speed * 1000.0 / 3600.0;
        }
        else
        {
            v2 = 0;
        }
        //다음 링크의 제한속도가 더 높은경우 미리 가속하지 않기 위해서 v2 제한
        if (vm < v2)
        {
            v2 = vm;
        }
        double dist = path[i].cost;

        if (i != 0)
        {
            double bound = (v1 + v2) * (fabs(v1 - v2) / maxAcceleration) / 2;
            if (bound > dist && v1 >= v2)
            {
                remakePrevProfile(i - 1, vm, v1, v2, dist, lastpos);
            }
            v1 = motion.back().v;
        }
        coeff(vm, v1, v2, dist); //경계시간 계산
        linktime = ta + tc + tb;
        double t = 0; //현재 링크기준 경과시간
        while (t <= linktime)
        {
            trapezoidal(t, vm, v1, v2, prevpos, lastpos);
            t += dt;
        }
        currentSpeed = motion.back().v;
        prevpos = motion.back().p;
        motionSize.push_back(motion.size());
        lastSize = static_cast<int>(motion.size());
    }

    //Save speed-profile as XLS format
//     FILE*fp = fopen(FILE_NAME, "w");
//     for(size_t i = 0; i < motion.size(); i++)
//     {
//         fprintf(fp, "%f \t%f \t%f \t%f\n", i * dt, motion[i].p, motion[i].v, motion[i].a);
//     }
//     fclose(fp);
}

void Trajectory::clearMotion()
{
    motionSize.clear();
    motion.clear();
    path.clear();
}

void Trajectory::getMotion(std::vector<Motion> &motion)
{
    motion = this->motion;
}

void Trajectory::getTime(double &time)
{
    time = (static_cast<int>(motion.size()) - 1) * dt;
}

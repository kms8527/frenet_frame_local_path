#include "trajectory.h"
const double dt = 0.01;

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

void Trajectory::trapezoidal(double localtime, double vm, double v1, double v2, double dist)
{
    Motion m;
    //가속구간
    if (localtime <= ta && ta != 0.0)
    {
        m.a = maxAcceleration;
        m.v = maxAcceleration * localtime + v1;
        m.p = (2.0 * v1 * localtime + maxAcceleration * localtime * localtime) / 2.0;
    }
    //등속구간
    else if (localtime <= tc + ta)
    {
        m.a = 0;
        m.v = vm;
        m.p = ((v1 + vm) * ta) / 2.0 + vm * (localtime - ta);
    }
    //감속구간
    else if (localtime < linktime)
    {
        m.a = -maxAcceleration;
        m.v = v2 + maxAcceleration * (linktime - localtime);
        m.p = dist - (2.0 * v2 + maxAcceleration * (linktime - localtime)) * (linktime - localtime) / 2.0;
    }
    //정지구간
    else
    {
        m.a = 0;
        m.v = 0;
        m.p = dist;
    }

    motion.push_back(m);
}

bool Trajectory::speedProfileDesign(double vm, double v1, double v2, double dist)
{
    if (vm < 0.01)
        vm = 0.1;
    ta = 0; //현재링크 가속시간
    tb = 0; //현재링크 감속시간
    tc = 0; //현재링크 등속시간

    double bound = (v1 + v2) * (fabs(v1 - v2) / maxAcceleration) / 2;
    if (bound > dist && v1 >= v2)
    {
        // std::cout << "Can't Make Profile" << std::endl;
        return false;
    }
    else
    {
        coeff(vm, v1, v2, dist);
        linktime = ta + tc + tb;

        double t = 0;
        std::vector<Motion> tempMotion;
        while (t <= linktime)
        {
            trapezoidal(t, vm, v1, v2, dist);
            t += dt;
        }
    }

    //Save speed-profile as XLS format
    // FILE*fp = fopen(FILE_NAME, "w");
    // for(size_t i = 0; i < motion.size(); i++)
    // {
    //     fprintf(fp, "%f \t%f \t%f \t%f\n", i * dt, motion[i].p, motion[i].v, motion[i].a);
    // }
    // fclose(fp);

    // std::cout << "Done!" << std::endl;
    return true;
}

void Trajectory::clearMotion()
{
    motion.clear();
}

void Trajectory::getMotion(std::vector<Motion> &motion)
{
    motion = this->motion;
}

void Trajectory::getTrajectory(pcl::PointCloud<pcl::PointXYZI> &path)
{
    double dis_path = 0.0;
    double dis_motion = 0.0;
    int iter_motion = 0;
    for (size_t iter_path = 1; iter_path < path.size(); iter_path++)
    {
        dis_path += hypot(path.at(iter_path).x - path.at(iter_path - 1).x, path.at(iter_path).y - path.at(iter_path - 1).y);
        while (true)
        {
            if (iter_motion + 1 >= motion.size())
            {
                path.at(iter_path).intensity = 0;
                break;
            }
            dis_motion = motion.at(iter_motion).p;
            if (dis_motion > dis_path)
            {
                path.at(iter_path).intensity = motion.at(iter_motion).v * 3.6;
                break;
            }
            else
                iter_motion++;
        }
    }
    if (path.size() > 1)
        path.front().intensity = path.at(1).intensity;
    if (!path.empty())
        path.back().intensity = motion.back().v * 3.6;
}

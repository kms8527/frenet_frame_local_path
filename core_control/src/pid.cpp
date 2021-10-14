#include "pid.h"

PID::PID() {}
PID::PID(double gain, double time_const_i, double time_const_d, double time_delta, double power_limit_min, double power_limit_max) : integral_(0),
                                                                                                                                     deriv_(0)
{
    SetParameter(gain, time_const_i, time_const_d, time_delta);
    power_limit_min_ = power_limit_min;
    power_limit_max_ = power_limit_max;
}
void PID::SetParameter(double gain, double time_const_i, double time_const_d, double time_delta)
{
    gain_ = gain;
    time_const_i_ = time_const_i;
    time_const_d_ = time_const_d;
    time_delta_ = time_delta;

    Reset();
}

double PID::update_new(double curr_feedback, double setpt, double max)
{

    double err = setpt - curr_feedback;
    double pout = gain_ * err;
    double dt = time_delta_;
    //        double intergral  += err * dt;

    double Iout = time_const_i_ * integral_;

    double derivati = (err - deriv_) / dt;
    double Dout = time_const_d_ * derivati;
    deriv_ = err;

    double out = (pout + Iout + Dout);
    // double out =
    if (out > max)
        out = max;
    else if (out < power_limit_min_)
        out = power_limit_min_;
    else
        integral_ += err * dt; //out이 max가 아닐때만 적분값 더함

    return out;
}

double PID::Update(double curr_feedback, double setpt, double max)
{
    double pidout;

    power_limit_max_ = max;

    if (flag_first_msg_)
    {
        flag_first_msg_ = false;
        pidout = 0.0;
    }
    else
    {
        double seterr;
        seterr = curr_feedback - setpt;

        // Proportional response
        pidout = seterr + integral_ * time_delta_ / time_const_i_;

        // drive controller output
        double change;
        change = curr_feedback - deriv_;
        pidout = pidout + change * (time_const_d_ / time_delta_);
        deriv_ = curr_feedback;

        pidout = pidout * (-gain_);

        // Enforce output limTdits and anti-windup latch
        if (pidout > power_limit_max_)
        {
            pidout = power_limit_max_;
        }
        else if (pidout < power_limit_min_)
        {
            pidout = power_limit_min_;
        }
        else
        {
            integral_ = integral_ + seterr;
        }
    }

    return pidout;
}

void PID::Reset()
{
    flag_first_msg_ = true;
    integral_ = 0.0;
    deriv_ = 0.0;
}

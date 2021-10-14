#pragma once

class PID
{

public:
    PID();
    PID(double gain, double time_const_i, double time_const_d, double time_delta, double power_limit_min, double power_limit_max);

public:
    void SetParameter(double gain, double time_const_i, double time_const_d, double time_delta);
    double update_new(double curr_feedback, double setpt, double max);
    double Update(double curr_feedback, double setpt, double max);
    void Reset();

private:
    double gain_;         // Loop gain parameter
    double time_const_i_; // Integrator time constant
    double time_const_d_; // Differentiator time constant
    double time_delta_;   // Update time interval

    //saturation
    double power_limit_min_;
    double power_limit_max_;

    double integral_; // Summation of setpoint errors
    double deriv_;    // Previous setpoint error

    bool flag_first_msg_;
};

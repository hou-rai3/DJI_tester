#ifndef PID_HPP
#define PID_HPP

// #include "Timer.h"

class PID
{
    public:
    PID(float kp, float ki, float kd);
    
    float do_pid(float actual);
    void set_goal(float goal);
    void set_dt(float time);
    float get_goal();
    
    private:
    const float kp;
    const float ki;
    const float kd;
    float dt;
    float goal;
    float pre_error;
    float pre_porp;
    float output;
    float pre_output;
    float _integral;
};

inline PID::PID(float kp, float ki, float kd) : kp(kp), ki(ki), kd(kd)
{
    pre_error = 0.0;
    pre_porp = 0.0;
    pre_output = 0.0;
    output = 0.0;
}
#endif // PID_HPP
#include <time.h>
#include "pid.h"
#include "utility.h"

//#define __USE_PID_METHOD1__
//#define __DEBUG_PID__

PID::PID()
{
    pid_param.e = 0;
    pid_param.e_ = 0;
    pid_param.e__ = 0;
    pid_param.Kd = 0;
    pid_param.Ki = 0;
    pid_param.Kp = 0;
    pid_param.PID_Saturation = 0;
    pid_param.Ts = 0.01;
    pid_param.ppart = 0;
    pid_param.ipart = 0;
    pid_param.dpart = 0;
    pid_param.dpart_alpha = 0.05;
    pid_param.u = 0;
    pid_param.u_ = 0;
    pid_param.flag_first_time = true;
}

PID::~PID()
{
}

void PID::pid_set_k_params(float Kp, float Ki, float Kd, float Ts, float PID_Saturation)
{
    pid_param.Kp = Kp;
    pid_param.Ki = Ki;
    pid_param.Kd = Kd;
    //pid_param.Ts = Ts;
    pid_param.PID_Saturation = PID_Saturation;
}

float PID::pid_process(float error, uint32_t time)
{
    float pre_dpart;
    uint32_t pre_time;

    if (pid_param.flag_first_time)
    {
        pid_param.flag_first_time = false;
        pid_param.time = time;
        pid_param.e = error;
        pid_param.u = pid_param.Kp * pid_param.e;
    }
    else
    {
        pre_time = pid_param.time;
        pid_param.time = time;
        pid_param.Ts = ((int32_t)(pid_param.time - pre_time)) / 1000.0f;
        //cout << (int)(pid_param.Ts*1000) << endl;
        pid_param.e__ = pid_param.e_;
        pid_param.e_ = pid_param.e;
        pid_param.e = error;
        pid_param.u_ = pid_param.u;

        pre_dpart = pid_param.dpart;

#ifdef __USE_PID_METHOD1__
        pid_param.dpart = (pid_param.Kd / pid_param.Ts) * (pid_param.e - (2 * pid_param.e_) + pid_param.e__);
        pid_param.dpart = pre_dpart + pid_param.dpart_alpha * (pid_param.dpart - pre_dpart);
        pid_param.ppart = pid_param.Kp * (pid_param.e - pid_param.e_);
        pid_param.u = pid_param.u_ + pid_param.ppart + pid_param.Ki * pid_param.Ts * pid_param.e + pid_param.dpart;
#else
        pid_param.dpart = (pid_param.Kd / pid_param.Ts) * (pid_param.e - pid_param.e_);
        pid_param.dpart = pre_dpart + pid_param.dpart_alpha * (pid_param.dpart - pre_dpart);
        pid_param.ppart = pid_param.Kp * pid_param.e;
        pid_param.ipart += pid_param.Ki * pid_param.Ts * pid_param.e;
        if (pid_param.ipart > pid_param.PID_Saturation / 2)
        {
            pid_param.ipart = pid_param.PID_Saturation / 2;
        }
        else if (pid_param.ipart < (-pid_param.PID_Saturation / 2))
        {
            pid_param.ipart = -pid_param.PID_Saturation / 2;
        }
        pid_param.u = pid_param.ppart + pid_param.ipart + pid_param.dpart;
#endif
    }
    if (pid_param.u > pid_param.PID_Saturation)
    {
        pid_param.u = pid_param.PID_Saturation;
    }
    else if (pid_param.u < (-pid_param.PID_Saturation))
    {
        pid_param.u = -pid_param.PID_Saturation;
    }
#ifdef __DEBUG_PID__
    cout << "PID " << (int)pid_param.u_ << "," << (int)pid_param.ppart << "," << (int)pid_param.ipart << "," << (int)pid_param.dpart << endl;
#endif
    return pid_param.u;
}

void PID::pid_reset()
{
    pid_param.e = 0;
    pid_param.e_ = 0;
    pid_param.e__ = 0;
    pid_param.u = 0;
    pid_param.u_ = 0;
    pid_param.ppart = 0;
    pid_param.ipart = 0;
    pid_param.dpart = 0;
    pid_param.flag_first_time = true;
}

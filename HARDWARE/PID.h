#ifndef __PID_H
#define __PID_H	 
#include <stdint.h>

typedef struct
{
    float                kp;         //P
    float                ki;         //I
    float                kd;         //D
    float                imax;       //积分限幅

    float                out_p;  //KP输出
    float                out_i;  //KI输出
    float                out_d;  //KD输出
    float                out;    //pid输出

    float                integrator; //< 积分值
    float                last_error; //< 上次误差
    float                last_derivative;//< 上次误差与上上次误差之差
    unsigned long        last_t;     //< 上次时间
}pid_param_t;

extern pid_param_t motorFL_PID;//左前轮
extern pid_param_t motorFR_PID;//右前轮
extern pid_param_t motorBL_PID;//左后轮
extern pid_param_t motorBR_PID;//右后轮




float constrain_float(float amt, float low, float high);

void PidInit(void);

float PidLocCtrl(pid_param_t * pid, float error);

float PidIncCtrl(pid_param_t * pid, float error);

float My_PidIncCtrl(pid_param_t * pid, float error);

#endif /* SRC_APPSW_TRICORE_USER_LQ_PID_H_ */

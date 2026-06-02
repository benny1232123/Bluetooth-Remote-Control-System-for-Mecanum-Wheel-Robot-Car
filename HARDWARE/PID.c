#include "ALL_Head.h"

pid_param_t motorFL_PID;//左前轮
pid_param_t motorFR_PID;//右前轮
pid_param_t motorBL_PID;//左后轮
pid_param_t motorBR_PID;//右后轮


/*************************************************************************
 *  函数名称：float constrain_float(float amt, float low, float high)
 *  功能说明：限幅函数
 *  参数说明：
  * @param    amt   ： 参数
  * @param    low   ： 最低值
  * @param    high  ： 最高值
 *  函数返回：无
 *  修改时间：2020年4月1日
 *  备    注：
 *************************************************************************/
float constrain_float(float amt, float low, float high)
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}


// pid参数初始化函数
void PidInit(void)
{
    //左前轮PID
    motorFL_PID.kp        = 0;
    motorFL_PID.ki        = 0;
    motorFL_PID.kd        = 0;
    motorFL_PID.imax      = 0;
    motorFL_PID.out_p     = 0;
    motorFL_PID.out_i     = 0;
    motorFL_PID.out_d     = 0;
    motorFL_PID.out       = 0;
    motorFL_PID.integrator= 0;
    motorFL_PID.last_error= 0;
    motorFL_PID.last_derivative   = 0;
    motorFL_PID.last_t    = 0;

    //右前轮PID
    motorFR_PID.kp        = 0;
    motorFR_PID.ki        = 0;
    motorFR_PID.kd        = 0;
    motorFR_PID.imax      = 0;
    motorFR_PID.out_p     = 0;
    motorFR_PID.out_i     = 0;
    motorFR_PID.out_d     = 0;
    motorFR_PID.out       = 0;
    motorFR_PID.integrator= 0;
    motorFR_PID.last_error= 0;
    motorFR_PID.last_derivative   = 0;
    motorFR_PID.last_t    = 0;

    //左后轮PID
    motorBL_PID.kp        = 0;
    motorBL_PID.ki        = 0;
    motorBL_PID.kd        = 0;
    motorBL_PID.imax      = 0;
    motorBL_PID.out_p     = 0;
    motorBL_PID.out_i     = 0;
    motorBL_PID.out_d     = 0;
    motorBL_PID.out       = 0;
    motorBL_PID.integrator= 0;
    motorBL_PID.last_error= 0;
    motorBL_PID.last_derivative   = 0;
    motorBL_PID.last_t    = 0;


    //右后轮PID
    motorBR_PID.kp        = 0;
    motorBR_PID.ki        = 0;
    motorBR_PID.kd        = 0;
    motorBR_PID.imax      = 0;
    motorBR_PID.out_p     = 0;
    motorBR_PID.out_i     = 0;
    motorBR_PID.out_d     = 0;
    motorBR_PID.out       = 0;
    motorBR_PID.integrator= 0;
    motorBR_PID.last_error= 0;
    motorBR_PID.last_derivative   = 0;
    motorBR_PID.last_t    = 0;


}

/*************************************************************************
 *  函数名称：float constrain_float(float amt, float low, float high)
 *  功能说明：pid位置式控制器输出
 *  参数说明：
  * @param    pid     pid参数
  * @param    error   pid输入误差
 *  函数返回：PID输出结果
 *  修改时间：2020年4月1日
 *  备    注：
 *************************************************************************/
float PidLocCtrl(pid_param_t * pid, float error)
{
    /* 累积误差 */
    pid->integrator += error;

    /* 误差限幅 */
    pid->integrator = constrain_float(pid->integrator, -pid->imax, pid->imax);

    pid->out_p = pid->kp * error;
    pid->out_i = pid->ki * pid->integrator;
    pid->out_d = pid->kd * (error - pid->last_error);

    pid->last_error = error;

    pid->out = pid->out_p + pid->out_i + pid->out_d;

    return pid->out;
}
/*************************************************************************
 *  函数名称：float constrain_float(float amt, float low, float high)
 *  功能说明：pid增量式控制器输出
 *  参数说明：
  * @param    pid     pid参数
  * @param    error   pid输入误差
 *  函数返回：PID输出结果   注意输出结果已经包涵了上次结果
 *  修改时间：2020年4月1日
 *  备    注：
 *************************************************************************/
float PidIncCtrl(pid_param_t * pid, float error)
{


    pid->out_p = pid->kp * (error - pid->last_error);
    pid->out_i = pid->ki * error;
    pid->out_d = pid->kd * ((error - pid->last_error) - pid->last_derivative);

    pid->last_derivative = error - pid->last_error;
    pid->last_error = error;

    pid->out += pid->out_p + pid->out_i + pid->out_d;

    return pid->out;
}

/*************************************************************************
 *  函数名称：float constrain_float(float amt, float low, float high)
 *  功能说明：pid增量式控制器输出
 *  参数说明：
  * @param    pid     pid参数
  * @param    error   pid输入误差
 *  函数返回：PID输出结果   注意输出结果已经包涵了上次结果
 *  修改时间：2020年4月1日
 *  备    注：
 *************************************************************************/
float My_PidIncCtrl(pid_param_t * pid, float error)
{

    pid->out_p = pid->kp * (error - pid->last_error);
    pid->out_i = pid->ki * error;
    pid->out_d = pid->kd * ((error - pid->last_error) - pid->last_derivative);

    pid->last_derivative = error - pid->last_error;
    pid->last_error = error;

    pid->out = pid->out_p + pid->out_i + pid->out_d;

    return pid->out;
}


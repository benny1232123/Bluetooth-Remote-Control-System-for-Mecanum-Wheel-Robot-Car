#ifndef __SERVO_H
#define __SERVO_H
#include "ALL_Head.h"

//用于麦克纳姆轮逆运动学（xy坐标系）
#define MECANUM_CAR_length  	20.0f //麦轮车车长(cm) (前进方向的轮距)
#define MECANUM_CAR_width    	22.0f //麦轮车车宽  （16+16+6+32）*2+80
#define HALF_LENGTH  (MECANUM_CAR_length/2.0f)  //车长的一半
#define HALF_WIDTH   (MECANUM_CAR_width/2.0f)   //车宽的一半

//用于麦轮正运动学解算
#define WHEEL_DIAMETE      		7.5f //麦轮直径 (cm)
#define PI  									3.1415926535898f                  //圆周率

//#define ENCODER_TIME_S     		0.020f   //编码器计数时间  (s)
//#define ENCODER_ACCURACY 			1560.0f  //编码器精度 (线数) = PRM*减速比*4

#define ENCODER_TIME_S     		0.01f   //光电编码器计数时间  (s)
#define ENCODER_ACCURACY 			53760.0f  //光电编码器精度 (线数) = PRM*减速比*4

#define ENCODER_GEAR     			1.0f      //连接编码器的齿轮齿数
#define WHEEL_GEAR       			1.0f     //连接车轮的齿轮齿数
#define Tooth_Proportion  		ENCODER_GEAR/WHEEL_GEAR	//编码器齿数与车模齿数的比例大小
#define Perimeter 						PI*WHEEL_DIAMETE							//轮子旋转一周产生的路程/cm

////////左前轮
//#define motorFL_PWM         GPIO_Pin_6
//#define motorFL_IN1       GPIO_Pin_14
//#define motorFL_IN2         GPIO_Pin_12
//#define motorFL_MAX    5999                  //电机输出pwm限幅
//#define motorFL_Die    0                  //电机输出pwm死区

////右前轮
//#define motorFR_PWM         GPIO_Pin_7
//#define motorFR_IN1        GPIO_Pin_8
//#define motorFR_IN2        GPIO_Pin_10
//#define motorFR_MAX    5999                  //电机输出pwm限幅
//#define motorFR_Die    0                  //电机输出pwm死区

////右后轮
//#define motorBR_PWM         GPIO_Pin_8
//#define motorBR_IN1          GPIO_Pin_15
//#define motorBR_IN2          GPIO_Pin_13
//#define motorBR_MAX    5999                  //电机输出pwm限幅
//#define motorBR_Die    0                  //电机输出pwm死区

////左后轮
//#define motorBL_PWM          GPIO_Pin_9
//#define motorBL_IN1        GPIO_Pin_9
//#define motorBL_IN2        GPIO_Pin_11
//#define motorBL_MAX    5999                  //速度环输出pwm限幅
//#define motorBL_Die    0                  //电机输出pwm死区


//左前轮
#define motorFL_PWM          GPIO_Pin_9
#define motorFL_IN1         GPIO_Pin_9
#define motorFL_IN2         GPIO_Pin_11
#define motorFL_MAX    5999                  //电机输出pwm限幅
#define motorFL_Die    0                  //电机输出pwm死区

//右前轮
#define motorFR_PWM          GPIO_Pin_6
#define motorFR_IN1         GPIO_Pin_12
#define motorFR_IN2         GPIO_Pin_14
#define motorFR_MAX    5999                  //电机输出pwm限幅
#define motorFR_Die    0                  //电机输出pwm死区

//右后轮
#define motorBR_PWM           GPIO_Pin_7
#define motorBR_IN1            GPIO_Pin_8
#define motorBR_IN2            GPIO_Pin_10
#define motorBR_MAX    5999                  //电机输出pwm限幅
#define motorBR_Die    0                  //电机输出pwm死区

//左后轮
#define motorBL_PWM           GPIO_Pin_8
#define motorBL_IN1         GPIO_Pin_13
#define motorBL_IN2          GPIO_Pin_15
#define motorBL_MAX    5999                  //速度环输出pwm限幅
#define motorBL_Die    0                  //电机输出pwm死区


extern int16_t  Encoder_FL;  //左前轮编码器值
extern int16_t  Encoder_FR;  //右前轮编码器值
extern int16_t  Encoder_BL;  //左后轮编码器值
extern int16_t  Encoder_BR;  //右后轮编码器值


#define HAVE_PID_INTEGRAL 
//--------------------------------------------------------------------------------------------位置式PID
typedef struct{
  float Kp;
  float Ki;
  float Kd;
#ifdef HAVE_PID_INTEGRAL
  int index;            // 积分分离系数
  float Integral;       // 积分项
  float I_outputMax;    // 积分限幅
#endif
	float Err;
  float Last_Err;       // 上次误差
  float Output;         // PID输出
  float OutputMax;      // 位置式PID输出限幅
}Position_PID;
//--------------------------------------------------------------------------------------------增量式PID
typedef struct{
  float Kp;
  float Ki;
  float Kd;
  float p_out;
  float i_out;
  float d_out;
  float Err;
  float Last_Err;       // 上次误差
  float Previous_Err;   // 上上次误差
  float Output;
  float OutputMax;      // 增量式式PID输出限幅
}Incremental_PID;

//--------------------------------------------------------------------------------------------电机参数
typedef struct {
	
	//电机初始化参数
	volatile float target_speed;		//目标速度（cm/s）
	volatile float speed;						//实时速度（cm/s）
	int32_t PWM;											//PWM值
	
	Incremental_PID s_pid;						//速度环pid
	

	int32_t encoder_count_all;				//编码器累计值
	int32_t encoder_count_r;					//编码器计数值(纠正极性)
	float	 encoder_count_f;					//编码器单位时间计数值（滤波）

}Motor;

//--------------------------------------------------------------------------------------------小车参数
typedef struct{
	
	Motor motor_1,motor_2,motor_3,motor_4;//右前，左前，左后，右后
	
	volatile float target_yaw;		//目标角度
	volatile float v_x;						//目标横移速度（向右为正）
	volatile float v_y;						//目标前进速度（向前为正）
	volatile float w;							//目标角速度
	double target_point[2];				//目标坐标点(x,y)
	volatile double target_x;					//当前x坐标
	volatile double target_y;					//当前y坐标
	
	volatile float now_v_x;						//当前横移速度（向右为正）
	volatile float now_v_y;						//当前前进速度（向前为正）
	volatile float yaw;							//当前角度（单位度）
	volatile double now_point[2];		//当前坐标点(x,y)
	volatile double now_x;					//当前x坐标
	volatile double now_y;					//当前y坐标
	volatile float now_the;				  //当前的角度（单位弧度）
	
	Position_PID yaw_pid;						//转向环pid
	Position_PID position_pid_x;			//位置环pid_x轴
	Position_PID position_pid_y;			//位置环pid_y轴

	Position_PID openmv_pid_x;			//位置环pid_x轴
	Position_PID openmv_pid_y;			//位置环pid_y轴
	
	bool stop_flag;
}Car;


extern Car my_car;  //定义小车结构体
extern float YAW ;//航向角
extern float W_Z ;//角速度




void Motor_parameter_Init(void);
void HWT_Init(void);//陀螺仪初始化
uint16_t constrain_long( uint16_t amt,  uint16_t low,  uint16_t high);
void Motor_Init(void);
float limit(float x, float y);// 限幅
void PositionPID_Calculate(Position_PID *pid,const float Target,const float Measure);//位置式PID
void IncrementalPID_Calculate(Incremental_PID *pid,const float Target,const float Measure);//增量式PID
/*************************************************************************
*  函数名称： mortor_TB6612Set(uint16_t motor_PWM, uint16_t motor_AIN1, uint16_t motor_AIN2, int32_t duty, uint16_t motor_MAX, uint16_t motor_Die)
*  功能说明：动量轮电机控制函数
*  参数说明：motor_PWM      PWM引脚
*  参数说明：motor_AIN1、motor_AIN2      方向引脚
*  参数说明：duty         占空比
*  参数说明：motor_MAX    限幅
*  参数说明：motor_Die    死区
*  函数返回：无
*  修改时间：2024年10月22日
*  备    注：TB6612
*************************************************************************/
void mortor_TB6612Set(uint16_t motor_PWM, uint16_t motor_AIN1, uint16_t motor_AIN2, int32_t duty, uint16_t motor_MAX, uint16_t motor_Die);
float sliding_average_filter(float value,float buffer[]);  //滑动窗口滤波函数
void encoder_count_get(void);//获取编码器值
void speed_translation(Motor *motor);//轮子速度换算
void RobotCalculate(void);//正向运动学解析，轮子编码值->底盘三轴里程计坐标

void car_yaw_pid(void);//角度环PID（位置式PID控制）
void mecanum(double v_y,double  v_x  ,double  w);


#endif // !__SERVO_H

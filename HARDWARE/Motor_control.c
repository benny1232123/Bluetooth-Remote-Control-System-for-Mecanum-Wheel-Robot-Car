#include "ALL_Head.h"

int16_t  Encoder_FL = 0;  //左前轮编码器值
int16_t  Encoder_FR = 0;  //右前轮编码器值
int16_t  Encoder_BL = 0;  //左后轮编码器值
int16_t  Encoder_BR = 0;  //右后轮编码器



float YAW = 0.0f;//陀螺仪获取的航向角
float W_Z = 0.0f;//陀螺仪获取的角速度

Car my_car;  //定义小车结构体
void Motor_parameter_Init(void)
{	
	/*******************转向环pid*************/
	my_car.yaw_pid.Kp = 0.06;
  my_car.yaw_pid.Ki = 0.001;
  my_car.yaw_pid.Kd = 0.05;
#ifdef HAVE_PID_INTEGRAL
  my_car.yaw_pid.index = 0;            // 积分分离系数
  my_car.yaw_pid.Integral = 0;       // 积分项
  my_car.yaw_pid.I_outputMax = 0;    // 积分限幅
#endif
	my_car.yaw_pid.Err = 0;
  my_car.yaw_pid.Last_Err = 0;       // 上次误差
  my_car.yaw_pid.Output = 0;         // PID输出
  my_car.yaw_pid.OutputMax = 3;      // 位置式PID输出限幅
	
	/*******************位置环pid*************/
	//x轴，右手方为正方向
	my_car.position_pid_x.Kp = 0.8;
  my_car.position_pid_x.Ki = 0;
  my_car.position_pid_x.Kd = 0;
#ifdef HAVE_PID_INTEGRAL
  my_car.position_pid_x.index = 0;          // 积分分离系数
  my_car.position_pid_x.Integral = 0;       // 积分项
  my_car.position_pid_x.I_outputMax = 0;    // 积分限幅
#endif
	my_car.position_pid_x.Err = 0;
  my_car.position_pid_x.Last_Err = 0;       // 上次误差
  my_car.position_pid_x.Output = 0;         // PID输出
  my_car.position_pid_x.OutputMax = 40;     // 位置式PID输出限幅
	
	//y轴，正前方为正方向
	my_car.position_pid_y.Kp = 0.8;
  my_car.position_pid_y.Ki = 0;
  my_car.position_pid_y.Kd = 0;
#ifdef HAVE_PID_INTEGRAL
  my_car.yaw_pid.index = 0;            // 积分分离系数
  my_car.position_pid_y.Integral = 0;       // 积分项
  my_car.position_pid_y.I_outputMax = 0;    // 积分限幅
#endif
	my_car.position_pid_y.Err = 0;
  my_car.position_pid_y.Last_Err = 0;        // 上次误差
  my_car.position_pid_y.Output = 0;          // PID输出
  my_car.position_pid_y.OutputMax = 40;      // 位置式PID输出限幅

	/*******************openmv位置环pid*************/
	//x轴，右手方为正方向
	my_car.openmv_pid_x.Kp = 0.15;
  my_car.openmv_pid_x.Ki = 0;
  my_car.openmv_pid_x.Kd = 0;
#ifdef HAVE_PID_INTEGRAL
  my_car.openmv_pid_x.index = 0;            // 积分分离系数
  my_car.openmv_pid_x.Integral = 0;       // 积分项
  my_car.openmv_pid_x.I_outputMax = 0;    // 积分限幅
#endif
	my_car.openmv_pid_x.Err = 0;
  my_car.openmv_pid_x.Last_Err = 0;       // 上次误差
  my_car.openmv_pid_x.Output = 0;         // PID输出
  my_car.openmv_pid_x.OutputMax = 5;      // 位置式PID输出限幅
	
	//y轴，正前方为正方向
	my_car.openmv_pid_y.Kp = 0.15;
  my_car.openmv_pid_y.Ki = 0;
  my_car.openmv_pid_y.Kd = 0;
#ifdef HAVE_PID_INTEGRAL
  my_car.yaw_pid.index = 0;            // 积分分离系数
  my_car.openmv_pid_y.Integral = 0;       // 积分项
  my_car.openmv_pid_y.I_outputMax = 0;    // 积分限幅
#endif
	my_car.openmv_pid_y.Err = 0;
  my_car.openmv_pid_y.Last_Err = 0;        // 上次误差
  my_car.openmv_pid_y.Output = 0;          // PID输出
  my_car.openmv_pid_y.OutputMax = 5;      // 位置式PID输出限幅

	/*******************电机增量式pid*************/
	//右前轮子
  my_car.motor_1.s_pid.Kp = 10;
  my_car.motor_1.s_pid.Ki = 3;
  my_car.motor_1.s_pid.Kd = 0;
  my_car.motor_1.s_pid.p_out = 0;
  my_car.motor_1.s_pid.i_out = 0;
  my_car.motor_1.s_pid.d_out = 0;
  my_car.motor_1.s_pid.Err = 0;
  my_car.motor_1.s_pid.Last_Err = 0;       // 上次误差
  my_car.motor_1.s_pid.Previous_Err = 0;   // 上上次误差
  my_car.motor_1.s_pid.Output = 0;
  my_car.motor_1.s_pid.OutputMax = motorFR_MAX;      // 增量式式PID输出限幅
	
	//左前轮子
  my_car.motor_2.s_pid.Kp = 10;
  my_car.motor_2.s_pid.Ki = 3;
  my_car.motor_2.s_pid.Kd = 0;
  my_car.motor_2.s_pid.p_out = 0;
  my_car.motor_2.s_pid.i_out = 0;
  my_car.motor_2.s_pid.d_out = 0;
  my_car.motor_2.s_pid.Err = 0;
  my_car.motor_2.s_pid.Last_Err = 0;       // 上次误差
  my_car.motor_2.s_pid.Previous_Err = 0;   // 上上次误差
  my_car.motor_2.s_pid.Output = 0;
  my_car.motor_2.s_pid.OutputMax = motorFL_MAX;      // 增量式式PID输出限幅

	//左后轮子
  my_car.motor_3.s_pid.Kp = 10;
  my_car.motor_3.s_pid.Ki = 3;
  my_car.motor_3.s_pid.Kd = 0;
  my_car.motor_3.s_pid.p_out = 0;
  my_car.motor_3.s_pid.i_out = 0;
  my_car.motor_3.s_pid.d_out = 0;
  my_car.motor_3.s_pid.Err = 0;
  my_car.motor_3.s_pid.Last_Err = 0;       // 上次误差
  my_car.motor_3.s_pid.Previous_Err = 0;   // 上上次误差
  my_car.motor_3.s_pid.Output = 0;
  my_car.motor_3.s_pid.OutputMax = motorBL_MAX;      // 增量式式PID输出限幅
	
	//右后轮子
  my_car.motor_4.s_pid.Kp = 10;
  my_car.motor_4.s_pid.Ki = 3;
  my_car.motor_4.s_pid.Kd = 0;
  my_car.motor_4.s_pid.p_out = 0;
  my_car.motor_4.s_pid.i_out = 0;
  my_car.motor_4.s_pid.d_out = 0;
  my_car.motor_4.s_pid.Err = 0;
  my_car.motor_4.s_pid.Last_Err = 0;       // 上次误差
  my_car.motor_4.s_pid.Previous_Err = 0;   // 上上次误差
  my_car.motor_4.s_pid.Output = 0;
  my_car.motor_4.s_pid.OutputMax = motorBR_MAX;      // 增量式式PID输出限幅
	
	my_car.stop_flag = 1;   //停车标志位

	
		//电机初始化参数
	my_car.motor_1.target_speed = 0;		//目标速度（cm/s）
	my_car.motor_1.speed = 0;						//实时速度（cm/s）
	my_car.motor_1.PWM = 0;											//PWM值
	my_car.motor_1.encoder_count_all = 0;				//编码器累计值
	my_car.motor_1.encoder_count_r = 0;					//编码器计数值(纠正极性)
	my_car.motor_1.encoder_count_f = 0;					//编码器单位时间计数值（滤波）

	
	my_car.motor_2.target_speed = 0;		//目标速度（cm/s）
	my_car.motor_2.speed = 0;						//实时速度（cm/s）
	my_car.motor_2.PWM = 0;											//PWM值	
	my_car.motor_2.encoder_count_all = 0;				//编码器累计值
	my_car.motor_2.encoder_count_r = 0;					//编码器计数值(纠正极性)
	my_car.motor_2.encoder_count_f = 0;					//编码器单位时间计数值（滤波）
	
	
	my_car.motor_3.target_speed = 0;		//目标速度（cm/s）
	my_car.motor_3.speed = 0;						//实时速度（cm/s）
	my_car.motor_3.PWM = 0;											//PWM值
	my_car.motor_3.encoder_count_all = 0;				//编码器累计值
	my_car.motor_3.encoder_count_r = 0;					//编码器计数值(纠正极性)
	my_car.motor_3.encoder_count_f = 0;					//编码器单位时间计数值（滤波）
	
	
	my_car.motor_4.target_speed = 0;		//目标速度（cm/s）
	my_car.motor_4.speed = 0;						//实时速度（cm/s）
	my_car.motor_4.PWM = 0;											//PWM值
	my_car.motor_4.encoder_count_all = 0;				//编码器累计值
	my_car.motor_4.encoder_count_r = 0;					//编码器计数值(纠正极性)
	my_car.motor_4.encoder_count_f = 0;					//编码器单位时间计数值（滤波）
	
	//小车参数
	my_car.target_yaw = 0;		//目标角度
	my_car.v_x = 0;						//目标横移速度（向右为正）
	my_car.v_y = 0;						//目标前进速度（向前为正）
	my_car.w = 0;							//目标角速度
	my_car.target_x = 0;					//目标x坐标
	my_car.target_y = 0;					//目标y坐标
	
	my_car.target_point[0] = 0;				//目标坐标点(x,y)的x
	my_car.target_point[1] = 0;				//目标坐标点(x,y)的y
	
	my_car.now_v_x = 0;						//当前横移速度（向右为正）
	my_car.now_v_y = 0;						//当前前进速度（向前为正）
	my_car.yaw = 0;							//当前角度（单位度）
	
	my_car.now_point[0] = 0;		//当前坐标点(x,y)的x
	my_car.now_point[1] = 0;		//当前坐标点(x,y)的y

	my_car.now_x = 0;					//当前x坐标
	my_car.now_y = 0;					//当前y坐标
	my_car.now_the = 0;				  //当前的角度（单位弧度）
	
}

//unsigned long限幅函数
 uint16_t constrain_long( uint16_t amt,  uint16_t low,  uint16_t high)
{
    return ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)));
}

void Motor_Init(void)
{
	  
	GPIO_InitTypeDef GPIO_InitStructure;

	   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE); //使能GPIO外设模块时钟
  //设置该引脚为复用输出功能,输出TIM8 CH的PWM脉冲波形	GPIO
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//复用推挽输出
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_9| GPIO_Pin_10 | GPIO_Pin_11 |GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	//左前轮子
	//			 GPIO_SetBits(GPIOD,GPIO_Pin_12); 			//PD12 输出高 
//			 GPIO_SetBits(GPIOD,GPIO_Pin_14); 			//PD14 输出高 
			 GPIO_ResetBits(GPIOD,GPIO_Pin_12); 			//PD12 输出低 
			 GPIO_ResetBits(GPIOD,GPIO_Pin_14); 			//PD14 输出低 
	
	//右前轮子
//			 GPIO_SetBits(GPIOD,GPIO_Pin_8); 			  //PD8 输出高 
//			 GPIO_SetBits(GPIOD,GPIO_Pin_10); 			//PD10 输出高 
			 GPIO_ResetBits(GPIOD,GPIO_Pin_8); 			  //PD8 输出低 
			 GPIO_ResetBits(GPIOD,GPIO_Pin_10); 			//PD10 输出低
	
	//右后轮子
//			 GPIO_SetBits(GPIOD,GPIO_Pin_13); 			//PD13 输出高 
//			 GPIO_SetBits(GPIOD,GPIO_Pin_15); 			//PD15 输出高 
			 GPIO_ResetBits(GPIOD,GPIO_Pin_13); 			//PD13 输出低 
			 GPIO_ResetBits(GPIOD,GPIO_Pin_15); 			//PD15 输出低 

	//左后轮子
//			 GPIO_SetBits(GPIOD,GPIO_Pin_9); 			  //PD9 输出高 
//			 GPIO_SetBits(GPIOD,GPIO_Pin_11); 			//PD11 输出高 
			 GPIO_ResetBits(GPIOD,GPIO_Pin_9); 			  //PD9 输出低 
			 GPIO_ResetBits(GPIOD,GPIO_Pin_11); 			//PD11 输出低 

		
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);	
	
		GPIO_InitTypeDef GPIO_InitStruct;
		GPIO_InitStruct.GPIO_Mode= GPIO_Mode_AF_PP ;
		GPIO_InitStruct.GPIO_Pin=GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_6|GPIO_Pin_7;
		GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
		GPIO_Init(GPIOC, &GPIO_InitStruct); 
	
	  TIM_InternalClockConfig(TIM8);
	
		TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
		TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV1 ;
		TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up ;
		TIM_TimeBaseInitStruct.TIM_Period=8000-1;
		TIM_TimeBaseInitStruct.TIM_Prescaler=1-1;
		TIM_TimeBaseInitStruct.TIM_RepetitionCounter=0;	
		TIM_TimeBaseInit(TIM8, &TIM_TimeBaseInitStruct);

		TIM_OCInitTypeDef TIM_OCInitStruct;
		TIM_OCStructInit(&TIM_OCInitStruct);
		TIM_OCInitStruct.TIM_OCMode=TIM_OCMode_PWM1;
		TIM_OCInitStruct.TIM_OCPolarity=TIM_OCPolarity_High  ;
		TIM_OCInitStruct.TIM_OutputState=TIM_OutputState_Enable ;
		TIM_OCInitStruct.TIM_Pulse=0;
		TIM_OC1Init(TIM8, &TIM_OCInitStruct);
		
		TIM_OCStructInit(&TIM_OCInitStruct);
		TIM_OCInitStruct.TIM_OCMode=TIM_OCMode_PWM1;
		TIM_OCInitStruct.TIM_OCPolarity=TIM_OCPolarity_High  ;
		TIM_OCInitStruct.TIM_OutputState=TIM_OutputState_Enable ;
		TIM_OCInitStruct.TIM_Pulse=0;
    TIM_OC2Init(TIM8, &TIM_OCInitStruct);
		
		TIM_OCStructInit(&TIM_OCInitStruct);
		TIM_OCInitStruct.TIM_OCMode=TIM_OCMode_PWM1;
		TIM_OCInitStruct.TIM_OCPolarity=TIM_OCPolarity_High  ;
		TIM_OCInitStruct.TIM_OutputState=TIM_OutputState_Enable ;
		TIM_OCInitStruct.TIM_Pulse=0;
    TIM_OC3Init(TIM8, &TIM_OCInitStruct);
		
		TIM_OCStructInit(&TIM_OCInitStruct);
		TIM_OCInitStruct.TIM_OCMode=TIM_OCMode_PWM1;
		TIM_OCInitStruct.TIM_OCPolarity=TIM_OCPolarity_High  ;
		TIM_OCInitStruct.TIM_OutputState=TIM_OutputState_Enable ;
		TIM_OCInitStruct.TIM_Pulse=0;
    TIM_OC4Init(TIM8, &TIM_OCInitStruct);
		
		TIM_Cmd(TIM8,ENABLE);
    TIM_CtrlPWMOutputs(TIM8, ENABLE);
}
void HWT_Init(void)//陀螺仪初始化
{
	uint8_t buff1[5] = {0XFF, 0XAA, 0X69, 0X88, 0XB5};//解锁陀螺仪配置指令
  uint8_t buff2[5] = {0XFF, 0XAA, 0X76, 0X00, 0X00};//Z轴归零
	//陀螺仪数据初始化
	uart_write_buffer(USART3, buff1, 5);
	delay_ms(10);                     //延时10ms
	uart_write_buffer(USART3, buff2, 5);
	delay_ms(10);                     //延时10ms
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      限幅
//  @param      x				被限幅的数据
//  @param      y				限幅范围(数据会被限制在-y至+y之间)
//  @return     float			限幅之后的数据			
//  Sample usage:				float dat = limit(500,300);//数据被限制在-300至+300之间  因此返回的结果是300
//-------------------------------------------------------------------------------------------------------------------
float limit(float x, float y)
{
    if(x>y)             return y;
    else if(x<-y)       return -y;
    else                return x;
}

//--------------------------------------------------------------------------------------------位置式PID
#define Integraldead_zone 300 // 积分死区 根据自己的需求定义
void PositionPID_Calculate(Position_PID *pid,const float Target,const float Measure)
{

  if(pid == NULL)
    return;
	pid->Err = Target - Measure;
	
  pid->Output = pid->Kp * pid->Err + pid->Kd * (pid->Err - pid->Last_Err);
  
#ifdef HAVE_PID_INTEGRAL
    /* 积分分离 */
  if(fabs(pid->Err) > Integraldead_zone)
  {
    pid->index=0;
  }else
  {
    pid->index = 1;
  }
  pid->Integral += pid->Ki * pid->Err * pid->index;
  
  pid->Integral = limit(pid->Integral,pid->I_outputMax);
  
  pid->Output += pid->Integral;
#endif
  
  pid->Output = limit(pid->Output,pid->OutputMax);
  
  pid->Last_Err = pid->Err;
  
}


//--------------------------------------------------------------------------------------------增量式PID      40     0     +40      0
void IncrementalPID_Calculate(Incremental_PID *pid,const float Target,const float Measure)	//							40      10    +30    30-40 = -10  缓慢接近，
																																														//							40      41    -1      0
																																															//            40      45    -5      -5-（-1）=-4  更快接近目标
{
  if(pid == NULL)									
    return;
  pid->Err = Target - Measure;	
	
  pid->p_out = pid->Kp * (pid->Err - pid->Last_Err);//变化量
  pid->i_out = pid->Ki * pid->Err;//误差 
  pid->d_out = pid->Kd * (pid->Err - 2.0f*pid->Last_Err + pid->Previous_Err);
  
  pid->Output += pid->p_out + pid->i_out + pid->d_out; 
  
  pid->Output = limit(pid->Output, pid->OutputMax); // 限幅
  
  pid->Previous_Err = pid->Last_Err;
  pid->Last_Err = pid->Err;
  
}



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
void mortor_TB6612Set(uint16_t motor_PWM, uint16_t motor_AIN1, uint16_t motor_AIN2, int32_t duty, uint16_t motor_MAX, uint16_t motor_Die)
{

		uint16_t temp_duty = 0;
		if(duty > 0 )   //正转
		{
				temp_duty = (uint16_t)duty + motor_Die;
				GPIO_SetBits(GPIOD,motor_AIN1); 			//输出高 
				GPIO_ResetBits(GPIOD,motor_AIN2); 			//输出低 

		}
		else if(duty < 0)  //反转
		{
				temp_duty = (uint16_t)(-duty) + motor_Die;
				GPIO_ResetBits(GPIOD,motor_AIN1); 			//输出低 
				GPIO_SetBits(GPIOD,motor_AIN2); 			//输出高 

		}
		else if(duty == 0)  //停
		{
				temp_duty = 0;
				GPIO_ResetBits(GPIOD,motor_AIN1); 			//输出低 
				GPIO_ResetBits(GPIOD,motor_AIN2); 			//输出低 

		}
		temp_duty = constrain_long(temp_duty, 0, motor_MAX);  //限幅

		if(motor_PWM == motorFR_PWM )//PC6
		{
				TIM_SetCompare1(TIM8, temp_duty);
		}
		else if(motor_PWM == motorBR_PWM )//PC7
		{
				TIM_SetCompare2(TIM8, temp_duty);

		}
		else if(motor_PWM == motorBL_PWM )//PC8
		{
				TIM_SetCompare3(TIM8, temp_duty);
			 
		}
		else if(motor_PWM == motorFL_PWM )//PC9
		{
				TIM_SetCompare4(TIM8, temp_duty);

		}

}


/************************ 滑动窗口滤波器 *****************************/
#define window_size 5            //滑动窗口长度
float buffer_1[window_size] = {0}; //滑动窗口数据buf
float buffer_2[window_size] = {0}; //滑动窗口数据buf
float buffer_3[window_size] = {0}; //滑动窗口数据buf
float buffer_4[window_size] = {0}; //滑动窗口数据buf
/*********************** 滑动窗口滤波函数 ****************************/
float sliding_average_filter(float value,float buffer[])  //滑动窗口滤波函数
{
  static int data_num = 0;
  float output = 0;

  if (data_num < window_size) //不满窗口，先填充
  {
    buffer[data_num++] = value;
    output = value; //返回相同的值
  }
  else
  {
    int i = 0;
    float sum = 0;

    memcpy(&buffer[0], &buffer[1], (window_size - 1) * 4); //将1之后的数据移到0之后，即移除头部
    buffer[window_size - 1] = value;                       //即添加尾部

    for (i = 0; i < window_size; i++) //每一次都计算，可以避免累计浮点计算误差
      sum += buffer[i];

    output = sum / window_size;
  }

  return output;
}

void encoder_count_get(void)//获取编码器值
{
	my_car.motor_1.encoder_count_r = Encoder_FR;
	my_car.motor_2.encoder_count_r = Encoder_FL;
	my_car.motor_3.encoder_count_r = Encoder_BL;
	my_car.motor_4.encoder_count_r = Encoder_BR;

	my_car.motor_1.encoder_count_all += my_car.motor_1.encoder_count_r;
	my_car.motor_2.encoder_count_all += my_car.motor_2.encoder_count_r;
	my_car.motor_3.encoder_count_all += my_car.motor_3.encoder_count_r;
	my_car.motor_4.encoder_count_all += my_car.motor_4.encoder_count_r;
	
	
	my_car.motor_1.encoder_count_f =  sliding_average_filter((float)(my_car.motor_1.encoder_count_r),buffer_1);
	my_car.motor_2.encoder_count_f =  sliding_average_filter((float)(my_car.motor_2.encoder_count_r),buffer_2);
	my_car.motor_3.encoder_count_f =  sliding_average_filter((float)(my_car.motor_3.encoder_count_r),buffer_3);
	my_car.motor_4.encoder_count_f =  sliding_average_filter((float)(my_car.motor_4.encoder_count_r),buffer_4);

}

void speed_translation(Motor *motor)//轮子速度换算
{
	motor->speed = ((motor->encoder_count_f/ENCODER_ACCURACY)*Tooth_Proportion)*Perimeter/ENCODER_TIME_S;
}



/**
  * @函数功能：正向运动学解析，轮子编码值->底盘三轴里程计坐标
  */
void RobotCalculate(void)//正向运动学解析，轮子编码值->底盘三轴里程计坐标
{
    // 根据每帧的motors[4]计算更新g_robot
//    float deltacounts[4]={0,0,0,0};
    float delta_x_o, delta_y_o;
    float delta_x, delta_y;
    float everycount;
    
    my_car.now_the = my_car.yaw * PI / 180.0f;    //当前角度转换成当前弧度
    
    while((my_car.now_the >= 2.0f * PI) || (my_car.now_the < 0))
    {
        if(my_car.now_the >= 2.0f * PI)
            my_car.now_the = my_car.now_the - 2.0f * PI;
        if(my_car.now_the < 0)
            my_car.now_the = my_car.now_the + 2.0f * PI;
    }
    
    everycount = ((1.0f/ENCODER_ACCURACY)*Tooth_Proportion)*Perimeter;//每个脉冲走多少cm
    
//    deltacounts[0] = my_car.motor_2.encoder_count_f; //FR
//    deltacounts[1] = my_car.motor_1.encoder_count_f; //FL
//    deltacounts[2] = my_car.motor_4.encoder_count_f; //BR
//    deltacounts[3] = my_car.motor_3.encoder_count_f; //BL
//    delta_x_o = (-deltacounts[1] + deltacounts[2]) / (2.0f) * everycount; //-FR+BR
//    delta_y_o = (deltacounts[0] + deltacounts[1]) / (2.0f) * everycount;  //FL+FL

//    delta_x_o = (-deltacounts[1] + deltacounts[2]) / (2.0f) * everycount; //-FR+BR
//    delta_y_o = (deltacounts[0] + deltacounts[1]) / (2.0f) * everycount;  //FL+FL
//		delta_x_o = (-my_car.motor_1.encoder_count_f + my_car.motor_4.encoder_count_f) / (2.0f) * everycount;  //-FR+BR
//    delta_y_o = ( my_car.motor_1.encoder_count_f + my_car.motor_2.encoder_count_f) / (2.0f) * everycount; //FL+FR

		delta_x_o = (-my_car.motor_1.encoder_count_f + my_car.motor_2.encoder_count_f) / (2.0f) * everycount;  //-FR+FL
    delta_y_o = ( my_car.motor_1.encoder_count_f + my_car.motor_4.encoder_count_f) / (2.0f) * everycount; //FR+BR
		
    if (my_car.now_the >= 0 && my_car.now_the < (PI/2.0f))
    {
        delta_x = delta_x_o * sin(PI/2.0f - my_car.now_the) + delta_y_o * cos(PI/2.0f - my_car.now_the);
        delta_y = -delta_x_o * cos(PI/2.0f - my_car.now_the) + delta_y_o * sin(PI/2.0f - my_car.now_the);
    }
    
    if (my_car.now_the >= (PI/2.0f) && my_car.now_the < PI)
    {
        delta_x = -delta_x_o * sin(my_car.now_the - PI/2.0f) + delta_y_o * cos(my_car.now_the - (PI/2.0f));
        delta_y = -delta_x_o * cos(my_car.now_the - PI/2.0f) - delta_y_o * sin(my_car.now_the - (PI/2.0f));
    }
    
    if (my_car.now_the >= PI && my_car.now_the < (3.0f*PI/2.0f))
    {
        delta_x = -delta_x_o * cos(my_car.now_the - PI) - delta_y_o * sin(my_car.now_the - PI);
        delta_y = delta_x_o * sin(my_car.now_the - PI) - delta_y_o * cos(my_car.now_the - PI);
    }
    
    if (my_car.now_the >= (3.0f*PI/2.0f) && my_car.now_the < (2.0f*PI))
    {
        delta_x = delta_x_o * sin(my_car.now_the - 3.0f*PI/2.0f) - delta_y_o * cos(my_car.now_the - 3.0f*PI/2.0f);
        delta_y = delta_x_o * cos(my_car.now_the - 3.0f*PI/2.0f) + delta_y_o * sin(my_car.now_the - 3.0f*PI/2.0f);
    }
    
		my_car.now_v_x = delta_x / ENCODER_TIME_S;
		my_car.now_v_y = delta_y / ENCODER_TIME_S;
    my_car.now_x += delta_x;
    my_car.now_y += delta_y;
		my_car.now_point[0] = my_car.now_x;
		my_car.now_point[1] = my_car.now_y;

}

void car_yaw_pid(void)//角度环PID（位置式PID控制）
{
	
	float temp_err,temp_target_yaw;
	
	my_car.yaw = YAW;  //当前角度赋值
	
	temp_err = my_car.target_yaw - my_car.yaw;
		//取最短路径，重新计算角度误差
	if(temp_err > 180.0f)
	{
	  temp_err = -360.0f + temp_err;
	}

	else if(temp_err < -180.0f)
	{
	  temp_err = 360.0f + temp_err;
	}
	
	else if(fabs(temp_err) <= 180.f)
	{
		temp_err = temp_err;
	}
	
	temp_target_yaw = temp_err + my_car.yaw;
	
	
	PositionPID_Calculate(&my_car.yaw_pid, temp_target_yaw, my_car.yaw);
	my_car.w = my_car.yaw_pid.Output;
}

void  mecanum(double v_y,double  v_x  ,double  w)
{

		my_car.motor_1.target_speed = ( v_y - v_x + w* (HALF_LENGTH +  HALF_WIDTH));//右前轮子，motor_1+
		my_car.motor_2.target_speed = ( v_y + v_x + w* (HALF_LENGTH +  HALF_WIDTH));//左前轮子，motor_2-
		my_car.motor_3.target_speed = ( v_y - v_x - w* (HALF_LENGTH +  HALF_WIDTH));//左后轮子，motor_3-
		my_car.motor_4.target_speed = ( v_y + v_x - w* (HALF_LENGTH +  HALF_WIDTH));//右后轮子，motor_4+

		
//		limit(my_car.motor_1.target_speed, 200.0f); // 限幅
//		limit(my_car.motor_2.target_speed, 200.0f); // 限幅
//		limit(my_car.motor_3.target_speed, 200.0f); // 限幅
//		limit(my_car.motor_4.target_speed, 200.0f); // 限幅
}


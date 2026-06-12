#include "ALL_Head.h"

int16_t  Encoder_FL = 0;  //��ǰ�ֱ�����ֵ
int16_t  Encoder_FR = 0;  //��ǰ�ֱ�����ֵ
int16_t  Encoder_BL = 0;  //����ֱ�����ֵ
int16_t  Encoder_BR = 0;  //�Һ��ֱ�����



float YAW = 0.0f;//�����ǻ�ȡ�ĺ����
float W_Z = 0.0f;//�����ǻ�ȡ�Ľ��ٶ�

Car my_car;  //����С���ṹ��
void Motor_parameter_Init(void)
{	
	/*******************ת��pid*************/
	my_car.yaw_pid.Kp = 0.06;
  my_car.yaw_pid.Ki = 0.001;
  my_car.yaw_pid.Kd = 0.05;
#ifdef HAVE_PID_INTEGRAL
  my_car.yaw_pid.index = 0;            // ���ַ���ϵ��
  my_car.yaw_pid.Integral = 0;       // ������
  my_car.yaw_pid.I_outputMax = 0;    // �����޷�
#endif
	my_car.yaw_pid.Err = 0;
  my_car.yaw_pid.Last_Err = 0;       // �ϴ����
  my_car.yaw_pid.Output = 0;         // PID���
  my_car.yaw_pid.OutputMax = 3;      // λ��ʽPID����޷�
	
	/*******************λ�û�pid*************/
	//x�ᣬ���ַ�Ϊ������
	my_car.position_pid_x.Kp = 0.8;
  my_car.position_pid_x.Ki = 0;
  my_car.position_pid_x.Kd = 0;
#ifdef HAVE_PID_INTEGRAL
  my_car.position_pid_x.index = 0;          // ���ַ���ϵ��
  my_car.position_pid_x.Integral = 0;       // ������
  my_car.position_pid_x.I_outputMax = 0;    // �����޷�
#endif
	my_car.position_pid_x.Err = 0;
  my_car.position_pid_x.Last_Err = 0;       // �ϴ����
  my_car.position_pid_x.Output = 0;         // PID���
  my_car.position_pid_x.OutputMax = 40;     // λ��ʽPID����޷�
	
	//y�ᣬ��ǰ��Ϊ������
	my_car.position_pid_y.Kp = 0.8;
  my_car.position_pid_y.Ki = 0;
  my_car.position_pid_y.Kd = 0;
#ifdef HAVE_PID_INTEGRAL
  my_car.yaw_pid.index = 0;            // ���ַ���ϵ��
  my_car.position_pid_y.Integral = 0;       // ������
  my_car.position_pid_y.I_outputMax = 0;    // �����޷�
#endif
	my_car.position_pid_y.Err = 0;
  my_car.position_pid_y.Last_Err = 0;        // �ϴ����
  my_car.position_pid_y.Output = 0;          // PID���
  my_car.position_pid_y.OutputMax = 40;      // λ��ʽPID����޷�

	/*******************openmvλ�û�pid*************/
	//x�ᣬ���ַ�Ϊ������
	my_car.openmv_pid_x.Kp = 0.008;
  my_car.openmv_pid_x.Ki = 0;
  my_car.openmv_pid_x.Kd = 0;
#ifdef HAVE_PID_INTEGRAL
  my_car.openmv_pid_x.index = 0;            // ���ַ���ϵ��
  my_car.openmv_pid_x.Integral = 0;       // ������
  my_car.openmv_pid_x.I_outputMax = 0;    // �����޷�
#endif
	my_car.openmv_pid_x.Err = 0;
  my_car.openmv_pid_x.Last_Err = 0;       // �ϴ����
  my_car.openmv_pid_x.Output = 0;         // PID���
  my_car.openmv_pid_x.OutputMax = 0.8;      // λ��ʽPID����޷�
	
	//y�ᣬ��ǰ��Ϊ������
	my_car.openmv_pid_y.Kp = 0.15;
  my_car.openmv_pid_y.Ki = 0;
  my_car.openmv_pid_y.Kd = 0;
#ifdef HAVE_PID_INTEGRAL
  my_car.yaw_pid.index = 0;            // ���ַ���ϵ��
  my_car.openmv_pid_y.Integral = 0;       // ������
  my_car.openmv_pid_y.I_outputMax = 0;    // �����޷�
#endif
	my_car.openmv_pid_y.Err = 0;
  my_car.openmv_pid_y.Last_Err = 0;        // �ϴ����
  my_car.openmv_pid_y.Output = 0;          // PID���
  my_car.openmv_pid_y.OutputMax = 5;      // λ��ʽPID����޷�

	/*******************�������ʽpid*************/
	//��ǰ����
  my_car.motor_1.s_pid.Kp = 10;
  my_car.motor_1.s_pid.Ki = 3;
  my_car.motor_1.s_pid.Kd = 0;
  my_car.motor_1.s_pid.p_out = 0;
  my_car.motor_1.s_pid.i_out = 0;
  my_car.motor_1.s_pid.d_out = 0;
  my_car.motor_1.s_pid.Err = 0;
  my_car.motor_1.s_pid.Last_Err = 0;       // �ϴ����
  my_car.motor_1.s_pid.Previous_Err = 0;   // ���ϴ����
  my_car.motor_1.s_pid.Output = 0;
  my_car.motor_1.s_pid.OutputMax = motorFR_MAX;      // ����ʽʽPID����޷�
	
	//��ǰ����
  my_car.motor_2.s_pid.Kp = 10;
  my_car.motor_2.s_pid.Ki = 3;
  my_car.motor_2.s_pid.Kd = 0;
  my_car.motor_2.s_pid.p_out = 0;
  my_car.motor_2.s_pid.i_out = 0;
  my_car.motor_2.s_pid.d_out = 0;
  my_car.motor_2.s_pid.Err = 0;
  my_car.motor_2.s_pid.Last_Err = 0;       // �ϴ����
  my_car.motor_2.s_pid.Previous_Err = 0;   // ���ϴ����
  my_car.motor_2.s_pid.Output = 0;
  my_car.motor_2.s_pid.OutputMax = motorFL_MAX;      // ����ʽʽPID����޷�

	//�������
  my_car.motor_3.s_pid.Kp = 10;
  my_car.motor_3.s_pid.Ki = 3;
  my_car.motor_3.s_pid.Kd = 0;
  my_car.motor_3.s_pid.p_out = 0;
  my_car.motor_3.s_pid.i_out = 0;
  my_car.motor_3.s_pid.d_out = 0;
  my_car.motor_3.s_pid.Err = 0;
  my_car.motor_3.s_pid.Last_Err = 0;       // �ϴ����
  my_car.motor_3.s_pid.Previous_Err = 0;   // ���ϴ����
  my_car.motor_3.s_pid.Output = 0;
  my_car.motor_3.s_pid.OutputMax = motorBL_MAX;      // ����ʽʽPID����޷�
	
	//�Һ�����
  my_car.motor_4.s_pid.Kp = 10;
  my_car.motor_4.s_pid.Ki = 3;
  my_car.motor_4.s_pid.Kd = 0;
  my_car.motor_4.s_pid.p_out = 0;
  my_car.motor_4.s_pid.i_out = 0;
  my_car.motor_4.s_pid.d_out = 0;
  my_car.motor_4.s_pid.Err = 0;
  my_car.motor_4.s_pid.Last_Err = 0;       // �ϴ����
  my_car.motor_4.s_pid.Previous_Err = 0;   // ���ϴ����
  my_car.motor_4.s_pid.Output = 0;
  my_car.motor_4.s_pid.OutputMax = motorBR_MAX;      // ����ʽʽPID����޷�
	
	my_car.stop_flag = 1;   //ͣ����־λ

	
		//�����ʼ������
	my_car.motor_1.target_speed = 0;		//Ŀ���ٶȣ�cm/s��
	my_car.motor_1.speed = 0;						//ʵʱ�ٶȣ�cm/s��
	my_car.motor_1.PWM = 0;											//PWMֵ
	my_car.motor_1.encoder_count_all = 0;				//�������ۼ�ֵ
	my_car.motor_1.encoder_count_r = 0;					//����������ֵ(��������)
	my_car.motor_1.encoder_count_f = 0;					//��������λʱ�����ֵ���˲���

	
	my_car.motor_2.target_speed = 0;		//Ŀ���ٶȣ�cm/s��
	my_car.motor_2.speed = 0;						//ʵʱ�ٶȣ�cm/s��
	my_car.motor_2.PWM = 0;											//PWMֵ	
	my_car.motor_2.encoder_count_all = 0;				//�������ۼ�ֵ
	my_car.motor_2.encoder_count_r = 0;					//����������ֵ(��������)
	my_car.motor_2.encoder_count_f = 0;					//��������λʱ�����ֵ���˲���
	
	
	my_car.motor_3.target_speed = 0;		//Ŀ���ٶȣ�cm/s��
	my_car.motor_3.speed = 0;						//ʵʱ�ٶȣ�cm/s��
	my_car.motor_3.PWM = 0;											//PWMֵ
	my_car.motor_3.encoder_count_all = 0;				//�������ۼ�ֵ
	my_car.motor_3.encoder_count_r = 0;					//����������ֵ(��������)
	my_car.motor_3.encoder_count_f = 0;					//��������λʱ�����ֵ���˲���
	
	
	my_car.motor_4.target_speed = 0;		//Ŀ���ٶȣ�cm/s��
	my_car.motor_4.speed = 0;						//ʵʱ�ٶȣ�cm/s��
	my_car.motor_4.PWM = 0;											//PWMֵ
	my_car.motor_4.encoder_count_all = 0;				//�������ۼ�ֵ
	my_car.motor_4.encoder_count_r = 0;					//����������ֵ(��������)
	my_car.motor_4.encoder_count_f = 0;					//��������λʱ�����ֵ���˲���
	//С������
	my_car.target_yaw = 0;		//Ŀ��Ƕ�
	my_car.v_x = 0;						//Ŀ������ٶȣ�����Ϊ����
	my_car.v_y = 0;						//Ŀ��ǰ���ٶȣ���ǰΪ����
	my_car.w = 0;							//Ŀ����ٶ�
	my_car.target_x = 0;					//Ŀ��x����
	my_car.target_y = 0;					//Ŀ��y����
	
	my_car.target_point[0] = 0;				//Ŀ�������(x,y)��x
	my_car.target_point[1] = 0;				//Ŀ�������(x,y)��y
	
	my_car.now_v_x = 0;						//��ǰ�����ٶȣ�����Ϊ����
	my_car.now_v_y = 0;						//��ǰǰ���ٶȣ���ǰΪ����
	my_car.yaw = 0;							//��ǰ�Ƕȣ���λ�ȣ�
	
	my_car.now_point[0] = 0;		//��ǰ�����(x,y)��x
	my_car.now_point[1] = 0;		//��ǰ�����(x,y)��y

	my_car.now_x = 0;					//��ǰx����
	my_car.now_y = 0;					//��ǰy����
	my_car.now_the = 0;				  //��ǰ�ĽǶȣ���λ���ȣ�
	
}

//unsigned long�޷�����
 uint16_t constrain_long( uint16_t amt,  uint16_t low,  uint16_t high)
{
    return ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)));
}

void Motor_Init(void)
{
	  
	GPIO_InitTypeDef GPIO_InitStructure;

	   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE); //ʹ��GPIO����ģ��ʱ��
  //���ø�����Ϊ�����������,���TIM8 CH��PWM���岨��	GPIO
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//�����������
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_9| GPIO_Pin_10 | GPIO_Pin_11 |GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	//��ǰ����
	//			 GPIO_SetBits(GPIOD,GPIO_Pin_12); 			//PD12 ����� 
//			 GPIO_SetBits(GPIOD,GPIO_Pin_14); 			//PD14 ����� 
			 GPIO_ResetBits(GPIOD,GPIO_Pin_12); 			//PD12 ����� 
			 GPIO_ResetBits(GPIOD,GPIO_Pin_14); 			//PD14 ����� 
	
	//��ǰ����
//			 GPIO_SetBits(GPIOD,GPIO_Pin_8); 			  //PD8 ����� 
//			 GPIO_SetBits(GPIOD,GPIO_Pin_10); 			//PD10 ����� 
			 GPIO_ResetBits(GPIOD,GPIO_Pin_8); 			  //PD8 ����� 
			 GPIO_ResetBits(GPIOD,GPIO_Pin_10); 			//PD10 �����
	
	//�Һ�����
//			 GPIO_SetBits(GPIOD,GPIO_Pin_13); 			//PD13 ����� 
//			 GPIO_SetBits(GPIOD,GPIO_Pin_15); 			//PD15 ����� 
			 GPIO_ResetBits(GPIOD,GPIO_Pin_13); 			//PD13 ����� 
			 GPIO_ResetBits(GPIOD,GPIO_Pin_15); 			//PD15 ����� 

	//�������
//			 GPIO_SetBits(GPIOD,GPIO_Pin_9); 			  //PD9 ����� 
//			 GPIO_SetBits(GPIOD,GPIO_Pin_11); 			//PD11 ����� 
			 GPIO_ResetBits(GPIOD,GPIO_Pin_9); 			  //PD9 ����� 
			 GPIO_ResetBits(GPIOD,GPIO_Pin_11); 			//PD11 ����� 

		
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
void HWT_Init(void)//�����ǳ�ʼ��
{
	uint8_t buff1[5] = {0XFF, 0XAA, 0X69, 0X88, 0XB5};//��������������ָ��
  uint8_t buff2[5] = {0XFF, 0XAA, 0X76, 0X00, 0X00};//Z�����
	//���������ݳ�ʼ��
	uart_write_buffer(USART3, buff1, 5);
	delay_ms(10);                     //��ʱ10ms
	uart_write_buffer(USART3, buff2, 5);
	delay_ms(10);                     //��ʱ10ms
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      �޷�
//  @param      x				���޷�������
//  @param      y				�޷���Χ(���ݻᱻ������-y��+y֮��)
//  @return     float			�޷�֮�������			
//  Sample usage:				float dat = limit(500,300);//���ݱ�������-300��+300֮��  ��˷��صĽ����300
//-------------------------------------------------------------------------------------------------------------------
float limit(float x, float y)
{
    if(x>y)             return y;
    else if(x<-y)       return -y;
    else                return x;
}

//--------------------------------------------------------------------------------------------λ��ʽPID
#define Integraldead_zone 300 // �������� �����Լ���������
void PositionPID_Calculate(Position_PID *pid,const float Target,const float Measure)
{

  if(pid == NULL)
    return;
	pid->Err = Target - Measure;
	
  pid->Output = pid->Kp * pid->Err + pid->Kd * (pid->Err - pid->Last_Err);
  
#ifdef HAVE_PID_INTEGRAL
    /* ���ַ��� */
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


//--------------------------------------------------------------------------------------------����ʽPID      40     0     +40      0
void IncrementalPID_Calculate(Incremental_PID *pid,const float Target,const float Measure)	//							40      10    +30    30-40 = -10  �����ӽ���
																																														//							40      41    -1      0
																																															//            40      45    -5      -5-��-1��=-4  ����ӽ�Ŀ��
{
  if(pid == NULL)									
    return;
  pid->Err = Target - Measure;	
	
  pid->p_out = pid->Kp * (pid->Err - pid->Last_Err);//�仯��
  pid->i_out = pid->Ki * pid->Err;//��� 
  pid->d_out = pid->Kd * (pid->Err - 2.0f*pid->Last_Err + pid->Previous_Err);
  
  pid->Output += pid->p_out + pid->i_out + pid->d_out; 
  
  pid->Output = limit(pid->Output, pid->OutputMax); // �޷�
  
  pid->Previous_Err = pid->Last_Err;
  pid->Last_Err = pid->Err;
  
}



/*************************************************************************
*  �������ƣ� mortor_TB6612Set(uint16_t motor_PWM, uint16_t motor_AIN1, uint16_t motor_AIN2, int32_t duty, uint16_t motor_MAX, uint16_t motor_Die)
*  ����˵���������ֵ�����ƺ���
*  ����˵����motor_PWM      PWM����
*  ����˵����motor_AIN1��motor_AIN2      ��������
*  ����˵����duty         ռ�ձ�
*  ����˵����motor_MAX    �޷�
*  ����˵����motor_Die    ����
*  �������أ���
*  �޸�ʱ�䣺2024��10��22��
*  ��    ע��TB6612
*************************************************************************/
void mortor_TB6612Set(uint16_t motor_PWM, uint16_t motor_AIN1, uint16_t motor_AIN2, int32_t duty, uint16_t motor_MAX, uint16_t motor_Die)
{

		uint16_t temp_duty = 0;
		if(duty > 0 )   //��ת
		{
				temp_duty = (uint16_t)duty + motor_Die;
				GPIO_SetBits(GPIOD,motor_AIN1); 			//����� 
				GPIO_ResetBits(GPIOD,motor_AIN2); 			//����� 

		}
		else if(duty < 0)  //��ת
		{
				temp_duty = (uint16_t)(-duty) + motor_Die;
				GPIO_ResetBits(GPIOD,motor_AIN1); 			//����� 
				GPIO_SetBits(GPIOD,motor_AIN2); 			//����� 

		}
		else if(duty == 0)  //ͣ
		{
				temp_duty = 0;
				GPIO_ResetBits(GPIOD,motor_AIN1); 			//����� 
				GPIO_ResetBits(GPIOD,motor_AIN2); 			//����� 

		}
		temp_duty = constrain_long(temp_duty, 0, motor_MAX);  //�޷�

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


/************************ ���������˲��� *****************************/
#define window_size 5            //�������ڳ���
float buffer_1[window_size] = {0}; //������������buf
float buffer_2[window_size] = {0}; //������������buf
float buffer_3[window_size] = {0}; //������������buf
float buffer_4[window_size] = {0}; //������������buf
/*********************** ���������˲����� ****************************/
float sliding_average_filter(float value, float buffer[], int *data_num)
{
  float output = 0;

  if (*data_num < window_size)
  {
    buffer[(*data_num)++] = value;
    output = value;
  }
  else
  {
    int i = 0;
    float sum = 0;

    memcpy(&buffer[0], &buffer[1], (window_size - 1) * 4);
    buffer[window_size - 1] = value;

    for (i = 0; i < window_size; i++)
      sum += buffer[i];

    output = sum / window_size;
  }

  return output;
}

void encoder_count_get(void)
{
	static int dn1 = 0, dn2 = 0, dn3 = 0, dn4 = 0;

	my_car.motor_1.encoder_count_r = Encoder_FR;
	my_car.motor_2.encoder_count_r = Encoder_FL;
	my_car.motor_3.encoder_count_r = Encoder_BL;
	my_car.motor_4.encoder_count_r = Encoder_BR;

	my_car.motor_1.encoder_count_all += my_car.motor_1.encoder_count_r;
	my_car.motor_2.encoder_count_all += my_car.motor_2.encoder_count_r;
	my_car.motor_3.encoder_count_all += my_car.motor_3.encoder_count_r;
	my_car.motor_4.encoder_count_all += my_car.motor_4.encoder_count_r;
	
	my_car.motor_1.encoder_count_f = sliding_average_filter((float)my_car.motor_1.encoder_count_r, buffer_1, &dn1);
	my_car.motor_2.encoder_count_f = sliding_average_filter((float)my_car.motor_2.encoder_count_r, buffer_2, &dn2);
	my_car.motor_3.encoder_count_f = sliding_average_filter((float)my_car.motor_3.encoder_count_r, buffer_3, &dn3);
	my_car.motor_4.encoder_count_f = sliding_average_filter((float)my_car.motor_4.encoder_count_r, buffer_4, &dn4);

}

void speed_translation(Motor *motor)//�����ٶȻ���
{
	motor->speed = ((motor->encoder_count_f/ENCODER_ACCURACY)*Tooth_Proportion)*Perimeter/ENCODER_TIME_S;
}



/**
  * @�������ܣ������˶�ѧ���������ӱ���ֵ->����������̼�����
  */
void RobotCalculate(void)//�����˶�ѧ���������ӱ���ֵ->����������̼�����
{
    // ����ÿ֡��motors[4]�������g_robot
//    float deltacounts[4]={0,0,0,0};
    float delta_x_o, delta_y_o;
    float delta_x, delta_y;
    float everycount;
    
    my_car.now_the = my_car.yaw * PI / 180.0f;    //��ǰ�Ƕ�ת���ɵ�ǰ����
    
    while((my_car.now_the >= 2.0f * PI) || (my_car.now_the < 0))
    {
        if(my_car.now_the >= 2.0f * PI)
            my_car.now_the = my_car.now_the - 2.0f * PI;
        if(my_car.now_the < 0)
            my_car.now_the = my_car.now_the + 2.0f * PI;
    }
    
    everycount = ((1.0f/ENCODER_ACCURACY)*Tooth_Proportion)*Perimeter;//ÿ�������߶���cm
    
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

void car_yaw_pid(void)//�ǶȻ�PID��λ��ʽPID���ƣ�
{
	
	float temp_err,temp_target_yaw;
	
	my_car.yaw = YAW;  //��ǰ�Ƕȸ�ֵ
	
	temp_err = my_car.target_yaw - my_car.yaw;
		//ȡ���·�������¼���Ƕ����
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

		my_car.motor_1.target_speed = ( v_y - v_x + w* (HALF_LENGTH +  HALF_WIDTH));//��ǰ���ӣ�motor_1+
		my_car.motor_2.target_speed = ( v_y + v_x + w* (HALF_LENGTH +  HALF_WIDTH));//��ǰ���ӣ�motor_2-
		my_car.motor_3.target_speed = ( v_y - v_x - w* (HALF_LENGTH +  HALF_WIDTH));//������ӣ�motor_3-
		my_car.motor_4.target_speed = ( v_y + v_x - w* (HALF_LENGTH +  HALF_WIDTH));//�Һ����ӣ�motor_4+

		
//		limit(my_car.motor_1.target_speed, 200.0f); // �޷�
//		limit(my_car.motor_2.target_speed, 200.0f); // �޷�
//		limit(my_car.motor_3.target_speed, 200.0f); // �޷�
//		limit(my_car.motor_4.target_speed, 200.0f); // �޷�
}

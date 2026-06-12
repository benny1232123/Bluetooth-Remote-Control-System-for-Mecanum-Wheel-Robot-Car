#ifndef __SERVO_H
#define __SERVO_H
#include "ALL_Head.h"

//���������ķ�����˶�ѧ��xy����ϵ��
#define MECANUM_CAR_length  	20.0f //���ֳ�����(cm) (ǰ��������־�)
#define MECANUM_CAR_width    	22.0f //���ֳ�����  ��16+16+6+32��*2+80
#define HALF_LENGTH  (MECANUM_CAR_length/2.0f)  //������һ��
#define HALF_WIDTH   (MECANUM_CAR_width/2.0f)   //������һ��

//�����������˶�ѧ����
#define WHEEL_DIAMETE      		7.5f //����ֱ�� (cm)
#define PI  									3.1415926535898f                  //Բ����

//#define ENCODER_TIME_S     		0.020f   //����������ʱ��  (s)
//#define ENCODER_ACCURACY 			1560.0f  //���������� (����) = PRM*���ٱ�*4

#define ENCODER_TIME_S     		0.01f   //������������ʱ��  (s)
#define ENCODER_ACCURACY 			53760.0f  //������������ (����) = PRM*���ٱ�*4

#define ENCODER_GEAR     			1.0f      //���ӱ������ĳ��ֳ���
#define WHEEL_GEAR       			1.0f     //���ӳ��ֵĳ��ֳ���
#define Tooth_Proportion  		ENCODER_GEAR/WHEEL_GEAR	//�����������복ģ�����ı�����С
#define Perimeter 						PI*WHEEL_DIAMETE							//������תһ�ܲ�����·��/cm

////////��ǰ��
//#define motorFL_PWM         GPIO_Pin_6
//#define motorFL_IN1       GPIO_Pin_14
//#define motorFL_IN2         GPIO_Pin_12
//#define motorFL_MAX    5999                  //������pwm�޷�
//#define motorFL_Die    0                  //������pwm����

////��ǰ��
//#define motorFR_PWM         GPIO_Pin_7
//#define motorFR_IN1        GPIO_Pin_8
//#define motorFR_IN2        GPIO_Pin_10
//#define motorFR_MAX    5999                  //������pwm�޷�
//#define motorFR_Die    0                  //������pwm����

////�Һ���
//#define motorBR_PWM         GPIO_Pin_8
//#define motorBR_IN1          GPIO_Pin_15
//#define motorBR_IN2          GPIO_Pin_13
//#define motorBR_MAX    5999                  //������pwm�޷�
//#define motorBR_Die    0                  //������pwm����

////�����
//#define motorBL_PWM          GPIO_Pin_9
//#define motorBL_IN1        GPIO_Pin_9
//#define motorBL_IN2        GPIO_Pin_11
//#define motorBL_MAX    5999                  //�ٶȻ����pwm�޷�
//#define motorBL_Die    0                  //������pwm����


//��ǰ��
#define motorFL_PWM          GPIO_Pin_9
#define motorFL_IN1         GPIO_Pin_9
#define motorFL_IN2         GPIO_Pin_11
#define motorFL_MAX    5999                  //������pwm�޷�
#define motorFL_Die    0                  //������pwm����

//��ǰ��
#define motorFR_PWM          GPIO_Pin_6
#define motorFR_IN1         GPIO_Pin_12
#define motorFR_IN2         GPIO_Pin_14
#define motorFR_MAX    5999                  //������pwm�޷�
#define motorFR_Die    0                  //������pwm����

//�Һ���
#define motorBR_PWM           GPIO_Pin_7
#define motorBR_IN1            GPIO_Pin_8
#define motorBR_IN2            GPIO_Pin_10
#define motorBR_MAX    5999                  //������pwm�޷�
#define motorBR_Die    0                  //������pwm����

//�����
#define motorBL_PWM           GPIO_Pin_8
#define motorBL_IN1         GPIO_Pin_13
#define motorBL_IN2          GPIO_Pin_15
#define motorBL_MAX    5999                  //�ٶȻ����pwm�޷�
#define motorBL_Die    0                  //������pwm����


extern int16_t  Encoder_FL;  //��ǰ�ֱ�����ֵ
extern int16_t  Encoder_FR;  //��ǰ�ֱ�����ֵ
extern int16_t  Encoder_BL;  //����ֱ�����ֵ
extern int16_t  Encoder_BR;  //�Һ��ֱ�����ֵ


#define HAVE_PID_INTEGRAL 
//--------------------------------------------------------------------------------------------λ��ʽPID
typedef struct{
  float Kp;
  float Ki;
  float Kd;
#ifdef HAVE_PID_INTEGRAL
  int index;            // ���ַ���ϵ��
  float Integral;       // ������
  float I_outputMax;    // �����޷�
#endif
	float Err;
  float Last_Err;       // �ϴ����
  float Output;         // PID���
  float OutputMax;      // λ��ʽPID����޷�
}Position_PID;
//--------------------------------------------------------------------------------------------����ʽPID
typedef struct{
  float Kp;
  float Ki;
  float Kd;
  float p_out;
  float i_out;
  float d_out;
  float Err;
  float Last_Err;       // �ϴ����
  float Previous_Err;   // ���ϴ����
  float Output;
  float OutputMax;      // ����ʽʽPID����޷�
}Incremental_PID;

//--------------------------------------------------------------------------------------------�������
typedef struct {
	
	//�����ʼ������
	volatile float target_speed;		//Ŀ���ٶȣ�cm/s��
	volatile float speed;						//ʵʱ�ٶȣ�cm/s��
	int32_t PWM;											//PWMֵ
	
	Incremental_PID s_pid;						//�ٶȻ�pid
	

	int32_t encoder_count_all;				//�������ۼ�ֵ
	int32_t encoder_count_r;					//����������ֵ(��������)
	float	 encoder_count_f;					//��������λʱ�����ֵ���˲���
}Motor;//--------------------------------------------------------------------------------------------С������
typedef struct{
	
	Motor motor_1,motor_2,motor_3,motor_4;//��ǰ����ǰ������Һ�
	
	volatile float target_yaw;		//Ŀ��Ƕ�
	volatile float v_x;						//Ŀ������ٶȣ�����Ϊ����
	volatile float v_y;						//Ŀ��ǰ���ٶȣ���ǰΪ����
	volatile float w;							//Ŀ����ٶ�
	double target_point[2];				//Ŀ�������(x,y)
	volatile double target_x;					//��ǰx����
	volatile double target_y;					//��ǰy����
	
	volatile float now_v_x;						//��ǰ�����ٶȣ�����Ϊ����
	volatile float now_v_y;						//��ǰǰ���ٶȣ���ǰΪ����
	volatile float yaw;							//��ǰ�Ƕȣ���λ�ȣ�
	volatile double now_point[2];		//��ǰ�����(x,y)
	volatile double now_x;					//��ǰx����
	volatile double now_y;					//��ǰy����
	volatile float now_the;				  //��ǰ�ĽǶȣ���λ���ȣ�
	
	Position_PID yaw_pid;						//ת��pid
	Position_PID position_pid_x;			//λ�û�pid_x��
	Position_PID position_pid_y;			//λ�û�pid_y��

	Position_PID openmv_pid_x;			//λ�û�pid_x��
	Position_PID openmv_pid_y;			//λ�û�pid_y��
	
	bool stop_flag;
}Car;


extern Car my_car;  //����С���ṹ��
extern float YAW ;//�����
extern float W_Z ;//���ٶ�




void Motor_parameter_Init(void);
void HWT_Init(void);//�����ǳ�ʼ��
uint16_t constrain_long( uint16_t amt,  uint16_t low,  uint16_t high);
void Motor_Init(void);
float limit(float x, float y);// �޷�
void PositionPID_Calculate(Position_PID *pid,const float Target,const float Measure);//λ��ʽPID
void IncrementalPID_Calculate(Incremental_PID *pid,const float Target,const float Measure);//����ʽPID
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
void mortor_TB6612Set(uint16_t motor_PWM, uint16_t motor_AIN1, uint16_t motor_AIN2, int32_t duty, uint16_t motor_MAX, uint16_t motor_Die);
float sliding_average_filter(float value,float buffer[],int *data_num);
void encoder_count_get(void);//��ȡ������ֵ
void speed_translation(Motor *motor);//�����ٶȻ���
void RobotCalculate(void);//�����˶�ѧ���������ӱ���ֵ->����������̼�����

void car_yaw_pid(void);//�ǶȻ�PID��λ��ʽPID���ƣ�
void mecanum(double v_y,double  v_x  ,double  w);


#endif // !__SERVO_H

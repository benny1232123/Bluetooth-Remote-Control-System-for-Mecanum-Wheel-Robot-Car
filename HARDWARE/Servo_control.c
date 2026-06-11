#include "ALL_Head.h"
uint16_t Servo_clip_duty = Servo_clip_loosen;
uint16_t Servo_turntable_duty = Servo_turntable_behind;
uint16_t Servo_turntable_target = Servo_turntable_behind;
uint16_t Servo_platform_duty = Servo_platform_two;
//uint16_t Servo_clip_duty = 180;
//uint16_t Servo_turntable_duty = 0;
//uint16_t Servo_platform_duty = 135;

/*************************************************************************
 *  �������ƣ�uint16_t constrain_uint16_t(uint16_t amt, uint16_t low, uint16_t high)
 *  ����˵�����޷�����
 *  ����˵����
  * @param    amt   �� ����
  * @param    low   �� ���ֵ
  * @param    high  �� ���ֵ
 *  �������أ���
 *  �޸�ʱ�䣺2020��4��1��
 *  ��    ע��
 *************************************************************************/
uint16_t constrain_uint16_t(uint16_t amt, uint16_t low, uint16_t high)
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}
void Servo_Init(void)
{
  /*  
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//ʹ��GPIO����ģ��ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);//ʹ�ܶ�ʱ��1ʱ��
  //���ø�����Ϊ�����������,���TIM8 CH��PWM���岨��	GPIO
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//�����������
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;//TIM���ϼ���ģʽ
    TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;//����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
    TIM_TimeBaseInitStructure.TIM_Period = 2000 - 1;//��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;//����ʱ�ӷָ�:TDTS = Tck_tim
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);//����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//ѡ��ʱ��ģʽ:TIM������ȵ���ģʽ1
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//�������:TIM����Ƚϼ��Ը�
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//�Ƚ����ʹ��
    TIM_OCInitStructure.TIM_Pulse = 1;
//����Tָ���Ĳ�����ʼ������TIM3 
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC4Init(TIM1, &TIM_OCInitStructure);

    TIM_Cmd(TIM1, ENABLE); //ʹ��TIM1
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
		*/
		
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);	
	
		GPIO_InitTypeDef GPIO_InitStruct;
		GPIO_InitStruct.GPIO_Mode= GPIO_Mode_AF_PP ;
		GPIO_InitStruct.GPIO_Pin=GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11;
		GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStruct); 
	
	  TIM_InternalClockConfig(TIM1);
	
		TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
		TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV1 ;
		TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up ;
    TIM_TimeBaseInitStruct.TIM_Prescaler = 72 - 1;//����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
    TIM_TimeBaseInitStruct.TIM_Period = 20000 - 1;//��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
		TIM_TimeBaseInitStruct.TIM_RepetitionCounter=0;	
		TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStruct);

		TIM_OCInitTypeDef TIM_OCInitStruct;
		TIM_OCStructInit(&TIM_OCInitStruct);
		TIM_OCInitStruct.TIM_OCMode=TIM_OCMode_PWM1;
		TIM_OCInitStruct.TIM_OCPolarity=TIM_OCPolarity_High  ;
		TIM_OCInitStruct.TIM_OutputState=TIM_OutputState_Enable ;
		TIM_OCInitStruct.TIM_Pulse=1;
		TIM_OC1Init(TIM1, &TIM_OCInitStruct);
		
		TIM_OCStructInit(&TIM_OCInitStruct);
		TIM_OCInitStruct.TIM_OCMode=TIM_OCMode_PWM1;
		TIM_OCInitStruct.TIM_OCPolarity=TIM_OCPolarity_High  ;
		TIM_OCInitStruct.TIM_OutputState=TIM_OutputState_Enable ;
		TIM_OCInitStruct.TIM_Pulse=1;
		TIM_OC2Init(TIM1, &TIM_OCInitStruct);
		
//		TIM_OCStructInit(&TIM_OCInitStruct);
//		TIM_OCInitStruct.TIM_OCMode=TIM_OCMode_PWM1;
//		TIM_OCInitStruct.TIM_OCPolarity=TIM_OCPolarity_High  ;
//		TIM_OCInitStruct.TIM_OutputState=TIM_OutputState_Enable ;
//		TIM_OCInitStruct.TIM_Pulse=1;
//    TIM_OC3Init(TIM1, &TIM_OCInitStruct);
		
		TIM_OCStructInit(&TIM_OCInitStruct);
		TIM_OCInitStruct.TIM_OCMode=TIM_OCMode_PWM1;
		TIM_OCInitStruct.TIM_OCPolarity=TIM_OCPolarity_High  ;
		TIM_OCInitStruct.TIM_OutputState=TIM_OutputState_Enable ;
		TIM_OCInitStruct.TIM_Pulse=1;
    TIM_OC4Init(TIM1, &TIM_OCInitStruct);
		
		TIM_Cmd(TIM1,ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
		
		// 启动后立即设置初始PWM值，避免舵机跳到0位
		Servo_clip_SetAngle(Servo_clip_duty);
		Servo_platform_SetAngle(Servo_platform_duty);
		Servo_turntable_SetAngle(Servo_turntable_duty);
		
}

void Servo_clip_SetAngle(uint16_t Angle)//צ��
{
    TIM_SetCompare1(TIM1,(Angle * 2000 / 270)  + 500);  //PA8
}

void Servo_platform_SetAngle(uint16_t Angle)//װ���ϵĻ���
{
    TIM_SetCompare2(TIM1,(Angle * 2000 / 270)  + 500);//PA9
}
//void Servo_turntable_SetAngle(uint16_t Angle)//��̨
//{
//    TIM_SetCompare3(TIM1,(Angle * 2000 / 270)  + 500);//PA10
//}
void Servo_turntable_SetAngle(uint16_t Angle)
{
    TIM_SetCompare4(TIM1,(Angle * 2000 / 270)  + 500);
}



#include "ALL_Head.h"
uint16_t Servo_clip_duty = Servo_clip_loosen;
uint16_t Servo_turntable_duty = Servo_turntable_behind;
uint16_t Servo_platform_duty = Servo_platform_two;
//uint16_t Servo_clip_duty = 180;
//uint16_t Servo_turntable_duty = 0;
//uint16_t Servo_platform_duty = 135;

/*************************************************************************
 *  函数名称：uint16_t constrain_uint16_t(uint16_t amt, uint16_t low, uint16_t high)
 *  功能说明：限幅函数
 *  参数说明：
  * @param    amt   ： 参数
  * @param    low   ： 最低值
  * @param    high  ： 最高值
 *  函数返回：无
 *  修改时间：2020年4月1日
 *  备    注：
 *************************************************************************/
uint16_t constrain_uint16_t(uint16_t amt, uint16_t low, uint16_t high)
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}
void Servo_Init(void)
{
  /*  
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//使能GPIO外设模块时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);//使能定时器1时钟
  //设置该引脚为复用输出功能,输出TIM8 CH的PWM脉冲波形	GPIO
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//复用推挽输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;//TIM向上计数模式
    TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;//设置用来作为TIMx时钟频率除数的预分频值
    TIM_TimeBaseInitStructure.TIM_Period = 2000 - 1;//设置在下一个更新事件装入活动的自动重装载寄存器周期的值
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;//设置时钟分割:TDTS = Tck_tim
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);//根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//选择定时器模式:TIM脉冲宽度调制模式1
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//输出极性:TIM输出比较极性高
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//比较输出使能
    TIM_OCInitStructure.TIM_Pulse = 1;
//根据T指定的参数初始化外设TIM3 
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC4Init(TIM1, &TIM_OCInitStructure);

    TIM_Cmd(TIM1, ENABLE); //使能TIM1
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
    TIM_TimeBaseInitStruct.TIM_Prescaler = 72 - 1;//设置用来作为TIMx时钟频率除数的预分频值
    TIM_TimeBaseInitStruct.TIM_Period = 20000 - 1;//设置在下一个更新事件装入活动的自动重装载寄存器周期的值
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
		
}

void Servo_clip_SetAngle(uint16_t Angle)//爪子
{
    TIM_SetCompare1(TIM1,(Angle * 2000 / 270)  + 500);  //PA8
}

void Servo_platform_SetAngle(uint16_t Angle)//装物料的换盘
{
    TIM_SetCompare2(TIM1,(Angle * 2000 / 270)  + 500);//PA9
}
//void Servo_turntable_SetAngle(uint16_t Angle)//云台
//{
//    TIM_SetCompare3(TIM1,(Angle * 2000 / 270)  + 500);//PA10
//}
void Servo_turntable_SetAngle(uint16_t Angle)
{
    TIM_SetCompare4(TIM1,(Angle * 2000 / 270)  + 500);
}



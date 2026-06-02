#include "ALL_Head.h"
uint32_t Timer7_count = 0;

void Timer6_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);//时钟使能
	
	TIM_InternalClockConfig(TIM6);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;//TIM向上计数模式
	TIM_TimeBaseInitStructure.TIM_Period = 1000 - 1;//设置在下一个更新事件装入活动的自动重装载寄存器周期的值200Hz，0.005s
	TIM_TimeBaseInitStructure.TIM_Prescaler = 720 - 1;  //设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseInitStructure); //根据指定的参数初始化TIMx的时间基数单位
	
	TIM_ClearFlag(TIM6, TIM_FLAG_Update);
	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
	 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;//TIM6中断
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//IRQ通道被使能
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;//先占优先级2级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;//从优先级1级
	NVIC_Init(&NVIC_InitStructure);//初始化NVIC寄存器
	TIM_Cmd(TIM6, ENABLE);//使能TIMx	
}

void Timer7_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);//时钟使能
	
	TIM_InternalClockConfig(TIM7);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;//TIM向上计数模式
	TIM_TimeBaseInitStructure.TIM_Period = 10000 - 1;//设置在下一个更新事件装入活动的自动重装载寄存器周期的值10Hz,，0.1s
	TIM_TimeBaseInitStructure.TIM_Prescaler = 720 - 1;  //设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseInitStructure); //根据指定的参数初始化TIMx的时间基数单位
	
	TIM_ClearFlag(TIM7, TIM_FLAG_Update);
	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
	 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;//TIM6中断
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//IRQ通道被使能
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;//先占优先级2级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;//从优先级1级
	NVIC_Init(&NVIC_InitStructure);//初始化NVIC寄存器
	TIM_Cmd(TIM7, ENABLE);//使能TIMx	
}




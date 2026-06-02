#include "ALL_Head.h"


#define KEY1   P22_0 //UP
#define KEY2   P22_1 //DOWN
#define KEY3   P22_2 //LEFT
#define KEY4   P22_3 //Right

#define SWITCH1 P33_12
#define SWITCH2 P33_13

//开关状态变量
unsigned char key1_status = 1;
unsigned char key2_status = 1;
unsigned char key3_status = 1;
unsigned char key4_status = 1;

unsigned char switch1_status = 1;
unsigned char switch2_status = 1;

//上一次开关状态变量
unsigned char key1_last_status;
unsigned char key2_last_status;
unsigned char key3_last_status;
unsigned char key4_last_status;

unsigned char switch1_last_status;
unsigned char switch2_last_status;

//开关标志位
unsigned char key1_flag;
unsigned char key2_flag;
unsigned char key3_flag;
unsigned char key4_flag;

unsigned char switch1_flag;
unsigned char switch2_flag;



void Button_Control_init(void)
{
    // 按键初始化
 	GPIO_InitTypeDef GPIO_InitStructure;
 
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE);//使能PORTE时钟

	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2;//KEY0-KEY2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //设置成上拉输入
 	GPIO_Init(GPIOE, &GPIO_InitStructure);//初始化PE0 1 2

	RCC_APB2PeriphClockCmd ( RCC_APB2Periph_GPIOA, ENABLE); 	//初始化KEY时钟	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);		// 开启IO口复用时钟
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);	//	禁用JTAG 只使用SWD	，不然PA15、PB3、PB4无法正常使用
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; //速度选择
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_15;	 
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
}




void Button_Control_scan(void)
{
    //保存按键状态
    key1_last_status = key1_status;
    key2_last_status = key2_status;
    key3_last_status = key3_status;
    key4_last_status = key4_status;

    //读取当前按键状态
    key1_status = GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_0);
    key2_status = GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_1);
    key3_status = GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_2);
    key4_status = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_15);


    //检测到按键按下之后  并放开置位标志位
    if (key1_status && !key1_last_status)    key1_flag = 1;
    if (key2_status && !key2_last_status)    key2_flag = 1;
    if (key3_status && !key3_last_status)    key3_flag = 1;
    if (key4_status && !key4_last_status)    key4_flag = 1;

}


#include "ALL_Head.h"


#define KEY1   P22_0 //UP
#define KEY2   P22_1 //DOWN
#define KEY3   P22_2 //LEFT
#define KEY4   P22_3 //Right

#define SWITCH1 P33_12
#define SWITCH2 P33_13

//����״̬����
unsigned char key1_status = 1;
unsigned char key2_status = 1;
unsigned char key3_status = 1;
unsigned char key4_status = 1;

unsigned char switch1_status = 1;
unsigned char switch2_status = 1;

//��һ�ο���״̬����
unsigned char key1_last_status;
unsigned char key2_last_status;
unsigned char key3_last_status;
unsigned char key4_last_status;

unsigned char switch1_last_status;
unsigned char switch2_last_status;

//���ر�־λ
unsigned char key1_flag;
unsigned char key2_flag;
unsigned char key3_flag;
unsigned char key4_flag;

unsigned char switch1_flag;
unsigned char switch2_flag;



void Button_Control_init(void)
{
    // ������ʼ��
 	GPIO_InitTypeDef GPIO_InitStructure;
 
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE);//ʹ��PORTEʱ��

	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2;//KEY0-KEY2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //���ó���������
 	GPIO_Init(GPIOE, &GPIO_InitStructure);//��ʼ��PE0 1 2

	RCC_APB2PeriphClockCmd ( RCC_APB2Periph_GPIOA, ENABLE); 	//��ʼ��KEYʱ��	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);		// ����IO�ڸ���ʱ��
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);	//	����JTAG ֻʹ��SWD	����ȻPA15��PB3��PB4�޷�����ʹ��
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; //�ٶ�ѡ��
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_15;	 
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
}




void Button_Control_scan(void)
{
    static uint8_t debounce_cnt1 = 0, debounce_cnt2 = 0, debounce_cnt3 = 0, debounce_cnt4 = 0;

    uint8_t cur1 = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_0);
    uint8_t cur2 = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_1);
    uint8_t cur3 = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_2);
    uint8_t cur4 = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15);

    if (cur1 == key1_status) { if (debounce_cnt1 < 3) debounce_cnt1++; }
    else                     { debounce_cnt1 = 0; }
    if (cur2 == key2_status) { if (debounce_cnt2 < 3) debounce_cnt2++; }
    else                     { debounce_cnt2 = 0; }
    if (cur3 == key3_status) { if (debounce_cnt3 < 3) debounce_cnt3++; }
    else                     { debounce_cnt3 = 0; }
    if (cur4 == key4_status) { if (debounce_cnt4 < 3) debounce_cnt4++; }
    else                     { debounce_cnt4 = 0; }

    if (debounce_cnt1 >= 3) { key1_last_status = key1_status; key1_status = cur1; }
    if (debounce_cnt2 >= 3) { key2_last_status = key2_status; key2_status = cur2; }
    if (debounce_cnt3 >= 3) { key3_last_status = key3_status; key3_status = cur3; }
    if (debounce_cnt4 >= 3) { key4_last_status = key4_status; key4_status = cur4; }

    if (key1_status && !key1_last_status)    key1_flag = 1;
    if (key2_status && !key2_last_status)    key2_flag = 1;
    if (key3_status && !key3_last_status)    key3_flag = 1;
    if (key4_status && !key4_last_status)    key4_flag = 1;
}


#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 
		   
extern unsigned char uart1_Res;
extern unsigned char uart2_Res;
extern unsigned char uart3_Res;
extern unsigned char uart4_Res;
extern unsigned char uart5_Res;



extern unsigned char uart2_str[12];
extern unsigned char uart2_str_count;
extern unsigned char uart2_str_T_count;
extern unsigned char uart2_str_flag;

//X=288,Y=201为中心点位置
#define opemv_middle_X		320
#define opemv_middle_Y		240

extern unsigned char QR_codeRxCnt;;
extern unsigned char QR_codeBuffer[20];  //二维码模块中接受的字符串保存数组
extern uint32_t  QR_codeRx_Date;

extern int opemv_X; 							
extern int opemv_Y;               //  openmv返回的色块点的W和Y值
extern unsigned char OpenmvBuffer[20];  //open mv接受的字符串保存数组
extern unsigned char OpenmvRxCnt;   //记录接受的位数
extern unsigned char Openmv_flag;   //接受到的数据标志位
extern unsigned char Openmv_track_start;   //开启openmv定点循迹标志位

void uart1_init(u32 bound);
void uart2_init(u32 bound);
void uart3_init(u32 bound);
void uart4_init(u32 bound);
void uart5_init(u32 bound);
void uart_write_buffer (USART_TypeDef* USARTx, uint8_t *buff, uint16_t len);
void uart_write_string (USART_TypeDef* USARTx, uint8_t *str);

#endif



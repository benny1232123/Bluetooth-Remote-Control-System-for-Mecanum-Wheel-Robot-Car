#ifndef __Button_H
#define __Button_H	 
  

//开关标志位
extern unsigned char key1_flag;
extern unsigned char key2_flag;
extern unsigned char key3_flag;
extern unsigned char key4_flag;



void Button_Control_init(void);

void Button_Control_scan(void);

		 				    
#endif

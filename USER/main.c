#include "ALL_Head.h"
int flag_do_task = -100; //-100=空闲, -3~5=自动任务进行中

#define BT_USE_UART5 1
#define BT_HOLD_MODE 1
#define BT_LIFT_UP_CLK 3000
#define BT_LIFT_DOWN_CLK 3000
#define BT_LIFT_VEL 150
#define BT_TURNTABLE_STEP 10
#define BT_PLATFORM_STEP 5
#define BT_CLIP_STEP 2
#define BT_TURNTABLE_STEP_180 180
#define BT_CMD_TURNTABLE_180_CW 'G'
#define BT_CMD_TURNTABLE_180_CCW 'I'
#define BT_PLATFORM_STEP_125 125
#define BT_CMD_PLATFORM_125_CW 'T'
#define BT_CMD_PLATFORM_125_CCW 'Y'
#define BT_CMD_TASK_START 'M'

static const uint16_t bt_clip_min = Servo_clip_loosen;
static const uint16_t bt_clip_max = Servo_clip_hold;
//static const uint16_t bt_turntable_min = Servo_turntable_front;
//static const uint16_t bt_turntable_max = Servo_turntable_behind;
//static const uint16_t bt_platform_min = Servo_platform_three;
//static const uint16_t bt_platform_max = Servo_platform_two;

static volatile uint8_t bt_cmd = 0;
static volatile uint8_t bt_speed_level = 2;
static volatile uint8_t bt_drive_mode = 1;
static volatile float bt_vx = 0.0f;
static volatile float bt_vy = 0.0f;
static volatile float bt_w = 0.0f;
static volatile uint16_t bt_clip_target = Servo_clip_loosen;
static volatile unsigned long bt_last_rx_tick = 0;
static volatile unsigned long bt_last_tx_tick = 0;
//static volatile int8_t bt_turntable_hold = 0; // -1 left, +1 right
//static volatile int8_t bt_platform_hold = 0;  // -1 right, +1 left
//static volatile uint8_t bt_hold_tick = 0;
static volatile uint8_t openmv_grab_pending = 0;
static volatile uint8_t openmv_camera_test = 0;
static volatile uint8_t openmv_drive_timer = 0;
static volatile uint8_t camera_auto_align = 0;

#define BT_TIMEOUT_TICKS 1 // 0.1s @ 10Hz
#define BT_HEARTBEAT_TICKS 2 // 0.2s @ 10Hz

static void BT_ApplyCommand(uint8_t raw_cmd)
{
	if (raw_cmd == 0)
		return;

	//按下保持模式：约定小写字符表示按下(start)，对应的大写字符表示松开(stop)
	uint8_t cmd = raw_cmd;
	uint8_t is_press = 1; // default treat as press
	if (cmd >= 'a' && cmd <= 'z')
	{
		// lower-case -> press; map to upper for switch convenience
		cmd = (uint8_t)(cmd - 'a' + 'A');
		is_press = 1;
	}
	else if (cmd >= 'A' && cmd <= 'Z')
	{
		// upper-case -> release in hold mode, press in legacy mode
		#if BT_HOLD_MODE
		is_press = 0;
		#else
		is_press = 1;
		#endif
	}

	float speed = 20.0f + (float)bt_speed_level * 10.0f; // 20..70
	float w = 0.4f + 0.1f * (float)bt_speed_level;       // 0.5..0.9
	uint16_t turntable_min = Servo_turntable_front;
	uint16_t turntable_max = Servo_turntable_behind;
	uint16_t turntable_mid = (uint16_t)((turntable_min + turntable_max) / 2);
	uint16_t platform_min = Servo_platform_three;
	uint16_t platform_max = Servo_platform_two;
	uint16_t platform_mid = Servo_platform_one;

	switch (cmd)
	{
		case '1': if (is_press) bt_speed_level = 0; break;
		case '2': if (is_press) bt_speed_level = 1; break;
		case '3': if (is_press) bt_speed_level = 2; break;
		case '4': if (is_press) bt_speed_level = 3; break;
		case '5': if (is_press) bt_speed_level = 4; break;
		case '+': // increase speed level (0..4)
			if (is_press)
			{
				if (bt_speed_level < 4) bt_speed_level++;
			}
			break;
		case '-': // decrease speed level
			if (is_press)
			{
				if (bt_speed_level > 0) bt_speed_level--;
			}
			break;

		case 'X': // explicit stop
		case 'S':
			bt_vx = 0.0f;
			bt_vy = 0.0f;
			bt_w = 0.0f;
			my_car.stop_flag = 1;
			break;

		case 'F': // forward
			if (is_press)
			{
				bt_vx = 0.0f; bt_vy = speed; bt_w = 0.0f; my_car.stop_flag = 0;
			}
			else
			{
				bt_vx = 0.0f; bt_vy = 0.0f; bt_w = 0.0f; my_car.stop_flag = 1;
			}
			break;
		case 'B': // backward
			if (is_press)
			{
				bt_vx = 0.0f; bt_vy = -speed; bt_w = 0.0f; my_car.stop_flag = 0;
			}
			else
			{
				bt_vx = 0.0f; bt_vy = 0.0f; bt_w = 0.0f; my_car.stop_flag = 1;
			}
			break;
		case 'L': // left
			if (is_press)
			{
				bt_vx = -speed; bt_vy = 0.0f; bt_w = 0.0f; my_car.stop_flag = 0;
			}
			else
			{
				bt_vx = 0.0f; bt_vy = 0.0f; bt_w = 0.0f; my_car.stop_flag = 1;
			}
			break;
		case 'R': // right
			if (is_press)
			{
				bt_vx = speed; bt_vy = 0.0f; bt_w = 0.0f; my_car.stop_flag = 0;
			}
			else
			{
				bt_vx = 0.0f; bt_vy = 0.0f; bt_w = 0.0f; my_car.stop_flag = 1;
			}
			break;
		case 'A': // rotate left
			if (is_press)
			{
				bt_vx = 0.0f; bt_vy = 0.0f; bt_w = -w; my_car.stop_flag = 0;
			}
			else
			{
				bt_vx = 0.0f; bt_vy = 0.0f; bt_w = 0.0f; my_car.stop_flag = 1;
			}
			break;
		case 'D': // rotate right
			if (is_press)
			{
				bt_vx = 0.0f; bt_vy = 0.0f; bt_w = w; my_car.stop_flag = 0;
			}
			else
			{
				bt_vx = 0.0f; bt_vy = 0.0f; bt_w = 0.0f; my_car.stop_flag = 1;
			}
			break;

		case 'U': // lift up
			if (is_press)
			{
				Emm_V5_Pos_Control(USART2, 1, 8, BT_LIFT_VEL, 0, BT_LIFT_UP_CLK, 0, 0);
			}
			else
			{
				Emm_V5_Stop_Now(USART2, 1, 0);
			}
			break;
		case 'N': // lift down
			if (is_press)
			{
				Emm_V5_Pos_Control(USART2, 1, 0, BT_LIFT_VEL, 0, BT_LIFT_DOWN_CLK, 0, 0);
			}
			else
			{
				Emm_V5_Stop_Now(USART2, 1, 0);
			}
			break;
		case 'Z': // lift stop
			Emm_V5_Stop_Now(USART2, 1, 0);
			break;

		case 'O': // claw open (instant)
			if (is_press) { Servo_clip_duty = bt_clip_min; bt_clip_target = bt_clip_min; }
			break;
		case 'P': // claw close (instant)
			if (is_press) { Servo_clip_duty = bt_clip_max; bt_clip_target = bt_clip_max; }
			break;

		case 'J': // turntable left
			if (is_press)
			{
				Servo_turntable_duty = constrain_uint16_t((uint16_t)(Servo_turntable_duty + BT_TURNTABLE_STEP), turntable_min, turntable_max);
			}
			break;
		case 'K': // turntable right
			if (is_press)
			{
				Servo_turntable_duty = constrain_uint16_t((uint16_t)(Servo_turntable_duty - BT_TURNTABLE_STEP), turntable_min, turntable_max);
			}
			break;
		case 'C': // turntable center
			if (is_press) Servo_turntable_duty = turntable_mid;
			break;
		case BT_CMD_TURNTABLE_180_CW: // turntable +180
			if (is_press)
			{
				Servo_turntable_duty = constrain_uint16_t((uint16_t)(Servo_turntable_duty + BT_TURNTABLE_STEP_180), turntable_min, turntable_max);
			}
			break;
		case BT_CMD_TURNTABLE_180_CCW: // turntable -180
			if (is_press)
			{
				Servo_turntable_duty = constrain_uint16_t((uint16_t)(Servo_turntable_duty - BT_TURNTABLE_STEP_180), turntable_min, turntable_max);
			}
			break;
		case 'Q': // platform left
			if (is_press)
			{
				Servo_platform_duty = constrain_uint16_t((uint16_t)(Servo_platform_duty + BT_PLATFORM_STEP), platform_min, platform_max);
			}
			break;
		case 'E': // platform right
			if (is_press)
			{
				Servo_platform_duty = constrain_uint16_t((uint16_t)(Servo_platform_duty - BT_PLATFORM_STEP), platform_min, platform_max);
			}
			break;
		case 'V': // platform center
			if (is_press) Servo_platform_duty = platform_mid;
			break;
		case BT_CMD_PLATFORM_125_CW: // platform +125
			if (is_press)
			{
				Servo_platform_duty = constrain_uint16_t((uint16_t)(Servo_platform_duty + BT_PLATFORM_STEP_125), platform_min, platform_max);
			}
			break;
		case BT_CMD_PLATFORM_125_CCW: // platform -125
			if (is_press)
			{
				Servo_platform_duty = constrain_uint16_t((uint16_t)(Servo_platform_duty - BT_PLATFORM_STEP_125), platform_min, platform_max);
			}
			break;
		case BT_CMD_TASK_START: // start auto task
			if (is_press)
			{
				flag_do_task = -3;
				bt_vx = 0.0f; bt_vy = 0.0f; bt_w = 0.0f; my_car.stop_flag = 1;
			}
			break;
		case 'W': // toggle camera test mode
			if (is_press) { openmv_camera_test = !openmv_camera_test; bt_drive_mode = 1; }
			break;
		case 'H': // toggle camera auto-tracking (car moves to center target)
			if (is_press)
			{
				camera_auto_align = !camera_auto_align;
				if (camera_auto_align)
				{
					Openmv_track_start = 1;
					bt_drive_mode = 0;
					bt_vx = 0.0f; bt_vy = 0.0f; bt_w = 0.0f; my_car.stop_flag = 1;
				}
				else
				{
					Openmv_track_start = 0;
					bt_drive_mode = 1;
				}
			}
			break;
		default:
			break;
	}
	bt_cmd = 0;
}

int main(void)
{	
	SystemInit();//初始化RCC 设置系统主频为72MHZ
	delay_init();	     //延时初始化
	LED_Init();
	Encoder_Init();
	#if !BT_USE_UART5
	uart1_init(9600); // HC-05/06 default, adjust if needed
	#endif
	uart2_init(115200);
	uart3_init(9600);//TX:GPIOB.10;  RX:GPIOB.11初始化,陀螺仪
	uart4_init(115200);//TX:GPIOC.10;   RX:GPIOC.11初始化 openmv
	uart5_init(9600); // UART5: PC12(TX), PD2(RX)
	Motor_Init();
	Servo_Init();
	
	LCD_Init();	   //液晶屏初始化
	Button_Control_init();
	HWT_Init();//陀螺仪初始化


	Motor_parameter_Init();//电机参数初始化
	
	Emm_V5_En_Control(USART2, 1, 1, 0);uart2_str_T_count = 4;
	delay_ms(500);                     //延时ms

//设置回零参数
//  Emm_V5_Origin_Modify_Params(USART2, 1, true, 2, 1, 100, 3500, 300, 1000, 60, false);
//	delay_ms(1000);                     //延时ms

//	Emm_V5_Origin_Trigger_Return(USART2, 1, 2, false);uart2_str_T_count = 10;
//	delay_ms(500);                     //延时ms

  Timer6_Init();
	delay_ms(7);                     //延时ms
	Timer7_Init();
	delay_ms(1000);                     //延时ms

	while(1)
	{
		char txt[32];
		unsigned int k = 4;
    unsigned int d = 16;
		

		sprintf(txt, "V_FL:%d ,%f  ",Encoder_FL, my_car.motor_2.speed);                    //将变量填充到字符串的对应位置，并将字符串存放到txt[]中
		Show_Str(2,k,BLUE,YELLOW,(u8 *) txt,16,0);
		k+=d;
		sprintf(txt, "TFL_V:%f  ",my_car.motor_2.target_speed);                    //将变量填充到字符串的对应位置，并将字符串存放到txt[]中
		Show_Str(2,k,BLUE,YELLOW,(u8 *) txt,16,0);
		k+=d;
		sprintf(txt, "V_FR:%d ,%f  ",Encoder_FR, my_car.motor_1.speed);                    //将变量填充到字符串的对应位置，并将字符串存放到txt[]中
		Show_Str(2,k,BLUE,YELLOW,(u8 *) txt,16,0);
		k+=d;
		sprintf(txt, "TFR_V:%f  ",my_car.motor_1.target_speed);                    //将变量填充到字符串的对应位置，并将字符串存放到txt[]中
		Show_Str(2,k,BLUE,YELLOW,(u8 *) txt,16,0);
		k+=d;
		sprintf(txt, "V_BR:%d ,%f  ",Encoder_BR, my_car.motor_4.speed);                    //将变量填充到字符串的对应位置，并将字符串存放到txt[]中
		Show_Str(2,k,BLUE,YELLOW,(u8 *) txt,16,0);
		k+=d;
		sprintf(txt, "TBR_V:%f  ",my_car.motor_4.target_speed);                    //将变量填充到字符串的对应位置，并将字符串存放到txt[]中
		Show_Str(2,k,BLUE,YELLOW,(u8 *) txt,16,0);
		k+=d;
		sprintf(txt, "V_BL:%d ,%f  ",Encoder_BL, my_car.motor_3.speed);                    //将变量填充到字符串的对应位置，并将字符串存放到txt[]中
		Show_Str(2,k,BLUE,YELLOW,(u8 *) txt,16,0);
		k+=d;
		sprintf(txt, "TBL_V:%f  ",my_car.motor_3.target_speed);                    //将变量填充到字符串的对应位置，并将字符串存放到txt[]中
		Show_Str(2,k,BLUE,YELLOW,(u8 *) txt,16,0);
		k+=d;
		sprintf(txt, "YAW:%f   ",YAW);                    //将变量填充到字符串的对应位置，并将字符串存放到txt[]中
		Show_Str(2,k,BLUE,YELLOW,(u8 *) txt,16,0);
		sprintf(txt, "T:%f   ",my_car.target_yaw);                    //将变量填充到字符串的对应位置，并将字符串存放到txt[]中
		Show_Str(120,k,BLUE,YELLOW,(u8 *) txt,16,0);
		k+=d;
		sprintf(txt, "v_x:%f   ",my_car.now_v_x);                    //将变量填充到字符串的对应位置，并将字符串存放到txt[]中
		Show_Str(2,k,BLUE,YELLOW,(u8 *) txt,16,0);
		sprintf(txt, "T:%f   ",my_car.v_x);                    //将变量填充到字符串的对应位置，并将字符串存放到txt[]中
		Show_Str(120,k,BLUE,YELLOW,(u8 *) txt,16,0);
		k+=d;
		sprintf(txt, "v_y:%f   ",my_car.now_v_y);                    //将变量填充到字符串的对应位置，并将字符串存放到txt[]中
		Show_Str(2,k,BLUE,YELLOW,(u8 *) txt,16,0);
		sprintf(txt, "T:%f ",my_car.v_y);                    //将变量填充到字符串的对应位置，并将字符串存放到txt[]中
		Show_Str(120,k,BLUE,YELLOW,(u8 *) txt,16,0);
		k+=d;
		sprintf(txt, "now_x:%f  ",my_car.now_x);                    //将变量填充到字符串的对应位置，并将字符串存放到txt[]中
		Show_Str(2,k,BLUE,YELLOW,(u8 *) txt,16,0);
		sprintf(txt, "T:%.2f ",my_car.target_x);                    //将变量填充到字符串的对应位置，并将字符串存放到txt[]中
		Show_Str(150,k,BLUE,YELLOW,(u8 *) txt,16,0);
		k+=d;
		sprintf(txt, "now_y:%f   ",my_car.now_y);                    //将变量填充到字符串的对应位置，并将字符串存放到txt[]中
		Show_Str(2,k,BLUE,YELLOW,(u8 *) txt,16,0);
		sprintf(txt, "T:%.2f ",my_car.target_y);                    //将变量填充到字符串的对应位置，并将字符串存放到txt[]中
		Show_Str(150,k,BLUE,YELLOW,(u8 *) txt,16,0);
		k+=d;
		sprintf(txt, "v_w:%f   ",my_car.w);                    //将变量填充到字符串的对应位置，并将字符串存放到txt[]中
		Show_Str(2,k,BLUE,YELLOW,(u8 *) txt,16,0);
		k+=d;
		sprintf(txt, "QR_code:%d, %d%d%d%d%d%d   ",QR_codeRx_Date,QR_codeRx_Date/100000,(QR_codeRx_Date/10000)%10,(QR_codeRx_Date/1000)%10,(QR_codeRx_Date/100)%10,(QR_codeRx_Date/10)%10,QR_codeRx_Date%10);                    //将变量填充到字符串的对应位置，并将字符串存放到txt[]中
		Show_Str(2,k,BLUE,YELLOW,(u8 *) txt,16,0);
	 
		k+=d;
		sprintf(txt, "mv:%c ",Openmv_flag);                    //将变量填充到字符串的对应位置，并将字符串存放到txt[]中
		Show_Str(2,k,BLUE,YELLOW,(u8 *) txt,16,0);
		sprintf(txt, "X:%d  ",opemv_X);                    //将变量填充到字符串的对应位置，并将字符串存放到txt[]中
		Show_Str(50,k,BLUE,YELLOW,(u8 *) txt,16,0);
		sprintf(txt, "Y:%d  ",opemv_Y);                    //将变量填充到字符串的对应位置，并将字符串存放到txt[]中
		Show_Str(120,k,BLUE,YELLOW,(u8 *) txt,16,0);
		
		k+=d;
		sprintf(txt, "task:%d ",flag_do_task);                    //将变量填充到字符串的对应位置，并将字符串存放到txt[]中
		Show_Str(2,k,BLUE,YELLOW,(u8 *) txt,16,0);
		sprintf(txt, "align:%d ",camera_auto_align);
		Show_Str(80,k,BLUE,YELLOW,(u8 *) txt,16,0);
		
		k+=d;
		sprintf(txt, "u1:%c ",uart1_Res);                    //将变量填充到字符串的对应位置，并将字符串存放到txt[]中
		Show_Str(2,k,BLUE,YELLOW,(u8 *) txt,16,0);
		sprintf(txt, "u2:%c ",uart2_Res);                    //将变量填充到字符串的对应位置，并将字符串存放到txt[]中
		Show_Str(2+5*8,k,BLUE,YELLOW,(u8 *) txt,16,0);
		sprintf(txt, "u3:%c ",uart3_Res);                    //将变量填充到字符串的对应位置，并将字符串存放到txt[]中
		Show_Str(2+5*8+5*8,k,BLUE,YELLOW,(u8 *) txt,16,0);
		sprintf(txt, "u4:%c ",uart4_Res);                    //将变量填充到字符串的对应位置，并将字符串存放到txt[]中
		Show_Str(2+5*8+5*8+5*8,k,BLUE,YELLOW,(u8 *) txt,16,0);
		sprintf(txt, "u5:%c ",uart5_Res);                    //将变量填充到字符串的对应位置，并将字符串存放到txt[]中
		Show_Str(2+5*8+5*8+5*8+5*8,k,BLUE,YELLOW,(u8 *) txt,16,0);
		k+=d;

		if(key1_flag == 1)
			{
				k+=d;
				Show_Str(2,k,BLUE,YELLOW,(u8 *)"key1_flag",16,0);

//				sprintf(txt, "clip:%d  ",Servo_clip_duty);                    //将变量填充到字符串的对应位置，并将字符串存放到txt[]中
//				Show_Str(2,k,BLUE,YELLOW,(u8 *) txt,16,0);
//				Servo_clip_duty ++;
				
//				sprintf(txt, "turntable:%d  ",Servo_turntable_duty);                    //将变量填充到字符串的对应位置，并将字符串存放到txt[]中
//				Show_Str(2,k,BLUE,YELLOW,(u8 *) txt,16,0);
//				Servo_turntable_duty ++;
				
//				sprintf(txt, "platform:%d  ",Servo_platform_duty);                    //将变量填充到字符串的对应位置，并将字符串存放到txt[]中
//				Show_Str(2,k,BLUE,YELLOW,(u8 *) txt,16,0);
//				Servo_platform_duty ++;
//				Servo_turntable_duty ++;
				
				HWT_Init();//陀螺仪初始化
				Motor_parameter_Init();//电机参数初始化
				my_car.stop_flag = 0; //停车标志位

				key1_flag = 0;
			}
			if(key2_flag == 1)
			{
				k+=d;
				Show_Str(2,k,BLUE,YELLOW,(u8 *)"key2_flag",16,0);
//				sprintf(txt, "clip:%d  ",Servo_clip_duty);                    //将变量填充到字符串的对应位置，并将字符串存放到txt[]中
//				Show_Str(2,k,BLUE,YELLOW,(u8 *) txt,16,0);
//				Servo_clip_duty --;

//				sprintf(txt, "turntable:%d  ",Servo_turntable_duty);                    //将变量填充到字符串的对应位置，并将字符串存放到txt[]中
//				Show_Str(2,k,BLUE,YELLOW,(u8 *) txt,16,0);
//				Servo_turntable_duty --;
//				
//				sprintf(txt, "platform:%d  ",Servo_platform_duty);                    //将变量填充到字符串的对应位置，并将字符串存放到txt[]中
//				Show_Str(2,k,BLUE,YELLOW,(u8 *) txt,16,0);
//				Servo_platform_duty --;
//				Servo_turntable_duty --;
				
				my_car.stop_flag =!my_car.stop_flag;//停车标志位翻转，控制车辆启停
//							Emm_V5_Pos_Control(USART2, 1, 8, 500, 0, 9000, false, false); //抬起爪子，目前位置1000
//					Emm_V5_Pos_Control(USART2, 1, 0, 500, 0, 10000, false, false); //放下爪子，目前位置10000

				key2_flag = 0;
			}


		LED1=!LED1;
		delay_ms(1);                     //延时300ms
			
			
	}
}


void TIM7_IRQHandler(void)//10Hz,，0.1s=100ms
{
	if (TIM_GetITStatus(TIM7, TIM_IT_Update) == SET)//检查TIM7更新中断发生置位
	{
				

				Timer7_count ++;//用于计时

						if ((Timer7_count - bt_last_tx_tick) >= BT_HEARTBEAT_TICKS)
						{
							#if BT_USE_UART5
							USART_SendData(UART5, 'H');
							#else
							USART_SendData(USART1, 'H');
							#endif
							bt_last_tx_tick = Timer7_count;
						}
						if ((Timer7_count - bt_last_rx_tick) > BT_TIMEOUT_TICKS)
						{
							bt_vx = 0.0f;
							bt_vy = 0.0f;
							bt_w = 0.0f;
							my_car.stop_flag = 1;
						}

					if (Servo_clip_duty < bt_clip_target)
					{
						Servo_clip_duty = (uint16_t)constrain_uint16_t((uint16_t)(Servo_clip_duty + BT_CLIP_STEP), bt_clip_min, bt_clip_max);
					}
					else if (Servo_clip_duty > bt_clip_target)
					{
						uint16_t next_clip = (Servo_clip_duty > BT_CLIP_STEP) ? (uint16_t)(Servo_clip_duty - BT_CLIP_STEP) : bt_clip_min;
						Servo_clip_duty = constrain_uint16_t(next_clip, bt_clip_min, bt_clip_max);
					}

					if (openmv_grab_pending)
					{
						Servo_clip_duty = bt_clip_max;
						openmv_grab_pending = 0;
					}

					if (openmv_drive_timer > 0)
					{
						bt_vx = 0.0f; bt_vy = 30.0f; bt_w = 0.0f; my_car.stop_flag = 0;
						openmv_drive_timer--;
						if (openmv_drive_timer == 0) { bt_vx = 0.0f; bt_vy = 0.0f; bt_w = 0.0f; my_car.stop_flag = 1; }
					}

				if (flag_do_task == -3)
				{
					Emm_V5_Pos_Control(USART2, 1, 0, 500, 0, 500, false, false);
					Timer7_count = 0;
					flag_do_task = -2;
				}
				if (flag_do_task == -2 && Timer7_count >= 10)
				{
					Servo_clip_duty = Servo_clip_hold;
					Timer7_count = 0;
					flag_do_task = -1;
				}
				if (flag_do_task == -1 && Timer7_count >= 10)
				{
					Emm_V5_Pos_Control(USART2, 1, 8, 500, 0, 12000, false, false);
					Timer7_count = 0;
					flag_do_task = 0;
				}
				if (flag_do_task == 0 && Timer7_count >= 10)
				{
					Servo_turntable_duty = Servo_turntable_behind;
					Timer7_count = 0;
					flag_do_task = 1;
				}
				if (flag_do_task == 1 && Timer7_count >= 10)
				{
					Emm_V5_Pos_Control(USART2, 1, 0, 500, 0, 500, false, false);
					Timer7_count = 0;
					flag_do_task = 2;
				}
				if (flag_do_task == 2 && Timer7_count >= 10)
				{
					Servo_clip_duty = Servo_clip_loosen;
					Timer7_count = 0;
					flag_do_task = 3;
				}
				if (flag_do_task == 3 && Timer7_count >= 10)
				{
					Emm_V5_Pos_Control(USART2, 1, 8, 500, 0, 3500, false, false);
					Timer7_count = 0;
					flag_do_task = 4;
				}
				if (flag_do_task == 4 && Timer7_count >= 10)
				{
					Servo_turntable_duty = Servo_turntable_front;
					Timer7_count = 0;
					flag_do_task = 5;
				}
				if (flag_do_task == 5)
				{
					flag_do_task = -100;
					bt_drive_mode = 1;
				}

				LED3=!LED3;

		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);//清除TIMx更新中断标志
	}
}

void TIM6_IRQHandler(void)//50Hz，0.020s=2ms中断
{
	if (TIM_GetITStatus(TIM6, TIM_IT_Update) == SET)//检查TIM6更新中断发生置位
	{
			Encoder_FR =  +(int16_t)TIM_GetCounter(TIM4); //右前轮子，motor_1
			Encoder_FL =  -(int16_t)TIM_GetCounter(TIM5); //左前轮子，motor_2
			Encoder_BR =  +(int16_t)TIM_GetCounter(TIM2); //右后轮子，motor_4
			Encoder_BL =  +(int16_t)TIM_GetCounter(TIM3); //左后轮子，motor_3
					//调试使用
//			my_car.yaw = -90;  //当前角度赋值
//			YAW = -90;
//			my_car.target_yaw = -90;  //旋转车身
//			Encoder_FR =  40; //右前轮子，motor_1
//			Encoder_FL =  40; //左前轮子，motor_2
//			Encoder_BR =  40; //右后轮子，motor_4
//			Encoder_BL =  40; //左后轮子，motor_3
		
//			Encoder_FR =  -40; //右前轮子，motor_1
//			Encoder_FL =  40; //左前轮子，motor_2
//			Encoder_BR =  40; //右后轮子，motor_4
//			Encoder_BL =  -40; //左后轮子，motor_3

			TIM_SetCounter(TIM2, 0);
			TIM_SetCounter(TIM3, 0);
			TIM_SetCounter(TIM4, 0);
			TIM_SetCounter(TIM5, 0);
		
		//四个电机的开环控制，确定占空比
		//实现四个电机的闭环控制:速度单位转换cm/s，输入是PID参数、目标速度、当前速度 -》输出是占空比
		//麦克纳姆轮逆运动学（xy坐标系）函数测试,输入是vx\vy\vw-》输出是目标速度
		//麦轮正运动学解算，函数测试，now_VY=正、now_Vx=0
		//x轴位置环：输入当前位置、目标位置、PID参数，输出是什么：Vx
		//y轴位置环：输入当前位置、目标位置、PID参数，输出是什么：Vy
		//角度闭环：输入当前角度、目标角度、PID参数，输出是什么：Vw
		
			encoder_count_get();//获取编码器的值
			
			speed_translation(&my_car.motor_1);speed_translation(&my_car.motor_2);speed_translation(&my_car.motor_3);speed_translation(&my_car.motor_4);//计算轮速

			//车坐标轴->地图坐标轴v_y、v_x、now_x、now_y
			RobotCalculate();//麦轮正运动学解算
			if (bt_drive_mode)
			{
				my_car.v_x = bt_vx;
				my_car.v_y = bt_vy;
				my_car.w = bt_w;
			}
			else
			{
				car_yaw_pid();
				my_car.w =  -my_car.w;

				PositionPID_Calculate(&my_car.position_pid_x, my_car.target_x, my_car.now_x);
				PositionPID_Calculate(&my_car.position_pid_y, my_car.target_y, my_car.now_y);

				my_car.v_x = my_car.position_pid_x.Output;
				my_car.v_y = my_car.position_pid_y.Output;

				if(my_car.target_yaw == 0.0f)
				{
						my_car.v_y = my_car.position_pid_y.Output;
						my_car.v_x = my_car.position_pid_x.Output;
				}
				else if(my_car.target_yaw == 90.0f)
				{
						my_car.v_x = -my_car.position_pid_y.Output;
						my_car.v_y = my_car.position_pid_x.Output;
				}
				else if(my_car.target_yaw == -90.0f)
				{
						my_car.v_x = my_car.position_pid_y.Output;
						my_car.v_y = -my_car.position_pid_x.Output;
				}
				else if(my_car.target_yaw == 180.0f||my_car.target_yaw == -180.0f)
				{
						my_car.v_x = -my_car.position_pid_x.Output;
						my_car.v_y = -my_car.position_pid_y.Output;
				}

				if(Openmv_track_start == 1)
				{
					PositionPID_Calculate(&my_car.openmv_pid_y, (float)opemv_middle_Y, (float)opemv_Y);
					PositionPID_Calculate(&my_car.openmv_pid_x, (float)opemv_middle_X, (float)opemv_X);
					my_car.v_x = my_car.openmv_pid_y.Output;
					my_car.v_y = -my_car.openmv_pid_x.Output;
				}
			}
			mecanum(my_car.v_y, my_car.v_x, my_car.w);
			//调试使用
			//Vy=正数，前面走
//			my_car.motor_1.target_speed = 40;
//			my_car.motor_2.target_speed = 40;
//			my_car.motor_3.target_speed = 40;
//			my_car.motor_4.target_speed = 40;
				//VX=正数，右边走
//			my_car.motor_1.target_speed = -40;//右前轮子，motor_1
//			my_car.motor_2.target_speed = 40;//左前轮子，motor_2
//			my_car.motor_3.target_speed = -40;//左后轮子，motor_3
//			my_car.motor_4.target_speed = 40;//右后轮子，motor_4
				//W=正数，顺时针转
//			my_car.motor_1.target_speed = 40;//右前轮子，motor_1
//			my_car.motor_2.target_speed = 40;//左前轮子，motor_2
//			my_car.motor_3.target_speed = -40;//左后轮子，motor_3
//			my_car.motor_4.target_speed = -40;//右后轮子，motor_4
			IncrementalPID_Calculate(&my_car.motor_1.s_pid, my_car.motor_1.target_speed, my_car.motor_1.speed);
			IncrementalPID_Calculate(&my_car.motor_2.s_pid, my_car.motor_2.target_speed, my_car.motor_2.speed);
			IncrementalPID_Calculate(&my_car.motor_3.s_pid, my_car.motor_3.target_speed, my_car.motor_3.speed);
			IncrementalPID_Calculate(&my_car.motor_4.s_pid, my_car.motor_4.target_speed, my_car.motor_4.speed);


			if(my_car.stop_flag == 1)   //停车处理
				{
						my_car.motor_1.s_pid.Output = 0;
						my_car.motor_2.s_pid.Output = 0;
						my_car.motor_3.s_pid.Output = 0;
						my_car.motor_4.s_pid.Output = 0;
				}
				
			if(Openmv_flag == 'Q' && Openmv_track_start == 1)//openmv循迹开启了，但接受到'Q'是没有识别到，则不进行移动
			{
					my_car.motor_1.s_pid.Output = 0;
					my_car.motor_2.s_pid.Output = 0;
					my_car.motor_3.s_pid.Output = 0;
					my_car.motor_4.s_pid.Output = 0;
			}
				
		  my_car.motor_2.s_pid.Output = constrain_float(my_car.motor_2.s_pid.Output, -motorFL_MAX, motorFL_MAX);
			my_car.motor_1.s_pid.Output = constrain_float(my_car.motor_1.s_pid.Output, -motorFR_MAX, motorFR_MAX);
			my_car.motor_3.s_pid.Output = constrain_float(my_car.motor_3.s_pid.Output, -motorFR_MAX, motorBL_MAX);
			my_car.motor_4.s_pid.Output = constrain_float(my_car.motor_4.s_pid.Output, -motorFR_MAX, motorBR_MAX);
		
			my_car.motor_1.PWM = my_car.motor_1.s_pid.Output;
			my_car.motor_2.PWM = my_car.motor_2.s_pid.Output;
			my_car.motor_3.PWM = my_car.motor_3.s_pid.Output;
			my_car.motor_4.PWM = my_car.motor_4.s_pid.Output;			
			
			//调试使用   Y正，往左走
//			my_car.motor_2.PWM = 2000;//左前轮子，motor_2
// 		  my_car.motor_1.PWM = 2000;//右前轮子，motor_1
//			my_car.motor_3.PWM = 2000;//左后轮子，motor_3
//			my_car.motor_4.PWM = 2000;//右后轮子，motor_4
			//旋转车身
//			my_car.motor_2.PWM = 2100;//左前轮子，motor_2
// 		  my_car.motor_1.PWM = 2100;//右前轮子，motor_1
//			my_car.motor_3.PWM = -2100;//左后轮子，motor_3
//			my_car.motor_4.PWM = -2100;//右后轮子，motor_4
			//X正，往前面走
//			my_car.motor_2.PWM = 2100;//左前轮子，motor_2
// 		  my_car.motor_1.PWM = -2100;//右前轮子，motor_1
//			my_car.motor_3.PWM = -2100;//左后轮子，motor_3
//			my_car.motor_4.PWM = 2100;//右后轮子，motor_4

			my_car.motor_2.PWM = constrain_float(my_car.motor_2.PWM, -motorFL_MAX, motorFL_MAX);
			my_car.motor_1.PWM = constrain_float(my_car.motor_1.PWM, -motorFR_MAX, motorFR_MAX);
			my_car.motor_3.PWM = constrain_float(my_car.motor_3.PWM, -motorBL_MAX, motorBL_MAX);
			my_car.motor_4.PWM = constrain_float(my_car.motor_4.PWM, -motorBR_MAX, motorBR_MAX);	
				

			mortor_TB6612Set(motorFL_PWM, motorFL_IN1, motorFL_IN2, (int32_t)my_car.motor_2.PWM, motorFL_MAX, motorFL_Die);
      mortor_TB6612Set(motorFR_PWM, motorFR_IN1, motorFR_IN2, (int32_t)my_car.motor_1.PWM, motorFR_MAX, motorFR_Die);
      mortor_TB6612Set(motorBL_PWM, motorBL_IN1, motorBL_IN2, (int32_t)my_car.motor_3.PWM, motorBL_MAX, motorBL_Die);
      mortor_TB6612Set(motorBR_PWM, motorBR_IN1, motorBR_IN2, (int32_t)my_car.motor_4.PWM, motorBR_MAX, motorBR_Die);

//			char txt[32];
//			sprintf(txt, "V:%f\n",my_car.motor_1.speed);                    //将变量填充到字符串的对应位置，并将字符串存放到txt[]中
//			uart_write_string(UART5, (uint8_t*)txt);//向串口发送数据
			
			Servo_clip_SetAngle(Servo_clip_duty);		//夹子舵机赋值
			Servo_turntable_SetAngle(Servo_turntable_duty);  //转盘舵机赋值
			Servo_platform_SetAngle(Servo_platform_duty);//装物料的平台舵机
			Button_Control_scan();
		LED2=!LED2;
		TIM_ClearITPendingBit(TIM6, TIM_IT_Update);//清除TIMx更新中断标志 
	}
}
		
void USART1_IRQHandler(void)                	//串口1中断服务程序
	{

	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //检查TIM7更新中断发生置位
		{
		   uart1_Res = USART_ReceiveData(USART1);	//读取接收到的数据
			#if !BT_USE_UART5
			bt_cmd = uart1_Res;
			bt_last_rx_tick = Timer7_count;
			BT_ApplyCommand(uart1_Res);
			bt_cmd = 0;
			USART_SendData(USART1, uart1_Res);
			#endif
				
  		 USART_ClearITPendingBit(USART1, USART_IT_RXNE);//清除TIMx更新中断标志 
     } 
} 
void USART2_IRQHandler(void)                	//串口2中断服务程序
	{

	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //检查TIM7更新中断发生置位
		{
		   uart2_Res = USART_ReceiveData(USART2);	//读取接收到的数据
				uart2_str[uart2_str_count] = uart2_Res;
				if(uart2_str_count == uart2_str_T_count)
					{
						uart2_str_flag = 1;
						uart2_str_count = 0;
					}
				uart2_str_count++;
			
  		 USART_ClearITPendingBit(USART2, USART_IT_RXNE);

     } 

} 
unsigned char Cmd_GetPkt(unsigned char byte);

//注意：右边为0°~-180°，左边为180°~0°
uint8_t HWT_ResBuffer[22] = {0};  //陀螺仪储存数组
unsigned char HWT_ResCnt = 0;
void USART3_IRQHandler(void)                	//串口3中断服务程序
	{

	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //检查TIM7更新中断发生置位
		{
		   uart3_Res = USART_ReceiveData(USART3);	//读取接收到的数据			
		HWT_ResBuffer[HWT_ResCnt] = uart3_Res;
			short yaw;	
		  short w;
			switch (HWT_ResCnt)
				{
					case 0:
						if (HWT_ResBuffer[HWT_ResCnt] == 0x55)
						{
							HWT_ResCnt++;
						}
						break;
					case 1:
						if (HWT_ResBuffer[HWT_ResCnt] == 0x52)
						{
							HWT_ResCnt++;
						}
						else
						{
							HWT_ResCnt = 0;
						}
						break;
		default:
						HWT_ResCnt++;
						break;
				}
				if(HWT_ResCnt ==  21)
				{
						yaw = (short)((((short)HWT_ResBuffer[18]) << 8) | HWT_ResBuffer[17]);
						YAW = (float)yaw / 32768 * 180;
						
						my_car.yaw = YAW;  //当前角度赋值


						w = (short)((((short)HWT_ResBuffer[7]) << 8) | HWT_ResBuffer[6]);
						W_Z = (float)w / 32768 * 2000;
					
					HWT_ResCnt = 0;
				}
  		 USART_ClearITPendingBit(USART3, USART_IT_RXNE);//清除TIMx更新中断标志 

     } 

} 

void UART4_IRQHandler(void)                	//串口4中断服务程序 F132X45Y
{

  if (USART_GetITStatus(UART4, USART_IT_RXNE) == SET)//检查TIM7更新中断发生置位
	{
			
			uart4_Res =USART_ReceiveData(UART4);//读取接收到的数据
		
				if(uart4_Res == 'A') Openmv_flag = 'A',OpenmvRxCnt = 0, openmv_grab_pending = 1, LED2 = !LED2;
				else if(uart4_Res == 'B') Openmv_flag = 'B', OpenmvRxCnt = 0, openmv_grab_pending = 1, LED2 = !LED2;
				else if(uart4_Res == 'C') Openmv_flag = 'C', OpenmvRxCnt = 0, openmv_grab_pending = 1, LED2 = !LED2;
				else if(uart4_Res == 'D') Openmv_flag = 'D', OpenmvRxCnt = 0, openmv_grab_pending = 1, LED2 = !LED2;
				else if(uart4_Res == 'E') Openmv_flag = 'E', OpenmvRxCnt = 0, openmv_grab_pending = 1, LED2 = !LED2;
				else if(uart4_Res == 'F') Openmv_flag = 'F', OpenmvRxCnt = 0, openmv_grab_pending = 1, LED2 = !LED2;
				else if(uart4_Res == 'G') Openmv_flag = 'G', OpenmvRxCnt = 0, openmv_grab_pending = 1, LED2 = !LED2;
				else if(uart4_Res == 'Q') Openmv_flag = 'Q', OpenmvRxCnt = 0, openmv_grab_pending = 0, LED3 = !LED3;
				else 
					{
						if(Openmv_flag != 0)
					{
						OpenmvBuffer[OpenmvRxCnt] = uart4_Res;
						if(OpenmvBuffer[OpenmvRxCnt] == 'X')  {   //获取X的误差 132X
								OpenmvBuffer[OpenmvRxCnt] = '\n';//132\n
								opemv_X = atoi((const char *)(OpenmvBuffer)); //132
								memset(OpenmvBuffer, '\n', OpenmvRxCnt);
								OpenmvRxCnt = 0;
						}
						if(OpenmvBuffer[OpenmvRxCnt] == 'Y')  {   //获取H的误差45Y
								OpenmvBuffer[OpenmvRxCnt] = '\n';//45\n
								opemv_Y = atoi((const char *)(OpenmvBuffer));//45
								memset(OpenmvBuffer, '\n', OpenmvRxCnt);
								OpenmvRxCnt = 0;
								Openmv_flag = 0;
								if (openmv_camera_test && openmv_drive_timer == 0) openmv_drive_timer = 10;
								LED1 = !LED1;
						}
						OpenmvRxCnt++;
					}
				}
				
		  
			USART_ClearITPendingBit(UART4, USART_IT_RXNE);//清除TIMx更新中断标志 
	}
}


/*
QR_codeRx_Date/100000 
(QR_codeRx_Date/10000)%10
(QR_codeRx_Date/1000)%10
(QR_codeRx_Date/100)%10
(QR_codeRx_Date/10)%10
QR_codeRx_Date%10 
*/
void UART5_IRQHandler(void)                	//串口5中断服务程序
{

  if (USART_GetITStatus(UART5, USART_IT_RXNE) == SET)//检查TIM7更新中断发生置位
	{
			uart5_Res = USART_ReceiveData(UART5);//读取接收到的数据
			#if BT_USE_UART5
			bt_cmd = uart5_Res;
			bt_last_rx_tick = Timer7_count;
			BT_ApplyCommand(uart5_Res);
			bt_cmd = 0;
			USART_SendData(UART5, uart5_Res);
			#else
			QR_codeBuffer[QR_codeRxCnt] = uart5_Res;
			QR_codeRxCnt++;
			if(QR_codeRxCnt == 20)  QR_codeRxCnt = 0;

//			if(QR_codeRxCnt >= 7 && uart5_Res == '+')
//			{
//				QR_codeBuffer[0] = QR_codeBuffer[QR_codeRxCnt-7];
//				QR_codeBuffer[1] = QR_codeBuffer[QR_codeRxCnt-6];
//				QR_codeBuffer[2] = QR_codeBuffer[QR_codeRxCnt-5];
//				QR_codeBuffer[3] = QR_codeBuffer[QR_codeRxCnt-4];
//				QR_codeBuffer[4] = QR_codeBuffer[QR_codeRxCnt-3];
//				QR_codeBuffer[5] = QR_codeBuffer[QR_codeRxCnt-2];
//				QR_codeBuffer[6] =  '\n';
//				QR_codeRx_Date = atoi((const char *)(QR_codeBuffer)); //213321
//				memset(QR_codeBuffer, '\n', QR_codeRxCnt);
//				QR_codeRxCnt = 0;
//			}
//			else if(QR_codeRxCnt < 7 && uart5_Res == '+')
//				QR_codeRxCnt = 0;
			

		
			if(QR_codeRxCnt == 7 && QR_codeBuffer[3] == '+')
			{
				QR_codeBuffer[3] = QR_codeBuffer[4];
				QR_codeBuffer[4] = QR_codeBuffer[5];
				QR_codeBuffer[5] = QR_codeBuffer[6];
				QR_codeBuffer[6] =  '\n';
				QR_codeRx_Date = atoi((const char *)(QR_codeBuffer)); //213321
				memset(QR_codeBuffer, '\n', QR_codeRxCnt);
				QR_codeRxCnt = 0;
			}
			else if(QR_codeRxCnt == 7)  QR_codeRxCnt = 0;
			#endif
			
				//321+213
//				if(QR_codeBuffer[QR_codeRxCnt] == '+') QR_codeRxCnt = 1;
//				else if(QR_codeRxCnt >= 1)
//					{
//						QR_codeBuffer[QR_codeRxCnt-1] = uart5_Res;
//						QR_codeRxCnt++;
//						if(QR_codeRxCnt >= 7)
//						{
//							QR_codeBuffer[QR_codeRxCnt-1] = '\n';//"213321"
//							QR_codeRx_Date = atoi((const char *)(QR_codeBuffer)); //213321
//							memset(QR_codeBuffer, '\n', QR_codeRxCnt);
//							QR_codeRxCnt = 0;
//						}
//					}

		  
			USART_ClearITPendingBit(UART5, USART_IT_RXNE);//清除TIMx更新中断标志 
	}
}



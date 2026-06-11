#include "ALL_Head.h"
int flag_do_task = -100; //-100=缁屾椽妫? -3~5=閼奉亜濮╂禒璇插鏉╂稖顢戞稉?

#define BT_USE_UART5 1
#define BT_HOLD_MODE 1
#define BT_LIFT_VEL 200
#define BT_LIFT_ACC 0
#define BT_LIFT_SLOW_CLK 200
#define BT_LIFT_SLOW_VEL 30
#define BT_TURNTABLE_STEP 30
#define BT_TURNTABLE_SMOOTH_STEP 10
#define BT_PLATFORM_STEP 125
#define BT_PLATFORM_SMOOTH_STEP 5
#define BT_CLIP_STEP 2
#define BT_FINE_VEL 30.0f
#define BT_FINE_W 1.0f
#define BT_CMD_TASK_START 'M'

static const uint16_t bt_clip_min = Servo_clip_loosen;
static const uint16_t bt_clip_max = Servo_clip_hold;
//static const uint16_t bt_turntable_min = Servo_turntable_front;
//static const uint16_t bt_turntable_max = Servo_turntable_behind;
//static const uint16_t bt_platform_min = Servo_platform_three;
//static const uint16_t bt_platform_max = Servo_platform_two;

// 载物盘三工位循环
static const uint16_t platform_pos[] = {
    Servo_platform_two,   // 260
    Servo_platform_one,   // 135
    Servo_platform_three  // 10
};
static volatile uint8_t platform_idx = 0; // 起始工位 = 260 (platform_two)

static volatile uint8_t bt_cmd = 0;
static volatile uint16_t task_turntable_pos = 250;	// 自动任务云台目标位置（默认250）
static volatile uint16_t task_reverse_down = 3000;	// 反流程第一次升降台下降量（默认3000）

static volatile uint8_t bt_drive_mode = 1;
static volatile uint8_t bt_lift_active = 0;
static volatile float bt_vx = 0.0f;
static volatile float bt_vy = 0.0f;
static volatile float bt_w = 0.0f;
static volatile uint16_t bt_clip_target = Servo_clip_loosen;
static volatile uint16_t bt_platform_target = Servo_platform_two; // 载物盘目标位（与初始工位一致）
static volatile unsigned long bt_last_rx_tick = 0;
static volatile unsigned long bt_last_tx_tick = 0;
//static volatile int8_t bt_turntable_hold = 0; // -1 left, +1 right
//static volatile int8_t bt_platform_hold = 0;  // -1 right, +1 left
//static volatile uint8_t bt_hold_tick = 0;


#define BT_TIMEOUT_TICKS 1 // ~0.2s @ 10Hz (> cmp: fires at 2nd tick)
#define BT_HEARTBEAT_TICKS 2 // 0.2s @ 10Hz

static void BT_ApplyCommand(uint8_t raw_cmd)
{
	if (raw_cmd == 0)
		return;

	//閹稿绗呮穱婵囧瘮濡€崇础閿涙氨瀹崇€规艾鐨崘娆忕摟缁楋箒銆冪粈鐑樺瘻娑?start)閿涘苯顕惔鏃傛畱婢堆冨晸鐎涙顑佺悰銊с仛閺夋儳绱?stop)
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

	uint16_t turntable_min = Servo_turntable_front;
	uint16_t turntable_max = Servo_turntable_behind;
	uint16_t turntable_mid = (uint16_t)((turntable_min + turntable_max) / 2);
	uint16_t platform_min = Servo_platform_three;
	uint16_t platform_max = Servo_platform_two;

	switch (cmd)
	{
		case 'X': // explicit stop
		case 'S':
			bt_vx = 0.0f;
			bt_vy = 0.0f;
			bt_w = 0.0f;
			my_car.stop_flag = 1;
			break;

		case '0': // 全局急停：停止所有部件
			if (is_press)
			{
				bt_vx = 0.0f; bt_vy = 0.0f; bt_w = 0.0f;
				my_car.stop_flag = 1;
				Emm_V5_Stop_Now(USART2, 1, 0);
				bt_lift_active = 0;
				flag_do_task = -100;
				bt_drive_mode = 1;
				bt_clip_target = Servo_clip_duty;
				Servo_turntable_target = Servo_turntable_duty;
				bt_platform_target = Servo_platform_duty;
			}
			break;

		case 'F': // forward (press=move, release=stop)
			if (is_press)
			{
				bt_vx = 0.0f; bt_vy = BT_FINE_VEL; bt_w = 0.0f; my_car.stop_flag = 0;
			}
			else
			{
				bt_vx = 0.0f; bt_vy = 0.0f; bt_w = 0.0f; my_car.stop_flag = 1;
			}
			break;
		case 'B': // backward (press=move, release=stop)
			if (is_press)
			{
				bt_vx = 0.0f; bt_vy = -BT_FINE_VEL; bt_w = 0.0f; my_car.stop_flag = 0;
			}
			else
			{
				bt_vx = 0.0f; bt_vy = 0.0f; bt_w = 0.0f; my_car.stop_flag = 1;
			}
			break;
		case 'L': // left (press=move, release=stop)
			if (is_press)
			{
				bt_vx = -BT_FINE_VEL; bt_vy = 0.0f; bt_w = 0.0f; my_car.stop_flag = 0;
			}
			else
			{
				bt_vx = 0.0f; bt_vy = 0.0f; bt_w = 0.0f; my_car.stop_flag = 1;
			}
			break;
		case 'R': // right (press=move, release=stop)
			if (is_press)
			{
				bt_vx = BT_FINE_VEL; bt_vy = 0.0f; bt_w = 0.0f; my_car.stop_flag = 0;
			}
			else
			{
				bt_vx = 0.0f; bt_vy = 0.0f; bt_w = 0.0f; my_car.stop_flag = 1;
			}
			break;
		case 'A': // rotate left (press=move, release=stop)
			if (is_press)
			{
				bt_vx = 0.0f; bt_vy = 0.0f; bt_w = -BT_FINE_W; my_car.stop_flag = 0;
			}
			else
			{
				bt_vx = 0.0f; bt_vy = 0.0f; bt_w = 0.0f; my_car.stop_flag = 1;
			}
			break;
		case 'D': // rotate right (press=move, release=stop)
			if (is_press)
			{
				bt_vx = 0.0f; bt_vy = 0.0f; bt_w = BT_FINE_W; my_car.stop_flag = 0;
			}
			else
			{
				bt_vx = 0.0f; bt_vy = 0.0f; bt_w = 0.0f; my_car.stop_flag = 1;
			}
			break;

		case 'U': // lift up (press=ascend, release=stop)
			if (is_press)
			{
				Emm_V5_Vel_Control(USART2, 1, 8, BT_LIFT_VEL, BT_LIFT_ACC, 0);
				bt_lift_active = 1;
			}
			else
			{
				Emm_V5_Stop_Now(USART2, 1, 0);
				bt_lift_active = 0;
			}
			break;
		case 'N': // lift down (press=descend, release=stop)
			if (is_press)
			{
				Emm_V5_Vel_Control(USART2, 1, 0, BT_LIFT_VEL, BT_LIFT_ACC, 0);
				bt_lift_active = 1;
			}
			else
			{
				Emm_V5_Stop_Now(USART2, 1, 0);
				bt_lift_active = 0;
			}
			break;
		case 'Z': // lift stop
			Emm_V5_Stop_Now(USART2, 1, 0);
			bt_lift_active = 0;
			break;
		case 'V': // lift slow descent (gentle placement)
			if (is_press)
			{
				Emm_V5_Vel_Control(USART2, 1, 0, BT_LIFT_SLOW_VEL, BT_LIFT_ACC, 0);
				bt_lift_active = 1;
			}
			else
			{
				Emm_V5_Stop_Now(USART2, 1, 0);
				bt_lift_active = 0;
			}
			break;
		case '6': // lift reset (use when stuck at limit)
			uart_write_string(UART5, (uint8_t *)"RESET\r\n");
			Emm_V5_Stop_Now(USART2, 1, 0);
			delay_ms(50);
			Emm_V5_Reset_CurPos_To_Zero(USART2, 1);
			delay_ms(50);
			Emm_V5_Reset_Clog_Pro(USART2, 1);  // reset stall protection
			delay_ms(50);
			Emm_V5_En_Control(USART2, 1, 1, 0); // re-enable motor
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
		case 'C': // turntable center (fixed at 250)
			if (is_press) Servo_turntable_target = 250;
			break;
		case 'G': // turntable position 250
			if (is_press) Servo_turntable_target = 250;
			break;
		case 'I': // turntable position 70
			if (is_press) Servo_turntable_target = 70;
			break;
		case 'H': // turntable position 210
			if (is_press) Servo_turntable_target = 210;
			break;
		case 'W': // turntable position 180
			if (is_press) Servo_turntable_target = 180;
			break;
		case 'Q': // platform left
			if (is_press)
			{
				bt_platform_target = constrain_uint16_t((uint16_t)(bt_platform_target + BT_PLATFORM_STEP), platform_min, platform_max);
			}
			break;
		case 'E': // platform right
			if (is_press)
			{
				bt_platform_target = constrain_uint16_t((uint16_t)(bt_platform_target - BT_PLATFORM_STEP), platform_min, platform_max);
			}
			break;
		case BT_CMD_TASK_START: // auto task (turntable -> 250)
			if (is_press)
			{
				task_turntable_pos = 250;
				flag_do_task = -4;
				bt_vx = 0.0f; bt_vy = 0.0f; bt_w = 0.0f; my_car.stop_flag = 1;
			}
			break;
		case 'T': // auto task (turntable -> 210)
			if (is_press)
			{
				task_turntable_pos = 210;
				flag_do_task = -4;
				bt_vx = 0.0f; bt_vy = 0.0f; bt_w = 0.0f; my_car.stop_flag = 1;
			}
			break;
		case 'Y': // auto task (turntable -> 180)
			if (is_press)
			{
				task_turntable_pos = 180;
				flag_do_task = -4;
				bt_vx = 0.0f; bt_vy = 0.0f; bt_w = 0.0f; my_car.stop_flag = 1;
			}
			break;
		case '3': // reverse task (turntable -> 250)
			if (is_press)
			{
				task_turntable_pos = 250;
				task_reverse_down = 3000;
				flag_do_task = -11;
				bt_vx = 0.0f; bt_vy = 0.0f; bt_w = 0.0f; my_car.stop_flag = 1;
			}
			break;
		case '4': // reverse task (turntable -> 210)
			if (is_press)
			{
				task_turntable_pos = 210;
				task_reverse_down = 4000;
				flag_do_task = -11;
				bt_vx = 0.0f; bt_vy = 0.0f; bt_w = 0.0f; my_car.stop_flag = 1;
			}
			break;
		case '5': // reverse task (turntable -> 180)
			if (is_press)
			{
				task_turntable_pos = 180;
				task_reverse_down = 4000;
				flag_do_task = -11;
				bt_vx = 0.0f; bt_vy = 0.0f; bt_w = 0.0f; my_car.stop_flag = 1;
			}
			break;
		default:
			break;
	}
	bt_cmd = 0;
}

int main(void)
{	
	SystemInit();//閸掓繂顫愰崠鏈C 鐠佸墽鐤嗙化鑽ょ埠娑撳顣舵稉?2MHZ
	delay_init();	     //瀵よ埖妞傞崚婵嗩潗閸?
	LED_Init();
	Encoder_Init();
	#if !BT_USE_UART5
	uart1_init(9600); // HC-05/06 default, adjust if needed
	#endif
	uart2_init(115200);
	uart3_init(9600);//TX:GPIOB.10;  RX:GPIOB.11
	uart5_init(9600); // UART5: PC12(TX), PD2(RX)
	Motor_Init();
	Servo_Init();
	
	LCD_Init();	   //濞戝弶娅犵仦蹇撳灥婵瀵?
	Button_Control_init();
	HWT_Init();//闂勨偓閾昏桨鍗庨崚婵嗩潗閸?


	Motor_parameter_Init();//閻㈠灚婧€閸欏倹鏆熼崚婵嗩潗閸?
	
	Emm_V5_En_Control(USART2, 1, 1, 0);uart2_str_T_count = 4;
	delay_ms(500);                     //瀵よ埖妞俶s

//鐠佸墽鐤嗛崶鐐烘祩閸欏倹鏆?
//  Emm_V5_Origin_Modify_Params(USART2, 1, true, 2, 1, 100, 3500, 300, 1000, 60, false);
//	delay_ms(1000);                     //瀵よ埖妞俶s

//	Emm_V5_Origin_Trigger_Return(USART2, 1, 2, false);uart2_str_T_count = 10;
//	delay_ms(500);                     //瀵よ埖妞俶s

  Timer6_Init();
	delay_ms(7);                     //瀵よ埖妞俶s
	Timer7_Init();
	delay_ms(1000);                     //瀵よ埖妞俶s

	// 启动后云台自动归位到70（同'i'指令）
	Servo_turntable_target = 70;
	Servo_turntable_duty = 70;

	// 启动后爪子自动松开
	Servo_clip_duty = Servo_clip_loosen;
	bt_clip_target = Servo_clip_loosen;
	Servo_clip_SetAngle(Servo_clip_duty);

	while(1)
	{
		char txt[32];
		unsigned int k = 4;
    unsigned int d = 16;
		

		sprintf(txt, "V_FL:%d ,%f  ",Encoder_FL, my_car.motor_2.speed);                    //鐏忓棗褰夐柌蹇擄綖閸忓懎鍩岀€涙顑佹稉鑼畱鐎电懓绨叉担宥囩枂閿涘苯鑻熺亸鍡楃摟缁楋缚瑕嗙€涙ɑ鏂侀崚鐨寈t[]娑?
		Show_Str(2,k,BLUE,YELLOW,(u8 *) txt,16,0);
		k+=d;
		sprintf(txt, "TFL_V:%f  ",my_car.motor_2.target_speed);                    //鐏忓棗褰夐柌蹇擄綖閸忓懎鍩岀€涙顑佹稉鑼畱鐎电懓绨叉担宥囩枂閿涘苯鑻熺亸鍡楃摟缁楋缚瑕嗙€涙ɑ鏂侀崚鐨寈t[]娑?
		Show_Str(2,k,BLUE,YELLOW,(u8 *) txt,16,0);
		k+=d;
		sprintf(txt, "V_FR:%d ,%f  ",Encoder_FR, my_car.motor_1.speed);                    //鐏忓棗褰夐柌蹇擄綖閸忓懎鍩岀€涙顑佹稉鑼畱鐎电懓绨叉担宥囩枂閿涘苯鑻熺亸鍡楃摟缁楋缚瑕嗙€涙ɑ鏂侀崚鐨寈t[]娑?
		Show_Str(2,k,BLUE,YELLOW,(u8 *) txt,16,0);
		k+=d;
		sprintf(txt, "TFR_V:%f  ",my_car.motor_1.target_speed);                    //鐏忓棗褰夐柌蹇擄綖閸忓懎鍩岀€涙顑佹稉鑼畱鐎电懓绨叉担宥囩枂閿涘苯鑻熺亸鍡楃摟缁楋缚瑕嗙€涙ɑ鏂侀崚鐨寈t[]娑?
		Show_Str(2,k,BLUE,YELLOW,(u8 *) txt,16,0);
		k+=d;
		sprintf(txt, "V_BR:%d ,%f  ",Encoder_BR, my_car.motor_4.speed);                    //鐏忓棗褰夐柌蹇擄綖閸忓懎鍩岀€涙顑佹稉鑼畱鐎电懓绨叉担宥囩枂閿涘苯鑻熺亸鍡楃摟缁楋缚瑕嗙€涙ɑ鏂侀崚鐨寈t[]娑?
		Show_Str(2,k,BLUE,YELLOW,(u8 *) txt,16,0);
		k+=d;
		sprintf(txt, "TBR_V:%f  ",my_car.motor_4.target_speed);                    //鐏忓棗褰夐柌蹇擄綖閸忓懎鍩岀€涙顑佹稉鑼畱鐎电懓绨叉担宥囩枂閿涘苯鑻熺亸鍡楃摟缁楋缚瑕嗙€涙ɑ鏂侀崚鐨寈t[]娑?
		Show_Str(2,k,BLUE,YELLOW,(u8 *) txt,16,0);
		k+=d;
		sprintf(txt, "V_BL:%d ,%f  ",Encoder_BL, my_car.motor_3.speed);                    //鐏忓棗褰夐柌蹇擄綖閸忓懎鍩岀€涙顑佹稉鑼畱鐎电懓绨叉担宥囩枂閿涘苯鑻熺亸鍡楃摟缁楋缚瑕嗙€涙ɑ鏂侀崚鐨寈t[]娑?
		Show_Str(2,k,BLUE,YELLOW,(u8 *) txt,16,0);
		k+=d;
		sprintf(txt, "TBL_V:%f  ",my_car.motor_3.target_speed);                    //鐏忓棗褰夐柌蹇擄綖閸忓懎鍩岀€涙顑佹稉鑼畱鐎电懓绨叉担宥囩枂閿涘苯鑻熺亸鍡楃摟缁楋缚瑕嗙€涙ɑ鏂侀崚鐨寈t[]娑?
		Show_Str(2,k,BLUE,YELLOW,(u8 *) txt,16,0);
		k+=d;
		sprintf(txt, "YAW:%f   ",YAW);                    //鐏忓棗褰夐柌蹇擄綖閸忓懎鍩岀€涙顑佹稉鑼畱鐎电懓绨叉担宥囩枂閿涘苯鑻熺亸鍡楃摟缁楋缚瑕嗙€涙ɑ鏂侀崚鐨寈t[]娑?
		Show_Str(2,k,BLUE,YELLOW,(u8 *) txt,16,0);
		sprintf(txt, "T:%f   ",my_car.target_yaw);                    //鐏忓棗褰夐柌蹇擄綖閸忓懎鍩岀€涙顑佹稉鑼畱鐎电懓绨叉担宥囩枂閿涘苯鑻熺亸鍡楃摟缁楋缚瑕嗙€涙ɑ鏂侀崚鐨寈t[]娑?
		Show_Str(120,k,BLUE,YELLOW,(u8 *) txt,16,0);
		k+=d;
		sprintf(txt, "v_x:%f   ",my_car.now_v_x);                    //鐏忓棗褰夐柌蹇擄綖閸忓懎鍩岀€涙顑佹稉鑼畱鐎电懓绨叉担宥囩枂閿涘苯鑻熺亸鍡楃摟缁楋缚瑕嗙€涙ɑ鏂侀崚鐨寈t[]娑?
		Show_Str(2,k,BLUE,YELLOW,(u8 *) txt,16,0);
		sprintf(txt, "T:%f   ",my_car.v_x);                    //鐏忓棗褰夐柌蹇擄綖閸忓懎鍩岀€涙顑佹稉鑼畱鐎电懓绨叉担宥囩枂閿涘苯鑻熺亸鍡楃摟缁楋缚瑕嗙€涙ɑ鏂侀崚鐨寈t[]娑?
		Show_Str(120,k,BLUE,YELLOW,(u8 *) txt,16,0);
		k+=d;
		sprintf(txt, "v_y:%f   ",my_car.now_v_y);                    //鐏忓棗褰夐柌蹇擄綖閸忓懎鍩岀€涙顑佹稉鑼畱鐎电懓绨叉担宥囩枂閿涘苯鑻熺亸鍡楃摟缁楋缚瑕嗙€涙ɑ鏂侀崚鐨寈t[]娑?
		Show_Str(2,k,BLUE,YELLOW,(u8 *) txt,16,0);
		sprintf(txt, "T:%f ",my_car.v_y);                    //鐏忓棗褰夐柌蹇擄綖閸忓懎鍩岀€涙顑佹稉鑼畱鐎电懓绨叉担宥囩枂閿涘苯鑻熺亸鍡楃摟缁楋缚瑕嗙€涙ɑ鏂侀崚鐨寈t[]娑?
		Show_Str(120,k,BLUE,YELLOW,(u8 *) txt,16,0);
		k+=d;
		sprintf(txt, "now_x:%f  ",my_car.now_x);                    //鐏忓棗褰夐柌蹇擄綖閸忓懎鍩岀€涙顑佹稉鑼畱鐎电懓绨叉担宥囩枂閿涘苯鑻熺亸鍡楃摟缁楋缚瑕嗙€涙ɑ鏂侀崚鐨寈t[]娑?
		Show_Str(2,k,BLUE,YELLOW,(u8 *) txt,16,0);
		sprintf(txt, "T:%.2f ",my_car.target_x);                    //鐏忓棗褰夐柌蹇擄綖閸忓懎鍩岀€涙顑佹稉鑼畱鐎电懓绨叉担宥囩枂閿涘苯鑻熺亸鍡楃摟缁楋缚瑕嗙€涙ɑ鏂侀崚鐨寈t[]娑?
		Show_Str(150,k,BLUE,YELLOW,(u8 *) txt,16,0);
		k+=d;
		sprintf(txt, "now_y:%f   ",my_car.now_y);                    //鐏忓棗褰夐柌蹇擄綖閸忓懎鍩岀€涙顑佹稉鑼畱鐎电懓绨叉担宥囩枂閿涘苯鑻熺亸鍡楃摟缁楋缚瑕嗙€涙ɑ鏂侀崚鐨寈t[]娑?
		Show_Str(2,k,BLUE,YELLOW,(u8 *) txt,16,0);
		sprintf(txt, "T:%.2f ",my_car.target_y);                    //鐏忓棗褰夐柌蹇擄綖閸忓懎鍩岀€涙顑佹稉鑼畱鐎电懓绨叉担宥囩枂閿涘苯鑻熺亸鍡楃摟缁楋缚瑕嗙€涙ɑ鏂侀崚鐨寈t[]娑?
		Show_Str(150,k,BLUE,YELLOW,(u8 *) txt,16,0);
		k+=d;
		sprintf(txt, "v_w:%f   ",my_car.w);                    //鐏忓棗褰夐柌蹇擄綖閸忓懎鍩岀€涙顑佹稉鑼畱鐎电懓绨叉担宥囩枂閿涘苯鑻熺亸鍡楃摟缁楋缚瑕嗙€涙ɑ鏂侀崚鐨寈t[]娑?
		Show_Str(2,k,BLUE,YELLOW,(u8 *) txt,16,0);
		k+=d;
		sprintf(txt, "QR_code:%d, %d%d%d%d%d%d   ",QR_codeRx_Date,QR_codeRx_Date/100000,(QR_codeRx_Date/10000)%10,(QR_codeRx_Date/1000)%10,(QR_codeRx_Date/100)%10,(QR_codeRx_Date/10)%10,QR_codeRx_Date%10);                    //鐏忓棗褰夐柌蹇擄綖閸忓懎鍩岀€涙顑佹稉鑼畱鐎电懓绨叉担宥囩枂閿涘苯鑻熺亸鍡楃摟缁楋缚瑕嗙€涙ɑ鏂侀崚鐨寈t[]娑?
		Show_Str(2,k,BLUE,YELLOW,(u8 *) txt,16,0);
	 
		k+=d;
		sprintf(txt, "task:%d ",flag_do_task);                    //鐏忓棗褰夐柌蹇擄綖閸忓懎鍩岀€涙顑佹稉鑼畱鐎电懓绨叉担宥囩枂閿涘苯鑻熺亸鍡楃摟缁楋缚瑕嗙€涙ɑ鏂侀崚鐨寈t[]娑?
		Show_Str(2,k,BLUE,YELLOW,(u8 *) txt,16,0);
		Show_Str(80,k,BLUE,YELLOW,(u8 *) txt,16,0);
		k+=d;
		sprintf(txt, "u1:%c ",uart1_Res);                    //鐏忓棗褰夐柌蹇擄綖閸忓懎鍩岀€涙顑佹稉鑼畱鐎电懓绨叉担宥囩枂閿涘苯鑻熺亸鍡楃摟缁楋缚瑕嗙€涙ɑ鏂侀崚鐨寈t[]娑?
		Show_Str(2,k,BLUE,YELLOW,(u8 *) txt,16,0);
		sprintf(txt, "u2:%c ",uart2_Res);                    //鐏忓棗褰夐柌蹇擄綖閸忓懎鍩岀€涙顑佹稉鑼畱鐎电懓绨叉担宥囩枂閿涘苯鑻熺亸鍡楃摟缁楋缚瑕嗙€涙ɑ鏂侀崚鐨寈t[]娑?
		Show_Str(2+5*8,k,BLUE,YELLOW,(u8 *) txt,16,0);
		sprintf(txt, "u3:%c ",uart3_Res);                    //鐏忓棗褰夐柌蹇擄綖閸忓懎鍩岀€涙顑佹稉鑼畱鐎电懓绨叉担宥囩枂閿涘苯鑻熺亸鍡楃摟缁楋缚瑕嗙€涙ɑ鏂侀崚鐨寈t[]娑?
		Show_Str(2+5*8+5*8,k,BLUE,YELLOW,(u8 *) txt,16,0);
		sprintf(txt, "u4:%c ",uart4_Res);                    //鐏忓棗褰夐柌蹇擄綖閸忓懎鍩岀€涙顑佹稉鑼畱鐎电懓绨叉担宥囩枂閿涘苯鑻熺亸鍡楃摟缁楋缚瑕嗙€涙ɑ鏂侀崚鐨寈t[]娑?
		Show_Str(2+5*8+5*8+5*8,k,BLUE,YELLOW,(u8 *) txt,16,0);
		sprintf(txt, "u5:%c ",uart5_Res);                    //鐏忓棗褰夐柌蹇擄綖閸忓懎鍩岀€涙顑佹稉鑼畱鐎电懓绨叉担宥囩枂閿涘苯鑻熺亸鍡楃摟缁楋缚瑕嗙€涙ɑ鏂侀崚鐨寈t[]娑?
		Show_Str(2+5*8+5*8+5*8+5*8,k,BLUE,YELLOW,(u8 *) txt,16,0);
		k+=d;

		if(key1_flag == 1)
			{
				k+=d;
				Show_Str(2,k,BLUE,YELLOW,(u8 *)"key1_flag",16,0);

//				sprintf(txt, "clip:%d  ",Servo_clip_duty);                    //鐏忓棗褰夐柌蹇擄綖閸忓懎鍩岀€涙顑佹稉鑼畱鐎电懓绨叉担宥囩枂閿涘苯鑻熺亸鍡楃摟缁楋缚瑕嗙€涙ɑ鏂侀崚鐨寈t[]娑?
//				Show_Str(2,k,BLUE,YELLOW,(u8 *) txt,16,0);
//				Servo_clip_duty ++;
				
//				sprintf(txt, "turntable:%d  ",Servo_turntable_duty);                    //鐏忓棗褰夐柌蹇擄綖閸忓懎鍩岀€涙顑佹稉鑼畱鐎电懓绨叉担宥囩枂閿涘苯鑻熺亸鍡楃摟缁楋缚瑕嗙€涙ɑ鏂侀崚鐨寈t[]娑?
//				Show_Str(2,k,BLUE,YELLOW,(u8 *) txt,16,0);
//				Servo_turntable_duty ++;
				
//				sprintf(txt, "platform:%d  ",Servo_platform_duty);                    //鐏忓棗褰夐柌蹇擄綖閸忓懎鍩岀€涙顑佹稉鑼畱鐎电懓绨叉担宥囩枂閿涘苯鑻熺亸鍡楃摟缁楋缚瑕嗙€涙ɑ鏂侀崚鐨寈t[]娑?
//				Show_Str(2,k,BLUE,YELLOW,(u8 *) txt,16,0);
//				Servo_platform_duty ++;
//				Servo_turntable_duty ++;
				
				HWT_Init();//闂勨偓閾昏桨鍗庨崚婵嗩潗閸?
				Motor_parameter_Init();//閻㈠灚婧€閸欏倹鏆熼崚婵嗩潗閸?
				my_car.stop_flag = 0; //閸嬫粏婧呴弽鍥х箶娴?

				key1_flag = 0;
			}
			if(key2_flag == 1)
			{
				k+=d;
				Show_Str(2,k,BLUE,YELLOW,(u8 *)"key2_flag",16,0);
//				sprintf(txt, "clip:%d  ",Servo_clip_duty);                    //鐏忓棗褰夐柌蹇擄綖閸忓懎鍩岀€涙顑佹稉鑼畱鐎电懓绨叉担宥囩枂閿涘苯鑻熺亸鍡楃摟缁楋缚瑕嗙€涙ɑ鏂侀崚鐨寈t[]娑?
//				Show_Str(2,k,BLUE,YELLOW,(u8 *) txt,16,0);
//				Servo_clip_duty --;

//				sprintf(txt, "turntable:%d  ",Servo_turntable_duty);                    //鐏忓棗褰夐柌蹇擄綖閸忓懎鍩岀€涙顑佹稉鑼畱鐎电懓绨叉担宥囩枂閿涘苯鑻熺亸鍡楃摟缁楋缚瑕嗙€涙ɑ鏂侀崚鐨寈t[]娑?
//				Show_Str(2,k,BLUE,YELLOW,(u8 *) txt,16,0);
//				Servo_turntable_duty --;
//				
//				sprintf(txt, "platform:%d  ",Servo_platform_duty);                    //鐏忓棗褰夐柌蹇擄綖閸忓懎鍩岀€涙顑佹稉鑼畱鐎电懓绨叉担宥囩枂閿涘苯鑻熺亸鍡楃摟缁楋缚瑕嗙€涙ɑ鏂侀崚鐨寈t[]娑?
//				Show_Str(2,k,BLUE,YELLOW,(u8 *) txt,16,0);
//				Servo_platform_duty --;
//				Servo_turntable_duty --;
				
				my_car.stop_flag =!my_car.stop_flag;//閸嬫粏婧呴弽鍥х箶娴ｅ秶鐐曟潪顒婄礉閹貉冨煑鏉烇箒绶犻崥顖氫粻
//							Emm_V5_Pos_Control(USART2, 1, 8, 500, 0, 9000, false, false); //閹额剝鎹ｉ悥顏勭摍閿涘瞼娲伴崜宥勭秴缂?000
//					Emm_V5_Pos_Control(USART2, 1, 0, 500, 0, 10000, false, false); //閺€鍙ョ瑓閻栴亜鐡欓敍宀€娲伴崜宥勭秴缂?0000

				key2_flag = 0;
			}


		LED1=!LED1;
		delay_ms(1);                     //瀵よ埖妞?00ms
			
			
	}
}


void TIM7_IRQHandler(void)//10Hz,閿?.1s=100ms
{
	if (TIM_GetITStatus(TIM7, TIM_IT_Update) == SET)//濡偓閺岊櫄IM7閺囧瓨鏌婃稉顓熸焽閸欐垹鏁撶純顔荤秴
	{
				

				Timer7_count ++;//閻劋绨拋鈩冩

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
							if (bt_lift_active)
							{
								bt_lift_active = 0;
								Emm_V5_Stop_Now(USART2, 1, 0);
							}
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

				if (Servo_turntable_duty < Servo_turntable_target)
				{
					Servo_turntable_duty = (uint16_t)(Servo_turntable_duty + BT_TURNTABLE_SMOOTH_STEP);
					if (Servo_turntable_duty > Servo_turntable_target)
						Servo_turntable_duty = Servo_turntable_target;
				}
				else if (Servo_turntable_duty > Servo_turntable_target)
				{
					if (Servo_turntable_duty > BT_TURNTABLE_SMOOTH_STEP)
						Servo_turntable_duty = (uint16_t)(Servo_turntable_duty - BT_TURNTABLE_SMOOTH_STEP);
					else
						Servo_turntable_duty = Servo_turntable_target;
				}

				if (Servo_platform_duty < bt_platform_target)
				{
					Servo_platform_duty = (uint16_t)(Servo_platform_duty + BT_PLATFORM_SMOOTH_STEP);
					if (Servo_platform_duty > bt_platform_target)
						Servo_platform_duty = bt_platform_target;
				}
				else if (Servo_platform_duty > bt_platform_target)
				{
					if (Servo_platform_duty > BT_PLATFORM_SMOOTH_STEP)
						Servo_platform_duty = (uint16_t)(Servo_platform_duty - BT_PLATFORM_SMOOTH_STEP);
					else
						Servo_platform_duty = bt_platform_target;
				}

				// ========== 自动取放任务 ==========
				// 流程: 升降台向下5000 -> 爪子闭合 -> 上升5000 -> 云台到基准位 -> 下降3000 -> 爪子释放 -> 载物盘转 -> 抬升3000+云台回70
				if (flag_do_task == -4)
				{
					Emm_V5_Pos_Control(USART2, 1, 0, 500, 0, 5000, false, false);	// Step 0: 升降台向下5000
					Timer7_count = 0;
					flag_do_task = -3;
				}
				if (flag_do_task == -3 && Timer7_count >= 10)	// 等待1s(下降完成)
				{
					Servo_clip_duty = Servo_clip_hold;			// Step 1: 爪子闭合(抓取)
					bt_clip_target = Servo_clip_hold;			// 同步目标，防止平滑拉回
					Timer7_count = 0;
					flag_do_task = -2;
				}
				if (flag_do_task == -2 && Timer7_count >= 10)	// 等待1s
				{
					Emm_V5_Pos_Control(USART2, 1, 8, 500, 0, 5000, false, false);	// Step 2: 升降台上升5000
					Timer7_count = 0;
					flag_do_task = -1;
				}
				if (flag_do_task == -1 && Timer7_count >= 10)	// 等待1s(上升完成)
				{
					Servo_turntable_target = task_turntable_pos;	// Step 3: 云台转到目标位
					Timer7_count = 0;
					flag_do_task = 0;
				}
				if (flag_do_task == 0 && Timer7_count >= 25)		// 等待2.5s(云台平滑到位)
				{
					Emm_V5_Pos_Control(USART2, 1, 0, 500, 0, 3000, false, false);	// Step 4: 升降台下降3000
					Timer7_count = 0;
					flag_do_task = 1;
				}
				if (flag_do_task == 1 && Timer7_count >= 5)			// 等待0.5s(下降完成)
				{
					Servo_clip_duty = Servo_clip_loosen;						// Step 5: 爪子释放
					bt_clip_target = Servo_clip_loosen;						// 同步目标
					Timer7_count = 0;
					flag_do_task = 2;
				}
				if (flag_do_task == 2 && Timer7_count >= 10)		// 等待1s
				{
					platform_idx = (platform_idx + 1) % 3;			// 载物盘转到下一工位
					bt_platform_target = platform_pos[platform_idx];
					Timer7_count = 0;
					flag_do_task = 3;
				}
				if (flag_do_task == 3 && Timer7_count >= 20)		// 等待2s(载物盘平滑到位)
				{
					Emm_V5_Pos_Control(USART2, 1, 8, 500, 0, 3000, false, false);	// 升降台抬升3000
					Servo_turntable_target = 70;								// 云台回位(同'i'指令)
					Timer7_count = 0;
					flag_do_task = 4;
				}
				if (flag_do_task == 4 && Timer7_count >= 25)		// 等待2.5s(抬升+云台到位)
				{
					flag_do_task = -100;
					bt_drive_mode = 1;
				}

				// ========== 反向取放任务 ==========
				// 流程: 载物盘转 -> 云台转目标位 -> 降 -> 爪合 -> 升3000 -> 云台回70 -> 降5000
				if (flag_do_task == -11)
				{
					platform_idx = (platform_idx + 1) % 3;			// 载物盘转到下一工位
					bt_platform_target = platform_pos[platform_idx];
					Timer7_count = 0;
					flag_do_task = -10;
				}
				if (flag_do_task == -10 && Timer7_count >= 20)		// 等待2s(载物盘到位)
				{
					Servo_turntable_target = task_turntable_pos;	// Step 2: 云台转到目标位
					Timer7_count = 0;
					flag_do_task = -9;
				}
				if (flag_do_task == -9 && Timer7_count >= 25)	// 等待2.5s(云台到位)
				{
					Emm_V5_Pos_Control(USART2, 1, 0, 500, 0, task_reverse_down, false, false);	// Step 2: 升降台下降
					Timer7_count = 0;
					flag_do_task = -8;
				}
				if (flag_do_task == -8 && Timer7_count >= 6)	// 等待0.6s(下降完成)
				{
					Servo_clip_duty = Servo_clip_hold;			// Step 3: 爪子闭合(抓取)
					bt_clip_target = Servo_clip_hold;
					Timer7_count = 0;
					flag_do_task = -7;
				}
				if (flag_do_task == -7 && Timer7_count >= 10)	// 等待1s
				{
					Emm_V5_Pos_Control(USART2, 1, 8, 500, 0, 3000, false, false);	// Step 4: 升降台上升3000
					Timer7_count = 0;
					flag_do_task = -6;
				}
				if (flag_do_task == -6 && Timer7_count >= 6)	// 等待0.6s(上升完成)
				{
					Servo_turntable_target = 70;				// Step 5: 云台转到70(同'i'指令)
					Timer7_count = 0;
					flag_do_task = -5;
				}
				if (flag_do_task == -5 && Timer7_count >= 25)	// 等待2.5s(云台到位)
				{
					Emm_V5_Pos_Control(USART2, 1, 0, 500, 0, 5000, false, false);	// Step 6: 升降台下降5000
					Timer7_count = 0;
					flag_do_task = 4;
				}
				if (flag_do_task == 4 && Timer7_count >= 10)	// 等待1s(下降完成)
				{
					flag_do_task = -100;
					bt_drive_mode = 1;
				}

				LED3=!LED3;

		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);//濞撳懘娅嶵IMx閺囧瓨鏌婃稉顓熸焽閺嶅洤绻?
	}
}

void TIM6_IRQHandler(void)//50Hz閿?.020s=2ms娑擃厽鏌?
{
	if (TIM_GetITStatus(TIM6, TIM_IT_Update) == SET)//濡偓閺岊櫄IM6閺囧瓨鏌婃稉顓熸焽閸欐垹鏁撶純顔荤秴
	{
			Encoder_FR =  +(int16_t)TIM_GetCounter(TIM4); //閸欏啿澧犳潪顔肩摍閿涘otor_1
			Encoder_FL =  -(int16_t)TIM_GetCounter(TIM5); //瀹革箑澧犳潪顔肩摍閿涘otor_2
			Encoder_BR =  +(int16_t)TIM_GetCounter(TIM2); //閸欏啿鎮楁潪顔肩摍閿涘otor_4
			Encoder_BL =  +(int16_t)TIM_GetCounter(TIM3); //瀹革箑鎮楁潪顔肩摍閿涘otor_3
					//鐠嬪啳鐦担璺ㄦ暏
//			my_car.yaw = -90;  //瑜版挸澧犵憴鎺戝鐠у鈧?
//			YAW = -90;
//			my_car.target_yaw = -90;  //閺冨娴嗘潪锕侀煩
//			Encoder_FR =  40; //閸欏啿澧犳潪顔肩摍閿涘otor_1
//			Encoder_FL =  40; //瀹革箑澧犳潪顔肩摍閿涘otor_2
//			Encoder_BR =  40; //閸欏啿鎮楁潪顔肩摍閿涘otor_4
//			Encoder_BL =  40; //瀹革箑鎮楁潪顔肩摍閿涘otor_3
		
//			Encoder_FR =  -40; //閸欏啿澧犳潪顔肩摍閿涘otor_1
//			Encoder_FL =  40; //瀹革箑澧犳潪顔肩摍閿涘otor_2
//			Encoder_BR =  40; //閸欏啿鎮楁潪顔肩摍閿涘otor_4
//			Encoder_BL =  -40; //瀹革箑鎮楁潪顔肩摍閿涘otor_3

			TIM_SetCounter(TIM2, 0);
			TIM_SetCounter(TIM3, 0);
			TIM_SetCounter(TIM4, 0);
			TIM_SetCounter(TIM5, 0);
		
		//閸ユ稐閲滈悽鍨簚閻ㄥ嫬绱戦悳顖涘付閸掕绱濈涵顔肩暰閸楃姷鈹栧В?
		//鐎圭偟骞囬崶娑楅嚋閻㈠灚婧€閻ㄥ嫰妫撮悳顖涘付閸?闁喎瀹抽崡鏇氱秴鏉烆剚宕瞔m/s閿涘矁绶崗銉︽ЦPID閸欏倹鏆熼妴浣烘窗閺嶅洭鈧喎瀹抽妴浣哥秼閸撳秹鈧喎瀹?-閵嗗绶崙鐑樻Ц閸楃姷鈹栧В?
		//妤癸箑鍘犵痪鍐差潒鏉烆噣鈧棜绻嶉崝銊ヮ劅閿涘澑y閸ф劖鐖ｇ化浼欑礆閸戣姤鏆熷ù瀣槸,鏉堟挸鍙嗛弰鐥竫\vy\vw-閵嗗绶崙鐑樻Ц閻╊喗鐖ｉ柅鐔峰
		//妤癸箒鐤嗗锝堢箥閸斻劌顒熺憴锝囩暬閿涘苯鍤遍弫鐗堢ゴ鐠囨洩绱漬ow_VY=濮濓絻鈧苟ow_Vx=0
		//x鏉炵繝缍呯純顔惧箚閿涙俺绶崗銉ョ秼閸撳秳缍呯純顔衡偓浣烘窗閺嶅洣缍呯純顔衡偓涓矷D閸欏倹鏆熼敍宀冪翻閸戠儤妲告禒鈧稊鍫窗Vx
		//y鏉炵繝缍呯純顔惧箚閿涙俺绶崗銉ョ秼閸撳秳缍呯純顔衡偓浣烘窗閺嶅洣缍呯純顔衡偓涓矷D閸欏倹鏆熼敍宀冪翻閸戠儤妲告禒鈧稊鍫窗Vy
		//鐟欐帒瀹抽梻顓犲箚閿涙俺绶崗銉ョ秼閸撳秷顫楁惔锔衡偓浣烘窗閺嶅洩顫楁惔锔衡偓涓矷D閸欏倹鏆熼敍宀冪翻閸戠儤妲告禒鈧稊鍫窗Vw
		
			encoder_count_get();//閼惧嘲褰囩紓鏍垳閸ｃ劎娈戦崐?
			
			speed_translation(&my_car.motor_1);speed_translation(&my_car.motor_2);speed_translation(&my_car.motor_3);speed_translation(&my_car.motor_4);//鐠侊紕鐣绘潪顕€鈧?

			//鏉烇箑娼楅弽鍥叡->閸︽澘娴橀崸鎰垼鏉炵_y閵嗕箍_x閵嗕苟ow_x閵嗕苟ow_y
			RobotCalculate();//妤癸箒鐤嗗锝堢箥閸斻劌顒熺憴锝囩暬
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

	
			}
			mecanum(my_car.v_y, my_car.v_x, my_car.w);
			//鐠嬪啳鐦担璺ㄦ暏
			//Vy=濮濓絾鏆熼敍灞藉闂堛垼铔?
//			my_car.motor_1.target_speed = 40;
//			my_car.motor_2.target_speed = 40;
//			my_car.motor_3.target_speed = 40;
//			my_car.motor_4.target_speed = 40;
				//VX=濮濓絾鏆熼敍灞藉礁鏉堢铔?
//			my_car.motor_1.target_speed = -40;//閸欏啿澧犳潪顔肩摍閿涘otor_1
//			my_car.motor_2.target_speed = 40;//瀹革箑澧犳潪顔肩摍閿涘otor_2
//			my_car.motor_3.target_speed = -40;//瀹革箑鎮楁潪顔肩摍閿涘otor_3
//			my_car.motor_4.target_speed = 40;//閸欏啿鎮楁潪顔肩摍閿涘otor_4
				//W=濮濓絾鏆熼敍宀勩€庨弮鍫曟嫛鏉?
//			my_car.motor_1.target_speed = 40;//閸欏啿澧犳潪顔肩摍閿涘otor_1
//			my_car.motor_2.target_speed = 40;//瀹革箑澧犳潪顔肩摍閿涘otor_2
//			my_car.motor_3.target_speed = -40;//瀹革箑鎮楁潪顔肩摍閿涘otor_3
//			my_car.motor_4.target_speed = -40;//閸欏啿鎮楁潪顔肩摍閿涘otor_4
			IncrementalPID_Calculate(&my_car.motor_1.s_pid, my_car.motor_1.target_speed, my_car.motor_1.speed);
			IncrementalPID_Calculate(&my_car.motor_2.s_pid, my_car.motor_2.target_speed, my_car.motor_2.speed);
			IncrementalPID_Calculate(&my_car.motor_3.s_pid, my_car.motor_3.target_speed, my_car.motor_3.speed);
			IncrementalPID_Calculate(&my_car.motor_4.s_pid, my_car.motor_4.target_speed, my_car.motor_4.speed);


			if(my_car.stop_flag == 1)   //閸嬫粏婧呮径鍕倞
				{
						my_car.motor_1.s_pid.Output = 0;
						my_car.motor_2.s_pid.Output = 0;
						my_car.motor_3.s_pid.Output = 0;
						my_car.motor_4.s_pid.Output = 0;
				}
				
			my_car.motor_2.s_pid.Output = constrain_float(my_car.motor_2.s_pid.Output, -motorFL_MAX, motorFL_MAX);
			my_car.motor_1.s_pid.Output = constrain_float(my_car.motor_1.s_pid.Output, -motorFR_MAX, motorFR_MAX);
			my_car.motor_3.s_pid.Output = constrain_float(my_car.motor_3.s_pid.Output, -motorBL_MAX, motorBL_MAX);
			my_car.motor_4.s_pid.Output = constrain_float(my_car.motor_4.s_pid.Output, -motorBR_MAX, motorBR_MAX);
		
			my_car.motor_1.PWM = my_car.motor_1.s_pid.Output;
			my_car.motor_2.PWM = my_car.motor_2.s_pid.Output;
			my_car.motor_3.PWM = my_car.motor_3.s_pid.Output;
			my_car.motor_4.PWM = my_car.motor_4.s_pid.Output;			
			
			//鐠嬪啳鐦担璺ㄦ暏   Y濮濓綇绱濆鈧锕佽泲
//			my_car.motor_2.PWM = 2000;//瀹革箑澧犳潪顔肩摍閿涘otor_2
// 		  my_car.motor_1.PWM = 2000;//閸欏啿澧犳潪顔肩摍閿涘otor_1
//			my_car.motor_3.PWM = 2000;//瀹革箑鎮楁潪顔肩摍閿涘otor_3
//			my_car.motor_4.PWM = 2000;//閸欏啿鎮楁潪顔肩摍閿涘otor_4
			//閺冨娴嗘潪锕侀煩
//			my_car.motor_2.PWM = 2100;//瀹革箑澧犳潪顔肩摍閿涘otor_2
// 		  my_car.motor_1.PWM = 2100;//閸欏啿澧犳潪顔肩摍閿涘otor_1
//			my_car.motor_3.PWM = -2100;//瀹革箑鎮楁潪顔肩摍閿涘otor_3
//			my_car.motor_4.PWM = -2100;//閸欏啿鎮楁潪顔肩摍閿涘otor_4
			//X濮濓綇绱濆鈧崜宥夋桨鐠?
//			my_car.motor_2.PWM = 2100;//瀹革箑澧犳潪顔肩摍閿涘otor_2
// 		  my_car.motor_1.PWM = -2100;//閸欏啿澧犳潪顔肩摍閿涘otor_1
//			my_car.motor_3.PWM = -2100;//瀹革箑鎮楁潪顔肩摍閿涘otor_3
//			my_car.motor_4.PWM = 2100;//閸欏啿鎮楁潪顔肩摍閿涘otor_4

			my_car.motor_2.PWM = constrain_float(my_car.motor_2.PWM, -motorFL_MAX, motorFL_MAX);
			my_car.motor_1.PWM = constrain_float(my_car.motor_1.PWM, -motorFR_MAX, motorFR_MAX);
			my_car.motor_3.PWM = constrain_float(my_car.motor_3.PWM, -motorBL_MAX, motorBL_MAX);
			my_car.motor_4.PWM = constrain_float(my_car.motor_4.PWM, -motorBR_MAX, motorBR_MAX);	
				

			mortor_TB6612Set(motorFL_PWM, motorFL_IN1, motorFL_IN2, (int32_t)my_car.motor_2.PWM, motorFL_MAX, motorFL_Die);
      mortor_TB6612Set(motorFR_PWM, motorFR_IN1, motorFR_IN2, (int32_t)my_car.motor_1.PWM, motorFR_MAX, motorFR_Die);
      mortor_TB6612Set(motorBL_PWM, motorBL_IN1, motorBL_IN2, (int32_t)my_car.motor_3.PWM, motorBL_MAX, motorBL_Die);
      mortor_TB6612Set(motorBR_PWM, motorBR_IN1, motorBR_IN2, (int32_t)my_car.motor_4.PWM, motorBR_MAX, motorBR_Die);

			if (my_car.stop_flag == 1)
			{
				uint16_t brake_pins = motorFL_IN1 | motorFL_IN2 | motorFR_IN1 | motorFR_IN2 | motorBR_IN1 | motorBR_IN2 | motorBL_IN1 | motorBL_IN2;
				GPIO_SetBits(GPIOD, brake_pins);
			}

//			char txt[32];
//			sprintf(txt, "V:%f\n",my_car.motor_1.speed);                    //鐏忓棗褰夐柌蹇擄綖閸忓懎鍩岀€涙顑佹稉鑼畱鐎电懓绨叉担宥囩枂閿涘苯鑻熺亸鍡楃摟缁楋缚瑕嗙€涙ɑ鏂侀崚鐨寈t[]娑?
//			uart_write_string(UART5, (uint8_t*)txt);//閸氭垳瑕嗛崣锝呭絺闁焦鏆熼幑?
			
			Servo_clip_SetAngle(Servo_clip_duty);		//婢剁懓鐡欓懜鍨簚鐠у鈧?
			Servo_turntable_SetAngle(Servo_turntable_duty);  //鏉烆剛娲忛懜鍨簚鐠у鈧?
			Servo_platform_SetAngle(Servo_platform_duty);//鐟佸懐澧块弬娆戞畱楠炲啿褰撮懜鍨簚
			Button_Control_scan();
		LED2=!LED2;
		TIM_ClearITPendingBit(TIM6, TIM_IT_Update);//濞撳懘娅嶵IMx閺囧瓨鏌婃稉顓熸焽閺嶅洤绻?
	}
}
		
void USART1_IRQHandler(void)                	//娑撴彃褰?娑擃厽鏌囬張宥呭缁嬪绨?
	{

	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //濡偓閺岊櫄IM7閺囧瓨鏌婃稉顓熸焽閸欐垹鏁撶純顔荤秴
		{
		   uart1_Res = USART_ReceiveData(USART1);	//鐠囪褰囬幒銉︽暪閸掓壆娈戦弫鐗堝祦
			#if !BT_USE_UART5
			bt_cmd = uart1_Res;
			bt_last_rx_tick = Timer7_count;
			BT_ApplyCommand(uart1_Res);
			bt_cmd = 0;
			USART_SendData(USART1, uart1_Res);
			#endif
  		 USART_ClearITPendingBit(USART1, USART_IT_RXNE);//濞撳懘娅嶵IMx閺囧瓨鏌婃稉顓熸焽閺嶅洤绻?
     } 
} 
void USART2_IRQHandler(void)                	//娑撴彃褰?娑擃厽鏌囬張宥呭缁嬪绨?
	{

	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //濡偓閺岊櫄IM7閺囧瓨鏌婃稉顓熸焽閸欐垹鏁撶純顔荤秴
		{
		   uart2_Res = USART_ReceiveData(USART2);	//鐠囪褰囬幒銉︽暪閸掓壆娈戦弫鐗堝祦
				if (uart2_str_count < sizeof(uart2_str))
					uart2_str[uart2_str_count] = uart2_Res;
				if(uart2_str_count == uart2_str_T_count)
					{
						uart2_str_flag = 1;
						uart2_str_count = 0;
					}
				else if (uart2_str_count < sizeof(uart2_str))
					uart2_str_count++;
			
  		 USART_ClearITPendingBit(USART2, USART_IT_RXNE);

     } 

} 
unsigned char Cmd_GetPkt(unsigned char byte);

//濞夈劍鍓伴敍姘礁鏉堥€涜礋0鎺硚-180鎺抽敍灞戒箯鏉堥€涜礋180鎺硚0鎺?
uint8_t HWT_ResBuffer[22] = {0};  //闂勨偓閾昏桨鍗庨崒銊ョ摠閺佹壆绮?
unsigned char HWT_ResCnt = 0;
void USART3_IRQHandler(void)                	//娑撴彃褰?娑擃厽鏌囬張宥呭缁嬪绨?
	{

	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //濡偓閺岊櫄IM7閺囧瓨鏌婃稉顓熸焽閸欐垹鏁撶純顔荤秴
		{
		   uart3_Res = USART_ReceiveData(USART3);	//鐠囪褰囬幒銉︽暪閸掓壆娈戦弫鐗堝祦			
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
						
						my_car.yaw = YAW;  //瑜版挸澧犵憴鎺戝鐠у鈧?


						w = (short)((((short)HWT_ResBuffer[7]) << 8) | HWT_ResBuffer[6]);
						W_Z = (float)w / 32768 * 2000;
					
					HWT_ResCnt = 0;
				}
  		 USART_ClearITPendingBit(USART3, USART_IT_RXNE);//濞撳懘娅嶵IMx閺囧瓨鏌婃稉顓熸焽閺嶅洤绻?

     } 

} 

void UART4_IRQHandler(void)
{
    if (USART_GetITStatus(UART4, USART_IT_RXNE) == SET)
    {
        uart4_Res = USART_ReceiveData(UART4);
        USART_ClearITPendingBit(UART4, USART_IT_RXNE);
    }
}

// QR_codeRx_Date/100000
// (QR_codeRx_Date/10000)%10
// (QR_codeRx_Date/1000)%10
// (QR_codeRx_Date/100)%10
// (QR_codeRx_Date/10)%10
// QR_codeRx_Date%10
void UART5_IRQHandler(void)                	//娑撴彃褰?娑擃厽鏌囬張宥呭缁嬪绨?
{

  if (USART_GetITStatus(UART5, USART_IT_RXNE) == SET)//濡偓閺岊櫄IM7閺囧瓨鏌婃稉顓熸焽閸欐垹鏁撶純顔荤秴
	{
			uart5_Res = USART_ReceiveData(UART5);//鐠囪褰囬幒銉︽暪閸掓壆娈戦弫鐗堝祦
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

		  
			USART_ClearITPendingBit(UART5, USART_IT_RXNE);//濞撳懘娅嶵IMx閺囧瓨鏌婃稉顓熸焽閺嶅洤绻?
	}
}

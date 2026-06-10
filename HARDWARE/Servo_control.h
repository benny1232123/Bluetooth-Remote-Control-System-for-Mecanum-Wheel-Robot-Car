#ifndef __PWM_H
#define __PWM_H

#include "ALL_Head.h"

#define Servo_clip_hold    280   //129-137
#define Servo_clip_loosen  150 //90-103

#define Servo_turntable_front   0
#define Servo_turntable_behind  250

#define Servo_platform_one   135
#define Servo_platform_two   260
#define Servo_platform_three  10


extern uint16_t Servo_clip_duty;
extern uint16_t Servo_turntable_duty ;
extern uint16_t Servo_platform_duty;


uint16_t constrain_uint16_t(uint16_t amt, uint16_t low, uint16_t high);
void Servo_Init(void);
void Servo_clip_SetAngle(uint16_t Angle);
void Servo_platform_SetAngle(uint16_t Angle);
void Servo_turntable_SetAngle(uint16_t Angle);

#endif // !__PWM_H

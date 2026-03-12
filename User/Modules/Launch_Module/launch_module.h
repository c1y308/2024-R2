#ifndef BALL_TASK_H
#define BALL_TASK_H
#include "main.h"
#include "chassis_module.h"
#include "dji_motor.h"
#include "can_trx.h"

void magnet_control(void);
void launch_motor_speed_normal(void);
void launch_motor_speed_change(uint16_t storm_motor_speed);
#endif

#include "launch_module.h"
#include "stdlib.h"
#include "user_config.h"

DjiMotorHandle_t *motor2006_front, *motor2006_back, *motor3508_lift;
void launch_motor_init()
{
    MotorInitConfig_t chassis_motor_config = {
        .can_init_config.can_handle = &hcan1,
        .controller_init_config = {
			.angle_PID = {
				.Kp = 0.01,
				.Ki = 0.007,
				.Kd = 0.0f,
			},
            .speed_PID = {
                .Kp = 10000.0f,
                .Ki = 10.0f,
                .Kd = 0.0f,
                .IntegralLimit = M3508_MOTOR_SPEED_PID_IOUT_LIMIT,
                .MaxOut = M3508_MOTOR_SPEED_PID_POUT_LIMIT,
            },
			.outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
			.motor_direction = MOTOR_DIRECTION_REVERSE,
        },
		.motor_id = 0,
        .motor_type = DJI_MOTOR_2006,
    };

    chassis_motor_config.motor_id = 4;
    motor2006_front = djimotor_init(&chassis_motor_config);

    chassis_motor_config.motor_id = 5;
    motor2006_back = djimotor_init(&chassis_motor_config);

    chassis_motor_config.motor_id = 6;
    motor3508_lift = djimotor_init(&chassis_motor_config);
}


void launch_motor_speed_normal(){
	dji_motor_setref(motor2006_front, M2006_GET_BALL_SPEED);
	dji_motor_setref(motor2006_back,  M2006_GET_BALL_SPEED);
	dji_motor_setref(motor3508_lift, 0);
}


void launch_motor_speed_change(uint16_t storm_motor_speed){
	dji_motor_setref(motor3508_lift, -0.9);
	set_pwm_motor_speed(&hcan1, storm_motor_speed, 0, 0, 0);
}


void magnet_control()
{
	static uint8_t tick = 0;
	tick++;
	if(tick <= 60)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
	}
	else
	{
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
	}
}

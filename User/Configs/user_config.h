#ifndef USER_CONFIG_H
#define USER_CONFIG_H
#include "main.h"
#include "chassis_module.h"

#define CHOICE_MOO  get_seed_type

extern float RUN_S;
#define LOW_SPEED 0.2//
#define LOW_DES_COM 700  // 开始减速的距离
#define INIT_OUT_TICK 70//

#define SEED_FORWARD_DES 330//

#define CHANGE_LINE_TICK 200//tick

/**********************************************************************/
#define GRAP_TICK_CLOSE 60//
#define GRAP_SPEED  -10//
#define GRAP_TICK_OPEN 60//
#define GRAP_SPEED_OPEN  10//

#define SPEED_LIFT_PLUS -6//
#define SPEED_ANGLE_PLUS -3//
#define GRAP_LIFT_TICK_PRE 120//
#define CAUTION_TICK 10//
#define CONTINUE_STORAGE_TICK 125//
#define GRAP_LIFT_TICK_AFT 160//
#define START_DOWN_COM_TICK 70//
#define GRAP_LIFT_TICK_PRE_COM 100//

#define GRAP_TICK_TOTAL_COM GRAP_LIFT_TICK_PRE+GRAP_LIFT_TICK_AFT-GRAP_FAST_TICK//
#define GRAP_FAST_TICK 160//

//-5.5
#define GRAP_LIFT_SPEED  -5//  4.5
#define GRAP_ANGLE_SPEED -2.5//
#define LOW_LIFT_SPEED   -2.5//2
#define LOW_ANGLE_SPEED  -2//
#define SAFE_GRAP_SPEED -0.001//

#define TRANS_TICK_F 260//
#define TRANS_TICK_S 410//
#define CRACK_SECOND_TICK 230//
#define CRACK_SPEED_SECOND 0.5//
#define CORRECT_DISTACNE 2320//

#define M2006_GET_BALL_SPEED 6
#define M3508_GET_BALL_SPEED -1.6
#define REMAKE_SPEEDX -0.41//
#define REMAKE_TX 300//
#define REMAKE_SPEEDY 0.9//
#define REMAKE_TY 840//

#define GET_ANGLE_SINGLE 1343//
#define PUT_ANGLE_SINGLE  0//


#define CHECK_TICK_SMALL 1
#define CHECK_TICK_BIG 20
#define CHASSIS_TOL_SMALL 9
#define CHASSIS_TOL_BIG 15

extern float crack_posY;//Y
extern float seed_pos_x[12];//X 
extern float seed_pos_x_single[12];//X
extern float put_pos_x_single[12];//X
extern float put_pos_x[6];//X
extern float put_pos_y[2];//Y
extern float put_pos_y_single[2];
extern float pos_get_special[2];//

extern float ball_pos_x[13];
extern float ball_pos_y[2];
extern float ball_launch_angle[13];
extern uint16_t ball_launch_speed[13];
extern uint16_t  line_change_tick_left[13];
extern uint16_t  line_change_tick_right[13];
extern float pos_get_1;// Y
extern float pos_get_2;//Y
extern float change_line_x_left;//
extern float change_line_x_right;//	

extern float real_lv100;
extern float manual_vx;
extern float manual_vy;
extern float manual_wz;
#endif

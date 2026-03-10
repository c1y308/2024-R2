#ifndef USER_LIB_H
#define USER_LIB_H

#include "main.h"
typedef float fp32;

typedef struct
{
    fp32 input;        //
    fp32 out;          //
    fp32 min_value;    //޷Сֵ
    fp32 max_value;    //޷ֵ
    fp32 frame_period; //ʱ
} ramp_function_source_t;

typedef struct
{
    fp32 input;        //
    fp32 out;          //˲
    fp32 num[1];       //˲
    fp32 frame_period; //˲ʱ λ s
} first_order_filter_type_t;

extern first_order_filter_type_t filter_chassis_vx;
extern first_order_filter_type_t filter_chassis_vy;
extern first_order_filter_type_t filter_chassis_vz;


extern fp32 invSqrt(fp32 num);


void ramp_init(ramp_function_source_t *ramp_source_type, fp32 frame_period, fp32 max, fp32 min);


void ramp_calc(ramp_function_source_t *ramp_source_type,fp32 input);

extern void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, const fp32 num);

extern void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input);

extern void abs_limit(fp32 *num, fp32 Limit);

extern int8_t sign(fp32 value);

extern float rc_dead_band_limit(float input, float dead_line);

extern fp32 float_deadline(fp32 Value, fp32 minValue, fp32 maxValue);

extern int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue);

extern fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue);

extern int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);

extern fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue);

extern fp32 theta_format(fp32 Ang);

#define rad_format(Ang) loop_fp32_constrain((Ang), -PI, PI)
#define ANGLE_TO_RADIAN (0.01745f)//Ƕת
#define RADIAN_TO_ANGLE (57.29577f)//ת
#endif

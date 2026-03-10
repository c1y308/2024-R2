#include "user_lib.h"
#include "main.h"
#include "chassis_module.h"
first_order_filter_type_t filter_chassis_vx;
first_order_filter_type_t filter_chassis_vy;
first_order_filter_type_t filter_chassis_vz;

first_order_filter_type_t filter_launch_m2006_v;

ramp_function_source_t filter_speed_limit_vx;
ramp_function_source_t filter_speed_limit_vy;
//ٿ
float invSqrt(float num)
{
    float halfnum = 0.5f * num;
    float y = num;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfnum * y * y));
    return y;
}

/**
  * @brief          бʼ
  * @author         RM
  * @param[in]      бṹ
  * @param[in]      ʱ䣬λ s
  * @param[in]      ֵ
  * @param[in]      Сֵ
  * @retval         ؿ
  */
void ramp_init(ramp_function_source_t *ramp_source_type, float frame_period, float max, float min)
{
    ramp_source_type->frame_period = frame_period;
    ramp_source_type->max_value = max;
    ramp_source_type->min_value = min;
    ramp_source_type->input = 0.0f;
    ramp_source_type->out = 0.0f;
}

/**
  * @brief          б㣬ֵеӣ 뵥λΪ /s һֵ
  * @author         RM
  * @param[in]      бṹ
  * @param[in]      ֵ
  * @param[in]      ˲
  * @retval         ؿ
  */
void ramp_calc(ramp_function_source_t *ramp_source_type, float input)
{
    ramp_source_type->input = input;
    ramp_source_type->out += ramp_source_type->input * ramp_source_type->frame_period;
    if (ramp_source_type->out > ramp_source_type->max_value)
    {
        ramp_source_type->out = ramp_source_type->max_value;
    }
    else if (ramp_source_type->out < ramp_source_type->min_value)
    {
        ramp_source_type->out = ramp_source_type->min_value;
    }
}
/**
  * @brief          һ׵ͨ˲ʼ
  * @author         RM
  * @param[in]      һ׵ͨ˲ṹ
  * @param[in]      ʱ䣬λ s
  * @param[in]      ˲
  * @retval         ؿ
  */
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float frame_period, const float num)
{
    first_order_filter_type->frame_period = frame_period;
    first_order_filter_type->num[0] = num;
    first_order_filter_type->input = 0.0f;
    first_order_filter_type->out = 0.0f;
}

/**
  * @brief          һ׵ͨ˲
  * @author         RM
  * @param[in]      һ׵ͨ˲ṹ
  * @param[in]      ʱ䣬λ s
  * @retval         ؿ
  */
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, float input)
{
    first_order_filter_type->input = input;
    first_order_filter_type->out =
    first_order_filter_type->num[0] / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->out + first_order_filter_type->frame_period / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->input;
}

//
void abs_limit(float *num, float Limit)
{
    if (*num > Limit)
    {
        *num = Limit;
    }
    else if (*num < -Limit)
    {
        *num = -Limit;
    }
}

//жϷλ
int8_t sign(float value)
{
    if (value >= 0.0f)
    {
        return 1.0f;
    }
    else
    {
        return -1.0f;
    }
}
float rc_dead_band_limit(float input, float dead_line)       \
{                                                            \
    if ((input > dead_line) || (input < -dead_line))         \
        return input;                                        \
    else                                                     \
        return 0;                                            \
}
//
float float_deadline(float Value, float minValue, float maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0.0f;
    }
    return Value;
}

//int26
int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0;
    }
    return Value;
}

//޷
float float_constrain(float Value, float minValue, float maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

//޷
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

//ѭ޷
float loop_float_constrain(float Input, float minValue, float maxValue)
{
    if (maxValue < minValue)
    {
        return Input;
    }

    if (Input > maxValue)
    {
        float len = maxValue - minValue;
        while (Input > maxValue)
        {
            Input -= len;
        }
    }
    else if (Input < minValue)
    {
        float len = maxValue - minValue;
        while (Input < minValue)
        {
            Input += len;
        }
    }
    return Input;
}

//ȸʽΪ-PI~PI

//ǶȸʽΪ-180~180
float theta_format(float Ang)
{
    return loop_float_constrain(Ang, -180.0f, 180.0f);
}


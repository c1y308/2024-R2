#include "bsp_dt35.h"

// DT35数据进行了取反操作，统一方向
// 绝对方向向前的DT35数据计算是反向的

//框数据
int dt35_place_point[5][2] = 
{
    {-80,-688},
    {-80,-1922},
    {-80,-3226},
    {-80,-4531},
    {-80,-5793}
};



//最终数据
int dt35_right_result = 0;
int dt35_left_result = 0;
int dt35_rightfront_result = 0;
int dt35_leftfront_result = 0;

//原始数据数组
int dt35_right[DT35_SAMPLE_NUM+1] = {0};
int dt35_left[DT35_SAMPLE_NUM+1] = {0};
int dt35_rightfront[DT35_SAMPLE_NUM+1] = {0};
int dt35_leftfront[DT35_SAMPLE_NUM+1] = {0};

void DT35DataUpdate()
{
    float adc_data[8] = {0};
    AD7689process(adc_data);
    //数据处理
    //新数据入队
    dt35_left[DT35_SAMPLE_NUM] = adc_data[0];
    dt35_leftfront[DT35_SAMPLE_NUM] = adc_data[1];
    dt35_right[DT35_SAMPLE_NUM] = adc_data[2];
    dt35_rightfront[DT35_SAMPLE_NUM] = adc_data[3];
    for(int i = 0; i < DT35_SAMPLE_NUM; i++)
    {
        dt35_right[i] = dt35_right[i+1];
        dt35_left[i] = dt35_left[i+1];
        dt35_rightfront[i] = dt35_rightfront[i+1];
        dt35_leftfront[i] = dt35_leftfront[i+1];
    }
    Mean_filter((int*)dt35_left,DT35_SAMPLE_NUM,&dt35_left_result);
    Mean_filter((int*)dt35_right,DT35_SAMPLE_NUM,&dt35_right_result);
    Mean_filter((int*)dt35_leftfront,DT35_SAMPLE_NUM,&dt35_leftfront_result);
    Mean_filter((int*)dt35_rightfront,DT35_SAMPLE_NUM,&dt35_rightfront_result);

        
}

#include "filter.h"


//均值滤波
void Mean_filter(int* rawdata,uint16_t len,int* result)
{
		arm_mean_q31(rawdata,len,result);
}

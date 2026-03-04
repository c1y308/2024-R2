#ifndef __BSPFILTER_H
#define __BSPFILTER_H

#ifdef __cplusplus
extern "C"{
#endif
#include "arm_math.h"
#include "stdint.h"
#include "main.h"



void Mean_filter(int* rawdata,uint16_t len,int* result);

#ifdef __cplusplus
}
#endif

#endif



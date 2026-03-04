#ifndef _BSPDT35_H
#define _BSPDT35_H

#ifdef __cplusplus
extern "C"{
#endif
#include "bsp_AD7689.h"
#include "filter.h"
    
    
//数组数量
#define DT35_SAMPLE_NUM 10
    //一区到二区巡左线距离
#define LEFT_NAVI_DIS -700 
    //二区到三区坡前前向距离
#define LF_NAVI_DIS -600
    //二区到三区上坡时侧边距离
#define LEFT_223 -7200
    //到达三区开始旋转抓球时，前向距离
#define LF_START_GRAP -3500


extern int dt35_right_result;
extern int dt35_left_result;
extern int dt35_rightfront_result;
extern int dt35_leftfront_result;

void DT35DataUpdate();
#ifdef __cplusplus
}

#endif
#endif

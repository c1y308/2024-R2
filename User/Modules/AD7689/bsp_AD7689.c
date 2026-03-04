/* 包含头文件 ----------------------------------------------------------------*/
#include "bsp_AD7689.h"

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

#define FILTER4_N 3
/* 私有变量 ------------------------------------------------------------------*/
SPI_HandleTypeDef hspi_AD7689;
static uint8_t  registerWord[2];
static  unsigned char buf[2] ={0,0};


/**
  * 函数功能: 简单延时
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 该函数被HAL库内部调用
*/
static void AD7689_Delay(uint16_t time)
{
	while(time-->0);
}

/**
  * 函数功能: 获取AD7689数值
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 该函数被HAL库内部调用
*/
uint16_t AD7689_Get_Data(uint16_t cmd)
{

  registerWord[0] = cmd>>8;
  registerWord[1] = cmd;
  AD7689_Delay(100);

  /* 片选使能 */
  AD7689_CS_LOW();
 
	HAL_SPI_TransmitReceive(&hspi1,registerWord,buf,2,0x04);

  AD7689_CS_HIGH();
    
	return ((buf[0]<<8) | buf[1]);
}

    uint16_t Adc_data1[8];
//data process
void AD7689process(float* result)
{
    
    __IO int32_t bias_data[8];               // 零点电压的AD转换结果
    
    bias_data[0]=BIAS_VOLTAGE_IN0;
    bias_data[1]=BIAS_VOLTAGE_IN1;
    bias_data[2]=BIAS_VOLTAGE_IN2;
    bias_data[3]=BIAS_VOLTAGE_IN3;
    bias_data[4]=BIAS_VOLTAGE_IN4;
    bias_data[5]=BIAS_VOLTAGE_IN5;
    
    __IO float  voltage_data[8];            // 电压值（单位：mV＿
    __IO float  current_data[8];            // 电流值（单位：uA＿
        
    static uint16_t     IN_DAT[8] = 
    {
        (0x3C49 << 2),                   // 单极性，全带宽，内部基准4.096V，禁用鿚道序列器，不回读CFG F124
        (0x3CC9 << 2),                   //      F324
        (0x3D49 << 2),                   //    f524
        (0x3DC9 << 2),                   //    f724
        (0x3E49 << 2),                   //    f924
        (0x3EC9 << 2),                   //      FB24      
        (0x3F49 << 2),                   //      FD24
        (0x3FC9 << 2)                   //      FF24
    };
	
    for(int i=0;i<8;i++)
    { 						
        uint16_t ad7689_cfg[8] = {IN_DAT[2],IN_DAT[3],IN_DAT[4],IN_DAT[5],IN_DAT[6],IN_DAT[7],IN_DAT[0],IN_DAT[1]};
        Adc_data1[i]=AD7689_Get_Data(ad7689_cfg[i]);			/* 发鿁读取AD数据指令 */			
        voltage_data[i]=(Adc_data1[i]-bias_data[i])*REFERENCE_VOLTAGE/OPA_RES_R2*OPA_RES_R1/0xFFFF;
        result[i] = voltage_data[i];
    }
}


/********** (C) COPYRIGHT 2019-2030 硬石嵌入式开发团队 *******END OF FILE************/


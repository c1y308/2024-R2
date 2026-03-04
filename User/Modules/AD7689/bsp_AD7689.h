#ifndef __AD7689_H
#define __AD7689_H

#ifdef __cplusplus
extern "C"{
#endif
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
     
#define AD7689_CS_HIGH()           HAL_GPIO_WritePin(AD7689_CS_GPIO_Port,AD7689_CS_Pin,GPIO_PIN_SET);
#define AD7689_CS_LOW()            HAL_GPIO_WritePin(AD7689_CS_GPIO_Port,AD7689_CS_Pin,GPIO_PIN_RESET);

#define AD7689_Speed            10000   // 采集模块获取速率，忼越大越慿
                                      

#define OPA_RES_R1              6800  // 6.8k 运放输入端电阿
#define OPA_RES_R2              2400  // 2k 运放反馈电阻
#define REFERENCE_VOLTAGE       4095  // 参迃电压（放大1000倍）
#define SAMPLE_RESISTANCE       150   // 电流采样电阻


#define BIAS_VOLTAGE_IN0        0x102F  // 输入0偏置电压，即把IN0和GND短接时AD7689转换结果
#define BIAS_VOLTAGE_IN1        0x102F  // 输入1偏置电压，即把IN1和GND短接时AD7689转换结果
#define BIAS_VOLTAGE_IN2        0x102F  // 输入2偏置电压，即把IN2和GND短接时AD7689转换结果
#define BIAS_VOLTAGE_IN3        0x102F  // 输入3偏置电压，即把IN3和GND短接时AD7689转换结果
#define BIAS_VOLTAGE_IN4        0x102F  // 输入4偏置电压，即把IN4和GND短接时AD7689转换结果
#define BIAS_VOLTAGE_IN5        0x102F  // 输入5偏置电压，即把IN5和GND短接时AD7689转换结果

/* AD7689 Register Map */
//#define IN0             (0x3c49 << 2)           // �����ԣ�ȫ�������ڲ���׼4.096V������ͨ�������������ض�CFG
//#define IN1             (0x3cc9 << 2)           
//#define IN2             (0x3d49 << 2)  
//#define IN3             (0x3dc9 << 2)  

#define Chanal                8                      // channel num  
  
extern SPI_HandleTypeDef hspi_AD7689;

void YS_AD7689_SPI_Init(void);
uint16_t AD7689_Get_Data(uint16_t cmd);

void AD7689process(float* result);
#ifdef __cplusplus
}

#endif
#endif  /* __BSP_SPIWEIGHT_H__ */

/******************* (C) COPYRIGHT 2019-2030 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/

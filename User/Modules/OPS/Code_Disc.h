#ifndef __CODE_DISC_H__
#define __CODE_DISC_H__

#include "ring_buffer.h"
// #define Code_Disc_on   //表示有码盘
#define Code_Disc_Offset//码盘安装位置不处于中心点
#define CD_UART_PORT huart3
#define ANG2RAD 0.01745329252f


#define CD_HEADER_1     0x0D
#define CD_HEADER_2     0x0A  
#define CD_FOOTER_1     0x0A
#define CD_FOOTER_2     0x0D
#define CD_PACKET_SIZE  28      // 整个数据包大小
#define CD_DATA_SIZE    24      // 数据部分大小


typedef struct
{
  float yaw;                //航向角
  float pitch;              //俯仰角
  float roll;               //翻滚角
  float x;                  //坐标x
  float y;                  //坐标y
  float pal_yaw;            //航向角速度
  uint32_t err;             //数据包错误次数
  uint8_t Ready_flag;       //初始化完成标志

} CodeDiscMeaser_t;//码盘原始信息


typedef struct
{
  float yaw_rad;                //航向角
  float pitch_rad;              //俯仰角
  float roll_rad;               //翻滚角
  float x;                      //坐标x
  float y;                      //坐标y
  float pal_yaw_rad;            //航向角速度
} CodeDiscRobotMeaser_t;        //由码盘信息转换得到的机器人中心位置信息，坐标轴方向与码盘坐标轴方向相同


typedef struct
{
  CodeDiscMeaser_t measer;
  CodeDiscRobotMeaser_t robot_measer;

  uint16_t err;
  uint16_t ok;
}CodeDisc_t;


extern CodeDiscMeaser_t Code_Disc;
extern CodeDiscRobotMeaser_t Code_Disc_robot;


/**
 * @brief 码盘坐标转换机器坐标（中心）
 * 
 * @param _Code_Disc_robot_ptr 机器人坐标指针
 * @param _Code_Disc 码盘原始信息
 */
void  code_disc_coodinate2robot_coodinate(CodeDiscRobotMeaser_t *_code_disc_robot_ptr, CodeDiscMeaser_t _code_disc);
void  CD_get_measer(uint8_t *pdata, uint8_t length);
void  CD_SET0(UART_HandleTypeDef *usart);


typedef union {
    float _float;
    char _char[4];
} float2char_u;

void CD_SETX(UART_HandleTypeDef *_usart, float _x);
void CD_SETZ(UART_HandleTypeDef *_usart, float _z);
void CD_SETY(UART_HandleTypeDef *_usart, float _y);
void CD_SETALL(UART_HandleTypeDef *_usart, float _x, float _y, float _z);

#endif

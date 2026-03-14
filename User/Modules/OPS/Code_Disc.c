#include "Code_Disc.h"
#include "string.h"
#include "math.h"

// Code_Disc_measer Code_Disc;
// Code_Disc_robot_measer Code_Disc_robot;

CodeDisc_t code_disc;
static RingBuffer_t ring_buf;


void CD_get_measer_ring_buffer(uint8_t *new_data, uint16_t length)
{
    if (new_data == NULL || length == 0) {
        return;
    }
    
    // 把所有新数据写到环形缓冲区
    uint16_t written = ring_buffer_write(&ring_buf, new_data, length);

    if (written < length) {
        Code_Disc.err++;
        // 可以选择丢弃最旧的一些数据来腾出空间
        if (ring_buffer_is_full(&ring_buf)) {
            ring_buffer_consume(&ring_buf, CD_PACKET_SIZE);
            // 重新尝试写入
            ring_buffer_write(&ring_buf, new_data + written, length - written);
        }
    }
    
    // 在环形缓冲区中搜索完整的数据包
    uint16_t packet_positions[5];
    int found_count = ring_buffer_find_packets(&ring_buf, packet_positions, 5);  // 从tail开始搜索
    
    // 处理找到的所有数据包
    for (int i = 0; i < found_count; i++) {
        uint8_t packet_data[CD_PACKET_SIZE];
        
        // 读取完整数据包
        if (ring_buffer_peek_multiple(&ring_buf, packet_data, CD_PACKET_SIZE, packet_positions[i]) == CD_PACKET_SIZE) {
            process_packet(packet_data);
        }
    }
    
    // 消费已处理的数据（从开始到最后一个包的末尾）
    if (found_count > 0) {
        uint16_t consume_length = packet_positions[found_count - 1] + CD_PACKET_SIZE;
        ring_buffer_consume(&ring_buf, consume_length);
    } else {
        // 如果没有找到完整包，但缓冲区已满，丢弃最旧的数据
        if (ring_buffer_is_full(&ring_buf)) {
            ring_buffer_consume(&ring_buf, 1); // 丢弃一个字节，腾出空间
        }
    }
}


void process_packet(const uint8_t *packet_data)
{
    static float last_yaw = 0;
    last_yaw = code_disc.measer.yaw;

    memcpy(&code_disc.measer, &packet_data[2], CD_DATA_SIZE);
    code_disc.ok++;
    
    // Yaw角度处理
    if ((last_yaw - Code_Disc.yaw) >= 180) {
        Code_Disc.yaw = 360 + Code_Disc.yaw;
    } else if ((last_yaw - Code_Disc.yaw) < -180) {
        Code_Disc.yaw = -(360 - Code_Disc.yaw);
    }
    last_yaw = Code_Disc.yaw;
    
    // 坐标转换
    code_disc_coodinate2robot_coodinate(&Code_Disc_robot, Code_Disc);
}


/**
  * @brief          清零码盘数据
  * @param[in]      串口句柄
  * @retval         none
  */
void CD_SET0(UART_HandleTypeDef *usart)
{
    HAL_UART_Transmit(usart, (uint8_t *)"ACT0", 4, 0xFF);
}


/**
 * @brief
 * @param _code_disc_robot_ptr
 * @param _code_disc
 */
void code_disc_coodinate2robot_coodinate(CodeDiscRobotMeaser_t *_code_disc_robot_ptr, CodeDiscMeaser_t _code_disc)
{
    const float OFFSET_X = 0; //以车中心为原点，码盘x正方向，码盘安装位置，单位mm
    const float OFFSET_Y = 190;//以车中心为原点，码盘y正方向，码盘安装位置，单位mm
    _code_disc_robot_ptr->pal_yaw_rad = _code_disc.pal_yaw * ANG2RAD;
    _code_disc_robot_ptr->yaw_rad = _code_disc.yaw * ANG2RAD;
//    _code_disc_robot_ptr->pitch = _code_disc.pitch;
//    _code_disc_robot_ptr->roll = _code_disc.roll;
    _code_disc_robot_ptr->y = _code_disc.x - OFFSET_X * cosf(_code_disc_robot_ptr->yaw_rad) +
                              OFFSET_Y * sinf(_code_disc_robot_ptr->yaw_rad) + OFFSET_X;
    _code_disc_robot_ptr->x = -(_code_disc.y - OFFSET_X * sinf(_code_disc_robot_ptr->yaw_rad) -
                              OFFSET_Y * cosf(_code_disc_robot_ptr->yaw_rad) + OFFSET_Y);
}


void CD_SETY(UART_HandleTypeDef *_usart, float _y) {
    static char send_string[8] = "ACTY";
    static float2char_u y;
    y._float = _y;
    for (int i = 0; i < 4; i++) {
        send_string[4 + i] = y._char[i];
    }
    HAL_UART_Transmit(_usart, (uint8_t *) send_string, 8, 0x0f);
}


void CD_SETZ(UART_HandleTypeDef *_usart, float _z) {
    static char send_string[8] = "ACTJ";
    static float2char_u z;
    z._float = _z;
    for (int i = 0; i < 4; i++) {
        send_string[4 + i] = z._char[i];
    }
    HAL_UART_Transmit(_usart, (uint8_t *) send_string, 8, 0x0f);
}


void CD_SETX(UART_HandleTypeDef *_usart, float _x) {
    static char send_string[8] = "ACTX";
    static float2char_u x;
    x._float = _x;
    for (int i = 0; i < 4; i++) {
        send_string[4 + i] = x._char[i];
    }
    HAL_UART_Transmit(_usart, (uint8_t *) send_string, 8, 0x0f);
}


void CD_SETALL(UART_HandleTypeDef *_usart,float _x,float _y, float _z) {
    static char send_string[16] = "ACTD";
	  static float2char_u x;
  	static float2char_u y;
    static float2char_u z;
  	z._float = _z;
	  x._float = _x;
    y._float = _y;
    for (int i = 0; i < 4; i++) {
        send_string[4 + i] = z._char[i];
    }
		for (int i = 0; i < 4; i++) {
        send_string[8 + i] = x._char[i];
    }
		for (int i = 0; i < 4; i++) {
        send_string[12 + i] = y._char[i];
    }
    HAL_UART_Transmit(_usart, (uint8_t *) send_string, 16, 0x0f);
}

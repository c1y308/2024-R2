#include "ring_buffer.h"

// 初始化环形缓冲区
void ring_buffer_init(RingBuffer_t *rb)
{
    memset(rb->buffer, 0, RING_BUFFER_SIZE);
    rb->writeto = 0;
    rb->readfrom = 0;
    rb->count = 0;
}


// 检查环形缓冲区是否为空
uint8_t ring_buffer_is_empty(const RingBuffer_t *rb)
{
    return (rb->count == 0);
}


// 检查环形缓冲区是否已满
uint8_t ring_buffer_is_full(const RingBuffer_t *rb)
{
    return (rb->count == RING_BUFFER_SIZE);
}


// 获取环形缓冲区中数据数量
uint16_t ring_buffer_count(const RingBuffer_t *rb)
{
    return rb->count;
}


// 获取环形缓冲区空闲空间
uint16_t ring_buffer_free_space(const RingBuffer_t *rb)
{
    return RING_BUFFER_SIZE - rb->count;
}


// 向环形缓冲区写入数据
uint16_t ring_buffer_write(RingBuffer_t *rb, const uint8_t *data, uint16_t length)
{
    uint16_t written = 0;
    
    for (uint16_t i = 0; i < length && !ring_buffer_is_full(rb); i++) {
        rb->buffer[rb->writeto] = data[i];
        rb->writeto = (rb->writeto + 1) & RING_BUFFER_SIZE;  // 超出后回到缓冲区开头
        rb->count++;
        written++;
    }
    
    return written;
}


// 从环形缓冲区消费数据（移动读指针）
void ring_buffer_consume(RingBuffer_t *rb, uint16_t length)
{
    if (length > rb->count) {  // 尝试消费超过缓冲区数据量的情况
        length = rb->count;
    }
    
    rb->readfrom = (rb->readfrom + length) & RING_BUFFER_SIZE;
    rb->count -= length;
}


// 从环形缓冲区读取一个字节（不移动读指针）
uint8_t ring_buffer_peek(const RingBuffer_t *rb, uint16_t offset)
{
    if (offset >= rb->count) {
        return 0;
    }
    
    uint16_t index = (rb->readfrom + offset) & RING_BUFFER_SIZE;
    return rb->buffer[index];
}


// 从环形缓冲区读取多个字节（不移动读指针）
uint16_t ring_buffer_peek_multiple(const RingBuffer_t *rb, uint8_t *data, uint16_t length, uint16_t offset)
{
    if (offset >= rb->count) {
        return 0;
    }
    
    uint16_t to_read = (length < (rb->count - offset)) ? length : (rb->count - offset);
    uint16_t read_count = 0;
    
    for (uint16_t i = 0; i < to_read; i++) {
        uint16_t index = (rb->readfrom + offset + i) & RING_BUFFER_SIZE;
        data[i] = rb->buffer[index];
        read_count++;
    }
    
    return read_count;
}


// 在环形缓冲区中搜索完整的数据包
// 返回找到的包数量，并通过参数返回包的位置
int ring_buffer_find_packets(RingBuffer_t *rb, uint16_t *packet_positions, int max_packets)
{
    int found_packets = 0;
    uint16_t search_pos = 0;
    
    // 搜索所有可能的位置
    while (search_pos <= rb->count - CD_PACKET_SIZE && found_packets < max_packets) {
        // 检查包头
        if (ring_buffer_peek(rb, search_pos) == CD_HEADER_1 && 
            ring_buffer_peek(rb, search_pos + 1) == CD_HEADER_2) {
            // 检查包尾
            if (ring_buffer_peek(rb, search_pos + 26) == CD_FOOTER_1 && 
                ring_buffer_peek(rb, search_pos + 27) == CD_FOOTER_2) {
                // 找到了完整数据包
                packet_positions[found_packets] = search_pos;
                found_packets++;
                search_pos += CD_PACKET_SIZE; // 跳到下一个可能的位置
                continue;
            }
        }
        search_pos++; // 继续搜索
    }
    
    return found_packets;
}

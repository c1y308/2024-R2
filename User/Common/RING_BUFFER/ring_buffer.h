#ifndef RING_BUFFER_H
#define RING_BUFFER_H
#include <stdint.h>

#define RING_BUFFER_SIZE 256

#define CD_HEADER_1     0x0D
#define CD_HEADER_2     0x0A  
#define CD_FOOTER_1     0x0A
#define CD_FOOTER_2     0x0D
#define CD_PACKET_SIZE  28      // 整个数据包大小
#define CD_DATA_SIZE    24      // 数据部分大小

typedef struct
{
  uint8_t  buffer[RING_BUFFER_SIZE];
  uint16_t readfrom;
  uint16_t writeto;
  uint16_t count;
} RingBuffer_t;


void    ring_buffer_init(RingBuffer_t *rb);

uint8_t ring_buffer_is_empty(const RingBuffer_t *rb);
uint8_t ring_buffer_is_full(const RingBuffer_t *rb);

uint16_t ring_buffer_count(const RingBuffer_t *rb);
uint16_t ring_buffer_free_space(const RingBuffer_t *rb);

uint16_t ring_buffer_write(RingBuffer_t *rb, const uint8_t *data, uint16_t length);
void     ring_buffer_consume(RingBuffer_t *rb, uint16_t length);
uint8_t  ring_buffer_peek(const RingBuffer_t *rb, uint16_t offset);
uint16_t ring_buffer_peek_multiple(const RingBuffer_t *rb, uint8_t *data, uint16_t length, uint16_t offset);

int      ring_buffer_find_packets(RingBuffer_t *rb, uint16_t *packet_positions, int max_packets);
#endif // RING_BUFFER_H

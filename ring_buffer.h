
#ifndef _ring_buffer_h_
#define _ring_buffer_h_

#include <stdint.h>

typedef struct _ring_buffer {
  uint8_t* storage;
  uint8_t* end;
  uint16_t size;
  uint8_t* read;
  uint8_t* write;
  uint16_t available;
} ring_buffer;

void ring_buffer_init(ring_buffer* ring, uint8_t* storage, uint16_t size);
void ring_buffer_read(ring_buffer* ring, uint8_t* buffer, uint16_t size);
uint8_t ring_buffer_read_byte(ring_buffer* ring);
void ring_buffer_write(ring_buffer* ring, const uint8_t* buffer, uint16_t size);
void ring_buffer_write_byte(ring_buffer* ring, uint8_t b);
uint16_t ring_buffer_readline(ring_buffer* ring, char* buffer, uint16_t size);
uint8_t ring_buffer_peekn(ring_buffer* ring, uint16_t i);

#endif

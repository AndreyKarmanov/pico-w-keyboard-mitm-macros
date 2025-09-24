#ifndef RING_BUFFER_UTILS_H
#define RING_BUFFER_UTILS_H

#include <stdint.h>
#include <stdbool.h>

#define REPORT_RING_BUFFER_SIZE 260 // capacity for multiple small items
#define MAX_REPORT_LEN 64

typedef struct {
    uint8_t buffer[REPORT_RING_BUFFER_SIZE];
    uint16_t head;
    uint16_t tail;
    uint16_t used_bytes;
    uint16_t item_count;
} ring_buffer_t;

void rb_init(ring_buffer_t* rb);
// Write a single item as a blob: [len][data...]. Returns false on overflow/too long.
bool rb_write(ring_buffer_t* rb, const uint8_t* data, uint16_t len);
// Read next item blob into 'data', returns length or negative on empty/too small.
int16_t rb_read(ring_buffer_t* rb, uint8_t* data, uint16_t max_len);
bool rb_is_empty(ring_buffer_t* rb);
uint16_t rb_get_item_count(ring_buffer_t* rb);

#endif // RING_BUFFER_UTILS_H

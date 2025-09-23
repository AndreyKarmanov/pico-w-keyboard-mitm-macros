#ifndef RING_BUFFER_UTILS_H
#define RING_BUFFER_UTILS_H

#include <stdint.h>
#include <stdbool.h>

#define REPORT_RING_BUFFER_SIZE 260 // (1 for id + 64 for report + 1 for len) * 4
#define MAX_REPORT_LEN 64

typedef struct {
    uint8_t buffer[REPORT_RING_BUFFER_SIZE];
    uint16_t head;
    uint16_t tail;
    uint16_t used_bytes;
    uint16_t item_count;
} report_ring_buffer_t;

void report_ring_buffer_init(report_ring_buffer_t* rb);
bool report_ring_buffer_write(report_ring_buffer_t* rb, uint8_t report_id, const uint8_t* data, uint8_t len);
int16_t report_ring_buffer_read(report_ring_buffer_t* rb, uint8_t* report_id, uint8_t* data, uint8_t max_len);
bool report_ring_buffer_is_empty(report_ring_buffer_t* rb);
uint16_t report_ring_buffer_get_item_count(report_ring_buffer_t* rb);

#endif // RING_BUFFER_UTILS_H

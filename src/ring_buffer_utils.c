#include "ring_buffer_utils.h"
#include <string.h>
#include <stdio.h>

void report_ring_buffer_init(report_ring_buffer_t* rb) {
    memset(rb, 0, sizeof(report_ring_buffer_t));
}

bool report_ring_buffer_write(report_ring_buffer_t* rb, const uint8_t* data, uint8_t len) {
    if (len == 0 || len > MAX_REPORT_LEN) {
        return false;
    }

    // Check for space: 1 byte for length + len for data
    uint16_t free_space;
    if (rb->head >= rb->tail) {
        free_space = REPORT_RING_BUFFER_SIZE - (rb->head - rb->tail);
    } else {
        free_space = rb->tail - rb->head;
    }

    if (free_space < (len + 1)) {
        printf("Ring buffer full\n");
        return false;
    }

    // Write length
    rb->buffer[rb->head] = len;
    rb->head = (rb->head + 1) % REPORT_RING_BUFFER_SIZE;

    // Write data, handling wrap-around
    uint16_t remaining_space_at_end = REPORT_RING_BUFFER_SIZE - rb->head;
    if (len <= remaining_space_at_end) {
        memcpy(&rb->buffer[rb->head], data, len);
    } else {
        uint16_t part1_len = remaining_space_at_end;
        uint16_t part2_len = len - part1_len;
        memcpy(&rb->buffer[rb->head], data, part1_len);
        memcpy(&rb->buffer[0], data + part1_len, part2_len);
    }

    rb->head = (rb->head + len) % REPORT_RING_BUFFER_SIZE;
    rb->item_count++;
    return true;
}

int16_t report_ring_buffer_read(report_ring_buffer_t* rb, uint8_t* data, uint8_t max_len) {
    if (report_ring_buffer_is_empty(rb)) {
        return -1;
    }

    // Read length
    uint8_t len = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % REPORT_RING_BUFFER_SIZE;

    if (len > max_len) {
        // Skip the data if buffer is too small
        rb->tail = (rb->tail + len) % REPORT_RING_BUFFER_SIZE;
        rb->item_count--;
        return -2; // Error: buffer too small
    }

    // Read data, handling wrap-around
    uint16_t remaining_space_at_end = REPORT_RING_BUFFER_SIZE - rb->tail;
    if (len <= remaining_space_at_end) {
        memcpy(data, &rb->buffer[rb->tail], len);
    } else {
        uint16_t part1_len = remaining_space_at_end;
        uint16_t part2_len = len - part1_len;
        memcpy(data, &rb->buffer[rb->tail], part1_len);
        memcpy(data + part1_len, &rb->buffer[0], part2_len);
    }

    rb->tail = (rb->tail + len) % REPORT_RING_BUFFER_SIZE;
    rb->item_count--;
    return len;
}

bool report_ring_buffer_is_empty(report_ring_buffer_t* rb) {
    return rb->item_count == 0;
}

uint16_t report_ring_buffer_get_item_count(report_ring_buffer_t* rb) {
    return rb->item_count;
}

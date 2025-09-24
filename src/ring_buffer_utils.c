#include "ring_buffer_utils.h"
#include <string.h>
#include <stdio.h>

void rb_init(ring_buffer_t* rb) {
    memset(rb, 0, sizeof(ring_buffer_t));
}

bool rb_write(ring_buffer_t* rb, const uint8_t* data, uint16_t len) {
    if (len == 0 || len > MAX_REPORT_LEN) {
        return false;
    }

    // total bytes needed: 1 length byte + payload
    uint16_t need = (uint16_t)(1 + len);
    uint16_t free_space = REPORT_RING_BUFFER_SIZE - rb->used_bytes;
    if (free_space <= need) {
        printf("Ring buffer full\n");
        return false;
    }

    // Write length
    rb->buffer[rb->head] = (uint8_t)len;
    rb->head = (rb->head + 1) % REPORT_RING_BUFFER_SIZE;

    // Write data, handling wrap-around
    uint16_t remaining_space_at_end = REPORT_RING_BUFFER_SIZE - rb->head;
    if (len <= remaining_space_at_end) {
        memcpy(&rb->buffer[rb->head], data, len);
    } else {
        uint16_t part1_len = remaining_space_at_end;
        uint16_t part2_len = (uint16_t)(len - part1_len);
        memcpy(&rb->buffer[rb->head], data, part1_len);
        memcpy(&rb->buffer[0], data + part1_len, part2_len);
    }

    rb->head = (rb->head + len) % REPORT_RING_BUFFER_SIZE;
    rb->item_count++;
    rb->used_bytes = (uint16_t)(rb->used_bytes + need);
    return true;
}

int16_t rb_read(ring_buffer_t* rb, uint8_t* data, uint16_t max_len) {
    if (rb_is_empty(rb)) {
        return -1;
    }

    // Read length
    uint8_t len = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % REPORT_RING_BUFFER_SIZE;

    if (len > max_len) {
        // Skip the data if buffer is too small
        rb->tail = (rb->tail + len) % REPORT_RING_BUFFER_SIZE;
        rb->item_count--;
        rb->used_bytes = (uint16_t)(rb->used_bytes - (1 + len));
        return -2; // Error: buffer too small
    }

    // Read data, handling wrap-around
    uint16_t remaining_space_at_end = REPORT_RING_BUFFER_SIZE - rb->tail;
    if (len <= remaining_space_at_end) {
        memcpy(data, &rb->buffer[rb->tail], len);
    } else {
        uint16_t part1_len = remaining_space_at_end;
        uint16_t part2_len = (uint16_t)(len - part1_len);
        memcpy(data, &rb->buffer[rb->tail], part1_len);
        memcpy(data + part1_len, &rb->buffer[0], part2_len);
    }

    rb->tail = (rb->tail + len) % REPORT_RING_BUFFER_SIZE;
    rb->item_count--;
    rb->used_bytes = (uint16_t)(rb->used_bytes - (1 + len));
    return len;
}

bool rb_is_empty(ring_buffer_t* rb) {
    return rb->item_count == 0;
}

uint16_t rb_get_item_count(ring_buffer_t* rb) {
    return rb->item_count;
}

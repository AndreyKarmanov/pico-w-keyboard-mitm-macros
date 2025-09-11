#ifndef TLV_UTILS_H
#define TLV_UTILS_H

#include <stdint.h>
#include <btstack.h>

#include "macro_itm_types.h"

// Initialize TLV backend implementation/context
void tlv_utils_init(void);

// Parse ASCII device string "AA:BB:CC:DD:EE:FF,<type>,<name>" into target_device_t
target_device_t tlv_parse_device_string(const char* s, uint16_t s_len);

// Target device persistence helpers
void tlv_delete_target(void);
target_device_t tlv_load_target_device(void);
void tlv_store_target_device(const target_device_t* dev);
void tlv_save_target_if_needed(const target_device_t* current);

// HID descriptor persistence helpers
uint16_t tlv_load_hid_descriptor(uint8_t* buffer, uint16_t buffer_size);
void tlv_store_hid_descriptor(const uint8_t* buffer, uint16_t buffer_size);

#endif // TLV_UTILS_H

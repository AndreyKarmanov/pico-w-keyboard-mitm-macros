#include "tlv_utils.h"

#include <stdio.h>
#include <string.h>

#include <btstack_tlv.h>

// TLV tag for target device ('TRGT')
#define TLV_TAG_TARGET_DEVICE 0x54524754u
// TLV tag for HID descriptor ('HIDD')
#define TLV_TAG_HID_DESCRIPTOR 0x48494444u
// TLV tag for Macros text ('MACR') and binary ('MBIN')
#define TLV_TAG_MACROS_TEXT 0x4D414352u
#define TLV_TAG_MACROS_BIN  0x4D42494Eu

static const btstack_tlv_t* tlv_impl;
static void* tlv_context;

void tlv_utils_init(void) {
    btstack_tlv_get_instance(&tlv_impl, &tlv_context);
}

target_device_t tlv_parse_device_string(const char* s, uint16_t s_len) {
    target_device_t temp_device = (target_device_t){ {0}, 0, "", HCI_CON_HANDLE_INVALID, 0 };

    char temp_addr_s[18];
    char s_buf[65]; // max 64 chars + null terminator
    if (s_len >= sizeof(s_buf)) {
        return temp_device;
    }
    memcpy(s_buf, s, s_len);
    s_buf[s_len] = '\0';

    // "AA:BB:CC:DD:EE:FF,<type>,<name>"
    int result = sscanf(s_buf, "%17[^,],%hhu,%31[^\r\n]", temp_addr_s, &temp_device.addr_type, temp_device.name);

    if (result == 3) {
        temp_device.valid = sscanf_bd_addr(temp_addr_s, temp_device.addr);
    }

    return temp_device;
}

void tlv_delete_target(void) {
    if (!tlv_impl) return;
    tlv_impl->delete_tag(tlv_context, TLV_TAG_TARGET_DEVICE);
}

target_device_t tlv_load_target_device(void) {
    if (!tlv_impl) return (target_device_t) { { 0 }, 0, "", HCI_CON_HANDLE_INVALID, 0 };
    uint8_t s_buf[64];
    int len = tlv_impl->get_tag(tlv_context, TLV_TAG_TARGET_DEVICE, s_buf, sizeof(s_buf));
    return tlv_parse_device_string((const char*)s_buf, (uint16_t)len);
}

void tlv_store_target_device(const target_device_t* device) {
    if (!tlv_impl || !device || !device->valid) return;
    char s_buf[64];
    int n = snprintf(s_buf, sizeof(s_buf), "%s,%u,%s", bd_addr_to_str(device->addr), (unsigned)device->addr_type, device->name);
    if (n < 0) return;
    size_t len = (size_t)n;
    if (len > sizeof(s_buf)) len = sizeof(s_buf);
    tlv_impl->store_tag(tlv_context, TLV_TAG_TARGET_DEVICE, (const uint8_t*)s_buf, (uint32_t)len);
}

uint16_t tlv_load_hid_descriptor(uint8_t* buffer, uint16_t buffer_size) {
    if (!tlv_impl) return 0;
    int len = tlv_impl->get_tag(tlv_context, TLV_TAG_HID_DESCRIPTOR, buffer, buffer_size);
    if (len > 0) {
        printf("Loaded HID descriptor from TLV (len=%u)\n", len);
        return (uint16_t)len;
    }
    return 0;
}

void tlv_store_hid_descriptor(const uint8_t* buffer, uint16_t buffer_size) {
    if (!tlv_impl || buffer == NULL || buffer_size == 0) return;
    tlv_impl->store_tag(tlv_context, TLV_TAG_HID_DESCRIPTOR, buffer, buffer_size);
    printf("Stored HID descriptor in TLV (len=%u)\n", buffer_size);
}

void tlv_save_target_if_needed(const target_device_t* current) {
    if (!current || !current->valid) return;
    target_device_t persisted = tlv_load_target_device();
    if (persisted.valid
        && (bd_addr_cmp(persisted.addr, current->addr) == 0)
        && (persisted.addr_type == current->addr_type)
        && (strncmp(persisted.name, current->name, sizeof(persisted.name)) == 0)) {
        printf("Persisted target matches, not updating\n");
        return;
    }
    printf("Persisted target differs, replacing\n");
    tlv_store_target_device(current);
}

// Legacy text tag constant retained but functions removed per request.

uint16_t tlv_load_macros_bin(uint8_t* buffer, uint16_t buffer_size) {
    if (!tlv_impl) return 0;
    int len = tlv_impl->get_tag(tlv_context, TLV_TAG_MACROS_BIN, buffer, buffer_size);
    if (len > 0) {
        printf("Loaded Macros BIN from TLV (len=%u)\n", len);
        return (uint16_t)len;
    }
    return 0;
}

void tlv_store_macros_bin(const uint8_t* buffer, uint16_t buffer_size) {
    if (!tlv_impl || buffer == NULL) return;
    tlv_impl->store_tag(tlv_context, TLV_TAG_MACROS_BIN, buffer, buffer_size);
    printf("Stored Macros BIN in TLV (len=%u)\n", buffer_size);
}

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


void tlv_delete_target(void) {
    if (!tlv_impl) return;
    tlv_impl->delete_tag(tlv_context, TLV_TAG_TARGET_DEVICE);
}

target_device_t tlv_load_target_device(void) {
    if (!tlv_impl) return (target_device_t) { { 0 }, 0, "", HCI_CON_HANDLE_INVALID, 0 };
    uint8_t buf[64];
    int len = tlv_impl->get_tag(tlv_context, TLV_TAG_TARGET_DEVICE, buf, sizeof(buf));
    target_device_t dev = (target_device_t){ {0}, 0, "", HCI_CON_HANDLE_INVALID, 0 };
    if (len < 8) return dev; // require addr(6)+type(1)+name_len(1)
    memcpy(dev.addr, buf, 6);
    dev.addr_type = buf[6];
    uint8_t name_len = buf[7];
    if (8 + name_len > len) name_len = (uint8_t)((len > 8) ? (len - 8) : 0);
    uint8_t cpy = name_len < sizeof(dev.name) - 1 ? name_len : (uint8_t)(sizeof(dev.name) - 1);
    if (cpy) memcpy(dev.name, &buf[8], cpy);
    dev.name[cpy] = '\0';
    dev.valid = true;
    return dev;
}

void tlv_store_target_device(const target_device_t* device) {
    if (!tlv_impl || !device || !device->valid) return;
    uint8_t buf[64];
    uint16_t p = 0;
    memcpy(&buf[p], device->addr, 6); p += 6;
    buf[p++] = (uint8_t)device->addr_type;
    uint8_t name_len = (uint8_t)strnlen(device->name, sizeof(device->name));
    buf[p++] = name_len;
    if (name_len) { memcpy(&buf[p], device->name, name_len); p = (uint16_t)(p + name_len); }
    tlv_impl->store_tag(tlv_context, TLV_TAG_TARGET_DEVICE, buf, p);
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

#ifndef MACRO_ITM_TYPES_H
#define MACRO_ITM_TYPES_H

#include <stdbool.h>
#include <stdint.h>
#include <btstack.h>

// Shared data types for the macro_itm application

typedef struct {
    bd_addr_t addr;
    bd_addr_type_t addr_type;
    char name[32];
    hci_con_handle_t con_handle;
    bool valid;
} target_device_t;

// Track host (our Peripheral link to PC/Phone)
typedef struct {
    bd_addr_type_t addr_type; // best-effort, filled on connect
    bd_addr_t addr;           // best-effort, filled on connect
    hci_con_handle_t con_handle;
    uint8_t report_protocol_mode;
    uint32_t subscribed_reports_bitmap; // bitmask for report IDs
} host_device_t;

#endif // MACRO_ITM_TYPES_H

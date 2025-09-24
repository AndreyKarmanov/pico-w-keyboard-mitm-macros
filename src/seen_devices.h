#ifndef SEEN_DEVICES_H
#define SEEN_DEVICES_H

#include <stdint.h>
#include <btstack.h>

#ifdef __cplusplus
extern "C" {
#endif

// Maximum size used by callers to allocate a buffer for binary listing
#define SEEN_DEVICES_BIN_BUF_MAX 1024

// Save an advertising report's device if it provides a (shortened or complete) local name
void seen_devices_save_from_adv(const bd_addr_t addr, uint8_t addr_type,
                                const uint8_t* adv_data, uint8_t adv_size);

// Clear the in-memory list
void seen_devices_clear(void);

// Build binary listing into 'out'. Returns number of bytes written.
// Format: count(1), then repeated entries {addr[6], type(1), name_len(1), name[name_len]}
uint16_t seen_devices_build_binary(uint8_t* out, uint16_t max_len);

// Get current count
int seen_devices_count(void);

#ifdef __cplusplus
}
#endif

#endif // SEEN_DEVICES_H

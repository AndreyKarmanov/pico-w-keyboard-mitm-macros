#ifndef MACROS_ENGINE_H
#define MACROS_ENGINE_H

#include <stdint.h>
#include <stdbool.h>

#ifndef MAX_REPORT_LEN
#define MAX_REPORT_LEN 64
#endif

#ifdef __cplusplus
extern "C" {
#endif

// Public API for macro rules engine
// Binary format (MBIN v1):
//   magic[4] = 'MBIN' (0x4D 0x42 0x49 0x4E)
//   version[1] = 1
//   count[1]
//   entries[count]: name_len[1], len[1], name[name_len], in[len], out[len]

// Initialize/clear internal state
void macros_engine_init(void);
void macros_engine_clear(void);

// Upload workflow for ATT chunks
void macros_engine_begin_upload(void);
bool macros_engine_append_chunk(const uint8_t* data, uint16_t len);
// Parse current upload buffer, load on success, and optionally persist via callback
// Returns true if parsed and loaded successfully. On success, if persist=true,
// calls the provided persist_fn(blob, len) to store the exact uploaded blob.
bool macros_engine_commit(bool persist, void (*persist_fn)(const uint8_t*, uint16_t));

// Load existing MBIN blob from storage (e.g., TLV); if invalid, clears state
void macros_engine_load_blob(const uint8_t* data, uint16_t len);

// Dump current rules to MBIN into out buffer; returns length written (0 if insufficient space)
uint16_t macros_engine_dump_blob(uint8_t* out, uint16_t max_len);

// Info
uint8_t macros_engine_rule_count(void);

// Transform HID keyboard-like report according to rules.
// Inputs/outputs may alias. Accepts report_with_id (len>=1). Returns output length.
// If no change, echoes input. Supports ID-preserving or ID-changing rules.
uint16_t macros_engine_apply(const uint8_t* in_bytes, uint16_t in_len,
                             uint8_t* out_bytes, uint16_t out_max_len,
                             uint8_t* out_report_id);

// Convenience: integrate TLV persistence so callers don't need to wire it.
// Loads the stored MBIN blob from TLV (if any); clears rules if invalid/missing.
void macros_engine_load_from_tlv(void);
// Commit the current upload buffer and persist to TLV on success.
bool macros_engine_commit_and_persist_to_tlv(void);
// Clear rules and clear TLV storage.
void macros_engine_clear_and_persist_to_tlv(void);

#ifdef __cplusplus
}
#endif

#endif // MACROS_ENGINE_H

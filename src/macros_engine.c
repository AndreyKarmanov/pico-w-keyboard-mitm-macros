#include "macros_engine.h"

#include <string.h>
#include <stdio.h>

#include "tlv_utils.h"

// Internal structures
#define MAX_KEYS 6
#define MAX_MACROS 32
#define MACROS_BIN_MAX 1400

typedef struct {
    char name[32];
    uint8_t in[MAX_REPORT_LEN + 1];
    uint8_t out[MAX_REPORT_LEN + 1];
    uint8_t len; // includes ID as first byte
} macro_rule_t;

static macro_rule_t g_rules[MAX_MACROS];
static uint8_t g_rules_count = 0;
static uint8_t g_upload_buf[MACROS_BIN_MAX];
static uint16_t g_upload_len = 0;

// Keyboard report helpers
static bool parse_kbd_report(const uint8_t* data, uint16_t len,
                             uint8_t* out_id, uint8_t* out_mod, bool* out_has_res,
                             uint8_t* out_keys, uint8_t* out_kcnt) {
    if (len < 2) return false; // id + at least 1 payload byte
    const uint8_t id = data[0];
    const uint16_t payload_len = (uint16_t)(len - 1);
    if (payload_len < 2) return false;
    const uint8_t* payload = data + 1;
    uint8_t mod = payload[0];
    bool has_res = false;
    uint8_t kstart = 1;
    if (payload_len >= 8) { has_res = true; kstart = 2; }
    else if (payload_len == 7) { has_res = false; kstart = 1; }
    else {
        uint8_t rem = (uint8_t)(payload_len - 1);
        has_res = !(rem <= 6);
        kstart = has_res ? 2 : 1;
    }
    uint8_t kbytes = (payload_len > kstart) ? (uint8_t)(payload_len - kstart) : 0;
    if (kbytes > MAX_KEYS) kbytes = MAX_KEYS;
    for (uint8_t i = 0; i < kbytes; i++) out_keys[i] = payload[kstart + i];
    *out_id = id; *out_mod = mod; *out_has_res = has_res; *out_kcnt = kbytes;
    return true;
}

static void uniq_keys_make(const uint8_t* keys, uint8_t cnt, uint8_t* uniq_out, uint8_t* uniq_cnt) {
    uint8_t u = 0;
    for (uint8_t i = 0; i < cnt; i++) {
        uint8_t k = keys[i];
        if (k == 0) continue;
        bool seen = false;
        for (uint8_t j = 0; j < u; j++) if (uniq_out[j] == k) { seen = true; break; }
        if (!seen && u < MAX_KEYS) uniq_out[u++] = k;
    }
    *uniq_cnt = u;
}

static bool keys_is_subset(const uint8_t* a, uint8_t ac, const uint8_t* b, uint8_t bc) {
    for (uint8_t i = 0; i < ac; i++){
        bool found = false;
        for (uint8_t j = 0; j < bc; j++) if (a[i] == b[j]) { found = true; break; }
        if (!found) return false;
    }
    return true;
}

// FIFO for physical key order across frames
static uint8_t key_fifo[32];
static uint8_t key_fifo_len = 0;
static void fifo_update_from_pressed(const uint8_t* pressed, uint8_t pcnt){
    for (uint8_t i = 0; i < key_fifo_len; ){
        uint8_t k = key_fifo[i];
        bool still = false;
        for (uint8_t j = 0; j < pcnt; j++) if (pressed[j] == k) { still = true; break; }
        if (!still){
            for (uint8_t m = i; m + 1 < key_fifo_len; m++) key_fifo[m] = key_fifo[m+1];
            key_fifo_len--;
        } else {
            i++;
        }
    }
    for (uint8_t j = 0; j < pcnt; j++){
        uint8_t k = pressed[j];
        bool present = false;
        for (uint8_t i = 0; i < key_fifo_len; i++) if (key_fifo[i] == k) { present = true; break; }
        if (!present && key_fifo_len < (uint8_t)sizeof(key_fifo)) key_fifo[key_fifo_len++] = k;
    }
}

static void rules_clear(){ memset(g_rules, 0, sizeof(g_rules)); g_rules_count = 0; }

static bool rules_parse_blob(const uint8_t* data, uint16_t len){
    if (len < 6) { rules_clear(); return false; }
    if (!(data[0]=='M' && data[1]=='B' && data[2]=='I' && data[3]=='N')) { rules_clear(); return false; }
    if (data[4] != 1) { rules_clear(); return false; }
    uint8_t count = data[5];
    uint16_t pos = 6;
    rules_clear();
    for (uint8_t i = 0; i < count && i < MAX_MACROS; i++){
        if (pos + 2 > len) break;
        uint8_t name_len = data[pos++];
        uint8_t rlen = data[pos++];
        if (rlen == 0 || rlen > (MAX_REPORT_LEN+1)) { rules_clear(); return false; }
        if (pos + name_len + rlen + rlen > len) { rules_clear(); return false; }
        macro_rule_t r; memset(&r, 0, sizeof(r));
        if (name_len){
            uint8_t cpy = name_len < sizeof(r.name)-1 ? name_len : (uint8_t)(sizeof(r.name)-1);
            memcpy(r.name, data+pos, cpy);
        }
        pos += name_len;
        memcpy(r.in, data+pos, rlen); pos += rlen;
        memcpy(r.out, data+pos, rlen); pos += rlen;
        r.len = rlen;
        g_rules[g_rules_count++] = r;
    }
    return true;
}

static uint16_t rules_dump_blob(uint8_t* out, uint16_t max_len){
    uint16_t pos = 0;
    if (max_len < 6) return 0;
    out[pos++] = 'M'; out[pos++] = 'B'; out[pos++] = 'I'; out[pos++] = 'N';
    out[pos++] = 1; // version
    out[pos++] = g_rules_count;
    for (uint8_t i = 0; i < g_rules_count; i++){
        macro_rule_t* r = &g_rules[i];
        uint8_t name_len = (uint8_t)strnlen(r->name, sizeof(r->name));
        if (pos + 2 + name_len + r->len + r->len > max_len) break;
        out[pos++] = name_len;
        out[pos++] = r->len;
        if (name_len){ memcpy(out+pos, r->name, name_len); pos += name_len; }
        memcpy(out+pos, r->in, r->len); pos += r->len;
        memcpy(out+pos, r->out, r->len); pos += r->len;
    }
    return pos;
}

void macros_engine_init(void){ rules_clear(); g_upload_len = 0; }
void macros_engine_clear(void){ rules_clear(); g_upload_len = 0; }
void macros_engine_begin_upload(void){ g_upload_len = 0; }

bool macros_engine_append_chunk(const uint8_t* data, uint16_t len){
    if (!data || !len) return false;
    if (g_upload_len + len > sizeof(g_upload_buf)) return false;
    memcpy(g_upload_buf + g_upload_len, data, len);
    g_upload_len += len;
    return true;
}

bool macros_engine_commit(bool persist, void (*persist_fn)(const uint8_t*, uint16_t)){
    if (g_upload_len == 0) { rules_clear(); return true; }
    bool ok = rules_parse_blob(g_upload_buf, g_upload_len);
    if (ok && persist && persist_fn){ persist_fn(g_upload_buf, g_upload_len); }
    return ok;
}

void macros_engine_load_blob(const uint8_t* data, uint16_t len){
    if (!data || !len || !rules_parse_blob(data, len)){
        rules_clear();
    }
}

uint16_t macros_engine_dump_blob(uint8_t* out, uint16_t max_len){ return rules_dump_blob(out, max_len); }
uint8_t macros_engine_rule_count(void){ return g_rules_count; }

uint16_t macros_engine_apply(const uint8_t* in_bytes, uint16_t in_len,
                             uint8_t* out_bytes, uint16_t out_max_len,
                             uint8_t* out_report_id){
    if (!in_bytes || in_len == 0) return 0;
    if (!out_bytes || out_max_len < in_len) {
        // If output buffer too small, we cannot transform; bail out
        return 0;
    }

    // Default pass-through
    memcpy(out_bytes, in_bytes, in_len);
    if (out_report_id) *out_report_id = in_bytes[0];

    // Attempt keyboard-like parse; if fails, just echo
    uint8_t in_id=0, in_mod=0, in_keys_raw[MAX_KEYS]={0}, in_kcnt=0; bool in_has_res=false;
    bool parsed_in = parse_kbd_report(in_bytes, in_len, &in_id, &in_mod, &in_has_res, in_keys_raw, &in_kcnt);
    if (!parsed_in) return in_len;

    // Build unique pressed set and update FIFO
    uint8_t in_keys_set[MAX_KEYS]={0}; uint8_t in_keys_n=0;
    uniq_keys_make(in_keys_raw, in_kcnt, in_keys_set, &in_keys_n);
    fifo_update_from_pressed(in_keys_set, in_keys_n);

    // Try rules
    for (uint8_t i = 0; i < g_rules_count; i++){
        macro_rule_t* r = &g_rules[i];
        if (r->len < 2) continue;
        if (r->in[0] != in_id) continue;

        uint8_t m_id=0, m_mod=0, m_keys_raw[MAX_KEYS]={0}, m_kcnt=0; bool m_has_res=false;
        if (!parse_kbd_report(r->in, r->len, &m_id, &m_mod, &m_has_res, m_keys_raw, &m_kcnt)) continue;
        uint8_t m_keys_set[MAX_KEYS]={0}; uint8_t m_keys_n=0; uniq_keys_make(m_keys_raw, m_kcnt, m_keys_set, &m_keys_n);
        if ((in_mod & m_mod) != m_mod) continue; // mods subset
        if (!keys_is_subset(m_keys_set, m_keys_n, in_keys_set, in_keys_n)) continue; // keys subset

        // Parse output
        uint8_t o_id=0, o_mod=0, o_keys_raw[MAX_KEYS]={0}, o_kcnt=0; bool o_has_res=false;
        if (!parse_kbd_report(r->out, r->len, &o_id, &o_mod, &o_has_res, o_keys_raw, &o_kcnt)) continue;

        uint8_t final_mod = (uint8_t)(in_mod | o_mod);

        uint8_t final_keys_set[16]; uint8_t final_keys_n = 0; memset(final_keys_set,0,sizeof(final_keys_set));
        for (uint8_t a = 0; a < in_keys_n; a++){
            uint8_t key = in_keys_set[a];
            bool in_macro = false; for (uint8_t b = 0; b < m_keys_n; b++) if (m_keys_set[b] == key) { in_macro = true; break; }
            if (!in_macro) {
                bool exists = false; for (uint8_t z = 0; z < final_keys_n; z++) if (final_keys_set[z] == key) { exists = true; break; }
                if (!exists && final_keys_n < sizeof(final_keys_set)) final_keys_set[final_keys_n++] = key;
            }
        }
        uint8_t o_keys_set[MAX_KEYS]={0}; uint8_t o_keys_n=0; uniq_keys_make(o_keys_raw, o_kcnt, o_keys_set, &o_keys_n);
        for (uint8_t a = 0; a < o_keys_n; a++){
            uint8_t key = o_keys_set[a];
            bool exists = false; for (uint8_t z = 0; z < final_keys_n; z++) if (final_keys_set[z] == key) { exists = true; break; }
            if (!exists && final_keys_n < sizeof(final_keys_set)) final_keys_set[final_keys_n++] = key;
        }

        uint8_t ordered_keys[MAX_KEYS] = {0}; uint8_t ord_n = 0;
        for (uint8_t a = 0; a < o_keys_n && ord_n < MAX_KEYS; a++) ordered_keys[ord_n++] = o_keys_set[a];
        for (uint8_t f = 0; f < key_fifo_len && ord_n < MAX_KEYS; f++){
            uint8_t k = key_fifo[f];
            bool in_final = false; for (uint8_t x = 0; x < final_keys_n; x++) if (final_keys_set[x] == k) { in_final = true; break; }
            if (!in_final) continue;
            bool dup = false; for (uint8_t y = 0; y < ord_n; y++) if (ordered_keys[y] == k) { dup = true; break; }
            if (!dup) ordered_keys[ord_n++] = k;
        }
        while (ord_n < MAX_KEYS) ordered_keys[ord_n++] = 0;

        // Build outgoing; reserve up to (1 + 1 + 1 + 6) bytes
        if (out_max_len < (uint16_t)(1 + 1 + (in_has_res ? 1 : 0) + MAX_KEYS)) return 0;
        uint16_t p = 0;
        out_bytes[p++] = o_id; // allow ID change
        out_bytes[p++] = final_mod;
        if (in_has_res) out_bytes[p++] = 0x00;
        for (uint8_t i2 = 0; i2 < MAX_KEYS; i2++) out_bytes[p++] = ordered_keys[i2];
        if (out_report_id) *out_report_id = o_id;
        return p;
    }

    // No rule matched, echo
    return in_len;
}

void macros_engine_load_from_tlv(void){
    uint8_t buf[MACROS_BIN_MAX];
    uint16_t len = tlv_load_macros_bin(buf, sizeof(buf));
    macros_engine_load_blob((len > 0) ? buf : NULL, len);
}

bool macros_engine_commit_and_persist_to_tlv(void){
    return macros_engine_commit(true, tlv_store_macros_bin);
}

void macros_engine_clear_and_persist_to_tlv(void){
    macros_engine_clear();
    tlv_store_macros_bin((const uint8_t*)"", 0);
}

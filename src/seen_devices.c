#include "seen_devices.h"

#include <string.h>
#include <stdio.h>

#include <btstack.h>

#define MAX_SEEN_DEVICES 32

typedef struct {
    bd_addr_t addr;
    uint8_t addr_type;
    char name[32];
} seen_device_t;

static seen_device_t g_seen[MAX_SEEN_DEVICES];
static int g_seen_count = 0;

void seen_devices_save_from_adv(const bd_addr_t addr, uint8_t addr_type,
                                const uint8_t* adv_data, uint8_t adv_size){
    // Skip if already seen
    for (int i = 0; i < g_seen_count; i++){
        if (bd_addr_cmp(addr, g_seen[i].addr) == 0) return;
    }
    if (g_seen_count >= MAX_SEEN_DEVICES) return;

    ad_context_t context;
    for (ad_iterator_init(&context, adv_size, adv_data); ad_iterator_has_more(&context); ad_iterator_next(&context)){
        uint8_t data_type = ad_iterator_get_data_type(&context);
        uint8_t size = ad_iterator_get_data_len(&context);
        const uint8_t* data = ad_iterator_get_data(&context);
        switch (data_type){
            case BLUETOOTH_DATA_TYPE_SHORTENED_LOCAL_NAME:
            case BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME: {
                memcpy(g_seen[g_seen_count].addr, addr, sizeof(bd_addr_t));
                g_seen[g_seen_count].addr_type = addr_type;
                uint8_t copy_len = size < (sizeof(g_seen[0].name) - 1) ? size : (uint8_t)(sizeof(g_seen[0].name) - 1);
                memcpy(g_seen[g_seen_count].name, data, copy_len);
                g_seen[g_seen_count].name[copy_len] = '\0';
                printf("Found device %s (%s, type=%u)\n", g_seen[g_seen_count].name, bd_addr_to_str(g_seen[g_seen_count].addr), addr_type);
                g_seen_count++;
                return;
            }
            default: break;
        }
    }
}

void seen_devices_clear(void){
    memset(g_seen, 0, sizeof(g_seen));
    g_seen_count = 0;
    printf("Seen devices cleared\n");
}

uint16_t seen_devices_build_binary(uint8_t* out, uint16_t max_len){
    uint16_t p = 0;
    if (max_len < 1) return 0;
    uint8_t count = (g_seen_count > 255) ? 255 : (uint8_t)g_seen_count;
    out[p++] = count;
    for (uint8_t i = 0; i < count; i++){
        uint8_t name_len = (uint8_t)strnlen(g_seen[i].name, sizeof(g_seen[i].name));
        if (p + 6 + 1 + 1 + name_len > max_len) break;
        memcpy(&out[p], g_seen[i].addr, 6); p += 6;
        out[p++] = g_seen[i].addr_type;
        out[p++] = name_len;
        if (name_len){ memcpy(&out[p], g_seen[i].name, name_len); p = (uint16_t)(p + name_len); }
    }
    return p;
}

int seen_devices_count(void){ return g_seen_count; }

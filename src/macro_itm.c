#include "macro_itm.h"

#include <inttypes.h>

#include <pico/cyw43_arch.h>
#include <pico/stdio.h>
#include <hardware/watchdog.h>

#include <btstack.h>
#include <btstack_tlv.h>
#include <btstack_config.h>

#include "macro_itm_types.h"
#include "tlv_utils.h"
#include "print_utils.h"
#include "ring_buffer_utils.h"
#include "macros_engine.h"
#include "seen_devices.h"


typedef enum {
    APP_STATE_IDLE = 0,
    APP_STATE_ACTIVE,
    APP_STATE_READY,
} app_state_t;
static app_state_t app_state;

typedef enum {
    HOST_STATE_IDLE = 0,
    HOST_STATE_ADVERTISING,
    HOST_STATE_CONNECTED,
} host_state_t;
static host_state_t host_state = HOST_STATE_IDLE;

typedef enum {
    TARGET_STATE_IDLE = 0,
    TARGET_STATE_SCANNING,
    TARGET_STATE_CONNECTING,
    TARGET_STATE_CONNECTED,
} target_state_t;

static target_state_t target_state = TARGET_STATE_IDLE;

static const char* app_state_strings[] = {
  "IDLE",
  "ACTIVE",
  "READY",
};

static const char* host_state_strings[] = {
  "NONE",
  "ADVERTISING",
  "CONNECTED",
};

static const char* target_state_strings[] = {
  "IDLE",
  "SCANNING",
  "CONNECTING",
  "CONNECTED",
};

static target_device_t target = { {0}, 0, "", HCI_CON_HANDLE_INVALID, 0 };

static host_device_t host = { 0, {0}, HCI_CON_HANDLE_INVALID, 0, 0, false };

static ring_buffer_t hid_report_ring_buffer;
static ring_buffer_t debug_report_ring_buffer;

static uint16_t hids_cid;
static uint8_t hid_descriptor_storage[500];

static uint8_t hid_descriptor[600];
static uint16_t hid_descriptor_len = 0;

#define SEEN_DEVICES_CHAR_BUFFER_SIZE 1024

static btstack_timer_source_t target_connection_timer;

static btstack_packet_callback_registration_t hci_event_callback_registration;
static btstack_packet_callback_registration_t sm_event_callback_registration;

// Config Advertisement Data
const uint8_t config_adv_data[] = {
    // Flags general discoverable, Classic not supported (LE only)
    0x02, BLUETOOTH_DATA_TYPE_FLAGS,
    0x06, // LE General Discoverable Mode

    // 16-bit Service UUIDs
    0x03,
    BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_16_BIT_SERVICE_CLASS_UUIDS,
    ORG_BLUETOOTH_SERVICE_HUMAN_INTERFACE_DEVICE & 0xff,
    ORG_BLUETOOTH_SERVICE_HUMAN_INTERFACE_DEVICE >> 8,
    // Appearance HID - Keyboard (Category 15, Sub-Category 1)
    0x03,
    BLUETOOTH_DATA_TYPE_APPEARANCE,
    0xC1,
    0x03,
};
const uint8_t config_adv_data_len = sizeof(config_adv_data);

const uint8_t scan_response_data[] = {
    // Name
    0x05, BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME, 'M', 'I', 'T', 'M', // "MITM"

    // 128-bit Service UUID
    0x11,
    BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS,
    0xEF, 0xCD, 0xAB, 0x89, 0x67, 0x45, 0x23, 0x01, 0xEF, 0xCD, 0xAB, 0x90,
    0x78, 0x56, 0x34, 0x12,
};
const uint8_t scan_response_data_len = sizeof(scan_response_data);

// from USB HID Specification 1.1, Appendix B.1
const uint8_t hid_descriptor_keyboard_boot_mode[] = {

    0x05, 0x01, // Usage Page (Generic Desktop)
    0x09, 0x06, // Usage (Keyboard)
    0xa1, 0x01, // Collection (Application)

    0x85, 0x01, // Report ID 1

    // Modifier byte

    0x75, 0x01, //   Report Size (1)
    0x95, 0x08, //   Report Count (8)
    0x05, 0x07, //   Usage Page (Key codes)
    0x19, 0xe0, //   Usage Minimum (Keyboard LeftControl)
    0x29, 0xe7, //   Usage Maxium (Keyboard Right GUI)
    0x15, 0x00, //   Logical Minimum (0)
    0x25, 0x01, //   Logical Maximum (1)
    0x81, 0x02, //   Input (Data, Variable, Absolute)

    // Reserved byte

    0x75, 0x01, //   Report Size (1)
    0x95, 0x08, //   Report Count (8)
    0x81, 0x03, //   Input (Constant, Variable, Absolute)

    // LED report + padding

    0x95, 0x05, //   Report Count (5)
    0x75, 0x01, //   Report Size (1)
    0x05, 0x08, //   Usage Page (LEDs)
    0x19, 0x01, //   Usage Minimum (Num Lock)
    0x29, 0x05, //   Usage Maxium (Kana)
    0x91, 0x02, //   Output (Data, Variable, Absolute)

    0x95, 0x01, //   Report Count (1)
    0x75, 0x03, //   Report Size (3)
    0x91, 0x03, //   Output (Constant, Variable, Absolute)

    // Keycodes

    0x95, 0x06, //   Report Count (6)
    0x75, 0x08, //   Report Size (8)
    0x15, 0x00, //   Logical Minimum (0)
    0x25, 0xff, //   Logical Maximum (1)
    0x05, 0x07, //   Usage Page (Key codes)
    0x19, 0x00, //   Usage Minimum (Reserved (no event indicated))
    0x29, 0xff, //   Usage Maxium (Reserved)
    0x81, 0x00, //   Input (Data, Array)

    0xc0, // End collection
};

static void hci_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size);
static void hids_client_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size);

static void app_set_state(app_state_t new_state)
{
    if (new_state == app_state) return;
    printf("[APP] State %s -> %s\n", app_state_strings[app_state], app_state_strings[new_state]);
    app_state = new_state;
}

static void target_set_state(target_state_t new_state)
{
    if (new_state == target_state) return;
    printf("[TARGET] State %s -> %s\n", target_state_strings[target_state], target_state_strings[new_state]);
    target_state = new_state;

    switch (target_state) {
    case TARGET_STATE_IDLE:
        gap_stop_scan();
        break;
    case TARGET_STATE_SCANNING:
        gap_start_scan();
        break;
    case TARGET_STATE_CONNECTING:
        break;
    case TARGET_STATE_CONNECTED:
        gap_stop_scan();
        if (host_state == HOST_STATE_CONNECTED) {
            app_set_state(APP_STATE_READY);
        } else {
            app_set_state(APP_STATE_ACTIVE);
        }
        // Persist target info when we complete target connection flow
        tlv_save_target_if_needed(&target);
        break;
    }
}

static void host_set_state(host_state_t new_state)
{
    if (new_state == host_state) return;
    printf("[HOST] State %s -> %s\n", host_state_strings[host_state], host_state_strings[new_state]);
    host_state = new_state;

    switch (host_state) {
    case HOST_STATE_IDLE:
        gap_advertisements_enable(0);
        host.con_handle = HCI_CON_HANDLE_INVALID;
        host.addr_type = 0;
        memset(host.addr, 0, sizeof(host.addr));
        break;
    case HOST_STATE_ADVERTISING:
        gap_advertisements_enable(1);
        host.con_handle = HCI_CON_HANDLE_INVALID;
        host.addr_type = 0;
        memset(host.addr, 0, sizeof(host.addr));
        break;
    case HOST_STATE_CONNECTED:
        if (target_state == TARGET_STATE_CONNECTED)
            app_set_state(APP_STATE_READY);
        else
            app_set_state(APP_STATE_ACTIVE);
        break;
    }
}

static void clear_target_device()
{
    if (target_state == TARGET_STATE_CONNECTING) {
        printf("[TARGET] Aborting connection attempt\n");
        gap_connect_cancel();
        btstack_run_loop_remove_timer(&target_connection_timer);
    } else if (target_state == TARGET_STATE_CONNECTED) {
        printf("[TARGET] Disconnecting\n");
        gap_disconnect(target.con_handle);
    }
    // the con_handle should only be removed in disconnection complete event
    printf("[TARGET] Clearing stored target\n");
    tlv_delete_target();
    hid_descriptor_len = 0;
    target.valid = false;
    target_set_state(TARGET_STATE_SCANNING);
}

static void disconnect_host()
{
    if (host.con_handle != HCI_CON_HANDLE_INVALID) {
        printf("[HOST] Disconnecting %s\n", bd_addr_to_str(host.addr));
        gap_disconnect(host.con_handle);
    }
}

static void connection_timeout(btstack_timer_source_t* ts)
{
    UNUSED(ts);
    printf("[TARGET] Connection timeout\n");
    gap_connect_cancel();
    target_set_state(TARGET_STATE_SCANNING);
}

static void attempt_gap_connect()
{
    if (!target.valid) {
        printf("[TARGET] No valid target set\n");
        return;
    }
    if (target_state == TARGET_STATE_CONNECTING || target_state == TARGET_STATE_CONNECTED) {
        printf("[TARGET] Already connecting or connected\n");
        return;
    }
    printf("[TARGET] Connecting to %s (type=%u)\n", bd_addr_to_str(target.addr), target.addr_type);
    target_set_state(TARGET_STATE_CONNECTING);
    btstack_run_loop_set_timer(&target_connection_timer, 10000); // 10s timeout
    btstack_run_loop_set_timer_handler(&target_connection_timer, &connection_timeout);
    btstack_run_loop_add_timer(&target_connection_timer);
    gap_connect(target.addr, target.addr_type);
}

static void hci_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size)
{
    UNUSED(channel);
    UNUSED(size);

    if (packet_type != HCI_EVENT_PACKET)
        return;

    uint8_t status;
    hci_con_handle_t con_handle;
    switch (hci_event_packet_get_type(packet)) {
    case BTSTACK_EVENT_STATE:
        switch (btstack_event_state_get_state(packet)) {
        case HCI_STATE_WORKING:
            target_set_state(TARGET_STATE_SCANNING);
            host_set_state(HOST_STATE_ADVERTISING);
            break;
        default:
            break;
        }
        break;
    case GAP_EVENT_ADVERTISING_REPORT: {
        bd_addr_t address;
        gap_event_advertising_report_get_address(packet, address);
        uint8_t addr_type = gap_event_advertising_report_get_address_type(packet);
        const uint8_t* data = gap_event_advertising_report_get_data(packet);
        uint8_t data_len = gap_event_advertising_report_get_data_length(packet);
        seen_devices_save_from_adv(address, addr_type, data, data_len);

        // connect if we have a target and see it
        if (target_state == TARGET_STATE_SCANNING
            && target.valid
            && bd_addr_cmp(address, target.addr) == 0
            && addr_type == target.addr_type) {
            attempt_gap_connect();
        }
        break;
    }
    case HCI_EVENT_META_GAP:
        if (hci_event_gap_meta_get_subevent_code(packet) != GAP_SUBEVENT_LE_CONNECTION_COMPLETE)
            break;

        bd_addr_t addr;
        gap_subevent_le_connection_complete_get_peer_address(packet, addr);
        status = gap_subevent_le_connection_complete_get_status(packet);

        if (status != ERROR_CODE_SUCCESS) {
            printf("[GAP] Connection failed %s, status %u\n", bd_addr_to_str(addr), status);
            break;
        }

        con_handle = gap_subevent_le_connection_complete_get_connection_handle(packet);
        if (target_state == TARGET_STATE_CONNECTING && target.valid && bd_addr_cmp(addr, target.addr) == 0) {
            btstack_run_loop_remove_timer(&target_connection_timer);
            target.con_handle = con_handle;
            printf("[TARGET] Initiating pairing\n");
            sm_request_pairing(target.con_handle);
        } else if (host_state != HOST_STATE_CONNECTED && gap_subevent_le_connection_complete_get_role(packet) == HCI_ROLE_SLAVE) {
            host.con_handle = con_handle;
            memcpy(host.addr, addr, sizeof(bd_addr_t));
            host.addr_type = gap_subevent_le_connection_complete_get_peer_address_type(packet);
            host_set_state(HOST_STATE_CONNECTED);
        } else {
            printf("[GAP] Unknown device connected: %s\n", bd_addr_to_str(addr));
            gap_disconnect(con_handle);
        }
        break;
    case HCI_EVENT_DISCONNECTION_COMPLETE:
        status = hci_event_disconnection_complete_get_status(packet);
        if (status != ERROR_CODE_SUCCESS) {
            printf("[GAP] Disconnection event error status %u\n", status);
            break;
        }

        con_handle = hci_event_disconnection_complete_get_connection_handle(packet);
        uint8_t reason = hci_event_disconnection_complete_get_reason(packet);
        printf("[GAP] Disconnection reason: 0x%02X\n", reason);
        if (con_handle == target.con_handle) {
            target.con_handle = HCI_CON_HANDLE_INVALID;
            target_set_state(TARGET_STATE_SCANNING);
        } else if (con_handle == host.con_handle) {
            host.con_handle = HCI_CON_HANDLE_INVALID;
            host_set_state(HOST_STATE_ADVERTISING);
        } else {
            printf("[GAP] Unknown device disconnected\n");
        }
        break;
    default:
        break;
    }
}

static void attempt_hids_client_connect()
{
    if (target.con_handle == HCI_CON_HANDLE_INVALID) {
        printf("No target connection handle; cannot connect HIDS client\n");
        return;
    }
    printf("[HIDS-CL] Connecting to HID service\n");
    uint8_t result = hids_client_connect(
        target.con_handle,
        hids_client_packet_handler,
        HID_PROTOCOL_MODE_REPORT,
        &hids_cid
    );

    switch (result) {
    case ERROR_CODE_SUCCESS:
        printf("[HIDS-CL] Connected\n");
        target_set_state(TARGET_STATE_CONNECTED);
        break;
    case ERROR_CODE_COMMAND_DISALLOWED:
        printf("[HIDS-CL] Already connected\n");
        target_set_state(TARGET_STATE_CONNECTED);
        break;
    default:
        break;
    }
}

static void hids_client_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size)
{
    UNUSED(channel);
    UNUSED(size);

    // Accept both forms due to upstream quirk: events can arrive as plain HCI_EVENT_PACKET
    // or already typed as HCI_EVENT_GATTSERVICE_META; filter only GATT Service meta
    if (packet_type == HCI_EVENT_PACKET) {
        if (hci_event_packet_get_type(packet) != HCI_EVENT_GATTSERVICE_META) return;
    } else if (packet_type != HCI_EVENT_GATTSERVICE_META) {
        return;
    }

    switch (hci_event_gattservice_meta_get_subevent_code(packet)) {
    case GATTSERVICE_SUBEVENT_HID_SERVICE_CONNECTED:
    {
        const uint8_t* desc = hids_client_descriptor_storage_get_descriptor_data(hids_cid, 0);
        uint16_t desc_len = hids_client_descriptor_storage_get_descriptor_len(hids_cid, 0);
        print_hid_descriptor(desc, desc_len);

        bool differs = (hid_descriptor_len != desc_len) || (memcmp(hid_descriptor, desc, desc_len) != 0);
        if (differs) {
            tlv_store_hid_descriptor(desc, desc_len);
            printf("[HIDS-CL] HID descriptor changed; rebooting in 100ms\n");
            watchdog_reboot(0, 0, 100);
        }
    }
    break;
    case GATTSERVICE_SUBEVENT_CLIENT_DISCONNECTED:
        printf("[HIDS-CL] Disconnected\n");
        if (target.valid) {
            target_set_state(TARGET_STATE_CONNECTING);
            attempt_hids_client_connect();
        } else {
            target_set_state(TARGET_STATE_SCANNING);
        }
        break;
    case GATTSERVICE_SUBEVENT_HID_REPORT:
    {
        const uint8_t* report_with_id = gattservice_subevent_hid_report_get_report(packet);
        uint16_t report_len_with_id = gattservice_subevent_hid_report_get_report_len(packet);
        uint8_t base_report_id = gattservice_subevent_hid_report_get_report_id(packet);
        print_hid_report(report_with_id, report_len_with_id);

        if (host.con_handle == HCI_CON_HANDLE_INVALID)
            break;

        // Apply macros via engine
        uint8_t transformed[1 + 1 + 1 + MAX_REPORT_LEN];
        uint8_t out_report_id = base_report_id;
        uint16_t out_len = macros_engine_apply(report_with_id, report_len_with_id,
                                               transformed, sizeof(transformed),
                                               &out_report_id);
        const uint8_t* out_bytes = (out_len ? transformed : report_with_id);
        if (!out_len) out_len = report_len_with_id;

        if (host.subscribed_reports_bitmap & (1u << out_report_id)) {
            const uint8_t* report_data = out_bytes;
            uint16_t report_len = out_len;
            if (out_report_id > 0) { report_data = out_bytes + 1; report_len = out_len - 1; }
            // Package for host send ring as: [report_id][payload...]
            uint8_t host_pkg[1 + MAX_REPORT_LEN];
            uint16_t host_pkg_len = 0;
            host_pkg[host_pkg_len++] = out_report_id;
            if (report_len > 0) { memcpy(&host_pkg[host_pkg_len], report_data, report_len); host_pkg_len += report_len; }
            if (rb_write(&hid_report_ring_buffer, host_pkg, host_pkg_len)) {
                hids_device_request_can_send_now_event(host.con_handle);
            } else {
                printf("[HIDS-HOST] Ring buffer full; dropped input report\n");
            }
        }

        if (host.enabled_notifications && host.con_handle != HCI_CON_HANDLE_INVALID) {
            // Build a composite debug frame without an extra marker:
            // New notify frame layout:
            //   [0] base_id
            //   [1] raw_len
            //   [...] raw_payload (without ID)
            //   [x] mod_id
            //   [x+1] mod_len
            //   [...] mod_payload (without ID)
            const uint8_t* raw_payload = report_with_id;
            uint16_t raw_len = report_len_with_id;
            uint8_t base_id = base_report_id;
            if (base_id > 0) { raw_payload = report_with_id + 1; raw_len = report_len_with_id - 1; }
            const uint8_t* mod_payload = out_bytes;
            uint16_t mod_len = out_len;
            uint8_t mod_id = out_report_id;
            if (mod_id > 0) { mod_payload = out_bytes + 1; mod_len = out_len - 1; }

            // Cap total payload to negotiated ATT MTU (minus ATT header) and ring buffer constraints
            uint16_t att_mtu = att_server_get_mtu(host.con_handle);
            uint16_t max_data = MAX_REPORT_LEN; // ring buffer item length cap
            if (att_mtu > 3) {
                uint16_t mtu_payload = (uint16_t)(att_mtu - 3);
                if (mtu_payload < max_data) max_data = mtu_payload;
            }
            // Minimum overhead: base_id + raw_len + mod_id + mod_len = 4
            if (max_data < 4) max_data = 4;
            uint16_t overhead = 4;
            uint16_t allowed = (max_data >= overhead) ? (max_data - overhead) : 0;
            uint16_t raw_fit = raw_len;
            uint16_t mod_fit = mod_len;
            if (raw_fit + mod_fit > allowed) {
                // Balance split so both sides are represented
                uint16_t raw_target = (uint16_t)(allowed / 2);
                if (raw_target < 1) raw_target = 1;
                uint16_t mod_target = (uint16_t)(allowed - raw_target);
                if (mod_target < 1) mod_target = 1;
                if (raw_fit > raw_target) raw_fit = raw_target;
                if (mod_fit > mod_target) mod_fit = mod_target;
                // If still exceeding allowed, adjust mod first then raw
                while ((uint16_t)(raw_fit + mod_fit) > allowed) {
                    if (mod_fit > 1) mod_fit--; else if (raw_fit > 1) raw_fit--; else break;
                }
            }

            // If either segment is too small to be useful, skip sending this debug frame
            if (raw_fit < 1 || mod_fit < 1) {
                break;
            }

            uint8_t comp[1 + 1 + MAX_REPORT_LEN + 1 + 1 + MAX_REPORT_LEN];
            uint16_t p = 0;
            comp[p++] = base_id;
            comp[p++] = (uint8_t)raw_fit;
            if (raw_fit) { memcpy(&comp[p], raw_payload, raw_fit); p += (uint16_t)raw_fit; }
            comp[p++] = mod_id;
            comp[p++] = (uint8_t)mod_fit;
            if (mod_fit) { memcpy(&comp[p], mod_payload, mod_fit); p += (uint16_t)mod_fit; }
            if (rb_write(&debug_report_ring_buffer, comp, p)) {
                att_server_request_can_send_now_event(host.con_handle);
            } else {
                printf("[ATT] Debug ring full; drop id=%u raw=%u mod=%u\n", base_id, (unsigned)raw_fit, (unsigned)mod_fit);
            }
        }

    }
    break;
    default:
        break;
    }
}

static void hids_host_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size)
{
    UNUSED(channel);
    UNUSED(size);

    if (packet_type != HCI_EVENT_PACKET) return;
    if (hci_event_packet_get_type(packet) != HCI_EVENT_HIDS_META) return;

    switch (hci_event_hids_meta_get_subevent_code(packet)) {
    case HIDS_SUBEVENT_INPUT_REPORT_ENABLE: {
        bool enabled = hids_subevent_input_report_enable_get_enable(packet) != 0;
        uint8_t report_id = hids_subevent_input_report_enable_get_report_id(packet);
        if (enabled) {
            host.subscribed_reports_bitmap |= (1u << report_id);
        } else {
            host.subscribed_reports_bitmap &= ~(1u << report_id);
        }
        host.report_protocol_mode = 1;
        printf("[HIDS-HOST] %s input report ID %u (con=0x%04X, bitmap=0x%08lX)\n", enabled ? "subscribed" : "unsubscribed", report_id, host.con_handle, host.subscribed_reports_bitmap);
        break;
    }
    case HIDS_SUBEVENT_BOOT_KEYBOARD_INPUT_REPORT_ENABLE:
        if (hids_subevent_boot_keyboard_input_report_enable_get_enable(packet)) {
            host.subscribed_reports_bitmap |= (1u << 1);
        } else {
            host.subscribed_reports_bitmap &= ~(1u << 1);
        }
        host.report_protocol_mode = 0;
        printf("[HIDS-HOST] Boot keyboard subscription (con=0x%04X, enabled=%u)\n", host.con_handle, (host.subscribed_reports_bitmap & (1u << 1)) != 0);
        break;
    case HIDS_SUBEVENT_PROTOCOL_MODE:
        host.report_protocol_mode = hids_subevent_protocol_mode_get_protocol_mode(packet);
        printf("[HIDS-HOST] Protocol mode: %s\n", host.report_protocol_mode ? "Report" : "Boot");
        break;
    case HIDS_SUBEVENT_CAN_SEND_NOW:
        if (!rb_is_empty(&hid_report_ring_buffer) && host.con_handle != HCI_CON_HANDLE_INVALID) {
            uint8_t report_buf[1 + MAX_REPORT_LEN];
            int16_t total_len = rb_read(&hid_report_ring_buffer, report_buf, sizeof(report_buf));

            if (total_len > 0 && total_len >= 1) {
                uint8_t report_id = report_buf[0];
                uint16_t report_len = (uint16_t)(total_len - 1);
                printf("[HIDS-HOST] Sending input report (id=%u, len=%u, mode=%s)\n", report_id, report_len, host.report_protocol_mode == 0 ? "Boot" : "Report");
                if (host.report_protocol_mode == 0) {
                    // Boot protocol doesn't use report IDs, but we assume it's a keyboard report
                    hids_device_send_boot_keyboard_input_report(host.con_handle, &report_buf[1], report_len);
                } else {
                    hids_device_send_input_report_for_id(host.con_handle, report_id, &report_buf[1], report_len);
                }
            }

            // If there are more reports, request to send again
            if (!rb_is_empty(&hid_report_ring_buffer)) {
                hids_device_request_can_send_now_event(host.con_handle);
            }
        }
        break;
    default:
        break;
    }
}

static void sm_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size)
{
    UNUSED(channel);
    UNUSED(size);

    if (packet_type != HCI_EVENT_PACKET)
        return;
    hci_con_handle_t con_handle;
    uint8_t status;
    switch (hci_event_packet_get_type(packet)) {
    case SM_EVENT_JUST_WORKS_REQUEST:
        printf("[SM] Just Works requested\n");
        sm_just_works_confirm(sm_event_just_works_request_get_handle(packet));
        break;
    case SM_EVENT_NUMERIC_COMPARISON_REQUEST:
        printf("[SM] Confirm numeric comparison: %" PRIu32 "\n",
               sm_event_numeric_comparison_request_get_passkey(packet));
        sm_numeric_comparison_confirm(
            sm_event_numeric_comparison_request_get_handle(packet));
        break;
    case SM_EVENT_PASSKEY_DISPLAY_NUMBER:
        printf("[SM] Display passkey: %" PRIu32 "\n",
               sm_event_passkey_display_number_get_passkey(packet));
        break;
    case SM_EVENT_PAIRING_COMPLETE:
        status = sm_event_pairing_complete_get_status(packet);
        con_handle = sm_event_pairing_complete_get_handle(packet);
        switch (status) {
        case ERROR_CODE_SUCCESS:
            printf("[SM] Pairing success\n");
            if (con_handle == target.con_handle) {
                attempt_hids_client_connect();
            } else if (con_handle == host.con_handle) {
                printf("[SM] Host paired successfully\n");
            }
            break;
        case ERROR_CODE_CONNECTION_TERMINATED_DUE_TO_MIC_FAILURE:
            printf("[SM] Pairing failed: MIC failure; deleting bond and retrying\n");
            if (con_handle == target.con_handle) {
                gap_delete_bonding(target.addr_type, target.addr);
                target_set_state(TARGET_STATE_SCANNING);
            } else if (con_handle == host.con_handle) {
                gap_delete_bonding(host.addr_type, host.addr);
                disconnect_host();
            }
            break;
        default:
            printf("[SM] Pairing failed, reason=%u\n", sm_event_pairing_complete_get_reason(packet));
            if (con_handle == target.con_handle) {
                target_set_state(TARGET_STATE_SCANNING);
            } else if (con_handle == host.con_handle) {
                disconnect_host();
            }
            break;
        }
        break;
    case SM_EVENT_REENCRYPTION_STARTED:
        con_handle = sm_event_reencryption_started_get_handle(packet);
        if (con_handle == target.con_handle) {
            printf("[SM] Target re-encryption started\n");
            target_set_state(TARGET_STATE_CONNECTING);
        } else if (con_handle == host.con_handle) {
            printf("[SM] Host re-encryption started\n");
        } else {
            printf("[SM] Re-encryption started for unknown connection\n");
        }
        break;
    case SM_EVENT_REENCRYPTION_COMPLETE:
        status = sm_event_reencryption_complete_get_status(packet);
        con_handle = sm_event_reencryption_complete_get_handle(packet);
        switch (status) {
        case ERROR_CODE_SUCCESS:
            if (target.con_handle == con_handle) {
                printf("[SM] Target re-encryption complete\n");
                attempt_hids_client_connect();
            } else if (host.con_handle == con_handle) {
                printf("[SM] Host re-encryption complete\n");
            }
            break;
        default:
            printf("[SM] Re-encryption failed, status %u\n", status);
            if (con_handle == target.con_handle) {
                printf("[SM] Retrying target pairing\n");
                sm_request_pairing(con_handle);
            } else if (con_handle == host.con_handle) {
                printf("Disconnecting host due to re-encryption failure\n");
                disconnect_host();
            }
            break;
        }
    default:
        break;
    }
}



static uint16_t att_read_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t offset, uint8_t* buffer, uint16_t buffer_size)
{
    static uint8_t devices_buf[SEEN_DEVICES_CHAR_BUFFER_SIZE];
    static uint16_t devices_buf_len = 0;
    static uint8_t target_buf[64];
    static uint16_t target_buf_len = 0;

    switch (att_handle) {
    case ATT_CHARACTERISTIC_12345678_90AB_CDEF_0123_456789ABCDF0_01_VALUE_HANDLE:
        if (offset == 0) {
            devices_buf_len = seen_devices_build_binary(devices_buf, sizeof(devices_buf));
        }
        return att_read_callback_handle_blob(devices_buf, devices_buf_len, offset, buffer, buffer_size);
    case ATT_CHARACTERISTIC_12345678_90AB_CDEF_0123_456789ABCDF1_01_VALUE_HANDLE:
        if (offset == 0) {
            if (target.valid) {
                uint8_t name_len = (uint8_t)strnlen(target.name, sizeof(target.name));
                uint16_t p = 0;
                memcpy(&target_buf[p], target.addr, 6); p += 6;
                target_buf[p++] = (uint8_t)target.addr_type;
                target_buf[p++] = name_len;
                if (name_len) { memcpy(&target_buf[p], target.name, name_len); p = (uint16_t)(p + name_len); }
                target_buf_len = p;
            } else {
                target_buf_len = 0;
            }
        }
        return att_read_callback_handle_blob(target_buf, target_buf_len, offset, buffer, buffer_size);
    case ATT_CHARACTERISTIC_12345678_90AB_CDEF_0123_456789ABCDF2_01_VALUE_HANDLE:
        return att_read_callback_handle_blob((uint8_t*)"Macro ITM Device", 17, offset, buffer, buffer_size);
    case ATT_CHARACTERISTIC_12345678_90AB_CDEF_0123_456789ABCDF3_01_VALUE_HANDLE: {
        static uint8_t dump[1400];
        static uint16_t dump_len = 0;
        if (offset == 0) { dump_len = macros_engine_dump_blob(dump, sizeof(dump)); }
        return att_read_callback_handle_blob(dump, dump_len, offset, buffer, buffer_size);
    }
    default:
        return 0;
    }
    return 0;
};

static int att_write_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t transaction_mode, uint16_t offset, uint8_t* buffer, uint16_t buffer_size)
{
    if (transaction_mode != ATT_TRANSACTION_MODE_NONE || offset != 0) return 0;

    switch (att_handle) {
    case ATT_CHARACTERISTIC_12345678_90AB_CDEF_0123_456789ABCDF0_01_VALUE_HANDLE:
        // Binary control: opcode 0x00 = CLEAR
        if (buffer_size >= 1 && buffer[0] == 0x00) { seen_devices_clear(); }
        break;
    case ATT_CHARACTERISTIC_12345678_90AB_CDEF_0123_456789ABCDF1_01_VALUE_HANDLE: {
        // Binary target set: opcode 0x01 = SET, 0x00 = CLEAR
        if (buffer_size >= 1 && buffer[0] == 0x00) {
            clear_target_device();
            break;
        }
        if (buffer_size >= 1 && buffer[0] == 0x01) {
            // payload: [0x01][addr(6)][type(1)][name_len(1)][name]
            if (buffer_size < 1 + 6 + 1 + 1) { printf("[TARGET] Invalid target payload\n"); break; }
            target_device_t temp_device = (target_device_t){ {0}, 0, "", HCI_CON_HANDLE_INVALID, 0 };
            memcpy(temp_device.addr, &buffer[1], 6);
            temp_device.addr_type = buffer[7];
            uint8_t name_len = buffer[8];
            if (1 + 6 + 1 + 1 + name_len > buffer_size) name_len = (uint8_t)(buffer_size - (1 + 6 + 1 + 1));
            uint8_t cpy = name_len < sizeof(temp_device.name) - 1 ? name_len : (uint8_t)(sizeof(temp_device.name) - 1);
            if (cpy) memcpy(temp_device.name, &buffer[9], cpy);
            temp_device.name[cpy] = '\0';
            temp_device.valid = true;

            if (target.valid && bd_addr_cmp(temp_device.addr, target.addr) == 0) {
                printf("[TARGET] Device unchanged\n");
                break;
            }
            if (target.valid) clear_target_device();
            target = temp_device;
            tlv_store_target_device(&target);
            printf("[TARGET] Set to %s (type=%u)\n", bd_addr_to_str(temp_device.addr), temp_device.addr_type);
        }
        break;
    }
    case ATT_CHARACTERISTIC_12345678_90AB_CDEF_0123_456789ABCDF2_01_CLIENT_CONFIGURATION_HANDLE:
        if (connection_handle != host.con_handle) {
            printf("[ATT] Ignore client config write from non-host\n");
            break;
        }
        host.enabled_notifications = little_endian_read_16(buffer, 0) == GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION;
        printf("[ATT] Host log notifications %s\n", host.enabled_notifications ? "enabled" : "disabled");
        break;
    case ATT_CHARACTERISTIC_12345678_90AB_CDEF_0123_456789ABCDF3_01_VALUE_HANDLE: {
        // Binary control: 0x00=CLEAR, 0x01=BEGIN, 0x02=COMMIT, else = raw chunk
        if (buffer_size >= 1 && buffer[0] <= 0x02 && buffer_size == 1) {
            if (buffer[0] == 0x00) { macros_engine_clear_and_persist_to_tlv(); printf("[MACROS] Cleared\n"); } else if (buffer[0] == 0x01) { macros_engine_begin_upload(); printf("[MACROS] Upload BEGIN\n"); } else if (buffer[0] == 0x02) { bool ok = macros_engine_commit_and_persist_to_tlv(); printf(ok ? "[MACROS] COMMIT\n" : "[MACROS] COMMIT failed: parse error\n"); }
        } else {
            if (!macros_engine_append_chunk(buffer, buffer_size)) {
                printf("[MACROS] Upload buffer overflow; dropped chunk len=%u\n", buffer_size);
            }
        }
        break;
    }
    default:
        printf("[ATT] Write to unknown handle 0x%04X\n", att_handle);
        break;
    }
    return 0;
}

static void att_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size)
{
    UNUSED(channel);
    UNUSED(size);

    // reduce ATT event verbosity
    if (packet_type != HCI_EVENT_PACKET)
        return;

    switch (hci_event_packet_get_type(packet)) {
    case ATT_EVENT_CAN_SEND_NOW:
        if (host.con_handle != HCI_CON_HANDLE_INVALID && !rb_is_empty(&debug_report_ring_buffer)) {
            uint8_t buf[1 + MAX_REPORT_LEN];
            int16_t len = rb_read(&debug_report_ring_buffer, buf, sizeof(buf));
            if (len > 0) {
                att_server_notify(host.con_handle,
                                  ATT_CHARACTERISTIC_12345678_90AB_CDEF_0123_456789ABCDF2_01_VALUE_HANDLE,
                                  buf,
                                  (uint16_t)len);
            }
            if (!rb_is_empty(&debug_report_ring_buffer)) {
                att_server_request_can_send_now_event(host.con_handle);
            }
        }
        break;
    default:
        break;
    }
}

int btstack_main(int argc, const char* argv[])
{
    UNUSED(argc);
    UNUSED(argv);

    tlv_utils_init();
    macros_engine_init();
    macros_engine_load_from_tlv();
    rb_init(&hid_report_ring_buffer);
    rb_init(&debug_report_ring_buffer);

    // setup transport layer
    l2cap_init();

    // setup security: handles pairing, authentication, encryption
    sm_init();
    sm_set_io_capabilities(IO_CAPABILITY_NO_INPUT_NO_OUTPUT);
    sm_set_authentication_requirements(SM_AUTHREQ_SECURE_CONNECTION | SM_AUTHREQ_BONDING);
    sm_event_callback_registration.callback = &sm_packet_handler;
    sm_add_event_handler(&sm_event_callback_registration);

    gatt_client_init();

    hids_client_init(hid_descriptor_storage, sizeof(hid_descriptor_storage));

    // setup ATT server (read + write callbacks, packet handler for ATT_EVENT_CAN_SEND_NOW)
    att_server_init(profile_data, att_read_callback, att_write_callback);
    att_server_register_packet_handler(&att_packet_handler);

    device_information_service_server_init();

    // setup HID Device service if descriptor is available
    memset(hid_descriptor, 0, sizeof(hid_descriptor));
    hid_descriptor_len = tlv_load_hid_descriptor(hid_descriptor, sizeof(hid_descriptor));
    printf("[HID-DEV] Descriptor length from TLV: %u\n", hid_descriptor_len);
    if (hid_descriptor_len > 0) {
        printf("[HID-DEV] Loaded from TLV; initializing HID Device Service\n");
        hids_device_init(0, hid_descriptor, hid_descriptor_len);
    } else {
        printf("[HID-DEV] No descriptor in TLV; using default boot keyboard descriptor\n");
        hids_device_init(0, hid_descriptor_keyboard_boot_mode, sizeof(hid_descriptor_keyboard_boot_mode));
    }

    // setup advertisements
    uint16_t adv_int_min = 0x0030;
    uint16_t adv_int_max = 0x0030;
    uint8_t adv_type = 0;
    bd_addr_t null_addr;
    memset(null_addr, 0, 6);
    gap_advertisements_set_params(adv_int_min, adv_int_max, adv_type, 0, null_addr, 0x07, 0x00);
    gap_advertisements_set_data(config_adv_data_len, (uint8_t*)config_adv_data);
    gap_scan_response_set_data(scan_response_data_len, (uint8_t*)scan_response_data);

    // Register for HCI events, main communication channel between btstack and application
    hci_event_callback_registration.callback = &hci_packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    // Register for hids events from host
    hids_device_register_packet_handler(&hids_host_packet_handler);

    // (stdout buffering tweak removed; keep defaults)

    {
        target_device_t restored = tlv_load_target_device();
        if (restored.valid) {
            target = restored;
            printf("[TLV] Restored target: %s (type=%u, name=\"%s\")\n", bd_addr_to_str(target.addr), target.addr_type, target.name);
        }
    }

    gap_set_scan_params(1, 0x0030, 0x0030, 0);

    hci_power_control(HCI_POWER_ON);

    return 0;
}

int main(int argc, const char* argv[])
{
    stdio_init_all();

    if (cyw43_arch_init()) {
        printf("[APP] Failed to initialise cyw43_arch\n");
        return -1;
    }

    btstack_main(argc, argv);

    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    btstack_run_loop_execute();
}

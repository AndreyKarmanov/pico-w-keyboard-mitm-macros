/*
 * Copyright (C) 2020 BlueKitchen GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 4. Any redistribution, use, or modification is done solely for
 *    personal benefit and not for any commercial purpose or for
 *    monetary gain.
 *
 * THIS SOFTWARE IS PROVIDED BY BLUEKITCHEN GMBH AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL BLUEKITCHEN
 * GMBH OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Please inquire about commercial licensing options at
 * contact@bluekitchen-gmbh.com
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include <btstack_tlv.h>

#include "pico/cyw43_arch.h"
#include "pico/stdio.h"
#include "pico/time.h"

#include "macro_itm.h"
#include "btstack_config.h"
#include "btstack.h"

#include "ble/gatt-service/battery_service_server.h"
#include "ble/gatt-service/device_information_service_server.h"
#include "ble/gatt-service/hids_device.h"
 // Simplified US Keyboard with Shift modifier

#define CHAR_ILLEGAL     0xff
#define CHAR_RETURN     '\n'
#define CHAR_ESCAPE      27
#define CHAR_TAB         '\t'
#define CHAR_BACKSPACE   0x7f

/**
 * English (US)
 */
static const uint8_t keytable_us_none[] = {
        CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL,             /*   0-3 */
        'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j',                   /*  4-13 */
        'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't',                   /* 14-23 */
        'u', 'v', 'w', 'x', 'y', 'z',                                       /* 24-29 */
        '1', '2', '3', '4', '5', '6', '7', '8', '9', '0',                   /* 30-39 */
        CHAR_RETURN, CHAR_ESCAPE, CHAR_BACKSPACE, CHAR_TAB, ' ',            /* 40-44 */
        '-', '=', '[', ']', '\\', CHAR_ILLEGAL, ';', '\'', 0x60, ',',       /* 45-54 */
        '.', '/', CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL,   /* 55-60 */
        CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL,             /* 61-64 */
        CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL,             /* 65-68 */
        CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL,             /* 69-72 */
        CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL,             /* 73-76 */
        CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL,             /* 77-80 */
        CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL,             /* 81-84 */
        '*', '-', '+', '\n', '1', '2', '3', '4', '5',                       /* 85-97 */
        '6', '7', '8', '9', '0', '.', 0xa7,                                 /* 97-100 */
};

static const uint8_t keytable_us_shift[] = {
        CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL,             /*  0-3  */
        'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J',                   /*  4-13 */
        'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T',                   /* 14-23 */
        'U', 'V', 'W', 'X', 'Y', 'Z',                                       /* 24-29 */
        '!', '@', '#', '$', '%', '^', '&', '*', '(', ')',                   /* 30-39 */
        CHAR_RETURN, CHAR_ESCAPE, CHAR_BACKSPACE, CHAR_TAB, ' ',            /* 40-44 */
        '_', '+', '{', '}', '|', CHAR_ILLEGAL, ':', '"', 0x7E, '<',         /* 45-54 */
        '>', '?', CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL,   /* 55-60 */
        CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL,             /* 61-64 */
        CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL,             /* 65-68 */
        CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL,             /* 69-72 */
        CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL,             /* 73-76 */
        CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL,             /* 77-80 */
        CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL,             /* 81-84 */
        '*', '-', '+', '\n', '1', '2', '3', '4', '5',                       /* 85-97 */
        '6', '7', '8', '9', '0', '.', 0xb1,                                 /* 97-100 */
};



typedef struct {
    bd_addr_t addr;
    bd_addr_type_t addr_type;
} le_device_addr_t;

// W4 = wait for
static enum {
    W4_WORKING,
    // start keyboard connection
    W4_HID_DEVICE_FOUND_INCOMING,
    W4_CONNECTED_INCOMING,
    W4_ENCRYPTED_INCOMING,
    W4_HID_CLIENT_CONNECTED,
    W4_HOST,
    W4_TIMEOUT_THEN_SCAN,
    W4_TIMEOUT_THEN_RECONNECT,
    // end keyboard connection
    W4_HOST_SUBSCRIPTION,
    READY,
} app_state;

static btstack_timer_source_t connection_timer;

static btstack_packet_callback_registration_t hci_event_callback_registration;
static btstack_packet_callback_registration_t sm_event_callback_registration;

// common vars
static hci_con_handle_t con_handle_keyboard = HCI_CON_HANDLE_INVALID;
static hci_con_handle_t con_handle_host = HCI_CON_HANDLE_INVALID;

// report or boot protocol
static hid_protocol_mode_t protocol_mode_incoming = HID_PROTOCOL_MODE_REPORT;
static hid_protocol_mode_t protocol_mode_host = HID_PROTOCOL_MODE_REPORT;

// keyboard (incoming) vars
static le_device_addr_t remote_device_keyboard;
static uint16_t hids_cid;
static uint8_t hid_descriptor_storage[500];
// tlv stuff
#define TLV_TAG_HOGD ((((uint32_t) 'H') << 24 ) | (((uint32_t) 'O') << 16) | (((uint32_t) 'G') << 8) | 'D')
static const btstack_tlv_t* btstack_tlv_singleton_impl;
static void* btstack_tlv_singleton_context;

// host (pc) (outgoing) vars
static uint8_t battery = 100;
const uint8_t adv_data[] = {
    // Flags general discoverable, BR/EDR not supported
    0x02, BLUETOOTH_DATA_TYPE_FLAGS, 0x06,
    // Name
    0x0d, BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME, 'M', 'I', 'T', 'M', ' ', 'K', 'e', 'y', 'b', 'o', 'a', 'r',
    // 16-bit Service UUIDs
    0x03, BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_16_BIT_SERVICE_CLASS_UUIDS, ORG_BLUETOOTH_SERVICE_HUMAN_INTERFACE_DEVICE & 0xff, ORG_BLUETOOTH_SERVICE_HUMAN_INTERFACE_DEVICE >> 8,
    // Appearance HID - Keyboard (Category 15, Sub-Category 1)
    0x03, BLUETOOTH_DATA_TYPE_APPEARANCE, 0xC1, 0x03,
};
const uint8_t adv_data_len = sizeof(adv_data);
// from USB HID Specification 1.1, Appendix B.1
const uint8_t hid_descriptor_keyboard_boot_mode[] = {

    0x05, 0x01,                    // Usage Page (Generic Desktop)
    0x09, 0x06,                    // Usage (Keyboard)
    0xa1, 0x01,                    // Collection (Application)

    0x85,  0x01,                   // Report ID 1

    // Modifier byte

    0x75, 0x01,                    //   Report Size (1)
    0x95, 0x08,                    //   Report Count (8)
    0x05, 0x07,                    //   Usage Page (Key codes)
    0x19, 0xe0,                    //   Usage Minimum (Keyboard LeftControl)
    0x29, 0xe7,                    //   Usage Maxium (Keyboard Right GUI)
    0x15, 0x00,                    //   Logical Minimum (0)
    0x25, 0x01,                    //   Logical Maximum (1)
    0x81, 0x02,                    //   Input (Data, Variable, Absolute)

    // Reserved byte

    0x75, 0x01,                    //   Report Size (1)
    0x95, 0x08,                    //   Report Count (8)
    0x81, 0x03,                    //   Input (Constant, Variable, Absolute)

    // LED report + padding

    0x95, 0x05,                    //   Report Count (5)
    0x75, 0x01,                    //   Report Size (1)
    0x05, 0x08,                    //   Usage Page (LEDs)
    0x19, 0x01,                    //   Usage Minimum (Num Lock)
    0x29, 0x05,                    //   Usage Maxium (Kana)
    0x91, 0x02,                    //   Output (Data, Variable, Absolute)

    0x95, 0x01,                    //   Report Count (1)
    0x75, 0x03,                    //   Report Size (3)
    0x91, 0x03,                    //   Output (Constant, Variable, Absolute)

    // Keycodes

    0x95, 0x06,                    //   Report Count (6)
    0x75, 0x08,                    //   Report Size (8)
    0x15, 0x00,                    //   Logical Minimum (0)
    0x25, 0xff,                    //   Logical Maximum (1)
    0x05, 0x07,                    //   Usage Page (Key codes)
    0x19, 0x00,                    //   Usage Minimum (Reserved (no event indicated))
    0x29, 0xff,                    //   Usage Maxium (Reserved)
    0x81, 0x00,                    //   Input (Data, Array)

    0xc0,                          // End collection
};


const uint8_t hid_descriptor_keyabord_mx_mechanical[] = {
    0x05, 0x01,                    // Usage Page (Generic Desktop)
    0x09, 0x06,                    // Usage (Keyboard)
    0xa1, 0x01,                    // Collection (Application)

    0x85, 0x01,                    // Report ID 1

    // modifier bytes
    0x05, 0x07,                    // Usage Page (Key codes)
    0x19, 0xE0,                    // Usage Minimum (Keyboard LeftControl)
    0x29, 0xE7,                    // Usage Maximum (Keyboard Right GUI)
    0x15, 0x00,                    // Logical Minimum (0)
    0x25, 0x01,                    // Logical Maximum (1)
    0x75, 0x01,                    // Report Size (1)
    0x95, 0x08,                    // Report Count (8)
    0x81, 0x02,                    // Input (Data, Variable, Absolute)

    // LEDs
    0x95, 0x05,                    // Report Count (5)
    0x05, 0x08,                    // Usage Page (LEDs)
    0x19, 0x01,                    // Usage Minimum (Num Lock)
    0x29, 0x05,                    // Usage Maximum (Kana)
    0x91, 0x02,                    // Output (Data, Variable, Absolute)

    // padding
    0x95, 0x01,                    // Report Count (1) 
    0x75, 0x03,                    // Report Size (3)
    0x91, 0x01,                    // Output (Constant)

    // keycodes
    0x95, 0x06,                    // Report Count (6)
    0x75, 0x08,                    // Report Size (8)
    0x15, 0x00,                    // Logical Minimum (0)
    0x26, 0xA4, 0x00,              // Logical Maximum (164)
    0x05, 0x07,                    // Usage Page (Key codes)
    0x19, 0x00,                    // Usage Minimum (0)
    0x2A, 0xA4, 0x00,              // Usage Maximum (164)
    0x81, 0x00,                    // Input (Data, Array)

    0xc0,                          // End collection

    // Consumer Control
    // 0x05, 0x0C,                    // Usage Page (Consumer)
    // 0x09, 0x01,                    // Usage (Consumer Control)
    // 0xA1, 0x01,                    // Collection (Application)
    // 0x85, 0x03,                    // Report ID (3)
    // 0x75, 0x10,                    // Report Size (16)
    // 0x95, 0x02,                    // Report Count (2)
    // 0x15, 0x01,                    // Logical Minimum (1)
    // 0x26, 0xFF, 0x02,              // Logical Maximum (767)
    // 0x19, 0x01,                    // Usage Minimum (1)
    // 0x2A, 0xFF, 0x02,              // Usage Maximum (767)
    // 0x81, 0x60,                    // Input (Data, Array, Absolute)
    // 0xc0,                          // End collection

    // Vendor Defined
    // 0x06, 0x43, 0xFF,             // Usage Page (Vendor Defined)
    // 0x0A, 0x02, 0x02,             // Usage (Vendor Defined)
    // 0xA1, 0x01,                    // Collection (Application)
    // 0x85, 0x11,                    // Report ID (17)
    // 0x75, 0x08,                    // Report Size (8)
    // 0x95, 0x13,                    // Report Count (19)
    // 0x15, 0x00,                    // Logical Minimum (0)
    // 0x26, 0xFF, 0x00,             // Logical Maximum (255)
    // 0x09, 0x02,                    // Usage (Vendor Defined)
    // 0x81, 0x00,                    // Input (Data, Array)
    // 0x09, 0x02,                    // Usage (Vendor Defined)
    // 0x91, 0x00,                    // Output (Data, Array)
    // 0xc0                           // End collection
};

static void human_readable_report(const uint8_t* report, uint16_t report_len) {
    for (int i = 0; i < report_len; i++) {
        printf("%02x ", report[i]);
    }
    printf("\n");
}

// modifier, 0, keycodes*6
uint8_t out_report[] = { 0, 0, 0, 0, 0, 0, 0 };
uint8_t out_boot_report[] = { 0, 0, 0, 0, 0, 0, 0, 0 };
static void apply_macros(const uint8_t modifiers, const uint8_t* keycodes, uint16_t keycodes_len) {
    // this takes in the modifiers + keycodes, and then updates out_report based on them. 
    // out_report is then sent to the host
    printf("applying macros\n%02x ", modifiers);
    human_readable_report(keycodes, keycodes_len);
    
    out_report[0] = modifiers; // modifiers are unchanged

    // bool shift = (modifiers & 0x02) != 0; // shift is pressed example


    // https://usb.org/sites/default/files/hut1_5.pdf

    bool capsLock = false;
    uint16_t out_keycodes_len = 0;
    for (int i = 0; i < keycodes_len; i++) {
        uint8_t keycode = keycodes[i];
        if (keycode == HID_USAGE_KEY_KEYBOARD_CAPS_LOCK) {
            capsLock = true;
            continue;
        }
        if (capsLock) {
            if (keycode == 0x04 || keycode == 0x0D) { // caps + (a || j)= left arrow
                keycode = 0x50;
            } else if (keycode == 0x07 || keycode == 0x0F) { // caps + (d || l) = right arrow
                keycode = 0x4F;
            } else if (keycode == 0x1A || keycode == 0x0C) { // caps + (w || i) = up arrow
                keycode = 0x52;
            } else if (keycode == 0x16 || keycode == 0x0E) { // caps + (s || k) = down arrow
                keycode = 0x51;
            } else if (keycode == 0x1C) { // y = volume up
                keycode = 0x80;
            } else if (keycode == 0x0B) { // h = volume dowm
                keycode = 0x81;
            } else if (keycode == 0x11) { // n = volume mute
                keycode = 0x7F;
            }
        }
        out_report[1 + out_keycodes_len++] = keycode;
    }

    human_readable_report(out_report, sizeof(out_report));
    hids_device_request_can_send_now_event(con_handle_host);
}

/*
    START OF INCOMING (KEYBOARD) CODE
*/


static void hid_handle_input_report(uint8_t service_index, const uint8_t* report, uint16_t report_len) {
    // check if HID Input Report
    if (report_len < 1) return;
    if (app_state != READY) {
        printf("HID Input Report, but not ready\n");
        return;
    }
    switch (protocol_mode_incoming) {
    case HID_PROTOCOL_MODE_BOOT:
        apply_macros(report[0], report + 2, report_len - 2); // skip reserved byte
        break;
    default:
        apply_macros(report[1], report + 2, report_len - 2); // skip id
        break;
    }
}

/**
 * @section Test if advertisement contains HID UUID
 * @param packet
 * @param size
 * @returns true if it does
 */
static bool adv_event_contains_hid_service(const uint8_t* packet) {
    const uint8_t* ad_data = gap_event_advertising_report_get_data(packet);
    uint8_t ad_len = gap_event_advertising_report_get_data_length(packet);
    return ad_data_contains_uuid16(ad_len, ad_data, ORG_BLUETOOTH_SERVICE_HUMAN_INTERFACE_DEVICE);
}

/**
 * Start scanning
 */
static void hog_start_scan(void) {
    printf("Scanning for LE HID devices...\n");
    app_state = W4_HID_DEVICE_FOUND_INCOMING;
    // Passive scanning, 100% (scan interval = scan window)
    gap_set_scan_parameters(0, 48, 48);
    gap_start_scan();
}

/**
 * Handle timeout for outgoing connection
 * @param ts
 */
static void hog_connection_timeout(btstack_timer_source_t* ts) {
    UNUSED(ts);
    printf("Timeout - abort connection\n");
    gap_connect_cancel();
    hog_start_scan();
}


/**
 * Connect to remote device but set timer for timeout
 */
static void hog_connect(void) {
    // set timer
    btstack_run_loop_set_timer(&connection_timer, 10000);
    btstack_run_loop_set_timer_handler(&connection_timer, &hog_connection_timeout);
    btstack_run_loop_add_timer(&connection_timer);
    app_state = W4_CONNECTED_INCOMING;
    gap_connect(remote_device_keyboard.addr, remote_device_keyboard.addr_type);
}

/**
 * Handle timer event to trigger reconnect
 * @param ts
 */
static void hog_reconnect_timeout(btstack_timer_source_t* ts) {
    UNUSED(ts);
    switch (app_state) {
    case W4_TIMEOUT_THEN_RECONNECT:
        hog_connect();
        break;
    case W4_TIMEOUT_THEN_SCAN:
        hog_start_scan();
        break;
    default:
        break;
    }
}

/**
 * Start connecting after boot up: connect to last used device if possible, start scan otherwise
 */
static void hog_start_connect(void) {

    // check if we have a bonded HID device
    btstack_tlv_get_instance(&btstack_tlv_singleton_impl, &btstack_tlv_singleton_context);
    if (btstack_tlv_singleton_impl) {
        int len = btstack_tlv_singleton_impl->get_tag(btstack_tlv_singleton_context, TLV_TAG_HOGD, (uint8_t*)&remote_device_keyboard, sizeof(remote_device_keyboard));
        if (len == sizeof(remote_device_keyboard)) {
            printf("Bonded, connect to device with %s address %s ...\n", remote_device_keyboard.addr_type == 0 ? "public" : "random", bd_addr_to_str(remote_device_keyboard.addr));
            hog_connect();
            return;
        }
    }

    // otherwise, scan for HID devices
    hog_start_scan();
}

/**
 * In case of error, disconnect and start scanning again
 */
static void handle_outgoing_connection_error(void) {
    printf("Error occurred, disconnect and start over\n");
    gap_disconnect(con_handle_keyboard);
    hog_start_scan();
}

/*
    START OF OUTGOING (HOST)
*/

// HID Report sending
static void send_report() {
    switch (protocol_mode_host) {
    case 0:
        out_boot_report[0] = out_report[0];
        memcpy(out_boot_report + 2, out_report + 1, 6);
        printf("Sending boot report\n");
        human_readable_report(out_boot_report, sizeof(out_boot_report));
        hids_device_send_boot_keyboard_input_report(con_handle_host, out_report, sizeof(out_report));
        break;
    case 1:
        hids_device_send_input_report(con_handle_host, out_report, sizeof(out_report));
        break;
    default:
        break;
    }
}

/**
 * Handle GATT Client Events dependent on current state
 *
 * @param packet_type
 * @param channel
 * @param packet
 * @param size
 */
static void handle_gatt_client_event(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size) {
    UNUSED(packet_type);
    UNUSED(channel);
    UNUSED(size);

    uint8_t status;

    if (hci_event_packet_get_type(packet) != HCI_EVENT_GATTSERVICE_META) {
        return;
    }

    switch (hci_event_gattservice_meta_get_subevent_code(packet)) {
    case GATTSERVICE_SUBEVENT_HID_SERVICE_CONNECTED:
        status = gattservice_subevent_hid_service_connected_get_status(packet);
        switch (status) {
        case ERROR_CODE_SUCCESS:
            printf("HID service client connected, found %d services\n",
                gattservice_subevent_hid_service_connected_get_num_instances(packet));

            // store device as bonded
            if (btstack_tlv_singleton_impl) {
                btstack_tlv_singleton_impl->store_tag(btstack_tlv_singleton_context, TLV_TAG_HOGD, (const uint8_t*)&remote_device_keyboard, sizeof(remote_device_keyboard));
            }
            // done
            printf("Found incoming - please start typing or mousing..\n");
            app_state = W4_HOST;
            // in future, we can set the io capabilities at this point ( for more secure bonding ! )
            gap_advertisements_enable(1);
            break;
        default:
            printf("HID service client connection failed, status 0x%02x.\n", status);
            handle_outgoing_connection_error();
            break;
        }
        break;

    case GATTSERVICE_SUBEVENT_HID_SERVICE_DISCONNECTED:
        printf("HID service client disconnected\n");
        hog_start_connect();
        break;

    case GATTSERVICE_SUBEVENT_HID_REPORT:
        hid_handle_input_report(
            gattservice_subevent_hid_report_get_service_index(packet),
            gattservice_subevent_hid_report_get_report(packet),
            gattservice_subevent_hid_report_get_report_len(packet));
        break;

    default:
        break;
    }
}

/* LISTING_START(packetHandler): Packet Handler */
static void hci_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size) {
    /* LISTING_PAUSE */
    UNUSED(channel);
    UNUSED(size);

    if (packet_type != HCI_EVENT_PACKET) return;

    switch (hci_event_packet_get_type(packet)) {
    case BTSTACK_EVENT_STATE:
        if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING) break;
        btstack_assert(app_state == W4_WORKING);
        hog_start_connect();
        break;
    case GAP_EVENT_ADVERTISING_REPORT:
        // looking for HID device
        if (app_state != W4_HID_DEVICE_FOUND_INCOMING) break;
        if (adv_event_contains_hid_service(packet) == false) break;
        // stop scan
        gap_stop_scan();
        // store remote device address and type
        gap_event_advertising_report_get_address(packet, remote_device_keyboard.addr);
        remote_device_keyboard.addr_type = gap_event_advertising_report_get_address_type(packet);
        // connect
        printf("Found, connect to device with %s address %s ...\n", remote_device_keyboard.addr_type == 0 ? "public" : "random", bd_addr_to_str(remote_device_keyboard.addr));
        hog_connect();
        break;
    case HCI_EVENT_DISCONNECTION_COMPLETE:
        uint16_t con_handle = hci_event_disconnection_complete_get_connection_handle(packet);

        // if we lost host, we go back to W4_host. we don't initiate host connection, 
        // but we do need to start advertising again I think(?)
        if (con_handle == con_handle_host) {
            printf("\nDisconnected from host, restarting advertising\n");
            app_state = W4_HOST;
            con_handle_host = HCI_CON_HANDLE_INVALID;
        }

        if (con_handle == con_handle_keyboard) {
            printf("\nDisconnected from keyboard, trying to reconnect...\n");
            con_handle_keyboard = HCI_CON_HANDLE_INVALID;
            app_state = W4_TIMEOUT_THEN_RECONNECT;

            btstack_run_loop_set_timer(&connection_timer, 100);
            btstack_run_loop_set_timer_handler(&connection_timer, &hog_reconnect_timeout);
            btstack_run_loop_add_timer(&connection_timer);
        }
        break;
    case HCI_EVENT_META_GAP:
        // wait for connection complete
        if (hci_event_gap_meta_get_subevent_code(packet) != GAP_SUBEVENT_LE_CONNECTION_COMPLETE) break;
        if (app_state != W4_CONNECTED_INCOMING) return;
        btstack_run_loop_remove_timer(&connection_timer);
        con_handle_keyboard = gap_subevent_le_connection_complete_get_connection_handle(packet);
        // request security
        app_state = W4_ENCRYPTED_INCOMING;
        sm_request_pairing(con_handle_keyboard);
        break;
    case HCI_EVENT_HIDS_META:
        switch (hci_event_hids_meta_get_subevent_code(packet)) {
        case HIDS_SUBEVENT_INPUT_REPORT_ENABLE:
            con_handle_host = hids_subevent_input_report_enable_get_con_handle(packet);
            printf("Report Characteristic Subscribed %u\n", hids_subevent_input_report_enable_get_enable(packet));
            if (app_state == W4_HOST_SUBSCRIPTION) {
                app_state = READY;
            }
            // W4_HOST -> READY
            // this is where we should be put into READY state
            break;
        case HIDS_SUBEVENT_BOOT_KEYBOARD_INPUT_REPORT_ENABLE:
            // not sure if it's needed here
            con_handle_host = hids_subevent_boot_keyboard_input_report_enable_get_con_handle(packet);
            printf("Boot Keyboard Characteristic Subscribed %u\n", hids_subevent_boot_keyboard_input_report_enable_get_enable(packet));
            break;
        case HIDS_SUBEVENT_PROTOCOL_MODE:
            // 0 = Boot Protocol Mode, 1 = Report Protocol Mode
            // mostly debug info
            protocol_mode_host = hids_subevent_protocol_mode_get_protocol_mode(packet);
            printf("Protocol Mode: %s mode\n", hids_subevent_protocol_mode_get_protocol_mode(packet) ? "Report" : "Boot");
            break;
        case HIDS_SUBEVENT_CAN_SEND_NOW:
            // should be in READY state
            send_report();
            break;
        default:
            break;
        }
        break;

    default:
        break;
    }
}
/* LISTING_END */

/* @section HCI packet handler
 *
 * @text The SM packet handler receives Security Manager Events required for pairing.
 * It also receives events generated during Identity Resolving
 * see Listing SMPacketHandler.
 */

 /* LISTING_START(SMPacketHandler): Scanning and receiving advertisements */

static void sm_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size) {
    UNUSED(channel);
    UNUSED(size);

    if (packet_type != HCI_EVENT_PACKET) return;

    bool connect_to_incoming = false;

    switch (hci_event_packet_get_type(packet)) {
    case SM_EVENT_JUST_WORKS_REQUEST:
        printf("Just works requested\n");
        sm_just_works_confirm(sm_event_just_works_request_get_handle(packet));
        break;
    case SM_EVENT_NUMERIC_COMPARISON_REQUEST:
        printf("Confirming numeric comparison: %"PRIu32"\n", sm_event_numeric_comparison_request_get_passkey(packet));
        sm_numeric_comparison_confirm(sm_event_passkey_display_number_get_handle(packet));
        break;
    case SM_EVENT_PASSKEY_DISPLAY_NUMBER:
        printf("Display Passkey: %"PRIu32"\n", sm_event_passkey_display_number_get_passkey(packet));
        break;
    case SM_EVENT_PAIRING_COMPLETE:
        switch (sm_event_pairing_complete_get_status(packet)) {
        case ERROR_CODE_SUCCESS:
            printf("Pairing complete, success\n");
            if (app_state == W4_ENCRYPTED_INCOMING) {
                connect_to_incoming = true;
            } else if (app_state == W4_HOST) {
                app_state = W4_HOST_SUBSCRIPTION;
            }
            break;
        case ERROR_CODE_CONNECTION_TIMEOUT:
            printf("Pairing failed, timeout\n");
            break;
        case ERROR_CODE_REMOTE_USER_TERMINATED_CONNECTION:
            printf("Pairing failed, disconnected\n");
            break;
        case ERROR_CODE_AUTHENTICATION_FAILURE:
            printf("Pairing failed, reason = %u\n", sm_event_pairing_complete_get_reason(packet));
            break;
        default:
            break;
        }
        break;
    case SM_EVENT_REENCRYPTION_COMPLETE:
        printf("Re-encryption complete, success\n");
        if (app_state == W4_ENCRYPTED_INCOMING) {
            connect_to_incoming = true;
        } else if (app_state == W4_HOST) {
            app_state = W4_HOST_SUBSCRIPTION;
        }
        break;
    default:
        break;
    }

    if (connect_to_incoming) {
        // continue - query primary services
        printf("Search for HID service.\n");
        app_state = W4_HID_CLIENT_CONNECTED;
        hids_client_connect(con_handle_keyboard, handle_gatt_client_event, protocol_mode_incoming, &hids_cid);
    }
}
/* LISTING_END */


int btstack_main(int argc, const char* argv[]);
int btstack_main(int argc, const char* argv[]) {

    (void)argc;
    (void)argv;

    /* LISTING_START(HogBootHostSetup): HID-over-GATT Host Setup */

    l2cap_init();

    // setup SM: Display only
    sm_init();
    sm_set_io_capabilities(IO_CAPABILITY_NO_INPUT_NO_OUTPUT); // probably have to change to keyboard only after pairing incoming? 
    sm_set_authentication_requirements(SM_AUTHREQ_SECURE_CONNECTION | SM_AUTHREQ_BONDING);

    gatt_client_init();

    // setup ATT server - only needed if LE Peripheral does ATT queries on its own, e.g. Android and iOS
    att_server_init(profile_data, NULL, NULL);

    // inc
    hids_client_init(hid_descriptor_storage, sizeof(hid_descriptor_storage));

    // out
    battery_service_server_init(battery);
    device_information_service_server_init();
    hids_device_init(0, hid_descriptor_keyabord_mx_mechanical, sizeof(hid_descriptor_keyabord_mx_mechanical));

    // setup advertisements
    uint16_t adv_int_min = 0x0030;
    uint16_t adv_int_max = 0x0030;
    uint8_t adv_type = 0;
    bd_addr_t null_addr;
    memset(null_addr, 0, 6);
    gap_advertisements_set_params(adv_int_min, adv_int_max, adv_type, 0, null_addr, 0x07, 0x00);
    gap_advertisements_set_data(adv_data_len, (uint8_t*)adv_data);

    // register for events from HCI
    hci_event_callback_registration.callback = &hci_packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    // register for events from Security Manager
    sm_event_callback_registration.callback = &sm_packet_handler;
    sm_add_event_handler(&sm_event_callback_registration);

    hids_device_register_packet_handler(hci_packet_handler);

    // Disable stdout buffering
    setvbuf(stdin, NULL, _IONBF, 0);

    app_state = W4_WORKING;

    // Turn on the device
    hci_power_control(HCI_POWER_ON);
    return 0;
}


int main(int argc, const char* argv[]) {

    stdio_init_all();

    if (cyw43_arch_init()) {
        printf("failed to initialise cyw43_arch\n");
        return -1;
    }

    btstack_main(argc, argv);

    btstack_run_loop_execute();
}
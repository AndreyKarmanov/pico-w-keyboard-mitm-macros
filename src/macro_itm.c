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

// target_device_t moved to app/macro_itm_types.h

static target_device_t target_device = { {0}, 0, "", HCI_CON_HANDLE_INVALID, 0 };

// Track host (our Peripheral link to PC/Phone)
typedef struct {
  bd_addr_type_t addr_type; // best-effort, filled on connect
  bd_addr_t addr;           // best-effort, filled on connect
  hci_con_handle_t con_handle;
  bool subscribed_to_reports;
} host_device_t;

static host_device_t host_device = { 0, {0}, HCI_CON_HANDLE_INVALID, false };
static uint8_t host_protocol_mode = 1; // 1 = Report, 0 = Boot

#define HID_REPORT_BUFFER_SIZE 16
static uint8_t hid_report_buffer[HID_REPORT_BUFFER_SIZE];
static uint16_t hid_report_len = 0;
static bool hid_report_pending = false;

static uint16_t hids_cid;
static uint8_t hid_descriptor_storage[500];
static uint16_t loaded_hid_descriptor_len = 0;


#define MAX_SEEN_DEVICES 32

typedef struct {
  bd_addr_t addr;
  uint8_t addr_type;
  char name[32];
} seen_device_t;

static seen_device_t seen_devices[MAX_SEEN_DEVICES];
static int seen_devices_count = 0;

#define SEEN_DEVICES_CHAR_BUFFER_SIZE 1024

static btstack_timer_source_t target_connection_timer;

static btstack_packet_callback_registration_t hci_event_callback_registration;
static btstack_packet_callback_registration_t sm_event_callback_registration;

// TLV backend and helpers are implemented in app/tlv_utils.*

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
  // Complete List of 128-bit Service UUIDs (complete means advertisement
  // contains all UUIDs available)
  0x11,
  BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS, // 0x07
  0xEF, 0xCD, 0xAB, 0x89, 0x67, 0x45, 0x23, 0x01, 0xEF, 0xCD, 0xAB, 0x90,
  0x78, 0x56, 0x34, 0x12, /* 128-bit UUIDs: 12345678-90AB-CDEF-0123-456789ABCDEF */
};
const uint8_t config_adv_data_len = sizeof(config_adv_data);

const uint8_t scan_response_data[] = {
  // Name
  0x05, BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME, 'M', 'I', 'T', 'M', // "MITM"
};
const uint8_t scan_response_data_len = sizeof(scan_response_data);

static void hci_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size);

static void app_set_state(app_state_t new_state)
{
  if (new_state == app_state) return;
  printf("[STATE] %s -> %s\n", app_state_strings[app_state], app_state_strings[new_state]);
  app_state = new_state;
}

static void target_set_state(target_state_t new_state)
{
  if (new_state == target_state) return;
  printf("[TARGET STATE] %s -> %s\n", target_state_strings[target_state], target_state_strings[new_state]);
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
    tlv_save_target_if_needed(&target_device);
    break;
  }
}

static void host_set_state(host_state_t new_state)
{
  if (new_state == host_state) return;
  printf("[HOST STATE] %s -> %s\n", host_state_strings[host_state], host_state_strings[new_state]);
  host_state = new_state;

  switch (host_state) {
  case HOST_STATE_IDLE:
    gap_advertisements_enable(0);
    host_device.con_handle = HCI_CON_HANDLE_INVALID;
    memset(host_device.addr, 0, sizeof(host_device.addr));
    break;
  case HOST_STATE_ADVERTISING:
    gap_advertisements_enable(1);
    host_device.con_handle = HCI_CON_HANDLE_INVALID;
    memset(host_device.addr, 0, sizeof(host_device.addr));
    break;
  case HOST_STATE_CONNECTED:
    gap_advertisements_enable(0);
    if (target_state == TARGET_STATE_CONNECTED)
      app_set_state(APP_STATE_READY);
    else
      app_set_state(APP_STATE_ACTIVE);
    break;
  }
}

static void connection_timeout(btstack_timer_source_t* ts)
{
  UNUSED(ts);
  printf("Connection Timeout\n");
  gap_connect_cancel();
  target_set_state(TARGET_STATE_SCANNING);
}

static void attempt_gap_connect()
{
  if (!target_device.valid) {
    printf("No valid target device set\n");
    return;
  }
  if (target_state == TARGET_STATE_CONNECTING || target_state == TARGET_STATE_CONNECTED) {
    printf("Already connecting or connected to target\n");
    return;
  }
  printf("Attempting connection to target %s (type=%u)\n", bd_addr_to_str(target_device.addr), target_device.addr_type);
  target_set_state(TARGET_STATE_CONNECTING);
  btstack_run_loop_set_timer(&target_connection_timer, 10000); // 10s timeout
  btstack_run_loop_set_timer_handler(&target_connection_timer, &connection_timeout);
  btstack_run_loop_add_timer(&target_connection_timer);
  gap_connect(target_device.addr, target_device.addr_type);
}

static void clear_target_device()
{
  if (target_state == TARGET_STATE_CONNECTING) {
    printf("Aborting target connection attempt\n");
    gap_connect_cancel();
    btstack_run_loop_remove_timer(&target_connection_timer);
  } else if (target_state == TARGET_STATE_CONNECTED) {
    printf("Disconnecting from target\n");
    gap_disconnect(target_device.con_handle);
  }
  // the con_handle should only be removed in disconnection complete event
  printf("Clearing target device\n");
  tlv_delete_target();
  loaded_hid_descriptor_len = 0;
  target_device.valid = 0;
  target_set_state(TARGET_STATE_SCANNING);
}

static void disconnect_host()
{
  if (host_device.con_handle != HCI_CON_HANDLE_INVALID) {
    printf("Disconnecting host %s\n", bd_addr_to_str(host_device.addr));
    gap_disconnect(host_device.con_handle);
  }
}

static void save_seen_device(bd_addr_t addr, uint8_t addr_type, const uint8_t* adv_data, uint8_t adv_size)
{
  // Skip if already seen
  for (int i = 0; i < seen_devices_count; i++) {
    if (bd_addr_cmp(addr, seen_devices[i].addr) == 0) {
      return;
    }
  }

  if (seen_devices_count >= MAX_SEEN_DEVICES)
    return;

  ad_context_t context;
  for (ad_iterator_init(&context, adv_size, adv_data); ad_iterator_has_more(&context); ad_iterator_next(&context)) {
    uint8_t data_type = ad_iterator_get_data_type(&context);
    uint8_t size = ad_iterator_get_data_len(&context);
    const uint8_t* data = ad_iterator_get_data(&context);

    switch (data_type) {
    case BLUETOOTH_DATA_TYPE_SHORTENED_LOCAL_NAME:
    case BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME: {
      memcpy(seen_devices[seen_devices_count].addr, addr, sizeof(bd_addr_t));
      seen_devices[seen_devices_count].addr_type = addr_type;
      uint8_t copy_len = size < (sizeof(seen_devices[0].name) - 1) ? size : (uint8_t)(sizeof(seen_devices[0].name) - 1);
      memcpy(seen_devices[seen_devices_count].name, data, copy_len);
      seen_devices[seen_devices_count].name[copy_len] = '\0';

      seen_devices_count++;
      printf("Found device %s (%s, type=%u)\n", seen_devices[seen_devices_count - 1].name, bd_addr_to_str(seen_devices[seen_devices_count - 1].addr), addr_type);
      return;
    }
    default:
      break;
    }
  }
}

static void clear_seen_devices()
{
  // Optionally zero out entries for cleanliness
  memset(seen_devices, 0, sizeof(seen_devices));
  seen_devices_count = 0;
  printf("Seen devices cleared\n");
}

static uint16_t build_seen_devices_listing(uint8_t* out, uint16_t max_len)
{
  uint16_t pos = 0;
  for (int i = 0; i < seen_devices_count; i++) {
    const char* name = seen_devices[i].name;
    int needed = snprintf(
      (char*)out + pos,
      (pos < max_len) ? (max_len - pos) : 0,
      "%02X:%02X:%02X:%02X:%02X:%02X,%u,%s\n",
      seen_devices[i].addr[0], seen_devices[i].addr[1], seen_devices[i].addr[2],
      seen_devices[i].addr[3], seen_devices[i].addr[4], seen_devices[i].addr[5],
      seen_devices[i].addr_type,
      name
    );
    if (needed < 0) break;
    if (pos + needed >= max_len) {
      if (max_len >= 4) {
        out[max_len - 4] = '.';
        out[max_len - 3] = '.';
        out[max_len - 2] = '.';
        out[max_len - 1] = '\n';
      }
      pos = max_len;
      break;
    }
    pos += (uint16_t)needed;
  }
  return pos;
}

// printing helpers moved to app/print_utils.*

void attempt_hids_connect()
{
  if (target_state != TARGET_STATE_CONNECTING) {
    printf("Not connected to target, cannot connect to HID service\n");
    return;
  }
  printf("Attempting HID service connection\n");
  uint8_t result = hids_client_connect(
    target_device.con_handle,
    hci_packet_handler,
    HID_PROTOCOL_MODE_REPORT,
    &hids_cid
  );

  switch (result) {
  case ERROR_CODE_SUCCESS:
    printf("HID service client connected\n");
    target_set_state(TARGET_STATE_CONNECTED);
    break;
  case ERROR_CODE_COMMAND_DISALLOWED:
    printf("HID service client already connected\n");
    target_set_state(TARGET_STATE_CONNECTED);
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

  switch (hci_event_packet_get_type(packet)) {
  case HCI_EVENT_HIDS_META:
    switch (hci_event_hids_meta_get_subevent_code(packet)) {
    case HIDS_SUBEVENT_INPUT_REPORT_ENABLE:
      host_device.con_handle = hids_subevent_input_report_enable_get_con_handle(packet);
      host_device.subscribed_to_reports = hids_subevent_input_report_enable_get_enable(packet) != 0;
      printf("Host subscribed to input reports (con_handle=0x%04X, enabled=%u)\n", host_device.con_handle, hids_subevent_input_report_enable_get_enable(packet));
      if (host_device.subscribed_to_reports) {
        hids_device_request_can_send_now_event(host_device.con_handle);
      }
      break;
    case HIDS_SUBEVENT_BOOT_KEYBOARD_INPUT_REPORT_ENABLE:
      host_device.con_handle = hids_subevent_boot_keyboard_input_report_enable_get_con_handle(packet);
      host_device.subscribed_to_reports = hids_subevent_boot_keyboard_input_report_enable_get_enable(packet) != 0;
      printf("Host subscribed to boot keyboard reports (con_handle=0x%04X, enabled=%u)\n", host_device.con_handle, hids_subevent_boot_keyboard_input_report_enable_get_enable(packet));
      if (host_device.subscribed_to_reports) {
        hids_device_request_can_send_now_event(host_device.con_handle);
      }
      break;
    case HIDS_SUBEVENT_PROTOCOL_MODE:
      host_protocol_mode = hids_subevent_protocol_mode_get_protocol_mode(packet);
      printf("Host protocol mode set to %s\n", host_protocol_mode ? "Report" : "Boot");
      break;
    case HIDS_SUBEVENT_CAN_SEND_NOW:
      if (hid_report_pending && host_device.subscribed_to_reports && host_device.con_handle != HCI_CON_HANDLE_INVALID) {
        printf("Sending HID report to host\n");
        if (host_protocol_mode == 0) {
          hids_device_send_boot_keyboard_input_report(host_device.con_handle, hid_report_buffer, hid_report_len);
        } else {
          hids_device_send_input_report(host_device.con_handle, hid_report_buffer, hid_report_len);
        }
        hid_report_pending = false;
      }
      break;
    default:
      break;
    }
    break;
  default:
    break;
  }
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
  case GAP_EVENT_ADVERTISING_REPORT:
    const uint8_t* data = gap_event_advertising_report_get_data(packet);
    uint8_t data_len = gap_event_advertising_report_get_data_length(packet);
    bd_addr_t address;
    gap_event_advertising_report_get_address(packet, address);
    uint8_t addr_type = gap_event_advertising_report_get_address_type(packet);

    save_seen_device(address, addr_type, data, data_len);

    // connect if we have a target and see it
    if (target_state == TARGET_STATE_SCANNING
        && target_device.valid
        && bd_addr_cmp(address, target_device.addr) == 0
        && addr_type == target_device.addr_type) {
      attempt_gap_connect();
    }
    break;
  case HCI_EVENT_GATTSERVICE_META:
    switch (hci_event_gattservice_meta_get_subevent_code(packet)) {
    case GATTSERVICE_SUBEVENT_HID_SERVICE_CONNECTED:
      if (loaded_hid_descriptor_len == 0) {
        const uint8_t* desc = hids_client_descriptor_storage_get_descriptor_data(hids_cid, 0);
        uint16_t desc_len = hids_client_descriptor_storage_get_descriptor_len(hids_cid, 0);
        print_hid_descriptor(desc, desc_len);
        // persist descriptor for later host mirroring setup
        tlv_store_hid_descriptor(desc, desc_len);
        printf("Rebooting in 100ms to apply HID descriptor\n");
        watchdog_reboot(0, 0, 100);
      }
      break;
    case GATTSERVICE_SUBEVENT_CLIENT_DISCONNECTED:
      printf("HID service client disconnected\n");
      if (target_device.valid) {
        target_set_state(TARGET_STATE_CONNECTING);
        attempt_hids_connect();
      } else {
        target_set_state(TARGET_STATE_SCANNING);
      }
      break;
    case GATTSERVICE_SUBEVENT_HID_REPORT:
    {
      const uint8_t* report = gattservice_subevent_hid_report_get_report(packet);
      uint16_t report_len = gattservice_subevent_hid_report_get_report_len(packet);
      print_hid_report(report, report_len);

      if (host_device.subscribed_to_reports && host_device.con_handle != HCI_CON_HANDLE_INVALID) {
        if (report_len <= HID_REPORT_BUFFER_SIZE) {
          memcpy(hid_report_buffer, report, report_len);
          hid_report_len = report_len;
          hid_report_pending = true;
          hids_device_request_can_send_now_event(host_device.con_handle);
        } else {
          printf("HID report too large for buffer\n");
        }
      }
    }
    break;
    default:
      break;
    }
    break;
  case HCI_EVENT_META_GAP:
    if (hci_event_gap_meta_get_subevent_code(packet) != GAP_SUBEVENT_LE_CONNECTION_COMPLETE)
      break;

    bd_addr_t addr;
    gap_subevent_le_connection_complete_get_peer_address(packet, addr);
    uint8_t status = gap_subevent_le_connection_complete_get_status(packet);

    if (status != ERROR_CODE_SUCCESS) {
      printf("Connection failed %s, status %u\n", bd_addr_to_str(addr), status);
      break;
    }

    con_handle = gap_subevent_le_connection_complete_get_connection_handle(packet);
    if (target_state == TARGET_STATE_CONNECTING && target_device.valid && bd_addr_cmp(addr, target_device.addr) == 0) {
      btstack_run_loop_remove_timer(&target_connection_timer);
      target_device.con_handle = con_handle;
      printf("Initiating Target Pairing\n");
      sm_request_pairing(target_device.con_handle);
    } else if (host_state != HOST_STATE_CONNECTED && gap_subevent_le_connection_complete_get_role(packet) == HCI_ROLE_SLAVE) {
      host_device.con_handle = con_handle;
      memcpy(host_device.addr, addr, sizeof(bd_addr_t));
      host_set_state(HOST_STATE_CONNECTED);
    } else {
      printf("Unknown Device Connected: %s\n", bd_addr_to_str(addr));
      gap_disconnect(con_handle);
    }
    break;
  case HCI_EVENT_DISCONNECTION_COMPLETE:
    status = hci_event_disconnection_complete_get_status(packet);
    if (status != ERROR_CODE_SUCCESS) {
      printf("Disconnection complete event with error status %u\n", status);
      break;
    }

    con_handle = hci_event_disconnection_complete_get_connection_handle(packet);
    if (con_handle == target_device.con_handle) {
      target_device.con_handle = HCI_CON_HANDLE_INVALID;
      target_set_state(TARGET_STATE_SCANNING);
    } else if (con_handle == host_device.con_handle) {
      host_device.con_handle = HCI_CON_HANDLE_INVALID;
      host_set_state(HOST_STATE_ADVERTISING);
    } else {
      printf("Unknown Device Disconnected\n");
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
    printf("Just works requested\n");
    sm_just_works_confirm(sm_event_just_works_request_get_handle(packet));
    break;
  case SM_EVENT_NUMERIC_COMPARISON_REQUEST:
    printf("Confirming numeric comparison: %" PRIu32 "\n",
           sm_event_numeric_comparison_request_get_passkey(packet));
    sm_numeric_comparison_confirm(
      sm_event_numeric_comparison_request_get_handle(packet));
    break;
  case SM_EVENT_PASSKEY_DISPLAY_NUMBER:
    printf("Display Passkey: %" PRIu32 "\n",
           sm_event_passkey_display_number_get_passkey(packet));
    break;
  case SM_EVENT_PAIRING_COMPLETE:
    status = sm_event_pairing_complete_get_status(packet);
    con_handle = sm_event_pairing_complete_get_handle(packet);
    switch (status) {
    case ERROR_CODE_SUCCESS:
      printf("Pairing success\n");
      if (con_handle == target_device.con_handle) {
        attempt_hids_connect();
      } else if (con_handle == host_device.con_handle) {
        printf("Host paired successfully\n");
      }
      break;
    case ERROR_CODE_CONNECTION_TERMINATED_DUE_TO_MIC_FAILURE:
      printf("Pairing failed: MIC failure. Deleting bond and retrying.\n");
      if (con_handle == target_device.con_handle) {
        gap_delete_bonding(target_device.addr_type, target_device.addr);
        target_set_state(TARGET_STATE_SCANNING);
      } else if (con_handle == host_device.con_handle) {
        gap_delete_bonding(host_device.addr_type, host_device.addr);
        disconnect_host();
      }
      break;
    default:
      printf("Pairing failed, reason = %u\n", status);
      if (con_handle == target_device.con_handle) {
        target_set_state(TARGET_STATE_SCANNING);
      } else if (con_handle == host_device.con_handle) {
        disconnect_host();
      }
      break;
    }
    break;
  case SM_EVENT_REENCRYPTION_STARTED:
    con_handle = sm_event_reencryption_started_get_handle(packet);
    if (con_handle == target_device.con_handle) {
      printf("Target re-encryption started\n");
      target_set_state(TARGET_STATE_CONNECTING);
    } else if (con_handle == host_device.con_handle) {
      printf("Host re-encryption started\n");
    } else {
      printf("Unknown re-encryption started\n");
    }
    break;
  case SM_EVENT_REENCRYPTION_COMPLETE:
    status = sm_event_reencryption_complete_get_status(packet);
    con_handle = sm_event_reencryption_complete_get_handle(packet);
    switch (status) {
    case ERROR_CODE_SUCCESS:
      if (target_device.con_handle == con_handle) {
        printf("Target re-encryption complete, success\n");
        attempt_hids_connect();
      } else if (host_device.con_handle == con_handle) {
        printf("Host re-encryption complete\n");
      }
      break;
    default:
      printf("Re-encryption failed, status %u\n", status);
      if (con_handle == target_device.con_handle) {
        printf("Retrying target pairing\n");
        sm_request_pairing(con_handle);
      }
      break;
    }
  default:
    break;
  }
}


static uint16_t att_read_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t offset, uint8_t* buffer, uint16_t buffer_size)
{
  if (att_handle == ATT_CHARACTERISTIC_12345678_90AB_CDEF_0123_456789ABCDF0_01_VALUE_HANDLE) {
    // Now returns lines: MAC,TYPE,NAME
    static uint8_t devices_buf[SEEN_DEVICES_CHAR_BUFFER_SIZE];
    uint16_t len = build_seen_devices_listing(devices_buf, sizeof(devices_buf));
    return att_read_callback_handle_blob(devices_buf, len, offset, buffer, buffer_size);
  } else if (att_handle == ATT_CHARACTERISTIC_12345678_90AB_CDEF_0123_456789ABCDF1_01_VALUE_HANDLE) {
    static char target_buf[55];
    uint16_t len = 0;
    if (target_device.valid) {
      len = (uint16_t)snprintf(target_buf, sizeof(target_buf), "%s,%u,%s", bd_addr_to_str(target_device.addr), target_device.addr_type, target_device.name);
    } else {
      target_buf[0] = '\0';
      len = 0;
    }
    return att_read_callback_handle_blob((uint8_t*)target_buf, len, offset, buffer, buffer_size);
  }
  return 0;
};

static int att_write_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t transaction_mode, uint16_t offset, uint8_t* buffer, uint16_t buffer_size)
{
  if (transaction_mode != ATT_TRANSACTION_MODE_NONE || offset != 0) return 0;

  if (att_handle == ATT_CHARACTERISTIC_12345678_90AB_CDEF_0123_456789ABCDF0_01_VALUE_HANDLE) {
    if (buffer_size == 5 && memcmp(buffer, "CLEAR", 5) == 0) {
      clear_seen_devices();
    } else {
      printf("Unknown command (expect CLEAR)\n");
    }
  } else if (att_handle == ATT_CHARACTERISTIC_12345678_90AB_CDEF_0123_456789ABCDF1_01_VALUE_HANDLE) {
    if (buffer_size == 5 && memcmp(buffer, "CLEAR", 5) == 0) {
      clear_target_device();
    } else {
      printf("Trying to set target device: %.*s\n", buffer_size, buffer);

      target_device_t temp_device = tlv_parse_device_string((const char*)buffer, buffer_size);

      if (!temp_device.valid) {
        printf("Invalid target format\n");
        return 0;
      }

      if (target_device.valid && bd_addr_cmp(temp_device.addr, target_device.addr) == 0) {
        printf("Target device unchanged\n");
        return 0;
      }

      if (target_device.valid) {
        clear_target_device();
      }

      target_device = temp_device;
      tlv_store_target_device(&target_device);
      printf("Target set to %s (type=%u)\n", bd_addr_to_str(temp_device.addr), temp_device.addr_type);
    }
  } else {
    printf("Write to unknown handle %04X\n", att_handle);
  }
  return 0;
}

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

int btstack_main(int argc, const char* argv[])
{
  UNUSED(argc);
  UNUSED(argv);

  /* Organized in a chronological manner of what is used and in what order */
  tlv_utils_init();

  // Register for HCI events, main communication channel between btstack and
  // application
  hci_event_callback_registration.callback = &hci_packet_handler;
  hci_add_event_handler(&hci_event_callback_registration);

  // setup transport layer
  l2cap_init();

  // setup discovery: advertisements
  uint16_t adv_int_min = 0x0030;
  uint16_t adv_int_max = 0x0030;
  uint8_t adv_type = 0;
  bd_addr_t null_addr;
  memset(null_addr, 0, 6);
  gap_advertisements_set_params(adv_int_min, adv_int_max, adv_type, 0, null_addr, 0x07, 0x00);
  gap_advertisements_set_data(config_adv_data_len, (uint8_t*)config_adv_data);
  gap_scan_response_set_data(scan_response_data_len, (uint8_t*)scan_response_data);

  // setup attributes: provide access to the services/characteristics/descriptors
  att_server_init(profile_data, att_read_callback, att_write_callback);
  device_information_service_server_init();


  // setup security: handles pairing, authentication, encryption
  sm_init();
  sm_set_io_capabilities(IO_CAPABILITY_NO_INPUT_NO_OUTPUT);
  sm_set_authentication_requirements(SM_AUTHREQ_SECURE_CONNECTION | SM_AUTHREQ_BONDING);
  sm_event_callback_registration.callback = &sm_packet_handler;
  sm_add_event_handler(&sm_event_callback_registration);

  // Init GATT Client to enable reading profiles?
  // TODO: see if this is needed
  gatt_client_init();

  // Init HID Service Client to access HID over GATT Service on target
  hids_client_init(hid_descriptor_storage, sizeof(hid_descriptor_storage));

  // setup HID Device service if descriptor is available
  uint8_t hid_descriptor[500];
  uint16_t hid_descriptor_len = tlv_load_hid_descriptor(hid_descriptor, sizeof(hid_descriptor));
  loaded_hid_descriptor_len = hid_descriptor_len;
  printf("HID descriptor length from TLV: %u\n", hid_descriptor_len);
  if (hid_descriptor_len > 0) {
    printf("HID descriptor loaded from TLV, initializing HID Device Service\n");
    hids_device_init(0, hid_descriptor, hid_descriptor_len);
  } else {
    printf("No HID descriptor in TLV, using default boot keyboard descriptor\n");
    hids_device_init(0, hid_descriptor_keyboard_boot_mode, sizeof(hid_descriptor_keyboard_boot_mode));
  }
  hids_device_register_packet_handler(&hids_host_packet_handler);


  // turn off stdout buffering to help with debugging (disable in prod tho)
  // setvbuf(stdin, NULL, _IONBF, 0);


  {
    target_device_t restored = tlv_load_target_device();
    if (restored.valid) {
      target_device = restored;
      printf("Restored target from TLV: %s (type=%u, name=\"%s\")\n", bd_addr_to_str(target_device.addr), target_device.addr_type, target_device.name);
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
    printf("failed to initialise cyw43_arch\n");
    return -1;
  }

  btstack_main(argc, argv);

  cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
  btstack_run_loop_execute();
}

#include "macro_itm.h"

#include <btstack_tlv.h>
#include <ctype.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "btstack.h"
#include "btstack_config.h"
#include "pico/cyw43_arch.h"
#include "pico/stdio.h"
#include "pico/time.h"


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

// TLV tag for target device ('TRGT')
#define TLV_TAG_TARGET_DEVICE 0x54524754u

typedef struct {
  bd_addr_t addr;
  uint8_t addr_type;
  char name[32];
  hci_con_handle_t con_handle;
  uint8_t valid;
} target_device_t;

static target_device_t target_device = { {0}, 0, "", HCI_CON_HANDLE_INVALID, 0 };

static uint16_t hids_cid;
static uint8_t hid_descriptor_storage[500];

#define MAX_SEEN_DEVICES 32
// struct {
//   bd_addr_t addr;
//   uint8_t addr_type;
//   char name[32];
// } seen_devices[MAX_SEEN_DEVICES];

bd_addr_t seen_device_addrs[MAX_SEEN_DEVICES];
static uint8_t seen_device_types[MAX_SEEN_DEVICES];
static char* seen_device_names[MAX_SEEN_DEVICES];
static int seen_devices_count = 0;

#define SEEN_DEVICES_CHAR_BUFFER_SIZE 1024

static btstack_timer_source_t connection_timer;

static btstack_packet_callback_registration_t hci_event_callback_registration;
static btstack_packet_callback_registration_t sm_event_callback_registration;

static const btstack_tlv_t* tlv_impl;
static void* tlv_context;

// Config Advertisement Data
const uint8_t config_adv_data[] = {
  // Flags general discoverable, Classic not supported (LE only)
  0x02, BLUETOOTH_DATA_TYPE_FLAGS,
  0x06, // LE General Discoverable Mode

  // Name
  0x05, BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME, 'M', 'I', 'T', 'M', // "MITM"

  // Complete List of 128-bit Service UUIDs (complete means advertisement
  // contains all UUIDs available)
  0x11,
  BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS, // 0x07
  0xEF, 0xCD, 0xAB, 0x89, 0x67, 0x45, 0x23, 0x01, 0xEF, 0xCD, 0xAB, 0x90,
  0x78, 0x56, 0x34, 0x12, /* 128-bit UUIDs: 12345678-90AB-CDEF-0123-456789ABCDEF */
};
const uint8_t config_adv_data_len = sizeof(config_adv_data);

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
    break;
  case HOST_STATE_ADVERTISING:
    gap_advertisements_enable(1);
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

// Attempt connection to selected target (called after seeing advertisement or setting target)
static void attempt_target_connection()
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
  btstack_run_loop_set_timer(&connection_timer, 10000); // 10s timeout
  btstack_run_loop_set_timer_handler(&connection_timer, &connection_timeout);
  btstack_run_loop_add_timer(&connection_timer);
  gap_connect(target_device.addr, target_device.addr_type);
}

static void clear_target_device()
{
  if (target_state == TARGET_STATE_CONNECTING) {
    printf("Aborting target connection attempt\n");
    gap_connect_cancel();
    btstack_run_loop_remove_timer(&connection_timer);
  } else if (target_state == TARGET_STATE_CONNECTED) {
    printf("Disconnecting from target\n");
    gap_disconnect(target_device.con_handle);
  }
  // the con_handle should only be removed in disconnection complete event
  target_device.valid = 0;
  target_set_state(TARGET_STATE_SCANNING);
}

static void save_found_device(bd_addr_t addr, uint8_t addr_type, const uint8_t* adv_data, uint8_t adv_size)
{
  for (int i = 0; i < seen_devices_count; i++) {
    if (bd_addr_cmp(addr, seen_device_addrs[i]) == 0) {
      return;
    }
  }

  ad_context_t context;
  if (seen_devices_count >= MAX_SEEN_DEVICES)
    return;

  for (ad_iterator_init(&context, adv_size, adv_data); ad_iterator_has_more(&context); ad_iterator_next(&context)) {
    uint8_t data_type = ad_iterator_get_data_type(&context);
    uint8_t size = ad_iterator_get_data_len(&context);
    const uint8_t* data = ad_iterator_get_data(&context);

    switch (data_type) {
    case BLUETOOTH_DATA_TYPE_SHORTENED_LOCAL_NAME:
    case BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME:
      memcpy(seen_device_addrs[seen_devices_count], addr, sizeof(bd_addr_t));
      seen_device_types[seen_devices_count] = addr_type;
      seen_device_names[seen_devices_count] = (char*)malloc(size + 1);
      if (seen_device_names[seen_devices_count]) {
        memcpy(seen_device_names[seen_devices_count], data, size);
        seen_device_names[seen_devices_count][size] = '\0';
      }
      seen_devices_count++;
      printf("Found device %s (%s, type=%u)\n", seen_device_names[seen_devices_count - 1], bd_addr_to_str(seen_device_addrs[seen_devices_count - 1]), addr_type);
      return;
    default:
      break;
    }
  }
}

// Helper: clear stored devices
static void clear_seen_devices(void)
{
  for (int i = 0; i < seen_devices_count; i++) {
    free(seen_device_names[i]);
    seen_device_names[i] = NULL;
  }
  seen_devices_count = 0;
  printf("Seen devices cleared\n");
}

// Helper: build textual representation into buffer, return length
static uint16_t build_seen_devices_listing(uint8_t* out, uint16_t max_len)
{
  uint16_t pos = 0;
  for (int i = 0; i < seen_devices_count; i++) {
    char* name = seen_device_names[i] ? seen_device_names[i] : "";
    int needed = snprintf(
      (char*)out + pos,
      (pos < max_len) ? (max_len - pos) : 0,
      "%02X:%02X:%02X:%02X:%02X:%02X,%u,%s\n",
      seen_device_addrs[i][0], seen_device_addrs[i][1], seen_device_addrs[i][2],
      seen_device_addrs[i][3], seen_device_addrs[i][4], seen_device_addrs[i][5],
      seen_device_types[i],
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

static void hci_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size)
{
  UNUSED(channel);
  UNUSED(size);

  if (packet_type != HCI_EVENT_PACKET)
    return;

  switch (hci_event_packet_get_type(packet)) {
  case BTSTACK_EVENT_STATE:
    switch (btstack_event_state_get_state(packet)) {
    case HCI_STATE_WORKING:
      printf("BTstack up and running.\n");
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

    save_found_device(address, addr_type, data, data_len);

    // connect if we have a target and see it
    if (target_state == TARGET_STATE_SCANNING
      && target_device.valid
      && bd_addr_cmp(address, target_device.addr) == 0) {
      attempt_target_connection();
    }
    break;
  case GAP_EVENT_PAIRING_STARTED:
    printf("GAP Pairing started\n");
    break;
  case GAP_EVENT_PAIRING_COMPLETE: // I believe this is for LE with profiles, because this doesn't get triggered for generic pairing
    switch (gap_event_pairing_complete_get_status(packet)) {
    case ERROR_CODE_SUCCESS:
      printf("GAP Pairing complete, success\n");
      break;
    case ERROR_CODE_CONNECTION_TIMEOUT:
      printf("GAP Pairing failed, timeout\n");
      break;
    case ERROR_CODE_REMOTE_USER_TERMINATED_CONNECTION:
      printf("GAP Pairing failed, disconnected\n");
      break;
    case ERROR_CODE_AUTHENTICATION_FAILURE:
      printf("GAP Pairing failed, Authentication failure\n");
      break;
    default:
      printf("GAP Pairing failed, status = %u\n", gap_event_pairing_complete_get_status(packet));
      break;
    }
    break;
  case HCI_EVENT_META_GAP:
    switch (hci_event_gap_meta_get_subevent_code(packet)) {
    case GAP_SUBEVENT_LE_CONNECTION_COMPLETE:
      switch (gap_subevent_le_connection_complete_get_status(packet)) {
      case ERROR_CODE_SUCCESS:
        uint8_t role = gap_subevent_le_connection_complete_get_role(packet);
        bd_addr_t addr;
        gap_subevent_le_connection_complete_get_peer_address(packet, addr);

        if (role == HCI_ROLE_SLAVE) {
          printf("Central Device Connected: %s\n", bd_addr_to_str(addr));
          host_set_state(HOST_STATE_CONNECTED);
        } else if (target_state == TARGET_STATE_CONNECTING && target_device.valid && bd_addr_cmp(addr, target_device.addr) == 0) {
          printf("Target Device Connected: %s\n", bd_addr_to_str(addr));
          btstack_run_loop_remove_timer(&connection_timer);
          printf("Initiating Pairing...\n");
          target_device.con_handle = gap_subevent_le_connection_complete_get_connection_handle(packet);
          sm_request_pairing(target_device.con_handle);
        } else {
          printf("Unknown Device Connected: %s\n", bd_addr_to_str(addr));
          gap_disconnect(gap_subevent_le_connection_complete_get_connection_handle(packet));
        }
        break;
      default:
        printf("Connection failed, status %u\n", gap_subevent_le_connection_complete_get_status(packet));
        break;
      }
      break;
    default:
      break;
    }
    break;
  case HCI_EVENT_DISCONNECTION_COMPLETE:
    switch (hci_event_disconnection_complete_get_status(packet)) {
    case ERROR_CODE_SUCCESS:
      hci_con_handle_t con_handle = hci_event_disconnection_complete_get_connection_handle(packet);
      uint8_t role = hci_connection_for_handle(con_handle)->role;
      if (con_handle == target_device.con_handle) {
        target_device.con_handle = HCI_CON_HANDLE_INVALID;
        target_set_state(TARGET_STATE_SCANNING);
        if (target_device.valid) {
          attempt_target_connection();
        }
        break;
      } else if (role == HCI_ROLE_SLAVE) {
        host_set_state(HOST_STATE_ADVERTISING);
      } else {
        printf("Unknown Device Disconnected\n");
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

static void sm_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size)
{
  UNUSED(channel);
  UNUSED(size);

  if (packet_type != HCI_EVENT_PACKET)
    return;

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
    switch (sm_event_pairing_complete_get_status(packet)) {
    case ERROR_CODE_SUCCESS:
      printf("Pairing complete, success\n");
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
    }
    break;
  case SM_EVENT_REENCRYPTION_COMPLETE:
    int is_target_event = sm_event_reencryption_complete_get_handle(packet) == target_device.con_handle;
    switch (sm_event_reencryption_complete_get_status(packet)) {
    case ERROR_CODE_PIN_OR_KEY_MISSING:
      printf("Re-encryption failed, PIN or Key missing - retry pairing\n");
      sm_request_pairing(sm_event_reencryption_complete_get_handle(packet));
      break;
    case ERROR_CODE_SUCCESS:
      printf("Re-encryption complete, success\n");
      break;
    default:
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

static target_device_t parse_device_string(const char* s, uint16_t s_len)
{
  target_device_t temp_device = { {0}, 0, "", HCI_CON_HANDLE_INVALID, 0 };

  char temp_addr_s[18];
  char s_buf[55];
  if (s_len >= sizeof(s_buf)) {
    return temp_device;
  }
  memcpy(s_buf, s, s_len);
  s_buf[s_len] = '\0';

  // "AA:BB:CC:DD:EE:FF,<type>,<name>"
  int result = sscanf(s_buf, "%17[^,],%hhu,%31", temp_addr_s, &temp_device.addr_type, temp_device.name);

  if (result == 3) {
    temp_device.valid = sscanf_bd_addr(temp_addr_s, temp_device.addr);
    printf("Address: %s\n", bd_addr_to_str(temp_device.addr));
    printf("Type: %u\n", temp_device.addr_type);
    printf("Name: %s\n", temp_device.name);
    printf("Valid: %s\n", temp_device.valid ? "true" : "false");
  }

  return temp_device;
}

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

      target_device_t temp_device = parse_device_string((const char*)buffer, buffer_size);

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
      printf("Target set to %s (type=%u)\n", bd_addr_to_str(temp_device.addr), temp_device.addr_type);
      attempt_target_connection();
    }
  } else {
    printf("Write to unknown handle %04X\n", att_handle);
  }
  return 0;
}

int btstack_main(int argc, const char* argv[])
{
  (void)argc;
  (void)argv;

  /* Organized in a chronological manner of what is used and in what order */

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
  gatt_client_init();

  // Init HID Service Client to access HID over GATT Service on target
  hids_client_init(hid_descriptor_storage, sizeof(hid_descriptor_storage));


  // turn off stdout buffering to help with debugging (disable in prod tho)
  // setvbuf(stdin, NULL, _IONBF, 0);

  btstack_tlv_get_instance(&tlv_impl, &tlv_context);

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
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
  APP_STATE_IDLE = 0,    // No connections active yet (BT stack not ready)
  APP_STATE_ADVERTISING, // Advertising to host, awaiting subscription
} app_state_t;

static app_state_t app_state;

static const char* app_state_strings[] = {
    "IDLE",
    "ADVERTISING",
};

#define MAX_SEEN_DEVICES 32
bd_addr_t seen_device_addrs[MAX_SEEN_DEVICES];
static char* seen_device_names[MAX_SEEN_DEVICES];
static int seen_devices_count = 0;

#define SEEN_DEVICES_CHAR_BUFFER_SIZE 1024

static btstack_timer_source_t connection_timer;

static btstack_packet_callback_registration_t hci_event_callback_registration;
static btstack_packet_callback_registration_t sm_event_callback_registration;

static const btstack_tlv_t* tlv_impl;
static void* tlv_context;

static hci_con_handle_t con_handle_host = HCI_CON_HANDLE_INVALID;

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
  if (new_state == app_state)
    return;
  printf("[STATE] %s -> %s\n", app_state_strings[app_state], app_state_strings[new_state]);
  app_state = new_state;
}

static void save_found_device(bd_addr_t addr, const uint8_t* adv_data, uint8_t adv_size)
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
      seen_device_names[seen_devices_count] = (char*)malloc(size + 1);
      if (seen_device_names[seen_devices_count]) {
        memcpy(seen_device_names[seen_devices_count], data, size);
        seen_device_names[seen_devices_count][size] = '\0';
      }
      seen_devices_count++;
      printf("Found device %s\n", seen_device_names[seen_devices_count - 1]);
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
      "%02X:%02X:%02X:%02X:%02X:%02X,%s\n",
      seen_device_addrs[i][0], seen_device_addrs[i][1], seen_device_addrs[i][2],
      seen_device_addrs[i][3], seen_device_addrs[i][4], seen_device_addrs[i][5],
      name
    );
    if (needed < 0) break;
    if (pos + needed >= max_len) { // truncated
      // overwrite end with ellipsis if space
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
      gap_start_scan();
      gap_advertisements_enable(1);
      app_set_state(APP_STATE_ADVERTISING);
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
    save_found_device(address, data, data_len);
    break;
  case HCI_EVENT_META_GAP:
    switch (hci_event_gap_meta_get_subevent_code(packet)) {
    case GAP_SUBEVENT_LE_CONNECTION_COMPLETE:
      uint8_t role = gap_subevent_le_connection_complete_get_role(packet);
      if (role == HCI_ROLE_SLAVE) {
        printf("Central Device Connected\n");
        gap_advertisements_enable(0);
        app_set_state(APP_STATE_IDLE);
      } else if (role == HCI_ROLE_MASTER) {
        printf("Peripheral Device Connected\n");
      }
      break;
    default:
      break;
    }
    break;
  case HCI_EVENT_DISCONNECTION_COMPLETE:
    switch (hci_event_disconnection_complete_get_status(packet)) {
    case ERROR_CODE_SUCCESS:
      uint8_t role =
        hci_connection_for_handle(
          hci_event_disconnection_complete_get_connection_handle(
            packet))
        ->role;
      if (role == HCI_ROLE_SLAVE) {
        printf("Central Device Disconnected\n");
        gap_stop_scan();
        gap_advertisements_enable(1);
        app_set_state(APP_STATE_ADVERTISING);
      } else if (role == HCI_ROLE_MASTER) {
        printf("Peripheral Device Disconnected\n");
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
      printf("Pairing failed, reason = %u\n",
        sm_event_pairing_complete_get_reason(packet));
      break;
    }
    break;
  case SM_EVENT_REENCRYPTION_COMPLETE:
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
    static uint8_t devices_buf[SEEN_DEVICES_CHAR_BUFFER_SIZE];
    uint16_t len = build_seen_devices_listing(devices_buf, sizeof(devices_buf));
    return att_read_callback_handle_blob(devices_buf, len, offset, buffer, buffer_size);
  }
  return 0;
};

static int att_write_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t transaction_mode, uint16_t offset, uint8_t* buffer, uint16_t buffer_size)
{
  if (att_handle == ATT_CHARACTERISTIC_12345678_90AB_CDEF_0123_456789ABCDF0_01_VALUE_HANDLE) {
    if (transaction_mode == ATT_TRANSACTION_MODE_NONE && offset == 0) {
      if (buffer_size == 5 && (memcmp(buffer, "CLEAR", 5) == 0)) {
        clear_seen_devices();
      } else {
        printf("Unknown command (expect CLEAR)\n");
      }
    } else {
      printf("Write with offset/transaction not supported (offset=%u mode=%u)\n", offset, transaction_mode);
    }
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

  // setup attributes: access to the services/characteristics/descriptors
  att_server_init(profile_data, att_read_callback, att_write_callback);

  // setup security: handles pairing, authentication, encryption
  sm_init();
  sm_set_io_capabilities(IO_CAPABILITY_NO_INPUT_NO_OUTPUT);
  sm_event_callback_registration.callback = &sm_packet_handler;
  sm_add_event_handler(&sm_event_callback_registration);

  device_information_service_server_init();

  // turn off stdout buffering to help with debugging (disable in prod tho)
  // setvbuf(stdin, NULL, _IONBF, 0);

  btstack_tlv_get_instance(&tlv_impl, &tlv_context);

  gap_set_scan_params(1, 0x0030, 0x0030, 0);

  // lets go!
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
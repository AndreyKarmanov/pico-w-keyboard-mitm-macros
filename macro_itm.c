#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <inttypes.h>
#include <btstack_tlv.h>

#include "pico/cyw43_arch.h"
#include "pico/stdio.h"
#include "pico/time.h"

#include "macro_itm.h"
#include "btstack_config.h"
#include "btstack.h"

typedef enum
{
  APP_STATE_IDLE = 0,    // No connections active yet (BT stack not ready)
  APP_STATE_ADVERTISING, // Advertising to host, awaiting subscription
} app_state_t;
static app_state_t app_state;

static const char *app_state_strings[] = {
    "IDLE",
    "ADVERTISING",
};

static btstack_timer_source_t connection_timer;

static btstack_packet_callback_registration_t hci_event_callback_registration;
static btstack_packet_callback_registration_t sm_event_callback_registration;

static const btstack_tlv_t *btstack_tlv_singleton_impl;
static void *btstack_tlv_singleton_context;
static hci_con_handle_t con_handle_temp = HCI_CON_HANDLE_INVALID;
static hci_con_handle_t con_handle_host = HCI_CON_HANDLE_INVALID;

// Config Advertisement Data
const uint8_t config_adv_data[] = {
    // Flags general discoverable, Classic not supported (LE only)
    0x02,
    BLUETOOTH_DATA_TYPE_FLAGS,
    0x06, // LE General Discoverable Mode

    // Name
    0x05,
    BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME,
    'M', 'I', 'T', 'M', // "MITM"

    // Complete List of 128-bit Service UUIDs (complete means advertisement contains all UUIDs available)
    0x11,
    BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS, // 0x07
    0xEF, 0xCD, 0xAB, 0x89, 0x67, 0x45, 0x23, 0x01,
    0xEF, 0xCD, 0xAB, 0x90, 0x78, 0x56, 0x34, 0x12, /* 128-bit UUIDs: 12345678-90AB-CDEF-0123-456789ABCDEF */
};
const uint8_t config_adv_data_len = sizeof(config_adv_data);

static void app_set_state(app_state_t new_state)
{
  if (new_state == app_state)
    return;
  printf("[STATE] %s -> %s\n", app_state_strings[app_state], app_state_strings[new_state]);
  app_state = new_state;
}

static void hci_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
  UNUSED(channel);
  UNUSED(size);

  if (packet_type != HCI_EVENT_PACKET)
    return;

  switch (hci_event_packet_get_type(packet))
  {
  case BTSTACK_EVENT_STATE:
    switch (btstack_event_state_get_state(packet))
    {
    case HCI_STATE_WORKING:
      gap_advertisements_enable(1);
      app_set_state(APP_STATE_ADVERTISING);
      break;
    default:
      break;
    }
    break;
  case HCI_EVENT_META_GAP:
    switch (hci_event_gap_meta_get_subevent_code(packet))
    {
    case GAP_SUBEVENT_LE_CONNECTION_COMPLETE:
      uint8_t role = gap_subevent_le_connection_complete_get_role(packet);
      if (role == HCI_ROLE_SLAVE)
      {
        printf("Central Device Connected\n");
        app_set_state(APP_STATE_IDLE);
      }
      else if (role == HCI_ROLE_MASTER)
      {
        printf("Peripheral Device Connected\n");
      }
      break;
    default:
      break;
    }
    break;
  case HCI_EVENT_DISCONNECTION_COMPLETE:
    switch (hci_event_disconnection_complete_get_status(packet))
    {
    case ERROR_CODE_SUCCESS:
      con_handle_temp = hci_event_disconnection_complete_get_connection_handle(packet);
      uint8_t role = hci_connection_for_handle(con_handle_temp)->role;
      con_handle_temp = HCI_CON_HANDLE_INVALID;
      if (role == HCI_ROLE_SLAVE)
      {
        printf("Central Device Disconnected\n");
        gap_advertisements_enable(1);
        app_set_state(APP_STATE_ADVERTISING);
      }
      else if (role == HCI_ROLE_MASTER)
      {
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

static void sm_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
  UNUSED(channel);
  UNUSED(size);

  if (packet_type != HCI_EVENT_PACKET)
    return;

  switch (hci_event_packet_get_type(packet))
  {
  case SM_EVENT_JUST_WORKS_REQUEST:
    printf("Just works requested\n");
    sm_just_works_confirm(sm_event_just_works_request_get_handle(packet));
    break;
  case SM_EVENT_NUMERIC_COMPARISON_REQUEST:
    printf("Confirming numeric comparison: %" PRIu32 "\n", sm_event_numeric_comparison_request_get_passkey(packet));
    sm_numeric_comparison_confirm(sm_event_numeric_comparison_request_get_handle(packet));
    break;
  case SM_EVENT_PASSKEY_DISPLAY_NUMBER:
    printf("Display Passkey: %" PRIu32 "\n", sm_event_passkey_display_number_get_passkey(packet));
    break;
  case SM_EVENT_PAIRING_COMPLETE:
    switch (sm_event_pairing_complete_get_status(packet))
    {
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
    switch (sm_event_reencryption_complete_get_status(packet))
    {
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

static uint16_t att_read_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t offset, uint8_t *buffer, uint16_t buffer_size)
{

  printf("ATT Read Callback: handle 0x%04X, offset %u, buffer size %u\n", att_handle, offset, buffer_size);

  if (att_handle == ATT_CHARACTERISTIC_12345678_90AB_CDEF_0123_456789ABCDF0_01_VALUE_HANDLE)
  {
    const uint8_t *status = app_state_strings[app_state];
    int len = strlen(status);
    if (offset >= len)
      return 0;
    return att_read_callback_handle_blob(status, len, offset, buffer, buffer_size);
  }

  return 0;
};

static int att_write_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size)
{
  printf("ATT Write Callback: handle 0x%04X, offset %u, buffer size %u\n", att_handle, offset, buffer_size);
  return 0;
}

int btstack_main(int argc, const char *argv[])
{

  (void)argc;
  (void)argv;

  /* Organized in a chronological manner of what is used and in what order */

  // Register for HCI events, main communication channel between btstack and application
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
  gap_advertisements_set_data(config_adv_data_len, (uint8_t *)config_adv_data);

  // setup attributes: access to the services/characteristics/descriptors
  att_server_init(profile_data, att_read_callback, att_write_callback);

  // setup security: handles pairing, authentication, encryption
  sm_init();
  sm_set_io_capabilities(IO_CAPABILITY_NO_INPUT_NO_OUTPUT);
  sm_event_callback_registration.callback = &sm_packet_handler;
  sm_add_event_handler(&sm_event_callback_registration);
  
  // Disable stdout buffering (for debugging)
  setvbuf(stdin, NULL, _IONBF, 0);

  // lets go!
  hci_power_control(HCI_POWER_ON);

  return 0;
}

int main(int argc, const char *argv[])
{

  stdio_init_all();

  if (cyw43_arch_init())
  {
    printf("failed to initialise cyw43_arch\n");
    return -1;
  }

  btstack_main(argc, argv);

  cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
  btstack_run_loop_execute();
}
# Copilot instructions for this repo

This repo is a Raspberry Pi Pico W (CYW43) + BTstack project that MITMs a BLE keyboard: it scans and connects to a target keyboard as a LE Central (HID over GATT Client) while advertising a configuration service as a LE Peripheral. Core logic lives in `macro_itm.c`.

## Architecture and data flow
- Dual-role app with small state machines:
  - `target_state` (scan → connect → bonded → HID client connected)
  - `host_state` (advertise → connected)
  - See transitions in `target_set_state()` / `host_set_state()` and event handlers in `hci_packet_handler()` and `sm_packet_handler()`.
- Central (to target keyboard):
  - Scan (`gap_start_scan`), remember seen devices, then connect (`gap_connect`) to the selected target.
  - Pair/bond via SM, then connect HIDS Client (`hids_client_connect`).
  - HID descriptor is printed and persisted; HID reports are printed. Persistence uses TLV (see below).
- Peripheral (to host):
  - Advertises with name "MITM" and exposes a custom GATT service for configuration, not yet a HID Device to the host.

## Persistence (TLV) and formats
- TLV backend provided by BTstack (`btstack_tlv_get_instance`).
- Tags:
  - `TRGT` (0x54524754): target device as single ASCII line `AA:BB:CC:DD:EE:FF,<type>,<name>`.
  - `HIDD` (0x48494444): raw HID descriptor blob from the target.
- Helper: `parse_device_string()` converts the ASCII line into `target_device_t`.

## Custom GATT service (configuration)
Defined in `macro_itm.gatt` and code-generated to `build/generated/macro_itm_gatt_header/macro_itm.h`:
- Service UUID: `12345678-90AB-CDEF-0123-456789ABCDEF`.
- Characteristics:
  - `…CDF0` (READ | WRITE WITHOUT RESP | DYNAMIC):
    - READ returns newline-delimited seen devices as `MAC,TYPE,NAME`.
    - WRITE accepts `CLEAR` to reset the seen list.
  - `…CDF1` (READ | WRITE WITHOUT RESP | DYNAMIC):
    - READ returns the current target as `MAC,TYPE,NAME`.
    - WRITE `CLEAR` removes the target and disconnects.
    - WRITE `AA:BB:CC:DD:EE:FF,<type>,<name>` sets new target (stored via TLV).
- Do not edit the generated header; change `macro_itm.gatt` and rebuild.

## Build and flash (VS Code tasks)
- Build: task "Compile Project" (Ninja `-C build`). Output: `build/macro_itm.uf2` plus `.elf/.hex`.
- Load over USB (UF2 via ROM or picotool): task "Run Project".
- Flash via SWD/OpenOCD: task "Flash".
- CMake focuses on Pico W (`PICO_BOARD pico_w`), links `pico_btstack_ble`, `pico_btstack_cyw43`, and generates GATT header via `pico_btstack_make_gatt_header()`.

## Logging / stdio
- In `CMakeLists.txt`: `pico_enable_stdio_uart(macro_itm 1)` and `pico_enable_stdio_usb(macro_itm 0)`. Logs go to UART (GP0/GP1). Switch these if you prefer USB CDC for logs, then rebuild.

## BTstack specifics and local quirk
- Events handled include `HCI_EVENT_GATTSERVICE_META` for HIDS/GATT service subevents and `HCI_EVENT_META_GAP` for GAP LE Connection Complete.
- README notes a local change in BTstack headers around HIDS event dispatching. If you update BTstack, verify event constants and rebuild the BTstack libs and this app.

## Key files
- App: `macro_itm.c` (state machines, GAP/SM/HIDS client, TLV, ATT read/write handlers).
- GATT definition: `macro_itm.gatt` → generated `macro_itm.h` included by `macro_itm.c`.
- BTstack config: `btstack_config.h` (increased connection counts, flow control for CYW43, max ATT DB size, etc.).
- Example for future host mirroring: `examples/example_keyboard.c` shows how to expose a HID Device (`hids_device_*`) and send input reports.

## Extending the project (examples)
- Forward target keyboard input to the host: subscribe to HIDS Client reports and emit via HIDS Device:
  - Receive in `GATTSERVICE_SUBEVENT_HID_REPORT` (client).
  - Send with `hids_device_send_input_report()` (device), as in `examples/example_keyboard.c`.
- Updating GATT: edit `macro_itm.gatt`, then rebuild to refresh `macro_itm.h` constants like `ATT_CHARACTERISTIC_…_VALUE_HANDLE` used in ATT callbacks.

If anything above is unclear (e.g., event constants, expected write formats, or build tasks on your setup), tell me and I’ll refine these notes.

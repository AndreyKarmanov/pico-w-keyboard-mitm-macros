# What is this project?

This project allows you to apply macros to keyboards before they even reach your computer, with no noticeable delay. 

This is especially useful to address issues such as
- remapping any keys 
  - ANY key: windows, command, alt, etc
  - done before it reaches the computer, so you can rebind keys without original shortcuts triggering 
- avoid security concerns with programs downloaded on the internet
  - this code is all hacked together from the btstack repo examples, and doesn't contain any capability to connect to the internet. All the code is in one file and (relatively) easy to follow.

future additions
- activity tracking
- mouse rebindings
- improved security in pairing (code request)
- support for mimicing the captured keyboard's BT advertisements

## Architecture

Here is a state diagram outlining the desired robust architecture for handling connections and disconnections:

```mermaid
stateDiagram-v2
    [*] --> Disconnected

    Disconnected: No connections active.
    Disconnected --> Scanning_for_Keyboard: Start searching for keyboard

    Scanning_for_Keyboard: Searching for a BLE HID keyboard to connect to.
    Scanning_for_Keyboard --> Connecting_to_Keyboard: Keyboard found
    Scanning_for_Keyboard --> Disconnected: Timeout or error

    Connecting_to_Keyboard: Attempting to connect to the found keyboard.
    Connecting_to_Keyboard --> Keyboard_Connected: Connection successful
    Connecting_to_Keyboard --> Scanning_for_Keyboard: Connection failed

    Keyboard_Connected: Connected to keyboard, not yet advertising to host.
    Keyboard_Connected --> Advertising_to_Host: Start advertising
    Keyboard_Connected --> Scanning_for_Keyboard: Keyboard disconnected

    Advertising_to_Host: Waiting for a host (PC) to connect.
    Advertising_to_Host --> Ready: Host connected
    Advertising_to_Host --> Scanning_for_Keyboard: Keyboard disconnected

    Ready: Connected to both keyboard and host. Ready to process and forward keystrokes.
    Ready --> Advertising_to_Host: Host disconnected
    Ready --> Scanning_for_Keyboard: Keyboard disconnected
```
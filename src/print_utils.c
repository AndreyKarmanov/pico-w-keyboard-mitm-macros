#include "print_utils.h"

#include <stdio.h>

void print_hid_report(const uint8_t* report, uint16_t len)
{
    char buffer[1024];  // adjust size depending on max HID report length
    int offset = 0;

    offset += snprintf(buffer + offset, sizeof(buffer) - offset, "HID Report: ");

    for (uint16_t i = 0; i < len && offset < (int)sizeof(buffer); i++) {
        offset += snprintf(buffer + offset, sizeof(buffer) - offset, "%02X ", report[i]);
    }

    snprintf(buffer + offset, sizeof(buffer) - offset, "\n");
    printf("%s", buffer);
}

void print_hid_descriptor(const uint8_t* descriptor, uint16_t len)
{
    char buffer[2048];  // adjust size depending on max HID descriptor length
    int offset = 0;

    offset += snprintf(buffer + offset, sizeof(buffer) - offset, "HID Descriptor: ");

    for (uint16_t i = 0; i < len && offset < (int)sizeof(buffer); i++) {
        offset += snprintf(buffer + offset, sizeof(buffer) - offset, "%02X ", descriptor[i]);
    }

    snprintf(buffer + offset, sizeof(buffer) - offset, "\n");
    printf("%s", buffer);
}

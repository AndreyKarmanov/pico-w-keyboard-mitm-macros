#ifndef PRINT_UTILS_H
#define PRINT_UTILS_H

#include <stdint.h>

void print_hid_report(const uint8_t* report, uint16_t len);
void print_hid_descriptor(const uint8_t* descriptor, uint16_t len);

#endif // PRINT_UTILS_H

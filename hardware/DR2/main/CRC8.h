#ifndef CRC8_H
#define CRC8_H

#include <stdint.h>
#include <stddef.h>

int get_bit(uint8_t data_buffer[], int bit_number);

void xor_bit(uint8_t data_buffer[], int bit_number, int bit);

uint8_t remainder(uint8_t data_buffer[], size_t data_size, uint16_t crc_key);

bool is_valid(uint8_t data_buffer[], size_t data_size, uint8_t remainder, uint16_t crc_key);

#endif
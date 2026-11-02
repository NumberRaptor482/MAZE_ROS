#include "CRC8.h"
#include "string.h"

int get_bit(uint8_t data_buffer[], size_t data_size, int bit_number) {
    int byte_number = data_size - ((int) bit_number / 8.0);
    int bit_offset = bit_number % 8;
    return (data_buffer[byte_number] >> bit_offset) & 1;
}

void xor_bit(uint8_t data_buffer[], size_t data_size, int bit_number, int bit) {
    int byte_number = data_size - (int) bit_number / 8.0;
    int bit_offset = bit_number % 8;
    uint8_t subtrahend = bit << bit_offset;
    data_buffer[byte_number] ^= subtrahend;
}

uint8_t remainder(uint8_t data_buffer[], size_t data_size, uint16_t crc_key) {
    uint8_t data_polynomial[data_size + 1];
    memcpy(data_polynomial, data_buffer, data_size);
    memset(data_polynomial + data_size, 0, 1);
    int data_bits = sizeof(data_polynomial) * 8;

    for( int i = data_bits - 1; i >= 8; i-- ) {
        if(get_bit(data_polynomial, data_size +1, i) == 1) {
            for(int j = 8; j >= 0; j--) {
                xor_bit(data_polynomial, data_size + 1, i - j, (crc_key >> j) & 1);
            }
        }
    }

    return data_polynomial[data_size];
}

bool is_valid(uint8_t data_buffer[], size_t data_size, uint8_t crc, uint16_t crc_key) {
    return remainder(data_buffer, data_size, crc_key) == crc;
}
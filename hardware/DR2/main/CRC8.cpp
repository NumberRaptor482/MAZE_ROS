#include "CRC8.h"
#include "string.h"

bool is_valid(char* data, size_t data_size, char crc, short crc_key) {
    // return remainder(data, data_size, crc_key) == crc;
    return false;
}

char remainder(char* data, size_t data_size, short crc_key) {
    // char data_buffer[data_size] = &data;
    // char data_polynomial[data_size + 1];
    // memcpy(data_polynomial, data_buffer, data_size);
    // memset(data_polynomial + data_size, 0, 1);
    // int data_bits = sizeof(data_polynomial) * 8; 

    // for( int i = data_bits - 1; i >= 9; i-- ) {
    //     if(get_bit(&data_polynomial) == 1) {
    //         for(int j = 8; j >= 0; j++) {
    //             xor_bit(&data_polynomial, sizeof(data_polynomial), j, crc_key >> j);
    //         }
    //     }
    // }

    return 0;
}

int get_bit(char* data, size_t data_size, int bit_number) {
    // char data_buffer[data_size] = &data;
    // int byte_number = (int) bit_number / 8.0;
    // int bit_offset = bit_number % 8;
    return 1; //data_buffer[byte_number] >> bit_offset;
}


void xor_bit(char* data, size_t data_size, int bit_number, int bit) {
    // char data_buffer[data_size] = &data;
    // int byte_number = (int) bit_number / 8.0;
    // int bit_offset = bit_number % 8;
    // char subtrahend = bit << bit_offset;
    // data_buffer[byte_number] ^= subtrahend;
}
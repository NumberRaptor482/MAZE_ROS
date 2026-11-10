#include "CRC8.h"
#include "string.h"

int get_bit(uint8_t data_buffer[], int bit_number) {
    int byte_number = ((int) bit_number / 8.0);
    int bit_offset = 7 - (bit_number % 8);
    return (data_buffer[byte_number] >> bit_offset) & 1;
}

void xor_bit(uint8_t data_buffer[], int bit_number, int bit) {
    int byte_number = (int) bit_number / 8.0;
    int bit_offset = 7 - (bit_number % 8);
    uint8_t subtrahend = bit << bit_offset;
    data_buffer[byte_number] ^= subtrahend;
}

uint8_t remainder(uint8_t data_buffer[], size_t data_size, uint16_t crc_key) {
    uint8_t data_polynomial[data_size + 1];
    memcpy(data_polynomial, data_buffer, data_size);
    memset(data_polynomial + data_size, 0, 1);
    int data_bits = sizeof(data_polynomial) * 8;

    for( int i = 0; i < data_bits - 8; i++ ) {
        if(get_bit(data_polynomial, i) == 1) {
            // printf("%d: \n", i);
            // printf("[");
            // for(int k = 0; k < sizeof(data_polynomial); k++) {
            //     printf("%d, ", data_polynomial[k]);
            // }
            // printf("]\n");
            for(int j = 8; j >= 0; j--) {
                xor_bit(data_polynomial, i + (8 - j), (crc_key >> j) & 1);
            }
            // printf("[");
            // for(int k = 0; k < sizeof(data_polynomial); k++) {
            //     printf("%d, ", data_polynomial[k]);
            // }
            // printf("]\n");
        }
    }

    return data_polynomial[data_size];
}

bool is_valid(uint8_t data_buffer[], size_t data_size, uint8_t crc, uint16_t crc_key) {
    return remainder(data_buffer, data_size, crc_key) == crc;
}

// int main(int argc, char ** argv) {
    // uint16_t crc_key = 0x12F;
    // CRC Checksum generated should be
    // uint8_t data_buffer[] = {1, 1, 1, 1, 1};

    // printf("%d\n", remainder(data_buffer, sizeof(data_buffer), crc_key));

    // return 0;
// }
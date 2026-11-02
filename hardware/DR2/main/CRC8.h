#ifndef CRC8_H
#define CRC8_H

bool is_valid(char* data, char remainder, short crc_key);

char remainder(char* data, short crc_key);

#endif CRC8_H
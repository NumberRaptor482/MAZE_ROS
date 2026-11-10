#include <errno.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <cstdio>
#include <stdint.h>
#include <stddef.h>
#include <chrono> // Required for time units
#include <thread> // Required for std::this_thread::sleep_fo
// #include <iostream>

#define PORT_NAME "/dev/ttyACM0"

struct COMMAND_VELOCITY {
    float linear_x;
    float yaw;
};


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

static int open_serializer_port(const char* devicePathName) {
    int fd = open(devicePathName, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        close(fd);
        return -1;
    }

    return fd;
}

static int configure_serial_port(int fd) {
    struct termios comm_config;
    tcgetattr(fd, &comm_config);

    // START OF AI SLOP \\
    // 1. Set Baud Rate to 9600
    cfsetispeed(&comm_config, B38400);
    cfsetospeed(&comm_config, B38400);

    // 2. Control Options (c_cflag)
    comm_config.c_cflag |= (CLOCAL | CREAD); // Enable receiver, ignore modem control lines
    comm_config.c_cflag &= ~PARENB;          // No parity bit (N)
    comm_config.c_cflag &= ~CSTOPB;          // 1 Stop bit (1)
    comm_config.c_cflag &= ~CSIZE;           // Clear data size bits
    comm_config.c_cflag |= CS8;              // 8 Data bits (8)
    comm_config.c_cflag &= ~CRTSCTS;         // Disable hardware flow control

    // 3. Input Options (c_iflag)
    // Disable software flow control (XON/XOFF) and disable line mappings
    comm_config.c_iflag &= ~(IXON | IXOFF | IXANY);
    comm_config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    // 4. Output Options (c_oflag)
    comm_config.c_oflag &= ~OPOST;           // Choose raw output (no character post-processing)
    comm_config.c_oflag &= ~ONLCR;           // Prevent mapping NL to CR-NL on output

    // 5. Local Options (c_lflag)
    // Choose raw input (turn off canonical processing, echoing, visual erasures, and signaling)
    comm_config.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // 6. Timeout / Blocking Rules (c_cc)
    comm_config.c_cc[VMIN]  = 1; // Minimum Read Size
    comm_config.c_cc[VTIME] = 10;             // 10 Second Timeout

    // Apply the settings immediately
    if (tcsetattr(fd, TCSANOW, &comm_config) != 0) {
        perror("Error applying termios settings");
        close(fd);
        return -1;
    }
    // END OF AI SLOP \\

    return fd;

}

static int read_serial_port(int fd, void *buf, size_t count) {
    return read(fd, buf, count);
}

static int write_serial_port(int fd, void *buf, size_t count) {
    return write(fd, buf, count);
}

int main(int argc, char ** argv) {
    (void) argc;
    (void) argv;

    uint8_t start_marker1_cmd_vel = 0xAA;
    uint8_t start_marker2_cmd_vel = 0x55;
    uint16_t crc8_key = 0x12F;

    int fd = open_serializer_port(PORT_NAME);
    if (fd < 0) {
        printf("%s\n", "Device failed to open");
        return 1;
    }
    if (configure_serial_port(fd) < 0) {
        printf("%s\n", "Device failed to configure");
        return 1;
    }

    // To let connection happen
    std::this_thread::sleep_for(std::chrono::seconds(2));

    struct COMMAND_VELOCITY velocity = {1.0f, 0.0f};
    struct COMMAND_VELOCITY received_velocity = {0.0f, 0.0f};

    for(int i = 0; i < 100; i++) {
        printf("%d: \n", i);
    	uint8_t serialized_velocity[sizeof(struct COMMAND_VELOCITY)];
    	memcpy(&serialized_velocity, &velocity, sizeof(struct COMMAND_VELOCITY));
        
        uint8_t crc_checksum = remainder(serialized_velocity, sizeof(serialized_velocity), crc8_key);        

        int w_success1 = write_serial_port(fd, &start_marker1_cmd_vel, sizeof(uint8_t));
        if(w_success1 < 0) {
            printf("%s\n", "Write1 failed!");
        } else {
            printf("%s\n", "Write1 Succeeded!");
            // printf("%d\n", w_success);
            // printf("%f\n", velocity.linear_x);
        }

        int w_success2 = write_serial_port(fd, &start_marker2_cmd_vel, sizeof(uint8_t));
        if(w_success2 < 0) {
            printf("%s\n", "Write2 failed!");
        } else {
            printf("%s\n", "Write2 Succeeded!");
            // printf("%d\n", w_success);
            // printf("%f\n", velocity.linear_x);
        }

    	int w_success3 = write_serial_port(fd, &serialized_velocity, sizeof(struct COMMAND_VELOCITY));
        if(w_success3 < 0) {
            printf("%s\n", "Write3 failed!");
        } else {
            printf("%s\n", "Writ3 Succeeded!");
        }

        int w_success4 = write_serial_port(fd, &crc_checksum, sizeof(uint8_t));
        if(w_success4 < 0) {
            printf("%s\n", "Write4 failed!");
        } else {
            printf("%s\n", "Write4 Succeeded!");
        }

        uint8_t received_marker1 = 0;
	    int r_success1 = read_serial_port(fd, &received_marker1, sizeof(uint8_t));
        if(r_success1 < 0) {
            printf("%s\n", "Read1 failed!");
        } else {
            printf("%s\n", "Read1 Succeeded!");
            printf("%d\n", received_marker1);
        }

        uint8_t received_marker2 = 0;
	    int r_success2 = read_serial_port(fd, &received_marker2, sizeof(uint8_t));
        if(r_success2 < 0) {
            printf("%s\n", "Read2 failed!");
        } else {
            printf("%s\n", "Read2 Succeeded!");
            printf("%d\n", received_marker2);
        }

        float received_markerA = 0;
	    int r_successA = read_serial_port(fd, &received_markerA, sizeof(float));
        if(r_successA < 0) {
            printf("%s\n", "ReadA failed!");
        } else {
            printf("%s\n", "ReadA Succeeded!");
            printf("%f\n", received_markerA);
        }

        float received_markerB = 0;
	    int r_successB = read_serial_port(fd, &received_markerB, sizeof(float));
        if(r_successB < 0) {
            printf("%s\n", "ReadB failed!");
        } else {
            printf("%s\n", "ReadB Succeeded!");
            printf("%f\n", received_markerB);
        }

        uint8_t serialized_received_velocity[sizeof(struct COMMAND_VELOCITY)];
	    int r_success3 = read_serial_port(
            fd, 
            &serialized_received_velocity, 
            sizeof(struct COMMAND_VELOCITY));
        if(r_success3 < 0) {
            printf("%s\n", "Read3 failed!");
        } else {
            printf("%s\n", "Read3 Succeeded!");
            memcpy(
                &received_velocity, 
                &serialized_received_velocity, 
                sizeof(struct COMMAND_VELOCITY));
            printf("%f\n", received_velocity.linear_x);
            printf("%f\n", received_velocity.yaw);
        }

        uint8_t crc_received = 0;
        int r_success4 = read_serial_port(fd, &received_velocity, sizeof(uint8_t));
        if(r_success4 < 0) {
            printf("%s\n", "Read4 failed!");
        } else {
            printf("%s\n", "Read4 Succeeded!");
            printf("%d\n", is_valid(
                serialized_received_velocity, 
                sizeof(struct COMMAND_VELOCITY),
                crc_received,
                crc8_key));
        }

        std::this_thread::sleep_for(std::chrono::seconds(5));
    }

    close(fd);

    return 0;
}

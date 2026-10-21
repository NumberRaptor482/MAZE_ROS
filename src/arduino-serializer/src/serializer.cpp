#include <errno.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <cstdio>
// #include <iostream>

#define PORT_NAME "/dev/ttyACM0"

struct CMD_VEL {
    float linear_x;
    float linear_y;
    float linear_z;
    float yaw;
    float pitch;
    float roll;
};

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
    // 1. Set Baud Rate to 38400
    cfsetispeed(&comm_config, B9600);
    cfsetospeed(&comm_config, B9600);

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
    comm_config.c_cc[VMIN]  = sizeof(struct CMD_VEL); // Minimum Read Size
    comm_config.c_cc[VTIME] = 0;             // 10 Second Timeout

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

    int fd = open_serializer_port(PORT_NAME);
    if (fd < 0) {
        printf("%s\n", "Device failed to open");
        return 1;
    }
    if (configure_serial_port(fd) < 0) {
        printf("%s\n", "Device failed to configure");
        return 1;
    }

    struct CMD_VEL velocity = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    struct CMD_VEL received_velocity = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    for(int i = 0; i < 100; i++) {
        printf("%d: \n", i);
    	char *serialized_velocity[sizeof(struct CMD_VEL)];
    	memcpy(serialized_velocity, &velocity, sizeof(struct CMD_VEL));

    	int w_success = write_serial_port(fd, serialized_velocity, sizeof(struct CMD_VEL));
        if(w_success < 0) {
            printf("%s\n", "Write failed!");
        } else {
            printf("%s\n", "Write Succeeded!");
            // printf("%d\n", w_success);
            // printf("%f\n", velocity.linear_x);
        }

	    char *received_serialized_velocity[sizeof(struct CMD_VEL)];
	    int r_success = read_serial_port(fd, received_serialized_velocity, sizeof(struct CMD_VEL));
        if(r_success < 0) {
            printf("%s\n", "Read failed!");
        } else {
            printf("%s\n", "Read Succeeded!");
            memcpy(&received_velocity, received_serialized_velocity, sizeof(struct CMD_VEL));
            printf("%d\n", r_success);
	        printf("%f\n", received_velocity.linear_x);
            printf("%f\n", received_velocity.linear_y);
	        printf("%f\n", received_velocity.linear_z);
	        printf("%f\n", received_velocity.yaw);
	        printf("%f\n", received_velocity.pitch);
	        printf("%f\n", received_velocity.roll);
        }

        printf("%s", "\n");
    }

    close(fd);

    return 0;
}

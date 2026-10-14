#include <string.h>
#include <cmath>

struct CMD_VEL {
    float linear_x;
    float linear_y;
    float linear_z;
    float yaw;
    float pitch;
    float roll;
};

struct CMD_VEL latest_command_velocity = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

// TODO: Get actual LED pin
int status_led_pin = -1;
int status_led_properly_received = 255;
int status_led_received = 128;
int status_led_off = 0;

// Temporary variable to test if serial data connection is working!
float test_desired_linear_x = 1.0f;

double epsilon = 1e-6;

void setup() {
    Serial.begin(38400);

    pinMode(status_led_pin, output);
}

void loop() {
    if(Serial.available() > sizeof(CMD_VEL)) {
        analogWrite(status_led_pin, status_led_received);
        memcpy(latest_command_velocity, &Serial.readBytes(sizeof(CMD_VEL)), sizeof(CMD_VEL))
    }

    if(std::abs(test_desired_linear_x - latest_command_velocity.linear_x) < epsilon) {
        analogWrite(status_led_pin, status_led_properly_received);
    }
}
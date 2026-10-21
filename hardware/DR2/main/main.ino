#include <string.h>
// #include <cmath>

struct CMD_VEL {
    float linear_x;
    float linear_y;
    float linear_z;
    float yaw;
    float pitch;
    float roll;
};

CMD_VEL latest_command_velocity = {10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f};

// TODO: Get actual LED pin
int status_led_pin = LED_BUILTIN;
int status_led_properly_received = 255;
int status_led_received = 128;
int status_led_off = 0;

// Temporary variable to test if serial data connection is working!
float test_desired_linear_x = 1.0f;

double epsilon = 1e-6;

void setup() {
    Serial.begin(38400);
    while (!Serial);

    pinMode(status_led_pin, OUTPUT);
    digitalWrite(status_led_pin, LOW);
}

void loop() {
    if(Serial.available() > sizeof(CMD_VEL)) {
        char buff[sizeof(CMD_VEL)];
        Serial.readBytes(buff, sizeof(CMD_VEL));
        memcpy(&latest_command_velocity, buff, sizeof(CMD_VEL));
        Serial.write(buff, sizeof(CMD_VEL));

    } else {
        digitalWrite(status_led_pin, LOW);
    }
}
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

static unsigned char buff[sizeof(struct CMD_VEL)];

CMD_VEL latest_command_velocity = {10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f};

// TODO: Get actual LED pin
int status_led_pin = LED_BUILTIN;

void setup() {
    Serial.begin(9600);
    while (!Serial);

    pinMode(status_led_pin, OUTPUT);
    digitalWrite(status_led_pin, LOW);
}

void loop() {
    // memcpy(buff, &latest_command_velocity, sizeof(latest_command_velocity));
    // Serial.write(buff, sizeof(struct CMD_VEL));
    bool has_read = false;

    if(Serial.available() >= sizeof(struct CMD_VEL)) {
        Serial.readBytes(buff, sizeof(struct CMD_VEL));
        memcpy(&latest_command_velocity, buff, sizeof(struct CMD_VEL));
        has_read = true;
    }

    // if(has_read) {
    Serial.write(buff, sizeof(struct CMD_VEL));
    // }
}
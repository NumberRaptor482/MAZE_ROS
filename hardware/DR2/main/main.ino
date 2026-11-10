#include <string.h>
#include "PIDController.h"
#include "DiffDriveKinematics.h"
#include "CRC8.h"


struct COMMAND_VELOCITY {
    float linear_x;
    float yaw_radiansps;
};

//////////////////// READING ENCODER DATA(Needs to be abstracted) ////////////////
int system_frequency_hz = 50;
float wheel_radius_m = 0.1;

bool directionLeft = true;
int durationLeft = 0;
int encoderLeftPinA = 1;
int encoderLeftPinB = 2;
float wheelSpeedLeftRadiansPS = 0;

void encoderInitLeft() {
    directionLeft = true;//default -> Forward  
    pinMode(encoderLeftPinB,INPUT);  
    attachInterrupt(0, wheelSpeedLeftUpdate, CHANGE);
}

void wheelSpeedLeftUpdate() {
    int Lstate = digitalRead(encoderLeftPinA);
    if((encoderLeftPinA == LOW) && Lstate==HIGH) {
        int val = digitalRead(encoderLeftPinB);
        if(val == LOW && directionLeft) {
            directionLeft = false; //Reverse
        }
        else if(val == HIGH && !directionLeft) {
            directionLeft = true;  //Forward
        }
    }
    encoderLeftPinA = Lstate;

    if(!directionLeft)  durationLeft++;
    else  durationLeft--;
    wheelSpeedLeftRadiansPS = durationLeft * system_frequency_hz * 2 * PI;
}

float get_left_motor_speed_mps() {
    return wheelSpeedLeftRadiansPS;
}

bool directionRight = true;
int durationRight = 0;
int encoderRightPinA = 1;
int encoderRightPinB = 2;
float wheelSpeedRightRadiansPS = 0;

void encoderInitRight() {
    directionRight = true;//default -> Forward  
    pinMode(encoderRightPinB,INPUT);  
    attachInterrupt(0, wheelSpeedRightUpdate, CHANGE);
}

void wheelSpeedRightUpdate() {
    int Lstate = digitalRead(encoderRightPinA);
    if((encoderRightPinA == LOW) && Lstate==HIGH) {
        int val = digitalRead(encoderRightPinB);
        if(val == LOW && directionRight) {
            directionRight = false; //Reverse
        }
        else if(val == HIGH && !directionRight) {
            directionRight = true;  //Forward
        }
    }
    encoderRightPinA = Lstate;

    if(!directionRight)  durationRight++;
    else  durationRight--;
    wheelSpeedRightRadiansPS = durationRight * system_frequency_hz * 2 * PI;
}

float get_right_motor_speed_mps() {
    return wheelSpeedRightRadiansPS;
}

//////////////////// CONTROL COMMANDS ////////////////
PIDController left_velocity_controller = PIDController(1.0, 0.0, 0.0, 1 / system_frequency_hz);
PIDController right_velocity_controller = PIDController(1.0, 0.0, 0.0, 1 / system_frequency_hz);

DiffDriveKinematics kinematics = DiffDriveKinematics(1.0, 1.0);

int leftIN1 = 0;
int leftIN2 = 0;
int leftPower = 0;
int rightIN1 = 0;
int rightIN2 = 0;
int rightPower = 0;

void init_motors() {
    pinMode (leftIN1, OUTPUT);
    pinMode (leftIN2, OUTPUT);
    pinMode (leftPower, OUTPUT);
    pinMode (rightIN1,   OUTPUT);
    pinMode (rightIN2, OUTPUT);
    pinMode (rightPower,   OUTPUT);
} 

void set_left_motor_pwm(int pwm_output) {
    if(pwm_output < 0) {
        digitalWrite(leftIN1, HIGH);
        digitalWrite(leftIN2, LOW);
        analogWrite(leftPower, pwm_output);
    } else {
        digitalWrite(leftIN1, LOW);
        digitalWrite(leftIN2, HIGH);
        analogWrite(leftPower, pwm_output);
    }
}

void set_right_motor_pwm(int pwm_output) {
    if(pwm_output < 0) {
        digitalWrite(rightIN1, HIGH);
        digitalWrite(rightIN2, LOW);
        analogWrite(rightPower, pwm_output);
    } else {
        digitalWrite(rightIN1, LOW);
        digitalWrite(rightIN2, HIGH);
        analogWrite(rightPower, pwm_output);
    }
}

void control_from_command_velocity(COMMAND_VELOCITY command_velocity) {
    DiffDriveKinematics::WHEEL_VELOCITIES reference_wheel_velocities = 
        kinematics.forward_kinematics(
            {command_velocity.linear_x, 
             command_velocity.yaw_radiansps});

    reference_wheel_velocities = kinematics.saturate_wheel_velocities(reference_wheel_velocities);

    set_left_motor_pwm(left_velocity_controller.calculate(
        reference_wheel_velocities.left_linear_mps,
        get_right_motor_speed_mps()));

    set_right_motor_pwm(right_velocity_controller.calculate(
        reference_wheel_velocities.right_linear_mps,
        get_right_motor_speed_mps()));

}

//////////////////// READING INPUT COMMAND ////////////////
static const char start_marker1_cmd_vel = 0xAA;
static const char start_marker2_cmd_vel = 0x55;
static const short crc8_key = 0x12F;

static bool reading_second_marker = false;
static bool reading_cmd_vel = false;
static int reading_index = 0;

unsigned char cmd_vel_buff[sizeof(struct COMMAND_VELOCITY)] = {0, 0};
static unsigned char crc8 = 0;

COMMAND_VELOCITY latest_command_velocity = {1.0f, 0.0f};

// Only data we get is command velocity, not robust to multiple structs at this point in time
void update_cmd_vel_from_serial() {
    while(Serial.available() > 0) {
        unsigned char current_byte = 0;
        // Guarunteed to be read because Serial.availabe() > 0
        Serial.readBytes(&current_byte, sizeof(current_byte));

        // Starts data reading
        if(!reading_second_marker && !reading_cmd_vel && current_byte == start_marker1_cmd_vel) {
            reading_cmd_vel = false;
            reading_second_marker = true;
            reading_index = 0;
            continue;
        }

        if(reading_second_marker && !reading_cmd_vel && current_byte == start_marker2_cmd_vel) {
            reading_cmd_vel = true;
            reading_second_marker = false;
            reading_index = 0;
            continue;
        }

        if(reading_cmd_vel) {
            if(reading_index < sizeof(struct COMMAND_VELOCITY)) {
                cmd_vel_buff[reading_index] = current_byte;
            } else if(reading_index == sizeof(struct COMMAND_VELOCITY)) {
                crc8 = current_byte;
                if ( is_valid(cmd_vel_buff, sizeof(struct COMMAND_VELOCITY), crc8, crc8_key)) {
                    memcpy(
                        &latest_command_velocity, 
                        cmd_vel_buff, 
                        sizeof(struct COMMAND_VELOCITY));
                    reading_second_marker = false;
                    reading_cmd_vel = false;
                    reading_index = 0;
                    memset(
                        cmd_vel_buff, 
                        0, 
                        sizeof(struct COMMAND_VELOCITY));
                    crc8 = 0;
                } else {
                    reading_second_marker = false;
                    reading_cmd_vel = false;
                    reading_index = 0;
                    memset(cmd_vel_buff, 0, sizeof(struct COMMAND_VELOCITY));
                    crc8 = 0;
                    continue;
                }
            } else {
                reading_second_marker = false;
                reading_cmd_vel = false;
                reading_index = 0;
                memset(cmd_vel_buff, 0, sizeof(struct COMMAND_VELOCITY));
                crc8 = 0;
                continue;
            }

            reading_index++;
        }
    }
}

// TODO: Get actual LED pin
int status_led_pin = LED_BUILTIN;

void setup() {
    Serial.begin(9600);
    while (!Serial);

    pinMode(status_led_pin, OUTPUT);
    digitalWrite(status_led_pin, LOW);

    // encoderInitLeft();
    // encoderInitRight();
    // init_motors();
}

void loop() {
    update_cmd_vel_from_serial();
    // control_from_command_velocity(latest_command_velocity);

    // delay(1000 / system_frequency_hz);
}


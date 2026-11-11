// #include <string.h>
#include "PIDController.h"
#include "DiffDriveKinematics.h"
#include "CRC8.h"
#include <stdint.h>
#include <stddef.h>


struct COMMAND_VELOCITY {
    float linear_x;
    float yaw_radiansps;
};

// // LED
int status_led_pin = LED_BUILTIN;

// //////////////////// READING ENCODER DATA(Needs to be abstracted) ////////////////
int system_frequency_hz = 50;
float wheel_radius_m = 0.1;

bool directionLeft = true;
int durationLeft = 0;
int encoderLeftPinA = 3;
int encoderLeftPinALast = 2;
int encoderLeftPinB = 7;
float wheelSpeedLeftRadiansPS = 0;

void encoderInitLeft() {
    directionLeft = true;//default -> Forward  
    pinMode(encoderLeftPinB,INPUT);  
    attachInterrupt(0, wheelSpeedLeftUpdate, CHANGE);
}

void wheelSpeedLeftUpdate() {
    int Lstate = digitalRead(encoderLeftPinA);
    if((encoderLeftPinALast == LOW) && Lstate==HIGH) {
        int val = digitalRead(encoderLeftPinB);
        if(val == LOW && directionLeft) {
            directionLeft = false; //Reverse
        }
        else if(val == HIGH && !directionLeft) {
            directionLeft = true;  //Forward
        }
    }
    encoderLeftPinALast = Lstate;

    if(!directionLeft)  durationLeft++;
    else  durationLeft--;
    wheelSpeedLeftRadiansPS = durationLeft * 2.0 * PI / 64.0;
}

float get_left_motor_speed_mps() {
    return wheelSpeedLeftRadiansPS * wheel_radius_m;
}

bool directionRight = true;
int durationRight = 0;
int encoderRightPinA = 2;
int encoderRightPinALast = 2;
int encoderRightPinB = 4;
float wheelSpeedRightRadiansPS = 0;

void encoderInitRight() {
    directionRight = true;//default -> Forward  
    pinMode(encoderRightPinB,INPUT);  
    attachInterrupt(1, wheelSpeedRightUpdate, CHANGE);
}

void wheelSpeedRightUpdate() {
    int Lstate = digitalRead(encoderRightPinA);
    if((encoderRightPinALast == LOW) && Lstate==HIGH) {
        int val = digitalRead(encoderRightPinB);
        if(val == LOW && directionRight) {
            directionRight = false; //Reverse
        }
        else if(val == HIGH && !directionRight) {
            directionRight = true;  //Forward
        }
    }
    encoderRightPinALast = Lstate;

    if(!directionRight)  durationRight++;
    else  durationRight--;
    wheelSpeedRightRadiansPS = durationRight * 2.0 * PI / 64.0;
}

float get_right_motor_speed_mps() {
    return wheelSpeedRightRadiansPS * wheel_radius_m;
}

void reset_encoder_duration() {
    durationLeft = 0;
    durationRight = 0;
}

// //////////////////// CONTROL COMMANDS ////////////////
PIDController left_velocity_controller = PIDController(1.0, 0.0, 0.0, 1 / system_frequency_hz);
PIDController right_velocity_controller = PIDController(1.0, 0.0, 0.0, 1 / system_frequency_hz);

DiffDriveKinematics kinematics = DiffDriveKinematics(1.0, 1.0);

int leftForwardPWM = 5;
int leftBackwardPWM = 6;
int left_enable_pin = 8;
int rightForwardPWM = 9;
int rightBackwardPWM = 10;
int right_enable_pin = 11;

void init_motors() {
    pinMode(left_enable_pin, OUTPUT);
    pinMode(right_enable_pin, OUTPUT);

    digitalWrite(left_enable_pin, HIGH);
    digitalWrite(right_enable_pin, HIGH);

    pinMode( leftForwardPWM, OUTPUT );
    pinMode( leftBackwardPWM, OUTPUT );
    pinMode( rightForwardPWM, OUTPUT );
    pinMode( rightBackwardPWM, OUTPUT );
} 


void set_left_motor_pwm(int pwm_output) {
    if(pwm_output < 0) {
        analogWrite(leftForwardPWM, 0);
        analogWrite(leftBackwardPWM, -pwm_output);
    } else {
        analogWrite(leftForwardPWM, pwm_output);
        analogWrite(leftBackwardPWM, 0);
    }
}

void set_right_motor_pwm(int pwm_output) {
    if(pwm_output < 0) {
        analogWrite(rightForwardPWM, 0);
        analogWrite(rightBackwardPWM, -pwm_output);
    } else {
        analogWrite(rightForwardPWM, pwm_output);
        analogWrite(rightBackwardPWM, 0);
    }
}

// void control_from_command_velocity(COMMAND_VELOCITY command_velocity) {
//     DiffDriveKinematics::WHEEL_VELOCITIES reference_wheel_velocities = 
//         kinematics.forward_kinematics(
//             {command_velocity.linear_x, 
//              command_velocity.yaw_radiansps});

//     reference_wheel_velocities = kinematics.saturate_wheel_velocities(reference_wheel_velocities);

//     set_left_motor_pwm( (int) 
//         (min(
//             left_velocity_controller.calculate(
//                 reference_wheel_velocities.left_linear_mps,
//                 get_right_motor_speed_mps()),
//             kinematics.get_max_wheel_speed()) 
//             * 
//         ( 255.0 
//             / 
//             kinematics.get_max_wheel_speed() )));

//     set_right_motor_pwm( (int) 
//         (min(
//             right_velocity_controller.calculate(
//                 reference_wheel_velocities.right_linear_mps,
//                 get_right_motor_speed_mps()),
//             kinematics.get_max_wheel_speed()) 
//             * 
//         ( 255.0 
//             / 
//             kinematics.get_max_wheel_speed() )));

// }

// //////////////////// READING INPUT COMMAND ////////////////
static const uint8_t start_marker1_cmd_vel = 0xAA;
static const uint8_t start_marker2_cmd_vel = 0x55;
static const uint16_t crc8_key = 0x12F;

static bool reading_second_marker = false;
static bool reading_cmd_vel = false;
static int reading_index = 0;

uint8_t cmd_vel_buff[sizeof(struct COMMAND_VELOCITY)] = {0, 0};
static uint8_t crc8 = 0;

COMMAND_VELOCITY latest_command_velocity = {1.0f, 0.0f};

// Only data we get is command velocity, not robust to multiple structs at this point in time
void update_cmd_vel_from_serial() {
    // digitalWrite(status_led_pin, HIGH);
    while(Serial.available() > 0) {
        uint8_t current_byte = Serial.read();
        // Guarunteed to be read because Serial.availabe() > 0
        // Serial.readBytes(&current_byte, sizeof(current_byte));

        // Starts data reading
        if(
            !reading_second_marker && 
            !reading_cmd_vel && 
            current_byte == start_marker1_cmd_vel) {

            reading_cmd_vel = false;
            reading_second_marker = true;
            reading_index = 0;
            Serial.write(current_byte);
            continue;
        }

        if (reading_second_marker && 
            !reading_cmd_vel && 
            current_byte == start_marker2_cmd_vel) {

            reading_cmd_vel = true;
            reading_second_marker = false;
            reading_index = 0;
            continue;
        }

        if(reading_cmd_vel) {
            if(reading_index < sizeof(struct COMMAND_VELOCITY)) {
                cmd_vel_buff[reading_index] = current_byte;
                Serial.write(current_byte);
            } else if(reading_index == sizeof(struct COMMAND_VELOCITY)) {
                crc8 = current_byte;
                if ( is_valid(cmd_vel_buff, sizeof(struct COMMAND_VELOCITY), crc8, crc8_key) ) {
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

void setup() {
    Serial.begin(38400);
    while (!Serial);
    // Serial.println("Program Version 1.0 - Loaded");

    encoderInitLeft();
    encoderInitRight();
    init_motors();

    pinMode(status_led_pin, OUTPUT);
    // digitalWrite(status_led_pin, LOW);
}

void motor_test() {
    set_left_motor_pwm(255);
    set_right_motor_pwm(255);
}

void encoder_test() {
    Serial.print("Left Pulse: ");
    Serial.println(durationLeft);

    Serial.print("Right Pulse: ");
    Serial.println(durationRight);
}


void loop() {
    // digitalWrite(status_led_pin, HIGH);
    // delay(1000);
    // digitalWrite(status_led_pin, LOW);
    // delay(1000);
    // update_cmd_vel_from_serial();

    // latest_command_velocity = {1.0, 0.0};
    // control_from_command_velocity(latest_command_velocity);
    analogWrite(leftForwardPWM, 128);
    analogWrite(leftBackwardPWM, 0);

    analogWrite(rightForwardPWM, 255);
    analogWrite(rightBackwardPWM, 0);
    // motor_test();
    encoder_test();

    // analogWrite(leftForwardPWM, 128);

    reset_encoder_duration();
    delay(1000);
}


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
int system_frequency_hz = 250;
float wheel_radius_m = 0.06;

volatile bool directionLeft = true;
volatile int durationLeft = 0;
volatile int encoderLeftPinA = 3;
volatile int encoderLeftInterruptPinA = encoderLeftPinA - 2;
volatile bool encoderLeftPinALastState = true;
volatile int encoderLeftPinB = 7;
volatile float wheelSpeedLeftRadiansPS = 0;

void encoderInitLeft() {
    directionLeft = true;//default -> Forward  
    pinMode(encoderLeftPinA, INPUT);
    pinMode(encoderLeftPinB, INPUT);
    attachInterrupt(encoderLeftInterruptPinA, wheelSpeedLeftUpdate, CHANGE);
}

void wheelSpeedLeftUpdate() {
    int Lstate = digitalRead(encoderLeftPinA);
    if( ( encoderLeftPinALastState == LOW ) && ( Lstate == HIGH) ) {
        int val = digitalRead(encoderLeftPinB);
        if(val == LOW && directionLeft) {
            directionLeft = false; //Reverse
        }
        else if(val == HIGH && !directionLeft) {
            directionLeft = true;  //Forward
        }
    }
    encoderLeftPinALastState = Lstate;

    if(!directionLeft)  durationLeft++;
    else  durationLeft--;
}

float get_left_motor_speed_mps() {
    return wheelSpeedLeftRadiansPS * wheel_radius_m;
}

volatile bool directionRight = true;
volatile int durationRight = 0;
volatile int encoderRightPinA = 2;
volatile int encoderRightInterruptPinA = encoderRightPinA - 2;
volatile bool encoderRightPinALast = true;
volatile int encoderRightPinB = 4;
volatile float wheelSpeedRightRadiansPS = 0;

void encoderInitRight() {
    directionRight = true;//default -> Forward  
    pinMode(encoderRightPinA, INPUT);
    pinMode(encoderRightPinB, INPUT);
    attachInterrupt(encoderRightInterruptPinA, wheelSpeedRightUpdate, CHANGE);
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
}

float get_right_motor_speed_mps() {
    return wheelSpeedRightRadiansPS * wheel_radius_m;
}

void reset_encoder_duration() {
    wheelSpeedLeftRadiansPS = (durationLeft * 2.0 * PI * system_frequency_hz) / (50 * 32.0);
    wheelSpeedRightRadiansPS = (durationRight * 2.0 * PI * system_frequency_hz) / (50 * 32.0);
    durationLeft = 0;
    durationRight = 0;
}

// //////////////////// CONTROL COMMANDS ////////////////
PIDController left_velocity_controller = PIDController(1.0, 0.0, 0.0, 1 / system_frequency_hz);
PIDController right_velocity_controller = PIDController(1.0, 0.0, 0.0, 1 / system_frequency_hz);

DiffDriveKinematics kinematics = DiffDriveKinematics(8 * 0.0254, 1.2);

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

// Takes in value between -255 and 255(inclusive)
void set_left_motor_pwm(int pwm) {
    // Motor Invert
    pwm *= -1;
    if(pwm < 0) {
        analogWrite(leftForwardPWM, 0);
        analogWrite(leftBackwardPWM, -pwm);
    } else {
        analogWrite(leftForwardPWM, pwm);
        analogWrite(leftBackwardPWM, 0);
    }
}

// Takes in value between -1 and 1
void set_left_motor_output(float output) {
    set_left_motor_pwm(
        (int) (255 * output)
    );
}

// Takes in value between -255 and 255
void set_right_motor_pwm(int pwm) {
    if(pwm < 0) {
        analogWrite(rightForwardPWM, 0);
        analogWrite(rightBackwardPWM, -pwm);
    } else {
        analogWrite(rightForwardPWM, pwm);
        analogWrite(rightBackwardPWM, 0);
    }
}

// Takes in value between -1 and 1
void set_right_motor_output(float output) {
    set_right_motor_pwm(
        (int) (255 * output)
    );
}

void control_from_command_velocity(COMMAND_VELOCITY command_velocity) {
    DiffDriveKinematics::WHEEL_VELOCITIES reference_wheel_velocities = 
        kinematics.forward_kinematics(
            {command_velocity.linear_x, 
             command_velocity.yaw_radiansps});

    reference_wheel_velocities = kinematics.saturate_wheel_velocities(reference_wheel_velocities);


}

void control_wheels(float left_wheel_mps, float right_wheel_mps) {
    float left_pid_output = left_velocity_controller.calculate(
        left_wheel_mps,
        get_left_motor_speed_mps());

    float right_pid_output = right_velocity_controller.calculate(
        right_wheel_mps,
        get_right_motor_speed_mps());

    if(left_pid_output < 0.0) {
        set_left_motor_pwm( (int) 
            min(
                abs(left_pid_output),
                kinematics.get_max_wheel_speed()));
    } else {
        set_left_motor_pwm( (int) 
            - min(
                abs(left_pid_output),
                kinematics.get_max_wheel_speed()));
    }

    if(right_pid_output < 0.0) {
        set_right_motor_pwm( (int) 
            min(
                abs(right_pid_output),
                kinematics.get_max_wheel_speed()));
    } else {
        set_right_motor_pwm( (int) 
            - min(
                abs(right_pid_output),
                kinematics.get_max_wheel_speed()));
    }

}

float output_trapezoid_accel(float max_output, float accel_time_s, float current_time_s) {
    return (max_output / accel_time_s) * min(current_time_s, accel_time_s);
}

float output_trapezoid_decel(float max_output, float decel_time_s, float current_time_s) {
    return max_output - output_trapezoid_accel(max_output, decel_time_s, current_time_s);
}

float output_trapezoid_profile(float max_output, float accel_time_s, float max_time_s, float decel_time_s, float current_time_s) {
    if( current_time_s >= 0.0 and current_time_s < accel_time_s) {
        return output_trapezoid_accel(max_output, accel_time_s, current_time_s);
    } else if(current_time_s >= accel_time_s and current_time_s < max_time_s + accel_time_s) {
        return max_output;
    } else {
        return output_trapezoid_decel(max_output, decel_time_s, current_time_s - max_time_s - accel_time_s);
    }
}

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


unsigned long start_time_ms = 5000;
unsigned long brake_time_ms = 1000;
unsigned long step_1_linear_ms = 4000;
unsigned long step_2_rotation_ms = 1400;
unsigned long step_3_linear_ms = 4000;
unsigned long step_4_rotation_ms = 1550;
unsigned long step_5_linear_ms = 4000;
unsigned long step_6_rotation_ms = 1550;
unsigned long step_7_linear_ms = 1875;
unsigned long step_8_rotation_ms = 1400;
unsigned long step_9_linear_ms = 2150;
unsigned long step_10_rotation_ms = 1550;
unsigned long step_11_linear_ms = 2225;
unsigned long step_12_rotation_ms = 1550;
unsigned long step_13_linear_ms = 1980;

unsigned long time_deltas_ms[] = {
    start_time_ms,
    step_1_linear_ms,
    // brake_time_ms,
    step_2_rotation_ms,
    // brake_time_ms,
    step_3_linear_ms,
    // brake_time_ms,
    step_4_rotation_ms,
    // brake_time_ms,
    step_5_linear_ms,
    // brake_time_ms,
    step_6_rotation_ms,
    // brake_time_ms,
    step_7_linear_ms,
    // brake_time_ms,
    step_8_rotation_ms,
    // brake_time_ms,
    step_9_linear_ms,
    // brake_time_ms,
    step_10_rotation_ms,
    // brake_time_ms,
    step_11_linear_ms,
    // brake_time_ms,
    step_12_rotation_ms,
    // brake_time_ms,
    step_13_linear_ms
};

unsigned long time_stamps_ms[15];

void setup() {
    Serial.begin(38400);
    while (!Serial);

    for (int i = 0; i < sizeof(time_stamps_ms) / sizeof(unsigned long); i++) {
        if(i == 0) {
            time_stamps_ms[i] = 0;
        } else {
            time_stamps_ms[i] = time_stamps_ms[i-1] + time_deltas_ms[i-1];
        }
        Serial.println(time_stamps_ms[i]);
    }

    encoderInitLeft();
    encoderInitRight();
    init_motors();

    pinMode(status_led_pin, OUTPUT);
}

void led_test() {
    digitalWrite(status_led_pin, HIGH);
    delay(1000);
    digitalWrite(status_led_pin, LOW);
    delay(1000);
}

void motor_test() {
    set_left_motor_pwm(255);
    set_right_motor_pwm(255);
}

void encoder_test() {
    Serial.print("Left Radians Per Second: ");
    Serial.println(wheelSpeedLeftRadiansPS);

    Serial.print("Left Meter Per Second: ");
    Serial.println(get_left_motor_speed_mps());

    Serial.print("Right Radians Per Second: ");
    Serial.println(wheelSpeedRightRadiansPS);

    Serial.print("Right Meter Per Second: ");
    Serial.println(get_right_motor_speed_mps());
}


void square_routine() {
    for(int i = 0; i < 4; i++) {
        // Forward
        analogWrite(leftForwardPWM, 192);
        analogWrite(leftBackwardPWM, 0);

        analogWrite(rightForwardPWM, 92);
        analogWrite(rightBackwardPWM, 0);
        delay(1000);

        // Turn
        analogWrite(leftForwardPWM, 32);
        analogWrite(leftBackwardPWM, 0);

        analogWrite(rightForwardPWM, 0);
        analogWrite(rightBackwardPWM, 32);
        delay(1000);
    }
}

void forward_pid() {
    control_wheels(1.0, 1.0);
}

void turn_pid() {
    control_wheels(1.0, 1.0);
}

// void rotate_left_maze(int i) {
//     float accel_ts = 0.4 * time_deltas_ms[i] / 1000.0;
//     float maxo_ts = 0.50 * time_deltas_ms[i] / 1000.0;
//     float decel_ts = 0.25 * time_deltas_ms[i] / 1000.0;
//     left_output = -output_trapezoid_profile(0.13, accel_ts, maxo_ts, decel_ts, time_s - time_stamps_ms[i] / 1000.0);
//     right_output = -left_output;
// }

// void rotate_right_maze(int i) {
//     float accel_ts = 0.4 * time_deltas_msi[i] / 1000.0;
//     float maxo_ts = 0.50 * time_deltas_ms[i] / 1000.0;
//     float decel_ts = 0.25 * time_deltas_ms[i] / 1000.0;
//     left_output = -output_trapezoid_profile(0.13, accel_ts, maxo_ts, decel_ts, time_s - time_stamps_ms[i] / 1000.0);
//     right_output = left_output;
// }

// void forward_maze(int i) {
//     float accel_ts = 0.4 * time_deltas_ms[i] / 1000.0;
//     float maxo_ts = 0.2 * time_deltas_ms[i] / 1000.0;
//     float decel_ts = 0.4 * time_deltas_ms[i] / 1000.0;
//     left_output = output_trapezoid_profile(0.3, accel_ts, maxo_ts, decel_ts, time_s - time_stamps_ms[i] / 1000.0);
//     right_output = left_output;
// }


void loop() {
    unsigned int time_zone = millis() % time_stamps_ms[15];
    float time_s = time_zone / 1000.0;

    float left_output = 0.0f;
    float right_output = 0.0f;

    if (time_zone >= time_stamps_ms[0] and time_zone < time_stamps_ms[1]) {

    } else if(time_zone >= time_stamps_ms[1] and time_zone < time_stamps_ms[2]) {
        float accel_ts = 0.4 * time_deltas_ms[1] / 1000.0;
        float maxo_ts = 0.2 * time_deltas_ms[1] / 1000.0;
        float decel_ts = 0.4 * time_deltas_ms[1] / 1000.0;
        left_output = output_trapezoid_profile(0.3, accel_ts, maxo_ts, decel_ts, time_s - time_stamps_ms[1] / 1000.0);
        right_output = left_output;
    }else if(time_zone >= time_stamps_ms[2] and time_zone < time_stamps_ms[3]) {
        float accel_ts = 0.25 * time_deltas_ms[2] / 1000.0;
        float maxo_ts = 0.50 * time_deltas_ms[2] / 1000.0;
        float decel_ts = 0.25 * time_deltas_ms[2] / 1000.0;
        left_output = -output_trapezoid_profile(0.13, accel_ts, maxo_ts, decel_ts, time_s - time_stamps_ms[2] / 1000.0);
        right_output = -left_output;
    } else if(time_zone >= time_stamps_ms[3] and time_zone < time_stamps_ms[4]) {
        float accel_ts = 0.4 * time_deltas_ms[3] / 1000.0;
        float maxo_ts = 0.2 * time_deltas_ms[3] / 1000.0;
        float decel_ts = 0.4 * time_deltas_ms[3] / 1000.0;
        left_output = output_trapezoid_profile(0.3, accel_ts, maxo_ts, decel_ts, time_s - time_stamps_ms[3] / 1000.0);
        right_output = left_output;
    } else if(time_zone >= time_stamps_ms[4] and time_zone < time_stamps_ms[5]) {
        float accel_ts = 0.25 * time_deltas_ms[4] / 1000.0;
        float maxo_ts = 0.50 * time_deltas_ms[4] / 1000.0;
        float decel_ts = 0.25 * time_deltas_ms[4] / 1000.0;
        left_output = -output_trapezoid_profile(0.13, accel_ts, maxo_ts, decel_ts, time_s - time_stamps_ms[4] / 1000.0);
        right_output = -left_output;
    } else if(time_zone >= time_stamps_ms[5] and time_zone < time_stamps_ms[6]) {
        float accel_ts = 0.4 * time_deltas_ms[5] / 1000.0;
        float maxo_ts = 0.2 * time_deltas_ms[5] / 1000.0;
        float decel_ts = 0.4 * time_deltas_ms[5] / 1000.0;
        left_output = output_trapezoid_profile(0.3, accel_ts, maxo_ts, decel_ts, time_s - time_stamps_ms[5] / 1000.0);
        right_output = left_output;
    } else if(time_zone >= time_stamps_ms[6] and time_zone < time_stamps_ms[7]) {
        float accel_ts = 0.25 * time_deltas_ms[6] / 1000.0;
        float maxo_ts = 0.50 * time_deltas_ms[6] / 1000.0;
        float decel_ts = 0.25 * time_deltas_ms[6] / 1000.0;
        left_output = -output_trapezoid_profile(0.13, accel_ts, maxo_ts, decel_ts, time_s - time_stamps_ms[6] / 1000.0);
        right_output = -left_output;
    } else if(time_zone >= time_stamps_ms[7] and time_zone < time_stamps_ms[8]) {
        float accel_ts = 0.4 * time_deltas_ms[7] / 1000.0;
        float maxo_ts = 0.2 * time_deltas_ms[7] / 1000.0;
        float decel_ts = 0.4 * time_deltas_ms[7] / 1000.0;
        left_output = output_trapezoid_profile(0.3, accel_ts, maxo_ts, decel_ts, time_s - time_stamps_ms[7] / 1000.0);
        right_output = left_output;
    } else if(time_zone >= time_stamps_ms[8] and time_zone < time_stamps_ms[9]) {
        float accel_ts = 0.25 * time_deltas_ms[8] / 1000.0;
        float maxo_ts = 0.50 * time_deltas_ms[8] / 1000.0;
        float decel_ts = 0.25 * time_deltas_ms[8] / 1000.0;
        left_output = output_trapezoid_profile(0.13, accel_ts, maxo_ts, decel_ts, time_s - time_stamps_ms[8] / 1000.0);
        right_output = -left_output;
    } else if(time_zone >= time_stamps_ms[9] and time_zone < time_stamps_ms[10]) {
        float accel_ts = 0.4 * time_deltas_ms[9] / 1000.0;
        float maxo_ts = 0.2 * time_deltas_ms[9] / 1000.0;
        float decel_ts = 0.4 * time_deltas_ms[9] / 1000.0;
        left_output = output_trapezoid_profile(0.3, accel_ts, maxo_ts, decel_ts, time_s - time_stamps_ms[9] / 1000.0);
        right_output = left_output;
    } else if(time_zone >= time_stamps_ms[10] and time_zone < time_stamps_ms[11]) {
        float accel_ts = 0.25 * time_deltas_ms[10] / 1000.0;
        float maxo_ts = 0.50 * time_deltas_ms[10] / 1000.0;
        float decel_ts = 0.25 * time_deltas_ms[10] / 1000.0;
        left_output = -output_trapezoid_profile(0.13, accel_ts, maxo_ts, decel_ts, time_s - time_stamps_ms[10] / 1000.0);
        right_output = -left_output;
    } else if(time_zone >= time_stamps_ms[11] and time_zone < time_stamps_ms[12]) {
        float accel_ts = 0.4 * time_deltas_ms[11] / 1000.0;
        float maxo_ts = 0.2 * time_deltas_ms[11] / 1000.0;
        float decel_ts = 0.4 * time_deltas_ms[11] / 1000.0;
        left_output = output_trapezoid_profile(0.3, accel_ts, maxo_ts, decel_ts, time_s - time_stamps_ms[11] / 1000.0);
        right_output = left_output;
    } else if(time_zone >= time_stamps_ms[12] and time_zone < time_stamps_ms[13]) {
        float accel_ts = 0.25 * time_deltas_ms[12] / 1000.0;
        float maxo_ts = 0.50 * time_deltas_ms[12] / 1000.0;
        float decel_ts = 0.25 * time_deltas_ms[12] / 1000.0;
        left_output = -output_trapezoid_profile(0.13, accel_ts, maxo_ts, decel_ts, time_s - time_stamps_ms[12] / 1000.0);
        right_output = -left_output;
    } else if(time_zone >= time_stamps_ms[13] and time_zone < time_stamps_ms[14]) {
        float accel_ts = 0.4 * time_deltas_ms[13] / 1000.0;
        float maxo_ts = 0.2 * time_deltas_ms[13] / 1000.0;
        float decel_ts = 0.4 * time_deltas_ms[13] / 1000.0;
        left_output = output_trapezoid_profile(0.3, accel_ts, maxo_ts, decel_ts, time_s - time_stamps_ms[13] / 1000.0);
        right_output = left_output;
    }

    set_left_motor_output(left_output);
    set_right_motor_output(right_output);
    

    // encoder_test();
    reset_encoder_duration();
    delay(1000 / system_frequency_hz);
}


#include <string.h>
#include "PIDController.h"
#include "DiffDriveKinematics.h"

struct COMMAND_VELOCITY {
    float linear_x;
    float linear_y;
    float linear_z;
    float yaw;
    float pitch;
    float roll;
};

static const char start_marker_cmd_vel = 0;
static const char end_marker_cmd_vel = 255;


static bool reading_data_type = false;
static int reading_index = 0;
static unsigned char start_marker = 127;
static unsigned char end_marker = 128;

static unsigned char cmd_vel_buff[sizeof(struct COMMAND_VELOCITY)];

COMMAND_VELOCITY latest_command_velocity = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};

// TODO: Get actual LED pin
int status_led_pin = LED_BUILTIN;

PIDController left_velocity_controller = PIDController(1.0, 0.0, 0.0, 0.02);
PIDController right_velocity_controller = PIDController(1.0, 0.0, 0.0, 0.02);

DiffDriveKinematics kinematics = DiffDriveKinematics(1.0, 1.0);
float wheel_radius_m;

void setup() {
    Serial.begin(9600);
    while (!Serial);

    pinMode(status_led_pin, OUTPUT);
    digitalWrite(status_led_pin, LOW);
}

void loop() {
    // if(Serial.available() > 0) {
    //     if(!reading_data) {
    //         Serial.readBytes(start_marker, sizeof(start_marker_cmd_vel));
    //     }

    //     if(start_marker == start_marker_cmd_vel) {
    //         reading_data = true;
    //         Serial.readBytes(cmd_vel_buff[reading_index], sizeof(char)]);
    //         reading_index += sizeof(char);
    //     }

    //     memcpy(&latest_command_velocity, cmd_vel_buff, sizeof(struct CMD_VEL));
    //     has_read = true;
    // }

    control_from_drive_velocity(latest_command_velocity);    
}

void control_from_drive_velocity(COMMAND_VELOCITY drive_velocity) {
    DiffDriveKinematics::WHEEL_VELOCITIES reference_wheel_velocities = 
        kinematics.forward_kinematics(
            {drive_velocity.linear_x, 
             drive_velocity.yaw});

    reference_wheel_velocities = kinematics.saturate_wheel_velocities(reference_wheel_velocities);

    set_left_motor_pwm(left_velocity_controller.calculate(
        reference_wheel_velocities.left_linear_mps,
        get_right_motor_speed_mps()));

    set_right_motor_pwm(right_velocity_controller.calculate(
        reference_wheel_velocities.right_linear_mps,
        get_right_motor_speed_mps()));

}

void set_left_motor_pwm(float pwm_output) {
    // TODO: ADD MOTOR "DRIVER"
}

float get_left_motor_speed_mps() {
    // TODO: ADD ENCODER CODE
    return 0.0;
}

void set_right_motor_pwm(float pwm_output) {
    // TODO: ADD MOTOR "DRIVER"
}

float get_right_motor_speed_mps() {
    // TODO: ADD ENCODER CODE
    return 0.0;
}
/*
  Differential drive with:
  - moveForward(mm), moveReverse(mm), turnCCW(deg), turnCW(deg), stopNow()
  - square demo: 300mm square at startup
  - anti-buzz end behavior: "done latch" with hysteresis + reduced min PWM near target
  Serial (optional):
    F <mm>
    T <deg>
    S
*/
#include <Arduino.h>
#include <PID_v1.h>
// ----- Pins -----
const uint8_t L_A = 2, L_B = 12; // left encoder A and B pins. A is the only one connected to the interrupt because uno only has 2 interrupt pins
const uint8_t R_A = 3, R_B = 11; // right encoder A and B pins. A is connected to interrupt pin 3. 
#define LEFT_MOTOR1  5
#define LEFT_MOTOR2  6
#define RIGHT_MOTOR1 9
#define RIGHT_MOTOR2 10
#define KILL_WIRE 8 // not used anymore
#define LED 13 // builtin LED on arduino that can be used for debugging
// ----- Encoder counts -----
volatile long leftCount  = 0; // signed total encoder counts used to calculate distance traveled of the left wheel
volatile long rightCount = 0; // signed total encoder counts used to calculate distance traveled of the right wheel
// ----- Mechanics/encoder constants -----
const float WHEEL_RADIUS_MM = 120.0f; // radius in mm of the black side wheels
const float GEAR_RATIO = 30.0f;       // gear ratio is 30:1 so the motor/encoder spins 30 times for every 1 revolution of the wheel
const float COUNTS_PER_MOTOR_REV = 64.0f; // encoder pattern repeats 64 times per motor revolution
const float COUNTS_PER_OUTPUT_REV = COUNTS_PER_MOTOR_REV * GEAR_RATIO; // 1920 frequency for the grays encoding of the wheels
const float WHEEL_CIRCUM_MM = 2.0f * PI * WHEEL_RADIUS_MM; // circumference of the black side wheels needed to find distance traveled
const float MM_PER_COUNT = WHEEL_CIRCUM_MM / COUNTS_PER_OUTPUT_REV; // mm traveled of the wheel per encoder count
// ----- Robot geometry -----
const float ROBOT_RADIUS_MM = 122.2375f;          // center to wheel radius. Used to do point turns
const double MAX_SPEED = 2500;
const double BASE_POWER = 30;
const int PWM_MAX = 150; // max speed. 255 is very fast
const int PWM_MIN  = 60; // min speed. Going below this will make the motors stall
double kp = 1.0;
double ki = 0.6;
double kd = 0.0;

// ----- Moving Average Filter -----
const int MA_WINDOW_SIZE = 40;  // number of samples to average (adjust as needed: 3-10 typical)
double left_velocity_history[MA_WINDOW_SIZE];   // circular buffer for left velocity
double right_velocity_history[MA_WINDOW_SIZE];  // circular buffer for right velocity
int ma_index = 0;              // current index in circular buffer
bool ma_buffer_filled = false; // tracks if buffer has been filled at least once

// current velocity for the left wheel in mm/s (filtered)
double left_current_velocity_mm = 0.0;
// current velocity for the right wheel in mm/s (filtered)
double right_current_velocity_mm = 0.0;
// raw (unfiltered) velocity readings
double left_raw_velocity_mm = 0.0;
double right_raw_velocity_mm = 0.0;

double left_pid_out = 0.0;
double right_pid_out = 0.0;
// target velocity for the left wheel in mm/s
double left_target_velocity_mm = 300.0;
// targte velocity for the right wheel in mm/s
double right_target_velocity_mm = 0.0;
PID  leftPID(&left_current_velocity_mm,  &left_pid_out,  &left_target_velocity_mm,  kp, ki, kd, DIRECT);
PID rightPID(&right_current_velocity_mm, &right_pid_out, &right_target_velocity_mm, kp, ki, kd, DIRECT);

// ----- Moving Average Function -----
void updateMovingAverage(double left_raw, double right_raw) {
  // Store new values in circular buffer
  left_velocity_history[ma_index] = left_raw;
  right_velocity_history[ma_index] = right_raw;
  
  // Advance index (wrap around)
  ma_index = (ma_index + 1) % MA_WINDOW_SIZE;
  
  // Check if buffer is filled
  if (ma_index == 0) {
    ma_buffer_filled = true;
  }
  
  // Calculate number of samples to average
  int samples = ma_buffer_filled ? MA_WINDOW_SIZE : ma_index;
  if (samples == 0) samples = 1; // prevent division by zero on first call
  
  // Calculate averages
  double left_sum = 0.0;
  double right_sum = 0.0;
  for (int i = 0; i < samples; i++) {
    left_sum += left_velocity_history[i];
    right_sum += right_velocity_history[i];
  }
  
  left_current_velocity_mm = left_sum / samples;
  right_current_velocity_mm = right_sum / samples;
}

void initMovingAverage() {
  for (int i = 0; i < MA_WINDOW_SIZE; i++) {
    left_velocity_history[i] = 0.0;
    right_velocity_history[i] = 0.0;
  }
  ma_index = 0;
  ma_buffer_filled = false;
}

// ----- ISRs -----
void isrLeft() {
  // read from the other encoder pin to determine the direction of motion. Slightly lower resolution doing it this way
  // instead of two interrupts for both sides but arduino doesn't support that and its not really necessary
  if (digitalRead(L_B)) leftCount++;
  else leftCount--;
}
// Right encoder direction FLIPPED
void isrRight() {
  // see isrLeft
  if (digitalRead(R_B)) rightCount--;
  else rightCount++;
}
// ----- Helpers -----
void getCounts(long &l, long &r) {
  noInterrupts();
  l = leftCount;
  r = rightCount;
  interrupts();
}
// ----- Motor control -----
void setLeftMotor(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm >= 0) { analogWrite(LEFT_MOTOR1, pwm);  analogWrite(LEFT_MOTOR2, 0); }
  else          { analogWrite(LEFT_MOTOR1, 0);    analogWrite(LEFT_MOTOR2, -pwm); }
}
void setRightMotor(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm >= 0) { analogWrite(RIGHT_MOTOR1, pwm); analogWrite(RIGHT_MOTOR2, 0); }
  else          { analogWrite(RIGHT_MOTOR1, 0);   analogWrite(RIGHT_MOTOR2, -pwm); }
}
void setVelocity(float left, float right) {
  left_target_velocity_mm = left;
  right_target_velocity_mm = right;
}
void stopMotors() {
  setLeftMotor(0);
  setRightMotor(0);
}
void setup() {
  Serial.begin(115200);
  // configure encoder pins
  pinMode(L_A, INPUT_PULLUP); pinMode(L_B, INPUT_PULLUP);
  pinMode(R_A, INPUT_PULLUP); pinMode(R_B, INPUT_PULLUP);
  // set up interrupt
  attachInterrupt(digitalPinToInterrupt(L_A), isrLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(R_A), isrRight, RISING);
  // configure motor control pins
  pinMode(LEFT_MOTOR1, OUTPUT);  pinMode(LEFT_MOTOR2, OUTPUT);
  pinMode(RIGHT_MOTOR1, OUTPUT); pinMode(RIGHT_MOTOR2, OUTPUT);
  // LED for debugging if necessary
  pinMode(LED, OUTPUT);
  
  stopMotors();
  initMovingAverage();  // Initialize moving average buffers
  leftPID.SetMode(AUTOMATIC);
  rightPID.SetMode(AUTOMATIC);
  leftPID.SetOutputLimits(-150, 150);
  rightPID.SetOutputLimits(-150, 150);
}
void loop() {
  static long beginning = 0;
  if (beginning == 0) beginning = millis();
  else {
    if ((((millis() - beginning) / 5000) % 2) == 0) left_target_velocity_mm = 500;
    else left_target_velocity_mm = 150;
  }

  // first update velocity
  static unsigned long last_update = 0;
  static long last_left_count = 0;
  static long last_right_count = 0;
  if (last_update == 0) {
    last_update = micros();
    getCounts(last_left_count, last_right_count);
    return;
  }
  float dt = (micros() - last_update) / 1.0E6;
  last_update = micros();
  long new_left_count;
  long new_right_count;
  getCounts(new_left_count, new_right_count);
  
  // Calculate raw velocities
  left_raw_velocity_mm  = (new_left_count - last_left_count) * MM_PER_COUNT / dt;
  right_raw_velocity_mm = (new_right_count - last_right_count) * MM_PER_COUNT / dt;
  
  // Apply moving average filter to get smoothed velocities
  updateMovingAverage(left_raw_velocity_mm, right_raw_velocity_mm);
  
  last_left_count  = new_left_count;
  last_right_count = new_right_count;
  leftPID.Compute();
  rightPID.Compute();
  //left_pid_out  += (left_target_velocity_mm / MAX_SPEED) * BASE_POWER;
  //right_pid_out += (right_target_velocity_mm / MAX_SPEED) * BASE_POWER;
  left_pid_out = constrain(left_pid_out, -255, 255);
  right_pid_out = constrain(right_pid_out, -255, 255);
  static long last_print = 0;
  if (millis() - last_print > 20) {
    Serial.print(left_current_velocity_mm);  // filtered reading
    Serial.print(",");
    Serial.print(left_target_velocity_mm);
    Serial.print(",");
    Serial.println(left_pid_out);
    last_print = millis();
  }
  setLeftMotor(left_pid_out);
  setRightMotor(right_pid_out);
}
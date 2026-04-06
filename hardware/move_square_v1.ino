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

// ----- Pins -----
const uint8_t L_A = 2, L_B = 12;
const uint8_t R_A = 3, R_B = 11;

#define LEFT_MOTOR1 5
#define LEFT_MOTOR2 6
#define RIGHT_MOTOR1 9
#define RIGHT_MOTOR2 10
#define KILL_WIRE 8
#define LED 13

// ----- Encoder counts -----
volatile long leftCount = 0;
volatile long rightCount = 0;

// ----- Mechanics/encoder constants -----
const float WHEEL_RADIUS_MM = 120.0f;
const float GEAR_RATIO = 30.0f;
const float COUNTS_PER_MOTOR_REV = 64.0f;                              // x1 counting
const float COUNTS_PER_OUTPUT_REV = COUNTS_PER_MOTOR_REV * GEAR_RATIO; // 1920
const float WHEEL_CIRCUM_MM = 2.0f * PI * WHEEL_RADIUS_MM;
const float MM_PER_COUNT = WHEEL_CIRCUM_MM / COUNTS_PER_OUTPUT_REV;

// ----- Robot geometry -----
const float ROBOT_RADIUS_MM = 122.2375f; // center to wheel
const float TRACK_WIDTH_MM = 2.0f * ROBOT_RADIUS_MM;

// ----- PID state -----
float Kp = 2.0f;
float Ki = 0.2f;
float Kd = 0.2f; // smaller D helps reduce chatter

long targetCountsL = 0;
long targetCountsR = 0;

float integL = 0.0f, prevErrL = 0.0f;
float integR = 0.0f, prevErrR = 0.0f;

unsigned long prevUs = 0;

const int PWM_MAX = 100;

// Your motors won't spin below ~100 (far from target)
const int PWM_MIN_FAR = 100;
// Near target, allow smaller minimum to reduce buzz (tune 0..100)
const int PWM_MIN_NEAR = 60;

// Done/hold hysteresis (prevents rapid restart near endpoint)
const long STOP_IN_COUNTS = 18; // stop when within this
const long NEAR_RANGE_COUNTS = 12;
const long MIN_RANGE_TIME = 1000;
const long RESTART_COUNTS = 18; // only restart if error grows beyond this

const float I_LIMIT = 2000.0f;

// ----- ISRs -----
void isrLeft()
{
    if (digitalRead(L_B))
        leftCount++;
    else
        leftCount--;
}
// Right encoder direction FLIPPED
void isrRight()
{
    if (digitalRead(R_B))
        rightCount--;
    else
        rightCount++;
}

// ----- Helpers -----
void getCounts(long &l, long &r)
{
    noInterrupts();
    l = leftCount;
    r = rightCount;
    interrupts();
}

int clampWithMin(int pwm, int pwmMin)
{
    pwm = constrain(pwm, -PWM_MAX, PWM_MAX);
    if (pwm == 0)
        return 0;
    if (pwm > 0 && pwm < pwmMin)
        return pwmMin;
    if (pwm < 0 && -pwm < pwmMin)
        return -pwmMin;
    return pwm;
}

int pwmMinForErrCounts(float errCounts)
{
    // Reduce minimum PWM near the target to avoid buzzing/clicking
    if (fabs(errCounts) > 80.0f)
        return PWM_MIN_FAR;
    return PWM_MIN_NEAR;
}

// ----- Motor control -----
void setLeftMotor(int pwm)
{
    if (pwm >= 0)
    {
        analogWrite(LEFT_MOTOR1, pwm);
        analogWrite(LEFT_MOTOR2, 0);
    }
    else
    {
        analogWrite(LEFT_MOTOR1, 0);
        analogWrite(LEFT_MOTOR2, -pwm);
    }
}
void setRightMotor(int pwm)
{
    if (pwm >= 0)
    {
        analogWrite(RIGHT_MOTOR1, pwm);
        analogWrite(RIGHT_MOTOR2, 0);
    }
    else
    {
        analogWrite(RIGHT_MOTOR1, 0);
        analogWrite(RIGHT_MOTOR2, -pwm);
    }
}

void stopMotors()
{
    setLeftMotor(0);
    setRightMotor(0);
    integL = integR = 0;
}

void stopNow()
{
    stopMotors();
    noInterrupts();
    targetCountsL = leftCount;
    targetCountsR = rightCount;
    interrupts();
}

// Set targets as relative moves from NOW (independent deltas, in mm at wheels)
void setWheelDeltasMm(float leftDeltaMm, float rightDeltaMm)
{
    noInterrupts();
    long nowL = leftCount;
    long nowR = rightCount;
    interrupts();

    long deltaCountsL = lroundf(leftDeltaMm / MM_PER_COUNT);
    long deltaCountsR = lroundf(rightDeltaMm / MM_PER_COUNT);

    targetCountsL = nowL + deltaCountsL;
    targetCountsR = nowR + deltaCountsR;

    integL = integR = 0.0f;
    prevErrL = prevErrR = 0.0f;
    prevUs = micros();
}

// =======================
// Public motion functions
// =======================
void moveForward(float mm) { setWheelDeltasMm(mm, mm); }
void moveReverse(float mm) { setWheelDeltasMm(-mm, -mm); }
void turnCCW(float deg)
{
    float arc = PI * TRACK_WIDTH_MM * (deg / 360.0f);
    setWheelDeltasMm(-arc, +arc);
}
void turnCW(float deg)
{
    float arc = PI * TRACK_WIDTH_MM * (deg / 360.0f);
    setWheelDeltasMm(+arc, -arc);
}

int time_done = 0;
bool isMotionDone()
{
    long l, r;
    getCounts(l, r);

    bool is_done = (labs(targetCountsL - l) <= STOP_IN_COUNTS) &&
                   (labs(targetCountsR - r) <= STOP_IN_COUNTS);

    if (is_done)
    {
        if (time_done == 0)
            time_done = millis();
        else if (millis() - time_done >= 500)
        {
            digitalWrite(LED, HIGH);
            return true;
        }
    }
    else
        time_done = 0;
    digitalWrite(LED, LOW);
    return false;
}

// ----- Serial line reader -----
bool readLine(String &out)
{
    static String buf;
    while (Serial.available())
    {
        char ch = (char)Serial.read();
        if (ch == '\n')
        {
            out = buf;
            buf = "";
            return true;
        }
        if (ch != '\r')
            buf += ch;
    }
    return false;
}

// ----- Square state machine -----
enum SquareState
{
    SQ_IDLE,
    SQ_FWD,
    SQ_TURN
};
SquareState sqState = SQ_IDLE;
int sqSide = 0;

void startSquare300()
{
    sqSide = 0;
    sqState = SQ_FWD;
    moveForward(300.0f);
}

void updateSquare300()
{
    if (sqState == SQ_IDLE)
        return;
    if (!isMotionDone())
        return;

    if (sqState == SQ_FWD)
    {
        sqState = SQ_TURN;
        turnCCW(90.0f);
    }
    else
    { // SQ_TURN
        moveForward(300.0f);
        sqState = SQ_FWD;
    }
}

// ----- Done/hold hysteresis flags (prevents buzzing at the end) -----
bool holdStoppedL = false;
bool holdStoppedR = false;

void resetHoldFlags()
{
    holdStoppedL = false;
    holdStoppedR = false;
}

void setup()
{
    Serial.begin(115200);

    pinMode(L_A, INPUT_PULLUP);
    pinMode(L_B, INPUT_PULLUP);
    pinMode(R_A, INPUT_PULLUP);
    pinMode(R_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(L_A), isrLeft, RISING);
    attachInterrupt(digitalPinToInterrupt(R_A), isrRight, RISING);

    pinMode(LEFT_MOTOR1, OUTPUT);
    pinMode(LEFT_MOTOR2, OUTPUT);
    pinMode(RIGHT_MOTOR1, OUTPUT);
    pinMode(RIGHT_MOTOR2, OUTPUT);

    pinMode(KILL_WIRE, INPUT_PULLUP);

    pinMode(LED, OUTPUT);

    stopNow();
    prevUs = micros();

    Serial.println("Commands:");
    Serial.println("  F <mm>   : forward/backward (e.g. F 200, F -100)");
    Serial.println("  T <deg>  : turn in place (CCW+, CW-) (e.g. T 90, T -45)");
    Serial.println("  S        : stop");

    resetHoldFlags();
    startSquare300(); // comment out if you don't want auto-square
}

void loop()
{
    updateSquare300();

    // ----- Kill wire -----
    /*if (digitalRead(KILL_WIRE) == 0) {
      stopNow();
      Serial.println("Kill wire pulled");
      while (1) {}
    }*/

    // ----- Serial commands (optional) -----
    String line;
    if (readLine(line))
    {
        line.trim();
        if (line.length() > 0)
        {
            char cmd = toupper(line.charAt(0));
            if (cmd == 'S')
            {
                stopNow();
                resetHoldFlags();
                Serial.println("Stop");
            }
            else
            {
                int sp = line.indexOf(' ');
                if (sp < 0)
                {
                    Serial.println("Bad input. Use: F <mm> or T <deg> or S");
                }
                else
                {
                    float val = line.substring(sp + 1).toFloat();
                    resetHoldFlags();
                    if (cmd == 'F')
                    {
                        moveForward(val);
                        Serial.print("Forward mm: ");
                        Serial.println(val, 3);
                    }
                    else if (cmd == 'T')
                    {
                        turnCCW(val);
                        Serial.print("Turn deg: ");
                        Serial.println(val, 3);
                    }
                    else
                    {
                        Serial.println("Unknown cmd. Use: F, T, S");
                    }
                }
            }
        }
    }

    // ----- PID update -----
    unsigned long nowUs = micros();
    float dt = (nowUs - prevUs) * 1e-6f;
    if (dt <= 0)
        dt = 1e-6f;
    prevUs = nowUs;

    long posL, posR;
    getCounts(posL, posR);

    float errL = (float)(targetCountsL - posL);
    float errR = (float)(targetCountsR - posR);

    // Update hold flags with hysteresis
    if (!holdStoppedL && labs((long)errL) <= STOP_IN_COUNTS)
        holdStoppedL = true;
    if (holdStoppedL && labs((long)errL) >= RESTART_COUNTS)
        holdStoppedL = false;

    if (!holdStoppedR && labs((long)errR) <= STOP_IN_COUNTS)
        holdStoppedR = true;
    if (holdStoppedR && labs((long)errR) >= RESTART_COUNTS)
        holdStoppedR = false;

    // LEFT
    if (holdStoppedL)
    {
        setLeftMotor(0);
        integL = 0;
    }
    else
    {
        integL += errL * dt;
        integL = constrain(integL, -I_LIMIT, I_LIMIT);
        float derivL = (errL - prevErrL) / dt;
        prevErrL = errL;

        float uL = Kp * errL + Ki * integL + Kd * derivL;
        int pwmL = (int)lroundf(uL);
        pwmL = clampWithMin(pwmL, pwmMinForErrCounts(errL));
        setLeftMotor(pwmL);
    }

    // RIGHT
    if (holdStoppedR)
    {
        setRightMotor(0);
        integR = 0;
    }
    else
    {
        integR += errR * dt;
        integR = constrain(integR, -I_LIMIT, I_LIMIT);
        float derivR = (errR - prevErrR) / dt;
        prevErrR = errR;

        float uR = Kp * errR + Ki * integR + Kd * derivR;
        int pwmR = (int)lroundf(uR);
        pwmR = clampWithMin(pwmR, pwmMinForErrCounts(errR));
        setRightMotor(pwmR);
    }

    // ----- Debug print (10 Hz) -----
    static unsigned long lastMs = 0;
    if (millis() - lastMs >= 100)
    {
        lastMs += 100;
        Serial.print("sqState=");
        Serial.print((int)sqState);
        Serial.print(" side=");
        Serial.print(sqSide);
        Serial.print(" holdL=");
        Serial.print(holdStoppedL ? 1 : 0);
        Serial.print(" holdR=");
        Serial.print(holdStoppedR ? 1 : 0);

        float posMmL = posL * MM_PER_COUNT;
        float tgtMmL = targetCountsL * MM_PER_COUNT;
        float posMmR = posR * MM_PER_COUNT;
        float tgtMmR = targetCountsR * MM_PER_COUNT;

        Serial.print(" | L ");
        Serial.print(posMmL, 1);
        Serial.print("/");
        Serial.print(tgtMmL, 1);
        Serial.print(" R ");
        Serial.print(posMmR, 1);
        Serial.print("/");
        Serial.println(tgtMmR, 1);
    }
}
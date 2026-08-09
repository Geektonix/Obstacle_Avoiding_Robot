#include <Arduino.h>
#include <NewPing.h>

constexpr uint8_t FRONT_TRIG = A0;
constexpr uint8_t FRONT_ECHO = A1;
constexpr uint8_t LEFT_TRIG  = A2;
constexpr uint8_t LEFT_ECHO  = A3;
constexpr uint8_t RIGHT_TRIG = A4;
constexpr uint8_t RIGHT_ECHO = A5;

constexpr uint8_t IN1 = 9;
constexpr uint8_t ENA = 10;
constexpr uint8_t IN2 = 8;
constexpr uint8_t IN3 = 3;
constexpr uint8_t IN4 = 2;
constexpr uint8_t ENB = 5;


constexpr bool INVERT_LEFT_MOTOR  = true;
constexpr bool INVERT_RIGHT_MOTOR = false;


constexpr int16_t KP_INT = 3;
constexpr int16_t KD_INT = 4;


constexpr uint8_t BASE_FORWARD_PWM = 160;
constexpr uint8_t MAX_PWM          = 185;
constexpr uint8_t MIN_PWM          = 145;
constexpr uint8_t REVERSE_PWM      = 160;
constexpr uint8_t PIVOT_PWM        = 165;


constexpr uint8_t MAX_RANGE_CM   = 60;
constexpr uint8_t WALL_LOST_CM   = 30;
constexpr uint8_t TARGET_WALL_CM = 15;
constexpr uint8_t BRAKE_DIST_CM  = 18;
constexpr uint8_t REVERSE_DIST_CM= 8;
constexpr uint8_t CLEAR_DIST_CM  = 25;


constexpr uint32_t PING_INTERVAL_MS   = 20;
constexpr uint32_t REVERSE_PULSE_MS   = 250;
constexpr uint32_t MIN_PIVOT_TIME_MS = 200;
constexpr uint32_t MAX_PIVOT_TIME_MS = 800;
constexpr uint32_t MOTOR_DEADTIME_MS = 15;

enum SystemState : uint8_t { TRACKING, REVERSING, PIVOTING };
enum SensorID    : uint8_t { S_FRONT = 0, S_LEFT = 1, S_RIGHT = 2 };

NewPing sonarFront(FRONT_TRIG, FRONT_ECHO, MAX_RANGE_CM);
NewPing sonarLeft(LEFT_TRIG, LEFT_ECHO, MAX_RANGE_CM);
NewPing sonarRight(RIGHT_TRIG, RIGHT_ECHO, MAX_RANGE_CM);

SystemState state = TRACKING;
uint8_t sensorIndex = 0;
uint8_t filteredDist[3] = { MAX_RANGE_CM, MAX_RANGE_CM, MAX_RANGE_CM };

uint32_t lastPingMs = 0;
uint32_t stateChangeMs = 0;

int8_t pivotDirection = 1;
int16_t lastError = 0;
uint8_t frontCloseCount = 0;

void driveChannel(uint8_t pin1, uint8_t pin2, uint8_t pwmPin, int16_t rawPwm);
void setMotorsDirect(int16_t leftPwm, int16_t rightPwm);
void setMotorsSafe(int16_t leftPwm, int16_t rightPwm, bool directionChange);
void updateSensor(uint8_t id);
void updateSensors();
int16_t calculateError(uint8_t lDist, uint8_t rDist);

void track();
void reverse();
void pivot();

void setup() {
    pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
    pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);

    setMotorsSafe(0, 0, false);
    delay(300);
}

void loop() {
    uint32_t now = millis();
    if (now - lastPingMs < PING_INTERVAL_MS) return;
    lastPingMs = now;

    updateSensors();

    switch (state) {
        case TRACKING:  track();   break;
        case REVERSING: reverse(); break;
        case PIVOTING:  pivot();   break;
    }
}

void track() {
    uint32_t now = millis();
    uint8_t fDist = filteredDist[S_FRONT];
    uint8_t lDist = filteredDist[S_LEFT];
    uint8_t rDist = filteredDist[S_RIGHT];

    if (fDist > BRAKE_DIST_CM) {
        frontCloseCount = 0;
    } else {
        frontCloseCount++;
    }

    if (frontCloseCount >= 2) {
        frontCloseCount = 0;
        stateChangeMs = now;

        pivotDirection = (lDist > rDist) ? -1 : 1;
        if (lDist == MAX_RANGE_CM && rDist == MAX_RANGE_CM) {
            pivotDirection = 1;
        }

        if (fDist <= REVERSE_DIST_CM) {
            state = REVERSING;
            setMotorsSafe(-REVERSE_PWM, -REVERSE_PWM, true);
            return;
        }

        state = PIVOTING;
        int16_t leftPwm  = (pivotDirection == -1) ? -PIVOT_PWM : PIVOT_PWM;
        int16_t rightPwm = (pivotDirection == -1) ? PIVOT_PWM : -PIVOT_PWM;
        setMotorsSafe(leftPwm, rightPwm, true);
        return;
    }

    int16_t error = calculateError(lDist, rDist);
    int16_t dError = error - lastError;
    lastError = error;

    int16_t steering = constrain((KP_INT * error) + (KD_INT * dError), -25, 25);
    setMotorsDirect(BASE_FORWARD_PWM + steering, BASE_FORWARD_PWM - steering);
}

void reverse() {
    uint32_t now = millis();
    if (now - stateChangeMs < REVERSE_PULSE_MS) return;

    state = PIVOTING;
    stateChangeMs = now;

    int16_t leftPwm  = (pivotDirection == -1) ? -PIVOT_PWM : PIVOT_PWM;
    int16_t rightPwm = (pivotDirection == -1) ? PIVOT_PWM : -PIVOT_PWM;
    setMotorsSafe(leftPwm, rightPwm, true);
}

void pivot() {
    uint32_t now = millis();
    uint8_t fDist = filteredDist[S_FRONT];

    bool minTimeElapsed = (now - stateChangeMs >= MIN_PIVOT_TIME_MS);
    bool pathCleared    = (fDist >= CLEAR_DIST_CM);
    bool watchdogActive = (now - stateChangeMs >= MAX_PIVOT_TIME_MS);

    if (!(minTimeElapsed && pathCleared) && !watchdogActive) return;

    lastError = 0;
    frontCloseCount = 0;
    state = TRACKING;
    setMotorsSafe(BASE_FORWARD_PWM, BASE_FORWARD_PWM, true);
}

void updateSensors() {
    if (state == PIVOTING) {
        updateSensor(S_FRONT);
        return;
    }
    updateSensor(sensorIndex);
    sensorIndex = (sensorIndex + 1) % 3;
}

void updateSensor(uint8_t id) {
    uint8_t raw = 0;
    switch (id) {
        case S_FRONT: raw = sonarFront.ping_cm(); break;
        case S_LEFT:  raw = sonarLeft.ping_cm();  break;
        case S_RIGHT: raw = sonarRight.ping_cm(); break;
    }

    if (raw == 0) raw = MAX_RANGE_CM;
    filteredDist[id] = (uint8_t)(((uint16_t)filteredDist[id] * 1 + (uint16_t)raw * 3) >> 2);
}

int16_t calculateError(uint8_t lDist, uint8_t rDist) {
    bool leftValid  = (lDist < WALL_LOST_CM);
    bool rightValid = (rDist < WALL_LOST_CM);

    if (leftValid && rightValid) return (int16_t)rDist - (int16_t)lDist;
    if (leftValid)  return ((int16_t)TARGET_WALL_CM - (int16_t)lDist);
    if (rightValid) return ((int16_t)rDist - (int16_t)TARGET_WALL_CM);
    return 0;
}

void driveChannel(uint8_t pin1, uint8_t pin2, uint8_t pwmPin, int16_t rawPwm) {
    uint16_t absSpeed = abs(rawPwm);
    if (rawPwm == 0 || absSpeed < MIN_PWM) {
        digitalWrite(pin1, LOW);
        digitalWrite(pin2, LOW);
        analogWrite(pwmPin, 0);
        return;
    }

    uint8_t speed = (uint8_t)constrain(absSpeed, MIN_PWM, MAX_PWM);
    bool fwd = (rawPwm > 0);

    digitalWrite(pin1, fwd ? HIGH : LOW);
    digitalWrite(pin2, fwd ? LOW : HIGH);
    analogWrite(pwmPin, speed);
}

void setMotorsDirect(int16_t leftPwm, int16_t rightPwm) {
    if (INVERT_LEFT_MOTOR)  leftPwm  = -leftPwm;
    if (INVERT_RIGHT_MOTOR) rightPwm = -rightPwm;

    driveChannel(IN1, IN2, ENA, leftPwm);
    driveChannel(IN3, IN4, ENB, rightPwm);
}

void setMotorsSafe(int16_t leftPwm, int16_t rightPwm, bool directionChange) {
    if (directionChange) {
        setMotorsDirect(0, 0);
        delay(MOTOR_DEADTIME_MS);
    }
    setMotorsDirect(leftPwm, rightPwm);
}

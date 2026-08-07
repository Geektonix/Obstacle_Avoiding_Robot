#include <Arduino.h>
#include <NewPing.h>

#define FRONT_TRIG A0
#define FRONT_ECHO A1
#define LEFT_TRIG  A2
#define LEFT_ECHO  A3
#define RIGHT_TRIG A4
#define RIGHT_ECHO A5

#define IN1 9  
#define ENA 10 
#define IN2 8  
#define IN3 3  
#define IN4 2  
#define ENB 5  

#define INVERT_LEFT_MOTOR  1  
#define INVERT_RIGHT_MOTOR 0  

#define KP_INT 3  
#define KD_INT 4

#define BASE_FORWARD_PWM 160 
#define MAX_PWM          185 
#define MIN_PWM          145 
#define REVERSE_PWM      160
#define PIVOT_PWM        165 

#define MAX_RANGE_CM       60  
#define WALL_LOST_CM       30  
#define TARGET_WALL_CM     15  
#define BRAKE_DIST_CM      18  
#define REVERSE_DIST_CM    8   
#define CLEAR_DIST_CM      25  

#define PING_INTERVAL_MS   20  
#define REVERSE_PULSE_MS   250 
#define MIN_PIVOT_TIME_MS  200 
#define MAX_PIVOT_TIME_MS  800
#define MOTOR_DEADTIME_MS  15  

NewPing sonarFront(FRONT_TRIG, FRONT_ECHO, MAX_RANGE_CM);
NewPing sonarLeft(LEFT_TRIG, LEFT_ECHO, MAX_RANGE_CM);
NewPing sonarRight(RIGHT_TRIG, RIGHT_ECHO, MAX_RANGE_CM);

enum SystemState : uint8_t { TRACKING, REVERSING, PIVOTING };
enum SensorID    : uint8_t { S_FRONT = 0, S_LEFT = 1, S_RIGHT = 2 };

SystemState state = TRACKING;
uint8_t sensorIndex = 0;
uint8_t filteredDist[3] = { MAX_RANGE_CM, MAX_RANGE_CM, MAX_RANGE_CM };

uint32_t lastPingMs = 0;
uint32_t stateChangeMs = 0;

int8_t pivotDirection = 1; 
int16_t lastError = 0;
uint8_t frontCloseCount = 0;

void setMotorsDirect(int16_t leftPwm, int16_t rightPwm) {
    #if INVERT_LEFT_MOTOR
    leftPwm = -leftPwm;
    #endif

    #if INVERT_RIGHT_MOTOR
    rightPwm = -rightPwm;
    #endif

    if (leftPwm == 0) {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        analogWrite(ENA, 0);
    } else {
        bool fwd = leftPwm > 0;
        uint16_t absSpeed = abs(leftPwm);
        uint8_t speed = (absSpeed < MIN_PWM) ? 0 : (uint8_t)constrain(absSpeed, MIN_PWM, MAX_PWM);
        
        if (speed == 0) {
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, LOW);
            analogWrite(ENA, 0);
        } else {
            digitalWrite(IN1, fwd ? HIGH : LOW);
            digitalWrite(IN2, fwd ? LOW : HIGH);
            analogWrite(ENA, speed);
        }
    }

    if (rightPwm == 0) {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
        analogWrite(ENB, 0);
    } else {
        bool fwd = rightPwm > 0;
        uint16_t absSpeed = abs(rightPwm);
        uint8_t speed = (absSpeed < MIN_PWM) ? 0 : (uint8_t)constrain(absSpeed, MIN_PWM, MAX_PWM);

        if (speed == 0) {
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, LOW);
            analogWrite(ENB, 0);
        } else {
            digitalWrite(IN3, fwd ? HIGH : LOW);
            digitalWrite(IN4, fwd ? LOW : HIGH);
            analogWrite(ENB, speed);
        }
    }
}

void setMotorsSafe(int16_t leftPwm, int16_t rightPwm, bool directionChange) {
    if (directionChange) {
        setMotorsDirect(0, 0);
        delay(MOTOR_DEADTIME_MS);
    }
    setMotorsDirect(leftPwm, rightPwm);
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

    if (state == PIVOTING) {
        updateSensor(S_FRONT);
    } else {
        updateSensor(sensorIndex);
        sensorIndex = (sensorIndex + 1) % 3;
    }

    uint8_t fDist = filteredDist[S_FRONT];
    uint8_t lDist = filteredDist[S_LEFT];
    uint8_t rDist = filteredDist[S_RIGHT];

    if (state == TRACKING) {
        if (fDist <= BRAKE_DIST_CM) {
            frontCloseCount++;
        } else {
            frontCloseCount = 0;
        }

        if (frontCloseCount >= 2) {
            frontCloseCount = 0;
            stateChangeMs = now;

            if (lDist == MAX_RANGE_CM && rDist == MAX_RANGE_CM) {
                pivotDirection = 1; 
            } else {
                pivotDirection = (lDist > rDist) ? -1 : 1;
            }

            if (fDist <= REVERSE_DIST_CM) {
                state = REVERSING;
                setMotorsSafe(-REVERSE_PWM, -REVERSE_PWM, true);
            } else {
                state = PIVOTING;
                setMotorsSafe((pivotDirection == -1) ? -PIVOT_PWM : PIVOT_PWM,
                              (pivotDirection == -1) ? PIVOT_PWM : -PIVOT_PWM, true);
            }
            return;
        }

        int16_t error = 0;
        bool leftValid  = (lDist < WALL_LOST_CM);
        bool rightValid = (rDist < WALL_LOST_CM);

        if (leftValid && rightValid) {
            error = (int16_t)rDist - (int16_t)lDist;
        } else if (leftValid) {
            error = ((int16_t)TARGET_WALL_CM - (int16_t)lDist);
        } else if (rightValid) {
            error = ((int16_t)rDist - (int16_t)TARGET_WALL_CM);
        }

        int16_t dError = error - lastError;
        lastError = error;

        int16_t steering = (KP_INT * error) + (KD_INT * dError);
        steering = constrain(steering, -25, 25); 

        setMotorsDirect(BASE_FORWARD_PWM + steering, BASE_FORWARD_PWM - steering);

    } else if (state == REVERSING) {
        if (now - stateChangeMs >= REVERSE_PULSE_MS) {
            state = PIVOTING;
            stateChangeMs = now;
            setMotorsSafe((pivotDirection == -1) ? -PIVOT_PWM : PIVOT_PWM,
                          (pivotDirection == -1) ? PIVOT_PWM : -PIVOT_PWM, true);
        }

    } else if (state == PIVOTING) {
        bool minTimeElapsed = (now - stateChangeMs >= MIN_PIVOT_TIME_MS);
        bool pathCleared    = (fDist >= CLEAR_DIST_CM);
        bool watchdogActive = (now - stateChangeMs >= MAX_PIVOT_TIME_MS);

        if ((minTimeElapsed && pathCleared) || watchdogActive) {
            lastError = 0;
            frontCloseCount = 0;
            state = TRACKING;
            setMotorsSafe(BASE_FORWARD_PWM, BASE_FORWARD_PWM, true);
        }
    }
}

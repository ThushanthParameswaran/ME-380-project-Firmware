#pragma once

#include <Arduino.h>
#include "driver/pcnt.h"

class DCMotor
{
public:
    DCMotor(int encA, int encB, int rpwm, int lpwm,
            int rpwmCh, int lpwmCh,
            pcnt_unit_t unit, float countsPerRev);

    void begin();
    void update();
    void moveToDegrees(float degrees);
    int getPosition();
    void setPID(float kp_val, float ki_val, float kd_val);

private:
    int encoderA;
    int encoderB;

    int rpwmPin;
    int lpwmPin;

    int rpwmChannel;
    int lpwmChannel;

    pcnt_unit_t pcntUnit;

    float cpr;

    float targetPosition = 0;

    float kp = 0;
    float ki = 0;
    float kd = 0;

    float error = 0;
    float prev_error = 0;
    float integral = 0;
    float derivative = 0;

    float output = 0;

    float dt = 0;

    unsigned long prevTime = 0;
};
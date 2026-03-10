#ifndef DCMOTOR_H
#define DCMOTOR_H

#include <Arduino.h>
#include "driver/pcnt.h"

class DCMotor {

  public:

    // Constructor
    DCMotor(int encA, int encB, int rpwm, int lpwm);

    void begin();

    void moveToDegrees(float degrees);

    void update();   // run PID loop

    int getPosition();

  private:

    int encoderA;
    int encoderB;

    int rpwmPin;
    int lpwmPin;

    int targetPosition;

    // PID variables
    float kp = 3.5;
    float ki = 0.002;
    float kd = 0.13;

    float error = 0;
    float prev_error = 0;
    float integral = 0;
    float derivative = 0;
    float output = 0;

    unsigned long prevTime = 0;
    float dt = 0;

};

#endif
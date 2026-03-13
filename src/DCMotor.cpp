#include "DCMotor.h"

#define PWM_FREQ 20000
#define PWM_RESOLUTION 8

DCMotor::DCMotor(int encA, int encB, int rpwm, int lpwm,
                 int rpwmCh, int lpwmCh,
                 pcnt_unit_t unit, float countPerRev)
{
    encoderA = encA;
    encoderB = encB;

    rpwmPin = rpwm;
    lpwmPin = lpwm;

    rpwmChannel = rpwmCh;
    lpwmChannel = lpwmCh;

    pcntUnit = unit;
    
    cpr = countPerRev;
}

void DCMotor::begin()
{
    pinMode(rpwmPin, OUTPUT);
    pinMode(lpwmPin, OUTPUT);
    pinMode(encoderA, INPUT_PULLUP);
    pinMode(encoderB, INPUT_PULLUP);

    digitalWrite(rpwmPin, LOW);
    digitalWrite(lpwmPin, LOW);

    ledcSetup(rpwmChannel, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(rpwmPin, rpwmChannel);

    ledcSetup(lpwmChannel, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(lpwmPin, lpwmChannel);

    pcnt_config_t pcnt_config = {};

    pcnt_config.pulse_gpio_num = encoderA;
    pcnt_config.ctrl_gpio_num = encoderB;

    pcnt_config.unit = pcntUnit;
    pcnt_config.channel = PCNT_CHANNEL_0;

    pcnt_config.pos_mode = PCNT_COUNT_INC;
    pcnt_config.neg_mode = PCNT_COUNT_DEC;

    pcnt_config.lctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config.hctrl_mode = PCNT_MODE_KEEP;

    pcnt_config.counter_h_lim = 32767;
    pcnt_config.counter_l_lim = -32768;

    pcnt_unit_config(&pcnt_config);

    pcnt_set_filter_value(pcntUnit, 10);
    pcnt_filter_enable(pcntUnit);

    pcnt_counter_pause(pcntUnit);
    pcnt_counter_clear(pcntUnit);
    pcnt_counter_resume(pcntUnit);

    prevTime = millis();
}

int DCMotor::getPosition()
{
    int16_t position;
    pcnt_get_counter_value(pcntUnit, &position);
    return position;
}

void DCMotor::moveToDegrees(float degrees)
{
    targetPosition = (degrees / 360.0) * cpr;

    // reset PID state
    integral = 0;
    prev_error = targetPosition - getPosition();
}

void DCMotor::update()
{
    int position = getPosition();

    unsigned long now = millis();
    dt = (now - prevTime) / 1000.0;
    prevTime = now;

    // in case dt is 0
    if (dt <= 0.0001) return;

    error = targetPosition - position;

    if (abs(error) < 15)
    {
        ledcWrite(rpwmChannel, 0);
        ledcWrite(lpwmChannel, 0);
        return;
    }

    integral += error * dt;
    integral = constrain(integral, -5000, 5000);

    derivative = (error - prev_error) / dt;

    output = kp * error + ki * integral + kd * derivative;

    prev_error = error;

    int pwm = constrain(abs(output), 0, 150);

    if (output > 0)
    {
        ledcWrite(rpwmChannel, pwm);
        ledcWrite(lpwmChannel, 0);
    }
    else
    {
        ledcWrite(rpwmChannel, 0);
        ledcWrite(lpwmChannel, pwm);
    }
}
void DCMotor::setPID(float kp_val, float ki_val, float kd_val)
{
    kp = kp_val;
    ki = ki_val;
    kd = kd_val;
}
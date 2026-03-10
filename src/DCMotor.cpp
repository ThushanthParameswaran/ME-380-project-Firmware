#include "DCMotor.h"

#define PCNT_UNIT PCNT_UNIT_0

#define PWM_FREQ 20000
#define PWM_RESOLUTION 8
#define RPWM_CHANNEL 0
#define LPWM_CHANNEL 1

DCMotor::DCMotor(int encA, int encB, int rpwm, int lpwm)
{
  encoderA = encA;
  encoderB = encB;
  rpwmPin = rpwm;
  lpwmPin = lpwm;
}

void DCMotor::begin()
{

  pinMode(encoderA, INPUT);
  pinMode(encoderB, INPUT);

  ledcSetup(RPWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(rpwmPin, RPWM_CHANNEL);

  ledcSetup(LPWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(lpwmPin, LPWM_CHANNEL);

  pcnt_config_t pcnt_config = {};

  pcnt_config.pulse_gpio_num = encoderA;
  pcnt_config.ctrl_gpio_num = encoderB;

  pcnt_config.unit = PCNT_UNIT;
  pcnt_config.channel = PCNT_CHANNEL_0;

  pcnt_config.pos_mode = PCNT_COUNT_INC;
  pcnt_config.neg_mode = PCNT_COUNT_DEC;

  pcnt_config.lctrl_mode = PCNT_MODE_REVERSE;
  pcnt_config.hctrl_mode = PCNT_MODE_KEEP;

  pcnt_config.counter_h_lim = 32767;
  pcnt_config.counter_l_lim = -32768;

  pcnt_unit_config(&pcnt_config);

  pcnt_counter_pause(PCNT_UNIT);
  pcnt_counter_clear(PCNT_UNIT);
  pcnt_counter_resume(PCNT_UNIT);

  prevTime = millis();
}

int DCMotor::getPosition()
{
  int16_t position;
  pcnt_get_counter_value(PCNT_UNIT, &position);
  return -position;
}

void DCMotor::moveToDegrees(float degrees)
{

  // 524 counts = 360 degrees
  targetPosition = (degrees / 360.0) * 524;
}

void DCMotor::update()
{

  int position = getPosition();

  unsigned long now = millis();
  dt = (now - prevTime) / 1000.0;
  prevTime = now;

  error = targetPosition - position;

  integral += error * dt;
  integral = constrain(integral, -5000, 5000);

  derivative = (error - prev_error) / dt;

  output = kp * error + ki * integral + kd * derivative;

  prev_error = error;

  int pwm = constrain(abs(output), 0, 200);

  if (output > 0)
  {
    ledcWrite(RPWM_CHANNEL, pwm);
    ledcWrite(LPWM_CHANNEL, 0);
  }
  else
  {
    ledcWrite(RPWM_CHANNEL, 0);
    ledcWrite(LPWM_CHANNEL, pwm);
  }
}
#include <Arduino.h>
#include <util/atomic.h>

/* ------------------- PINS ------------------- */
#define ENCODER_A 2
#define ENCODER_B 3
#define PWM_PIN   9
#define IN1_PIN   8
#define IN2_PIN   7

/* ------------------- ENCODER ------------------- */
volatile int posi = 0;

/* ------------------- PID ------------------- */
float kp = 3;
float kd = 0.19;
float ki = 0.02;

float eprev = 0;
float eintegral = 0;

long prevT = 0;

/* ------------------- CONTROL ------------------- */
int target = -214;        //  SET YOUR TARGET ENCODER POSITION HERE
int deadband = 2;       // encoder counts
int minPWM = 130;       // minimum PWM to overcome friction

/* ------------------- ISR ------------------- */
void readEncoderA() {
  if (digitalRead(ENCODER_B)) posi++;
  else posi--;
}

/* ------------------- SETUP ------------------- */
void setup() {
  Serial.begin(9600);

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), readEncoderA, RISING);

  pinMode(PWM_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);

  prevT = micros();
}

/* ------------------- LOOP ------------------- */
void loop() {

  // Read encoder safely
  int pos;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = posi;
  }

  // Time step
  long currT = micros();
  float deltaT = (currT - prevT) * 1e-6;
  prevT = currT;
  if (deltaT <= 0) return;

  // Error
  int e = target - pos;

  // DEADZONE → STOP MOTOR
  if (abs(e) <= deadband) {
    analogWrite(PWM_PIN, 0);
    eintegral = 0;
    eprev = e;
    return;
  }

  // PID terms
  float dedt = (e - eprev) / deltaT;
  eintegral += e * deltaT;
  eintegral = constrain(eintegral, -300, 300);

  float u = kp * e + kd * dedt + ki * eintegral;

  // Direction
  if (u > 0) {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
  } else {
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
  }

  // Power with minimum PWM
  float pwr = fabs(u);
  pwr = constrain(pwr, minPWM, 255);
  analogWrite(PWM_PIN, pwr);

  eprev = e;

  // Debug
  Serial.print("Target: ");
  Serial.print(target);
  Serial.print("  Pos: ");
  Serial.println(pos);
}

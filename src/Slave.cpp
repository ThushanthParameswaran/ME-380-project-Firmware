#include <Arduino.h>

#define ENA 9
#define IN1 8
#define IN2 7

#define MOTOR_PWM 100      // speed (0–255)
#define MOVE_TIME 1000     // milliseconds
#define PAUSE_TIME 500

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
}

void loop() {
  // ---- Forward ----
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, MOTOR_PWM);
  delay(MOVE_TIME);

  // Stop
  analogWrite(ENA, 0);
  delay(PAUSE_TIME);

  // ---- Backward ----
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, MOTOR_PWM);
  delay(MOVE_TIME);

  // Stop
  analogWrite(ENA, 0);
  delay(PAUSE_TIME);
}

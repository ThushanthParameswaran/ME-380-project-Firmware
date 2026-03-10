#include <Arduino.h>
#include "DCMotor.h"

// Create motor object
DCMotor motor1(25, 26, 27, 14);

void setup()
{
  Serial.begin(115200);
  motor1.begin();

  Serial.println("Motor Ready");
}

void loop()
{
  // Check if Python sent data
  if (Serial.available() > 0)
  {
    String input = Serial.readStringUntil('\n');  // read angle
    int angle = input.toInt();

    Serial.print("Received Angle: ");
    Serial.println(angle);

    motor1.moveToDegrees(angle);
  }

  motor1.update();

  // Print current position
  Serial.print("Motor Position: ");
  Serial.println(motor1.getPosition());

  delay(100);
}
#include <Arduino.h>
#include "DCMotor.h"

const float CPR = 4192;

DCMotor motor1(25, 26, 27, 14, 0, 1, PCNT_UNIT_0,CPR);
DCMotor motor2(34, 35, 32, 33, 2, 3, PCNT_UNIT_1,CPR);


void setup()
{
    Serial.begin(115200);

    pinMode(27, OUTPUT);
    pinMode(14, OUTPUT);
    pinMode(32, OUTPUT);
    pinMode(33, OUTPUT);

    digitalWrite(27, LOW);
    digitalWrite(14, LOW);
    digitalWrite(32, LOW);
    digitalWrite(33, LOW);


    motor1.begin();
    motor2.begin();

    motor1.setPID(1.5, 0.0, 0.3);
    motor2.setPID(1.5, 0.0, 0.3);



    Serial.println("Motors Ready");
}

void loop()
{
    
    if (Serial.available())
    {
        String input = Serial.readStringUntil('\n');
        input.trim();

        int commaIndex = input.indexOf(',');

        if (commaIndex != -1)
        {
            int angle1 = input.substring(0, commaIndex).toInt();
            int angle2 = input.substring(commaIndex + 1).toInt();

            motor1.moveToDegrees(angle1);
            motor2.moveToDegrees(angle2);
        }
    }

    motor1.update();
    motor2.update();

    Serial.print("M1: ");
    Serial.print(motor1.getPosition());
    Serial.print(" | M2: ");
    Serial.println(motor2.getPosition());

    delay(10);
}
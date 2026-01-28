#include <arduino.h>


// pin connections for stepper
#define dirPinSteppper1 4
#define stepPinSteppper1 5




void setup() {
  Serial.begin (9600);


  pinMode(dirPinSteppper1, OUTPUT);
  pinMode(stepPinSteppper1, OUTPUT);

  // set direction of rotation to clockwise
  digitalWrite(dirPinSteppper1, HIGH);


}

void loop() {

  digitalWrite(stepPinSteppper1, HIGH);

  digitalWrite(stepPinSteppper1, LOW);
  delayMicroseconds(100);

}


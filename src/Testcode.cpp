#include <arduino.h>


// pin connections for stepper1
#define dirPinSteppper1 4
#define stepPinSteppper1 5

// pin connections for stepper2
#define dirPinSteppper2 6
#define stepPinSteppper2 7

// pin connections for stepper3 (Base Motor)
#define dirPinSteppper3 8
#define stepPinSteppper3 9

// pin COnnection for Encoder1
#define endoer1chA 2
#define endoer1chB 3

#define CPR 1024  // Encoder counts per full 360° rotation


volatile long temp, counter = 0; //This variable will increase or decrease depending on the rotation of encoder




// ---- REQUIRED FORWARD DECLARATIONS ----
void ai0();
void ai1();

void setup() {
  Serial.begin (9600);

  pinMode(endoer1chA, INPUT_PULLUP);
  pinMode(endoer1chB, INPUT_PULLUP);

  //Setting up interrupt
  //A rising pulse from encodenren activated ai0(). AttachInterrupt 0 is DigitalPin nr 2 on moust Arduino.
  attachInterrupt(0, ai0, RISING);
   
  //B rising pulse from encodenren activated ai1(). AttachInterrupt 1 is DigitalPin nr 3 on moust Arduino.
  attachInterrupt(1, ai1, RISING);




  pinMode(dirPinSteppper1, OUTPUT);
  pinMode(stepPinSteppper1, OUTPUT);

  pinMode(dirPinSteppper2, OUTPUT);
  pinMode(stepPinSteppper2, OUTPUT);

  pinMode(dirPinSteppper3, OUTPUT);
  pinMode(stepPinSteppper3, OUTPUT);


  // set direction of rotation to clockwise
  digitalWrite(dirPinSteppper1, HIGH);
  digitalWrite(dirPinSteppper2, HIGH);
  digitalWrite(dirPinSteppper3, HIGH);





}

void loop() {
  // Send the value of counter
  if( counter != temp ){
    
    Serial.println (counter);
    temp = counter;
  }
  

  digitalWrite(stepPinSteppper1, HIGH);
  digitalWrite(stepPinSteppper2, HIGH);
  digitalWrite(stepPinSteppper3, HIGH);
  delayMicroseconds(100);

  digitalWrite(stepPinSteppper1, LOW);
  digitalWrite(stepPinSteppper2, LOW);
  digitalWrite(stepPinSteppper3, LOW);
  delayMicroseconds(100);


}

void ai0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if(digitalRead(3)==LOW) {
  counter++;
  }else{
  counter--;
  }
  }
   
  void ai1() {
  // ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check with pin 2 to determine the direction
  if(digitalRead(2)==LOW) {
  counter--;
  }else{
  counter++;
  }
  }

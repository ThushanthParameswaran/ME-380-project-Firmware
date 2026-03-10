#include <Arduino.h>

#define STEP_PIN 5
#define DIR_PIN  4


const int microstepsPerRev = 3200;  // 200 * 16* Gear ratio

long stepPosition = 0;  // Keeps track of current position



int pause = 500;

// direction = true (CW), false (CCW)
// delayMicros controls speed (lower = faster)
void moveSteps(long steps, bool direction, int delayMicros) {

  digitalWrite(DIR_PIN, direction);

  for (long i = 0; i < steps; i++) {

    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(delayMicros);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(delayMicros);

    // Update position counter
    if (direction)
      stepPosition++;
    else
      stepPosition--;
  }
}

void setup() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
 


  Serial.begin(115200);
}

void loop() {

  // Example: Rotate 1 revolution forward
  moveSteps(microstepsPerRev, true, pause);
  delay(1000);

  // Rotate 1 revolution backward
  moveSteps(microstepsPerRev, false, pause);
  delay(2000);

  Serial.print("Current Step Position: ");
  Serial.println(stepPosition);

}













// /* ------------------- PINS ------------------- */
// #define ENCODER_A 2
// #define ENCODER_B 3
// #define PWM_PIN   9
// #define IN1_PIN   8
// #define IN2_PIN   7
// #define SWITCH_PIN 5

// /* ------------------- ENCODER ------------------- */
// volatile long posi = 0;

// /* ------------------- PID ------------------- */
// float kp = 2.1;
// float kd = 0.19;
// float ki = 0.05;

// float eprev = 0;
// float eintegral = 0;
// long prevT = 0;

// /* ------------------- CONTROL ------------------- */
// int target = -60;    // CW target encoder position
// int deadband = 10;    // encoder counts
// int minPWM = 130;

// /* ------------------- ISR ------------------- */
// void readEncoderA() {
//   if (digitalRead(ENCODER_B)) posi++;
//   else posi--;
// }

// /* ------------------- SETUP ------------------- */
// void setup() {
//   Serial.begin(115200);

//   pinMode(ENCODER_A, INPUT_PULLUP);
//   pinMode(ENCODER_B, INPUT_PULLUP);
//   pinMode(SWITCH_PIN, INPUT_PULLUP);   // HIGH when pressed

//   attachInterrupt(digitalPinToInterrupt(ENCODER_A), readEncoderA, RISING);

//   pinMode(PWM_PIN, OUTPUT);
//   pinMode(IN1_PIN, OUTPUT);
//   pinMode(IN2_PIN, OUTPUT);

//   prevT = micros();

//   // CW
//   // digitalWrite(IN1_PIN, HIGH);
//   // digitalWrite(IN2_PIN, LOW);
//   // analogWrite(PWM_PIN, 130);
// }

// /* ------------------- LOOP ------------------- */
// void loop() {
//   if (Serial.available()) {
//     Serial.println(Serial.readStringUntil('\n'));
//   }

//   // Read encoder safely
//   int pos;
//   ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
//     pos = posi;
//   }

//   // Select target based on switch
//   int desired;
//   desired = target;


  
//   // Time step
//   long currT = micros();
//   float deltaT = (currT - prevT) * 1e-6;
//   prevT = currT;
//   if (deltaT <= 0) return;

//   // Position error
//   int e = desired - pos;

//   if (abs(e) <= deadband) {
//     analogWrite(PWM_PIN, 0);

//     eintegral = 0;
//     eprev = 0;
//   }

  

//   // PID
//   float dedt = (e - eprev) / deltaT;
//   eintegral += e * deltaT;
//   eintegral = constrain(eintegral, -300, 300);

//   float u = kp * e + kd * dedt + ki * eintegral;

//   // Position error
 

//   if (abs(e) <= deadband) {
//     analogWrite(PWM_PIN, 0);
//     digitalWrite(IN1_PIN, LOW);
//     digitalWrite(IN2_PIN, LOW);
 

//     eintegral = 0;
//     eprev = 0;
//     return;
//   }


//   // Direction from error sign
//   if (u > 0) {
//     // CW
//     digitalWrite(IN1_PIN, HIGH);
//     digitalWrite(IN2_PIN, LOW);
//   } else {
//     // CCW
//     digitalWrite(IN1_PIN, LOW);
//     digitalWrite(IN2_PIN, HIGH);
//   }

//   // Power with minimum PWM
//   float pwr = fabs(u);
//   pwr = constrain(pwr, minPWM, 200);
//   analogWrite(PWM_PIN, pwr);

//   eprev = e;

//   // long pos = 0;
//   // noInterrupts();
//   // pos = posi;
//   // interrupts();


//   // Debug
//   Serial.print("Desired: ");
//   Serial.print(desired);
//   Serial.print("  Pos: ");
//   Serial.println(pos);
//   delay(50);
 
// }

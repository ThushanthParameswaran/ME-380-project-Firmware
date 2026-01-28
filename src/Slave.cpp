#include <Arduino.h>
#include <util/atomic.h>

#define ENCODER_A 2
#define ENCODER_B 3
#define PWM_PIN   9
#define IN1_PIN   8
#define IN2_PIN   7

volatile int posi = 0;

long prevT = 0;
float eprev = 0;
float eintegral = 0;

int minPos = 0;
int maxPos = 120;
bool forward = true;  // global direction flag

void readEncoderA();


void setup() {
  Serial.begin(9600);

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  // Use RISING edge for both channels
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), readEncoderA, RISING);


  pinMode(PWM_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);

  prevT = micros();
}

// A rising edge → read B to determine direction
void readEncoderA() {
  if (digitalRead(ENCODER_B)) posi++;
  else posi--;
}


void loop() {


  float t = micros() * 1e-6;
  int target = 60 * sin(2 * PI * 0.2 * t);  // 0.2 Hz motion


  // PID constants

  float kp = .5;
  float kd = 0.5;
  float ki = 0.01;

  // time difference

  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );

  prevT = currT;

  if (deltaT <= 0) return;  

  int pos = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { pos = posi; }

  Serial.println(pos);

  // error
  int e = target - pos;


  // derivative
  float dedt = (e-eprev)/(deltaT);


  // integral
  eintegral = eintegral + e*deltaT;

  eintegral = constrain(eintegral, -300, 300);

  // control signal

  float u = kp*e + kd*dedt + ki*eintegral;





  // motor direction
if (u > 0) {
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
} else {
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, HIGH);
}

// motor power
  float pwr = constrain(fabs(u), 100, 255);
  analogWrite(PWM_PIN, pwr);



  eprev = e;

}

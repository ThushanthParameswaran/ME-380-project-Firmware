#include <Arduino.h>

// ============================================================
// PIN DEFINITIONS (ESP32-WROOM-32)
// ============================================================

// --- Motor 1 (Left) ---
#define M1_ENA  32  // PWM Speed Control
#define M1_IN1  33  // Direction Pin 1
#define M1_IN2  25  // Direction Pin 2
#define M1_ENC_A 26 // Yellow Wire (Phase A)
#define M1_ENC_B 27 // Green Wire (Phase B)

// --- Motor 2 (Right) ---
#define M2_ENB  14  // PWM Speed Control
#define M2_IN3  12  // Direction Pin 1
#define M2_IN4  13  // Direction Pin 2
#define M2_ENC_A 18 // Yellow Wire (Phase A)
#define M2_ENC_B 19 // Green Wire (Phase B)

// ============================================================
// PID CONSTANTS (TUNING REQUIRED)
// ============================================================
// Start with Kp, then add Ki slightly, then Kd to dampen.
float Kp = 1.2;  // Power proportional to error 0.5 - 5.0
float Ki = 0.05; // Power scales with *time* (fixes small gaps) 0.01 - 0.5
float Kd = 0.1;  // Power opposes *change* (dampens oscillation) 0.05 to 1.0

// Deadzone (Stops motor buzz when close)
int deadzone = 5; 

// PWM Properties
const int freq = 5000;
const int resolution = 8;
const int M1_PWM_CHANNEL = 0;
const int M2_PWM_CHANNEL = 1;

// ============================================================
// GLOBAL VARIABLES
// ============================================================
volatile long currentTicks1 = 0;
volatile long currentTicks2 = 0;
long targetTicks1 = 0;
long targetTicks2 = 0;

// PID Variables
long prevError1 = 0;
long prevError2 = 0;
float integral1 = 0;
float integral2 = 0;
unsigned long lastTime = 0;

// ============================================================
// INTERRUPT SERVICE ROUTINES
// ============================================================
void IRAM_ATTR readEncoder1() {
  if (digitalRead(M1_ENC_B) > 0) {
    currentTicks1++;
  } else {
    currentTicks1--;
  }
}

void IRAM_ATTR readEncoder2() {
  if (digitalRead(M2_ENC_B) > 0) {
    currentTicks2--; // Reversed for symmetry if needed
  } else {
    currentTicks2++;
  }
}

// ============================================================
// MOTOR DRIVER LOGIC
// ============================================================
void setMotor(int pwmChannel, int in1, int in2, int speed) {
  // Constrain speed to valid PWM range
  if (speed > 255) speed = 255;
  if (speed < -255) speed = -255;

  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    ledcWrite(pwmChannel, speed);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    ledcWrite(pwmChannel, abs(speed));
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    ledcWrite(pwmChannel, 0);
  }
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);

  // Motor 1
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M1_ENC_A, INPUT_PULLUP);
  pinMode(M1_ENC_B, INPUT_PULLUP);
  ledcSetup(M1_PWM_CHANNEL, freq, resolution);
  ledcAttachPin(M1_ENA, M1_PWM_CHANNEL);

  // Motor 2
  pinMode(M2_IN3, OUTPUT);
  pinMode(M2_IN4, OUTPUT);
  pinMode(M2_ENC_A, INPUT_PULLUP);
  pinMode(M2_ENC_B, INPUT_PULLUP);
  ledcSetup(M2_PWM_CHANNEL, freq, resolution);
  ledcAttachPin(M2_ENB, M2_PWM_CHANNEL);

  // Interrupts
  attachInterrupt(digitalPinToInterrupt(M1_ENC_A), readEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(M2_ENC_A), readEncoder2, RISING);

  Serial.println("ESP32 PID Ready.");
}

// ============================================================
// MAIN LOOP
// ============================================================
void loop() {
  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0;
  lastTime = now;

  // --- NEW ROBUST SERIAL PARSING ---
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n'); // Read the whole line
    if (data.startsWith("<") && data.endsWith(">")) {
      // Remove the brackets and parse
      data = data.substring(1, data.length() - 1);
      int commaIndex = data.indexOf(',');
      if (commaIndex != -1) {
        targetTicks1 = data.substring(0, commaIndex).toInt();
        targetTicks2 = data.substring(commaIndex + 1).toInt();
        
        integral1 = 0;
        integral2 = 0;
        
        // SEND IMMEDIATE FEEDBACK
        Serial.print("Target Recv: ");
        Serial.print(targetTicks1);
        Serial.print(",");
        Serial.println(targetTicks2);
      }
    }
  }

  // 3. Compute Errors
  long error1 = targetTicks1 - currentTicks1;
  long error2 = targetTicks2 - currentTicks2;

  // 4. PID Calculations (Motor 1)
  integral1 += error1 * dt;
  float derivative1 = (error1 - prevError1) / dt;
  float output1 = (Kp * error1) + (Ki * integral1) + (Kd * derivative1);
  prevError1 = error1;

  // 5. PID Calculations (Motor 2)
  integral2 += error2 * dt;
  float derivative2 = (error2 - prevError2) / dt;
  float output2 = (Kp * error2) + (Ki * integral2) + (Kd * derivative2);
  prevError2 = error2;

  // 6. Anti-Windup (Integral Clamping)
  // Prevents the "I" term from growing huge if the motor stalls
  if (integral1 > 1000) integral1 = 1000;
  if (integral1 < -1000) integral1 = -1000;
  if (integral2 > 1000) integral2 = 1000;
  if (integral2 < -1000) integral2 = -1000;

  // 7. Drive Motors
  // If close to target, cut power to prevent jitter
  if (abs(error1) < deadzone) {
    setMotor(M1_PWM_CHANNEL, M1_IN1, M1_IN2, 0);
    integral1 = 0; // Clear built-up pressure
  } else {
    setMotor(M1_PWM_CHANNEL, M1_IN1, M1_IN2, output1);
  }

  if (abs(error2) < deadzone) {
    setMotor(M2_PWM_CHANNEL, M2_IN3, M2_IN4, 0);
    integral2 = 0;
  } else {
    setMotor(M2_PWM_CHANNEL, M2_IN3, M2_IN4, output2);
  }
  
  // Optional: Debugging (Comment out for production)
  // Serial.printf("E1:%ld  Out1:%.2f\n", error1, output1);
}
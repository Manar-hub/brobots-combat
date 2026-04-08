#include <Servo.h>

// Receiver pins
const int ch1Pin = 2;  // Steering  → INT0 (hardware interrupt)
const int ch2Pin = 3;  // Throttle  → INT1 (hardware interrupt)
const int ch3Pin = 4;  // Inversion toggle → pin-change interrupt

// Motor driver pins
const int L_RPWM = 5;
const int L_LPWM = 6;
const int R_RPWM = 9;
const int R_LPWM = 10;

// Weapon ESC
const int weaponPin = 11;
Servo weaponESC;

// PARAMETERS
int deadzone = 20;
int center   = 1500;

// ESC values
int escStop  = 1000;
int escPower = 1600;

// MOTOR DIRECTION FLAGS
bool invertLeftMotor  = false;
bool invertRightMotor = false;

// Inversion
bool inverted        = false;
bool lastToggleState = false;

// Smooth acceleration
float smoothFactor     = 0.1; // lower = smoother
float leftMotorSmooth  = 0;
float rightMotorSmooth = 0;

// SOFT-START
float weaponSmooth     = 1000.0;
float weaponSmoothRate = 0.02;   // lower = slower ramp 

// FAILSAFE 
const unsigned long failsafeTimeout = 100000UL; // 100ms

// INTERRUPT-DRIVEN RC READING
volatile unsigned long ch1Start    = 0, ch2Start    = 0, ch3Start    = 0;
volatile int           ch1Val      = 1500, ch2Val   = 1500, ch3Val   = 1000;
volatile unsigned long ch1LastTime = 0, ch2LastTime = 0, ch3LastTime = 0;
volatile byte          lastPortD   = 0;

// CH1 — Steering (hardware INT0, pin 2)
void ch1ISR() {
  if (digitalRead(ch1Pin) == HIGH) {
    ch1Start = micros();
  } else {
    unsigned long w = micros() - ch1Start;
    if (w >= 800 && w <= 2200) {
      ch1Val      = (int)w;
      ch1LastTime = micros();
    }
  }
}

// CH2 — Throttle (hardware INT1, pin 3)
void ch2ISR() {
  if (digitalRead(ch2Pin) == HIGH) {
    ch2Start = micros();
  } else {
    unsigned long w = micros() - ch2Start;
    if (w >= 800 && w <= 2200) {
      ch2Val      = (int)w;
      ch2LastTime = micros();
    }
  }
}

// CH3 — Inversion toggle (pin 4 / PD4) via pin-change interrupt
ISR(PCINT2_vect) {
  unsigned long now     = micros();
  byte          current = PIND;
  byte          changed = current ^ lastPortD;
  lastPortD             = current;

  if (changed & (1 << PD4)) {
    if (current & (1 << PD4)) {
      ch3Start = now;
    } else {
      unsigned long w = now - ch3Start;
      if (w >= 800 && w <= 2200) {
        ch3Val      = (int)w;
        ch3LastTime = now;
      }
    }
  }
}


bool signalValid() {
  unsigned long now = micros();
  return (now - ch1LastTime < failsafeTimeout) &&
         (now - ch2LastTime < failsafeTimeout);
}

void setMotor(int rpwm, int lpwm, int speed, bool invert) {
  if (invert) speed = -speed;
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    analogWrite(rpwm, speed);
    analogWrite(lpwm, 0);
  } else if (speed < 0) {
    analogWrite(rpwm, 0);
    analogWrite(lpwm, -speed);
  } else {
    analogWrite(rpwm, 0);
    analogWrite(lpwm, 0);
  }
}

void stopAll() {
  setMotor(L_RPWM, L_LPWM, 0, invertLeftMotor);
  setMotor(R_RPWM, R_LPWM, 0, invertRightMotor);
  weaponESC.writeMicroseconds(escStop);
  leftMotorSmooth  = 0;
  rightMotorSmooth = 0;
}

void setup() {
  pinMode(ch1Pin, INPUT);
  pinMode(ch2Pin, INPUT);
  pinMode(ch3Pin, INPUT);

  pinMode(L_RPWM, OUTPUT);
  pinMode(L_LPWM, OUTPUT);
  pinMode(R_RPWM, OUTPUT);
  pinMode(R_LPWM, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ch1Pin), ch1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ch2Pin), ch2ISR, CHANGE);

  lastPortD  = PIND;
  PCICR     |= (1 << PCIE2);
  PCMSK2    |= (1 << PCINT20); // pin 4

  weaponESC.attach(weaponPin);
  weaponESC.writeMicroseconds(escStop);
  delay(3000);
}

void loop() {
  noInterrupts();
  int steering = ch1Val;
  int throttle = ch2Val;
  int toggle   = ch3Val;
  interrupts();

  if (!signalValid()) {
    stopAll();
    return;
  }

  if (abs(steering - center) < deadzone) steering = center;
  if (abs(throttle - center) < deadzone) throttle = center;

  bool currentToggleState = (toggle > 1500);
  if (currentToggleState && !lastToggleState) inverted = !inverted;
  lastToggleState = currentToggleState;

  int steerVal    = map(steering, 1000, 2000, -255, 255);
  int throttleVal = map(throttle, 1000, 2000, -255, 255);

  if (inverted) {
    throttleVal = -throttleVal;
    steerVal    = -steerVal;
  }

  int leftMotorTarget  = constrain(throttleVal + steerVal, -255, 255);
  int rightMotorTarget = constrain(throttleVal - steerVal, -255, 255);

  leftMotorSmooth  += (leftMotorTarget  - leftMotorSmooth)  * smoothFactor;
  rightMotorSmooth += (rightMotorTarget - rightMotorSmooth) * smoothFactor;

  setMotor(L_RPWM, L_LPWM, (int)leftMotorSmooth, invertLeftMotor);
  setMotor(R_RPWM, R_LPWM, (int)rightMotorSmooth, invertRightMotor);

  weaponSmooth += (escPower - weaponSmooth) * weaponSmoothRate;
  weaponESC.writeMicroseconds((int)weaponSmooth);
}

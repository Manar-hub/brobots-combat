#include <Servo.h>

// Receiver pins
const int ch1Pin = 2; // Steering
const int ch2Pin = 3; // Throttle
const int ch3Pin = 4; // Inversion toggle

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
int center = 1500;

// ESC values
int escStop = 1000;
int escPower = 1600; 

// Inversion
bool inverted = false;
bool lastToggleState = false;

// Smooth acceleration
float smoothFactor = 0.1; // lower = smoother
float leftMotorSmooth = 0;
float rightMotorSmooth = 0;

void setup() {
  pinMode(ch1Pin, INPUT);
  pinMode(ch2Pin, INPUT);
  pinMode(ch3Pin, INPUT);

  pinMode(L_RPWM, OUTPUT);
  pinMode(L_LPWM, OUTPUT);
  pinMode(R_RPWM, OUTPUT);
  pinMode(R_LPWM, OUTPUT);

  weaponESC.attach(weaponPin);

  weaponESC.writeMicroseconds(escStop);
  delay(3000); // wait for ESC to arm
}

void loop() {
  int steering = pulseIn(ch1Pin, HIGH, 25000);
  int throttle = pulseIn(ch2Pin, HIGH, 25000);
  int toggle   = pulseIn(ch3Pin, HIGH, 25000);

  // DEADZONE
  if (abs(steering - center) < deadzone) steering = center;
  if (abs(throttle - center) < deadzone) throttle = center;

  // NORMALIZE
  int steerVal = map(steering, 1000, 2000, -255, 255);
  int throttleVal = map(throttle, 1000, 2000, -255, 255);

  // INVERSION TOGGLE
  bool currentToggleState = (toggle > 1500);

  if (currentToggleState && !lastToggleState) {
    inverted = !inverted;
  }
  lastToggleState = currentToggleState;

  if (inverted) {
    throttleVal = -throttleVal;
    steerVal = -steerVal;
  }

  // DIFFERENTIAL DRIVE
  int leftMotorTarget  = throttleVal + steerVal;
  int rightMotorTarget = throttleVal - steerVal;

  leftMotorTarget = constrain(leftMotorTarget, -255, 255);
  rightMotorTarget = constrain(rightMotorTarget, -255, 255);

  // SMOOTH ACCELERATION
  leftMotorSmooth  += (leftMotorTarget - leftMotorSmooth) * smoothFactor;
  rightMotorSmooth += (rightMotorTarget - rightMotorSmooth) * smoothFactor;

  setMotor(L_RPWM, L_LPWM, (int)leftMotorSmooth);
  setMotor(R_RPWM, R_LPWM, (int)rightMotorSmooth);

  weaponESC.writeMicroseconds(escPower);
}

void setMotor(int rpwm, int lpwm, int speed) {
  if (speed > 0) {
    analogWrite(rpwm, speed);
    analogWrite(lpwm, 0);
  } else {
    analogWrite(rpwm, 0);
    analogWrite(lpwm, -speed);
  }
}
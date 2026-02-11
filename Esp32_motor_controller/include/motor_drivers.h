#ifndef MOTOR_DRIVERS_H
#define MOTOR_DRIVERS_H

#include <Arduino.h>
#include "commands.h"

// =====================
// Shared enable pins
// =====================
#define BTS_R_EN 32
#define BTS_L_EN 33

// =====================
// Motor pin mapping (BTS7960)
// =====================
// Motor #1 (Front Left)
#define FL_RPWM 25
#define FL_LPWM 26

// Motor #2 (Front Right)
#define FR_RPWM 27
#define FR_LPWM 14

// Motor #3 (Rear Left)
#define RL_RPWM 13
#define RL_LPWM 23

// Motor #4 (Rear Right)
#define RR_RPWM 21
#define RR_LPWM 22

// =====================
// PWM settings
// =====================
const int PWM_FREQ_HZ = 20000;
const int PWM_BITS    = 10;       // 0..1023
const int PWM_MAX     = (1 << PWM_BITS) - 1;

// 2 channels per motor (explicit)
const int CH_FL_R = 0;
const int CH_FL_L = 1;
const int CH_FR_R = 2;
const int CH_FR_L = 3;
const int CH_RL_R = 4;
const int CH_RL_L = 5;
const int CH_RR_R = 6;
const int CH_RR_L = 7;

static inline int pwmDutyFrom255(int spdAbs255) {
  if (spdAbs255 < 0) spdAbs255 = -spdAbs255;
  if (spdAbs255 > 255) spdAbs255 = 255;
  return (spdAbs255 * PWM_MAX) / 255;
}

void robot_setup() {
  pinMode(BTS_R_EN, OUTPUT);
  pinMode(BTS_L_EN, OUTPUT);
  digitalWrite(BTS_R_EN, HIGH);
  digitalWrite(BTS_L_EN, HIGH);

  // Setup PWM channels + attach pins
  ledcSetup(CH_FL_R, PWM_FREQ_HZ, PWM_BITS);
  ledcSetup(CH_FL_L, PWM_FREQ_HZ, PWM_BITS);
  ledcAttachPin(FL_RPWM, CH_FL_R);
  ledcAttachPin(FL_LPWM, CH_FL_L);

  ledcSetup(CH_FR_R, PWM_FREQ_HZ, PWM_BITS);
  ledcSetup(CH_FR_L, PWM_FREQ_HZ, PWM_BITS);
  ledcAttachPin(FR_RPWM, CH_FR_R);
  ledcAttachPin(FR_LPWM, CH_FR_L);

  ledcSetup(CH_RL_R, PWM_FREQ_HZ, PWM_BITS);
  ledcSetup(CH_RL_L, PWM_FREQ_HZ, PWM_BITS);
  ledcAttachPin(RL_RPWM, CH_RL_R);
  ledcAttachPin(RL_LPWM, CH_RL_L);

  ledcSetup(CH_RR_R, PWM_FREQ_HZ, PWM_BITS);
  ledcSetup(CH_RR_L, PWM_FREQ_HZ, PWM_BITS);
  ledcAttachPin(RR_RPWM, CH_RR_R);
  ledcAttachPin(RR_LPWM, CH_RR_L);

  // Stop everything
  ledcWrite(CH_FL_R, 0); ledcWrite(CH_FL_L, 0);
  ledcWrite(CH_FR_R, 0); ledcWrite(CH_FR_L, 0);
  ledcWrite(CH_RL_R, 0); ledcWrite(CH_RL_L, 0);
  ledcWrite(CH_RR_R, 0); ledcWrite(CH_RR_L, 0);
}

// spd is -255..+255
void setWheelPWM(int wheel, int spd) {
  int duty = pwmDutyFrom255(spd);

  if (wheel == FL) {
    if (spd > 0) { ledcWrite(CH_FL_L, 0);   ledcWrite(CH_FL_R, duty); }
    else if (spd < 0) { ledcWrite(CH_FL_R, 0); ledcWrite(CH_FL_L, duty); }
    else { ledcWrite(CH_FL_R, 0); ledcWrite(CH_FL_L, 0); }
  }
  else if (wheel == FR) {
    if (spd > 0) { ledcWrite(CH_FR_L, 0);   ledcWrite(CH_FR_R, duty); }
    else if (spd < 0) { ledcWrite(CH_FR_R, 0); ledcWrite(CH_FR_L, duty); }
    else { ledcWrite(CH_FR_R, 0); ledcWrite(CH_FR_L, 0); }
  }
  else if (wheel == RL) {
    if (spd > 0) { ledcWrite(CH_RL_L, 0);   ledcWrite(CH_RL_R, duty); }
    else if (spd < 0) { ledcWrite(CH_RL_R, 0); ledcWrite(CH_RL_L, duty); }
    else { ledcWrite(CH_RL_R, 0); ledcWrite(CH_RL_L, 0); }
  }
  else { // RR
    if (spd > 0) { ledcWrite(CH_RR_L, 0);   ledcWrite(CH_RR_R, duty); }
    else if (spd < 0) { ledcWrite(CH_RR_R, 0); ledcWrite(CH_RR_L, duty); }
    else { ledcWrite(CH_RR_R, 0); ledcWrite(CH_RR_L, 0); }
  }
}

// Left applies to FL+RL, Right applies to FR+RR
void setSidePWMs(int leftPWM, int rightPWM) {
  setWheelPWM(FL, leftPWM);
  setWheelPWM(RL, leftPWM);
  setWheelPWM(FR, rightPWM);
  setWheelPWM(RR, rightPWM);
}

void stopAllMotors() {
  setSidePWMs(0, 0);
}

#endif

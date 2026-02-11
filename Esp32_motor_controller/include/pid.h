#ifndef PID_H
#define PID_H

#include <Arduino.h>
#include "commands.h"
#include "encoders.h"
#include "motor_drivers.h"

// =====================
// PID settings
// =====================
#define PID_RATE_HZ 50
const int PID_INTERVAL_MS = 1000 / PID_RATE_HZ;

// =====================
// PID parameters (globals)
// =====================
int Kp = 20;
int Kd = 12;
int Ki = 0;
int Ko = 50;

#define MAX_PWM 255

unsigned char moving = 0;
unsigned long nextPID = PID_INTERVAL_MS;

// =====================
// PID struct (kept like original)
// =====================
typedef struct {
  double TargetTicksPerFrame;
  long Encoder;
  long PrevEnc;
  int PrevInput;
  int ITerm;
  long output;
} SetPointInfo;

SetPointInfo leftPID, rightPID;

// Avg encoders for each side
static inline long leftEncAvg() {
  return (readEncoder(FL) + readEncoder(RL)) / 2;
}
static inline long rightEncAvg() {
  return (readEncoder(FR) + readEncoder(RR)) / 2;
}

void resetPID() {
  leftPID.TargetTicksPerFrame = 0.0;
  leftPID.Encoder = leftEncAvg();
  leftPID.PrevEnc = leftPID.Encoder;
  leftPID.output = 0;
  leftPID.PrevInput = 0;
  leftPID.ITerm = 0;

  rightPID.TargetTicksPerFrame = 0.0;
  rightPID.Encoder = rightEncAvg();
  rightPID.PrevEnc = rightPID.Encoder;
  rightPID.output = 0;
  rightPID.PrevInput = 0;
  rightPID.ITerm = 0;
}

void doPID(SetPointInfo *p) {
  long Perror;
  long output;
  int input;

  input = (int)(p->Encoder - p->PrevEnc);
  Perror = (long)(p->TargetTicksPerFrame - input);

  output = ((long)Kp * Perror
          - (long)Kd * (input - p->PrevInput)
          + (long)p->ITerm) / (long)Ko;

  p->PrevEnc = p->Encoder;

  output += p->output;

  if (output >= MAX_PWM) output = MAX_PWM;
  else if (output <= -MAX_PWM) output = -MAX_PWM;
  else p->ITerm += Ki * (int)Perror;

  p->output = output;
  p->PrevInput = input;
}

void updatePID() {
  leftPID.Encoder = leftEncAvg();
  rightPID.Encoder = rightEncAvg();

  if (!moving) {
    if (leftPID.PrevInput != 0 || rightPID.PrevInput != 0) resetPID();
    return;
  }

  doPID(&rightPID);
  doPID(&leftPID);

  setSidePWMs((int)leftPID.output, (int)rightPID.output);
}

#endif

#ifndef ENCODERS_H
#define ENCODERS_H

#include <Arduino.h>
#include "commands.h"

// =====================
// Encoder pin mapping
// =====================
// Motor #1 (Front Left)
#define ENC_FL_A 34
#define ENC_FL_B 35

// Motor #2 (Front Right)
#define ENC_FR_A 36
#define ENC_FR_B 39

// Motor #3 (Rear Left)
#define ENC_RL_A 18
#define ENC_RL_B 19  // (you had a typo GPIP19; this is GPIO19)

// Motor #4 (Rear Right)
#define ENC_RR_A 16
#define ENC_RR_B 17

// =====================
// Encoder calibration
// =====================
const float TICKS_PER_REV = 1975.0f;

// =====================
// Encoder state (globals)
// =====================
extern volatile long encFL;
extern volatile long encFR;
extern volatile long encRL;
extern volatile long encRR;

// =====================
// Functions
// =====================
void init_encoders();

long readEncoder(int wheel);
void resetEncoder(int wheel);
void resetEncoders();

#endif

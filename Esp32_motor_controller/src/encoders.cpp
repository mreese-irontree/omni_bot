#include "encoders.h"

// -------- encoder counters (globals) --------
volatile long encFL = 0;
volatile long encFR = 0;
volatile long encRL = 0;
volatile long encRR = 0;

// =======================================================
// Quadrature ISR style (kept explicit like your repo code)
// =======================================================

// ---- Front Left (FL) ----
void IRAM_ATTR doEncFL_A() {
  if (digitalRead(ENC_FL_A) == HIGH) {
    if (digitalRead(ENC_FL_B) == LOW) encFL++;
    else encFL--;
  } else {
    if (digitalRead(ENC_FL_B) == HIGH) encFL++;
    else encFL--;
  }
}
void IRAM_ATTR doEncFL_B() {
  if (digitalRead(ENC_FL_B) == HIGH) {
    if (digitalRead(ENC_FL_A) == HIGH) encFL++;
    else encFL--;
  } else {
    if (digitalRead(ENC_FL_A) == LOW) encFL++;
    else encFL--;
  }
}

// ---- Front Right (FR) ----
// NOTE: If FR direction counts opposite of others, swap ++/-- here.
// For now we keep same convention as FL.
void IRAM_ATTR doEncFR_A() {
  if (digitalRead(ENC_FR_A) == HIGH) {
    if (digitalRead(ENC_FR_B) == LOW) encFR++;
    else encFR--;
  } else {
    if (digitalRead(ENC_FR_B) == HIGH) encFR++;
    else encFR--;
  }
}
void IRAM_ATTR doEncFR_B() {
  if (digitalRead(ENC_FR_B) == HIGH) {
    if (digitalRead(ENC_FR_A) == HIGH) encFR++;
    else encFR--;
  } else {
    if (digitalRead(ENC_FR_A) == LOW) encFR++;
    else encFR--;
  }
}

// ---- Rear Left (RL) ----
void IRAM_ATTR doEncRL_A() {
  if (digitalRead(ENC_RL_A) == HIGH) {
    if (digitalRead(ENC_RL_B) == LOW) encRL++;
    else encRL--;
  } else {
    if (digitalRead(ENC_RL_B) == HIGH) encRL++;
    else encRL--;
  }
}
void IRAM_ATTR doEncRL_B() {
  if (digitalRead(ENC_RL_B) == HIGH) {
    if (digitalRead(ENC_RL_A) == HIGH) encRL++;
    else encRL--;
  } else {
    if (digitalRead(ENC_RL_A) == LOW) encRL++;
    else encRL--;
  }
}

// ---- Rear Right (RR) ----
void IRAM_ATTR doEncRR_A() {
  if (digitalRead(ENC_RR_A) == HIGH) {
    if (digitalRead(ENC_RR_B) == LOW) encRR++;
    else encRR--;
  } else {
    if (digitalRead(ENC_RR_B) == HIGH) encRR++;
    else encRR--;
  }
}
void IRAM_ATTR doEncRR_B() {
  if (digitalRead(ENC_RR_B) == HIGH) {
    if (digitalRead(ENC_RR_A) == HIGH) encRR++;
    else encRR--;
  } else {
    if (digitalRead(ENC_RR_A) == LOW) encRR++;
    else encRR--;
  }
}

void init_encoders() {
  pinMode(ENC_FL_A, INPUT_PULLUP);
  pinMode(ENC_FL_B, INPUT_PULLUP);

  pinMode(ENC_FR_A, INPUT_PULLUP);
  pinMode(ENC_FR_B, INPUT_PULLUP);

  pinMode(ENC_RL_A, INPUT_PULLUP);
  pinMode(ENC_RL_B, INPUT_PULLUP);

  pinMode(ENC_RR_A, INPUT_PULLUP);
  pinMode(ENC_RR_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_FL_A), doEncFL_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_FL_B), doEncFL_B, CHANGE);

  attachInterrupt(digitalPinToInterrupt(ENC_FR_A), doEncFR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_FR_B), doEncFR_B, CHANGE);

  attachInterrupt(digitalPinToInterrupt(ENC_RL_A), doEncRL_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RL_B), doEncRL_B, CHANGE);

  attachInterrupt(digitalPinToInterrupt(ENC_RR_A), doEncRR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RR_B), doEncRR_B, CHANGE);
}

long readEncoder(int wheel) {
  switch (wheel) {
    case FL: return encFL;
    case FR: return encFR;
    case RL: return encRL;
    case RR: return encRR;
    default: return 0;
  }
}

void resetEncoder(int wheel) {
  noInterrupts();
  switch (wheel) {
    case FL: encFL = 0; break;
    case FR: encFR = 0; break;
    case RL: encRL = 0; break;
    case RR: encRR = 0; break;
  }
  interrupts();
}

void resetEncoders() {
  resetEncoder(FL);
  resetEncoder(FR);
  resetEncoder(RL);
  resetEncoder(RR);
}

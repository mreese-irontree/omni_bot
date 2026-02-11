#include <Arduino.h>
#include "motor_drivers.h"
#include "commands.h"
#include "pid.h"
#include "encoders.h"

// =====================
// Auto-stop watchdog
// =====================
#define AUTO_STOP_INTERVAL_MS 2000
long lastMotorCommand = AUTO_STOP_INTERVAL_MS;

// =====================
// Command parser (kept like original)
// =====================
int arg = 0;
int idx = 0;

char chr;
char cmd;

long arg1;
long arg2;

char argv1[24];
char argv2[24];

void resetCommand() {
  cmd = 0;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  idx = 0;
}

int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];

  arg1 = atol(argv1);
  arg2 = atol(argv2);

  switch (cmd) {
    case READ_ENCODERS:
      Serial.print(readEncoder(FL)); Serial.print(" ");
      Serial.print(readEncoder(FR)); Serial.print(" ");
      Serial.print(readEncoder(RL)); Serial.print(" ");
      Serial.println(readEncoder(RR));
      break;

    case RESET_ENCODERS:
      resetEncoders();
      resetPID();
      Serial.println("OK");
      break;

    case MOTOR_SPEEDS:
      lastMotorCommand = millis();

      if (arg1 == 0 && arg2 == 0) {
        setSidePWMs(0, 0);
        resetPID();
        moving = 0;
      } else {
        moving = 1;
      }

      leftPID.TargetTicksPerFrame  = arg1;
      rightPID.TargetTicksPerFrame = arg2;
      Serial.println("OK");
      break;

    case MOTOR_RAW_PWM:
      lastMotorCommand = millis();
      resetPID();
      moving = 0; // disable PID
      setSidePWMs((int)arg1, (int)arg2);
      Serial.println("OK");
      break;

    case UPDATE_PID:
      while ((str = strtok_r(p, ":", &p)) != NULL) {
        pid_args[i] = atoi(str);
        i++;
        if (i >= 4) break;
      }
      if (i >= 1) Kp = pid_args[0];
      if (i >= 2) Kd = pid_args[1];
      if (i >= 3) Ki = pid_args[2];
      if (i >= 4) Ko = pid_args[3];
      Serial.println("OK");
      break;

    default:
      Serial.println("Invalid Command");
      break;
  }

  return 1;
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("begun");

  robot_setup();
  resetPID();
  init_encoders();

  resetCommand();
  lastMotorCommand = millis();

  Serial.println("ESP32 4WD skid ready");
  Serial.println("Commands:");
  Serial.println("  e            -> read encoders (FL FR RL RR)");
  Serial.println("  r            -> reset encoders + PID");
  Serial.println("  o L R        -> raw PWM (-255..255) left/right");
  Serial.println("  m L R        -> target ticks/frame left/right (PID)");
  Serial.println("  u Kp:Kd:Ki:Ko-> set PID gains");
}

void loop() {
  while (Serial.available() > 0) {
    chr = Serial.read();

    // Accept either CR or LF as command terminator
    if (chr == '\n') chr = 13;

    if (chr == 13) {
      if (arg == 1) argv1[idx] = 0;
      else if (arg == 2) argv2[idx] = 0;

      runCommand();
      resetCommand();
    }
    else if (chr == ' ') {
      if (arg == 0) arg = 1;
      else if (arg == 1) {
        argv1[idx] = 0;
        arg = 2;
        idx = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        cmd = chr;
      }
      else if (arg == 1) {
        if (idx < (int)sizeof(argv1) - 1) argv1[idx++] = chr;
      }
      else if (arg == 2) {
        if (idx < (int)sizeof(argv2) - 1) argv2[idx++] = chr;
      }
    }
  }

  // PID update at fixed interval
  if (millis() > nextPID) {
    updatePID();
    nextPID += PID_INTERVAL_MS;
  }

  // Auto-stop if no command recently
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL_MS) {
    stopAllMotors();
    moving = 0;
  }
}

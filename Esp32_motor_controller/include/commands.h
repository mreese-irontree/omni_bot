#ifndef COMMANDS_H
#define COMMANDS_H

// ---- Serial command letters ----
#define READ_ENCODERS   'e'
#define RESET_ENCODERS  'r'
#define MOTOR_SPEEDS    'm'   // PID mode: target ticks/frame for LEFT and RIGHT
#define MOTOR_RAW_PWM   'o'   // Raw PWM mode: -255..255 for LEFT and RIGHT
#define UPDATE_PID      'u'   // "u Kp:Kd:Ki:Ko"

// ---- Wheel IDs ----
#define FL 0
#define FR 1
#define RL 2
#define RR 3

// ---- Sides ----
#define LEFT_SIDE  0
#define RIGHT_SIDE 1

#endif

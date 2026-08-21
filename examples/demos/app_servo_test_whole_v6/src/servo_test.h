#pragma once

#include <stdint.h>

#define SERVO_CMD_IDLE  0
#define SERVO_CMD_CCW   1
#define SERVO_CMD_CW    2

void servoControlInit(void);
void servoControlMoveCw(void);
void servoControlMoveCcw(void);
void servoControlStop(void);
uint8_t servoControlGetLastCommand(void);

#pragma once

#include <stdbool.h>
#include <stdint.h>

#define MAX_TRAJECTORY_POINTS 100
#define CRTP_PORT_TRAJECTORY  0x0F

typedef struct __attribute__((packed)) {
  float x;
  float y;
} TrajectoryPoint;

extern TrajectoryPoint trajectory_buffer[MAX_TRAJECTORY_POINTS];
extern int trajectory_length;
extern bool trajectory_received;

void trajectoryInit(void);
void clearTrajectory(void);

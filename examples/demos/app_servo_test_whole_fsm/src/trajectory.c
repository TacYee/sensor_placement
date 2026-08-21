// trajectory.c - CRTP trajectory receiver, compatible with the old working protocol.
// Host sends one point per CRTP packet on port 0x0F: struct '<ff' = x, y.
// Host sends NaN, NaN as the end marker.

#include "trajectory.h"

#include <string.h>
#include <stdbool.h>
#include <math.h>

#include "crtp.h"
#include "log.h"
#include "debug.h"

#define DEBUG_MODULE "TRAJ_RX"

TrajectoryPoint trajectory_buffer[MAX_TRAJECTORY_POINTS];
int trajectory_length = 0;
bool trajectory_received = false;

static void crtpTrajectoryHandler(CRTPPacket *pk)
{
  if (pk->size != sizeof(TrajectoryPoint)) {
    return;
  }

  TrajectoryPoint pt;
  memcpy(&pt, pk->data, sizeof(TrajectoryPoint));

  if (isnan(pt.x) && isnan(pt.y)) {
    trajectory_received = true;
    DEBUG_PRINT("Trajectory end received, len=%d\n", trajectory_length);
    return;
  }

  if (trajectory_length >= MAX_TRAJECTORY_POINTS) {
    DEBUG_PRINT("Trajectory buffer full\n");
    return;
  }

  trajectory_buffer[trajectory_length++] = pt;
}

void trajectoryInit(void)
{
  crtpRegisterPortCB(CRTP_PORT_TRAJECTORY, crtpTrajectoryHandler);
  DEBUG_PRINT("CRTP trajectory receiver registered on port 0x%02x\n", CRTP_PORT_TRAJECTORY);
}

void clearTrajectory(void)
{
  trajectory_length = 0;
  trajectory_received = false;
}

LOG_GROUP_START(traj)
LOG_ADD(LOG_FLOAT, traj_x, &trajectory_buffer[0].x)
LOG_ADD(LOG_FLOAT, traj_y, &trajectory_buffer[0].y)
LOG_GROUP_STOP(traj)

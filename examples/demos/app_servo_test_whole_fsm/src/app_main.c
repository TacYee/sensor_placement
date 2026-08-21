/**
 * app_main.c - Thin app layer for CRTP trajectory mission FSM.
 *
 * Responsibilities here are intentionally small:
 *   1. Read stateEstimate.x/y/z/yaw through the log framework.
 *   2. Update the independent mission FSM.
 *   3. Convert the FSM command into Crazyflie commander setpoints.
 */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"
#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"
#include "commander.h"
#include "log.h"

#include "mission_fsm.h"
#include "servo_test.h"

#define DEBUG_MODULE "APP"
#define APP_LOOP_PERIOD_MS     50U
#define COMMANDER_APP_PRIORITY 3

static setpoint_t setpoint;

static void send_body_velocity_height_setpoint(float vx_body, float vy_body, float height, float yaw_rate_deg_s)
{
  memset(&setpoint, 0, sizeof(setpoint_t));

  setpoint.mode.x = modeVelocity;
  setpoint.mode.y = modeVelocity;
  setpoint.mode.z = modeAbs;
  setpoint.mode.yaw = modeVelocity;

  setpoint.velocity.x = vx_body;
  setpoint.velocity.y = vy_body;
  setpoint.position.z = height;
  setpoint.attitudeRate.yaw = yaw_rate_deg_s;

  // Same convention as the old working setHoverSetpoint(): vx/vy are body-frame.
  setpoint.velocity_body = true;

  commanderSetSetpoint(&setpoint, COMMANDER_APP_PRIORITY);
}

static void send_position_setpoint(float x, float y, float z, float yaw_deg)
{
  memset(&setpoint, 0, sizeof(setpoint_t));

  setpoint.mode.x = modeAbs;
  setpoint.mode.y = modeAbs;
  setpoint.mode.z = modeAbs;
  setpoint.mode.yaw = modeAbs;

  setpoint.position.x = x;
  setpoint.position.y = y;
  setpoint.position.z = z;
  setpoint.attitude.yaw = yaw_deg;

  commanderSetSetpoint(&setpoint, COMMANDER_APP_PRIORITY);
}

static void send_stop_setpoint(void)
{
  memset(&setpoint, 0, sizeof(setpoint_t));
  commanderSetSetpoint(&setpoint, COMMANDER_APP_PRIORITY);
}

void appMain(void)
{
  DEBUG_PRINT("Independent mission FSM app starting\n");

  vTaskDelay(M2T(3000));

  logVarId_t idStateX = logGetVarId("stateEstimate", "x");
  logVarId_t idStateY = logGetVarId("stateEstimate", "y");
  logVarId_t idStateZ = logGetVarId("stateEstimate", "z");
  logVarId_t idStateYaw = logGetVarId("stateEstimate", "yaw");
  logVarId_t idNoDynPX = logGetVarId("kalman", "statePX_noDyn");
  logVarId_t idNoDynPY = logGetVarId("kalman", "statePY_noDyn");

  servoControlInit();
  missionFsmInit();

  DEBUG_PRINT("Send CRTP trajectory points to port 0x0F, end with NaN/NaN, then set traj.start=1\n");
  DEBUG_PRINT("Manual test: set traj.forceCollision=1 to force CONTACT from HOVERING/FOLLOWING/FORWARD\n");

  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1) {
    const float state_x = logGetFloat(idStateX);
    const float state_y = logGetFloat(idStateY);
    const float state_z = logGetFloat(idStateZ);
    const float state_yaw_deg = logGetFloat(idStateYaw);
    const float no_dyn_vx = logGetFloat(idNoDynPX);
    const float no_dyn_vy = logGetFloat(idNoDynPY);

    missionFsmUpdate(state_x, state_y, state_z, state_yaw_deg, no_dyn_vx, no_dyn_vy);

    MissionCommandMode mode = MISSION_CMD_STOP;
    float a = 0.0f;
    float b = 0.0f;
    float c = 0.0f;
    float d = 0.0f;

    missionFsmGetCommand(&mode, &a, &b, &c, &d);

    if (mode == MISSION_CMD_POSITION) {
      // a=x, b=y, c=z, d=yaw_deg
      send_position_setpoint(a, b, c, d);
    } else if (mode == MISSION_CMD_BODY_VEL_HEIGHT) {
      // a=vx_body, b=vy_body, c=height, d=yaw_rate_deg_s
      send_body_velocity_height_setpoint(a, b, c, d);
    } else {
      send_stop_setpoint();
    }

    vTaskDelayUntil(&xLastWakeTime, M2T(APP_LOOP_PERIOD_MS));
  }
}

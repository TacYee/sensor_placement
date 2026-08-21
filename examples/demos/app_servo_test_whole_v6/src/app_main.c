/**
 * app_main.c - Trajectory tracking + collision handling + servo CW + backup + landing
 *
 * This version reads position/yaw from the log framework, matching the older
 * working app style:
 *   stateEstimate.x, stateEstimate.y, stateEstimate.z, stateEstimate.yaw
 */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"
#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"
#include "app_channel.h"
#include "commander.h"
#include "log.h"

#include "trajectory_tracking.h"
#include "servo_test.h"

#define DEBUG_MODULE "APP"

#define TRAJECTORY_CMD_CLEAR_ID     0xA0
#define TRAJECTORY_CMD_WAYPOINT_ID  0xA1
#define APP_LOOP_PERIOD_MS          20U
#define COMMANDER_APP_PRIORITY      3

static uint8_t rx_buffer[256];
static setpoint_t setpoint;

static void send_velocity_setpoint(float vx, float vy, float vz, float yaw_rate)
{
  memset(&setpoint, 0, sizeof(setpoint_t));

  setpoint.mode.x = modeVelocity;
  setpoint.mode.y = modeVelocity;
  setpoint.mode.z = modeVelocity;
  setpoint.mode.yaw = modeVelocity;

  setpoint.velocity.x = vx;
  setpoint.velocity.y = vy;
  setpoint.velocity.z = vz;
  setpoint.attitudeRate.yaw = yaw_rate;

  // false: vx/vy are in world frame. The trajectory module converts body-forward
  // final-forward/backward commands to world frame using the current yaw.
  setpoint.velocity_body = false;

  commanderSetSetpoint(&setpoint, COMMANDER_APP_PRIORITY);
}

void appMain(void)
{
  DEBUG_PRINT("Trajectory + servo app starting\n");

  // Give the estimator/log variables a moment to become available.
  vTaskDelay(M2T(3000));

  logVarId_t idStateX = logGetVarId("stateEstimate", "x");
  logVarId_t idStateY = logGetVarId("stateEstimate", "y");
  logVarId_t idStateZ = logGetVarId("stateEstimate", "z");
  logVarId_t idStateYaw = logGetVarId("stateEstimate", "yaw");

  servoControlInit();
  trajectoryTrackingInit();

  DEBUG_PRINT("Send waypoints by app_channel cmd 0xA1, then set param traj.start=1\n");
  DEBUG_PRINT("Manual test: set traj.forceCollision=1 to trigger servo/back/land sequence\n");

  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1) {
    const float state_x = logGetFloat(idStateX);
    const float state_y = logGetFloat(idStateY);
    const float state_z = logGetFloat(idStateZ);
    const float state_yaw = logGetFloat(idStateYaw);

    trajectoryTrackingUpdateState(state_x, state_y, state_z, state_yaw);

    size_t length = appchannelReceiveDataPacket(rx_buffer, sizeof(rx_buffer), 0);
    if (length >= 1U) {
      if (rx_buffer[0] == TRAJECTORY_CMD_CLEAR_ID) {
        trajectoryTrackingClearWaypoints();
      } else if (length > 1U && rx_buffer[0] == TRAJECTORY_CMD_WAYPOINT_ID) {
        trajectoryTrackingReceiveWaypoints(&rx_buffer[1], length - 1U);
      }
    }

    float vx = 0.0f;
    float vy = 0.0f;
    float vz = 0.0f;
    float yaw_rate = 0.0f;
    trajectoryTrackingGetControl(&vx, &vy, &vz, &yaw_rate);
    send_velocity_setpoint(vx, vy, vz, yaw_rate);

    vTaskDelayUntil(&xLastWakeTime, M2T(APP_LOOP_PERIOD_MS));
  }
}

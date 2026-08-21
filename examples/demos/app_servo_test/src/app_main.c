/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2026 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * app_main.c - Trajectory tracking application
 * Receives waypoint data from Python ground station via app_channel
 * Executes waypoint-based trajectory with collision detection and end-effector control
 */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "debug.h"
#include "app_channel.h"
#include "commander.h"
#include "estimator.h"

#include "trajectory_tracking.h"

#define DEBUG_MODULE "TRAJ_APP"

// Communication protocol
#define TRAJECTORY_CMD_WAYPOINT_ID  0xA1

// Static buffers to avoid stack overflow
static uint8_t static_data_buffer[256];
static state_t static_state;
static setpoint_t static_setpoint;

void appMain() {
  DEBUG_PRINT("Trajectory tracking application starting...\n");
  
  // Initialize trajectory tracking system
  trajectoryTrackingInit();
  
  DEBUG_PRINT("Trajectory tracking ready. Waiting for trajectory data...\n");
  DEBUG_PRINT("Expected data format: [0xA1] + [waypoint_data]\n");
  DEBUG_PRINT("Each waypoint: 5 floats (x, y, z, yaw, duration_ms)\n");
  
  // Main loop
  while (1) {
    // Update trajectory with current drone position
    if (estimatorGetState(&static_state)) {
      trajectoryTrackingUpdateState(
        static_state.position.x,
        static_state.position.y,
        static_state.position.z,
        static_state.attitude.yaw
      );
    }
    
    // Receive and process trajectory commands
    uint32_t length = 0;
    
    if (appchannelReceiveDataPacket(static_data_buffer, &length)) {
      if (length > 0) {
        uint8_t cmd_id = static_data_buffer[0];
        
        if (cmd_id == TRAJECTORY_CMD_WAYPOINT_ID && length > 1) {
          DEBUG_PRINT("Waypoint command received, length=%d\n", length);
          trajectoryTrackingReceiveWaypoints(&static_data_buffer[1], length - 1);
        }
      }
    }
    
    // Get control commands from trajectory tracker
    float vx, vy, vz, yaw_rate;
    trajectoryTrackingGetControl(&vx, &vy, &vz, &yaw_rate);
    
    // Create and send setpoint to commander
    memset(&static_setpoint, 0, sizeof(setpoint_t));
    
    static_setpoint.mode = modeVelocityWorld;
    static_setpoint.velocity.x = vx;
    static_setpoint.velocity.y = vy;
    static_setpoint.velocity.z = vz;
    static_setpoint.attitudeRate.yaw = yaw_rate;
    static_setpoint.velocity_body = false;
    
    // Send to commander with high priority
    commanderSetSetpoint(&static_setpoint, COMMANDER_PRIORITY_HIGHLEVEL);
    
    // Run at 100 Hz
    vTaskDelay(M2T(10));
  }
}

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
 * trajectory_tracking.c - Trajectory tracking implementation
 */

#include <string.h>
#include <stdbool.h>
#include <math.h>

#include "trajectory_tracking.h"
#include "debug.h"
#include "app_channel.h"
#include "commander.h"
#include "deck.h"

#define DEBUG_MODULE "TRAJECTORY"

// Global context
static TrajectoryTrackingContext g_context = {0};
static bool g_initialized = false;

// Deck GPIO for IO_3 (PB4)
static uint32_t g_io3_pin = DECK_GPIO_IO3;

// ============================================================
// Helper Functions
// ============================================================

static float wrap_angle(float angle) {
  while (angle > M_PI) angle -= 2.0f * M_PI;
  while (angle < -M_PI) angle += 2.0f * M_PI;
  return angle;
}

static float distance(float x1, float y1, float x2, float y2) {
  float dx = x2 - x1;
  float dy = y2 - y1;
  return sqrtf(dx * dx + dy * dy);
}

static void activate_end_effector(void) {
  DEBUG_PRINT("Activating end-effector on IO_3 (PB4)\n");
  // Check if pin is available in deck configuration
  if (deckIsUsingIO3()) {
    pinMode(g_io3_pin, OUTPUT);
    digitalWrite(g_io3_pin, 1);  // Set HIGH to activate motor
    DEBUG_PRINT("End-effector activated\n");
  } else {
    DEBUG_PRINT("Warning: IO_3 not available\n");
  }
}

static void deactivate_end_effector(void) {
  DEBUG_PRINT("Deactivating end-effector on IO_3 (PB4)\n");
  if (deckIsUsingIO3()) {
    digitalWrite(g_io3_pin, 0);  // Set LOW to deactivate
    DEBUG_PRINT("End-effector deactivated\n");
  }
}

// ============================================================
// Collision Detection
// ============================================================

static void detect_collision(void) {
  // Simple nominal dynamics-based collision detection:
  // Check if drone position hasn't changed significantly despite velocity commands
  
  float distance_moved = distance(g_context.collision_state.last_x,
                                   g_context.collision_state.last_y,
                                   g_context.drone_x,
                                   g_context.drone_y);
  
  // If moving forward but position doesn't change, likely collision
  if (g_context.ref_vx > 0.1f && distance_moved < 0.01f) {
    g_context.collision_state.collision_counter++;
  } else {
    g_context.collision_state.collision_counter = 0;
  }
  
  if (g_context.collision_state.collision_counter >= COLLISION_COUNT_THRESHOLD) {
    g_context.collision_state.collision_detected = true;
    DEBUG_PRINT("Collision detected!\n");
  }
  
  // Update last known position
  g_context.collision_state.last_x = g_context.drone_x;
  g_context.collision_state.last_y = g_context.drone_y;
  g_context.collision_state.last_z = g_context.drone_z;
}

// ============================================================
// Waypoint Navigation
// ============================================================

static void compute_waypoint_control(void) {
  if (g_context.current_waypoint_index >= g_context.trajectory.num_waypoints) {
    // No more waypoints
    return;
  }
  
  TrajectoryWaypoint current_wp = g_context.trajectory.waypoints[g_context.current_waypoint_index];
  
  // Compute direction to waypoint
  float dx = current_wp.x - g_context.drone_x;
  float dy = current_wp.y - g_context.drone_y;
  float dz = current_wp.z - g_context.drone_z;
  float dist_xy = sqrtf(dx * dx + dy * dy);
  
  // Check if waypoint is reached
  if (dist_xy < WAYPOINT_REACH_DISTANCE && fabsf(dz) < WAYPOINT_REACH_DISTANCE) {
    DEBUG_PRINT("Waypoint %d reached\n", g_context.current_waypoint_index);
    g_context.current_waypoint_index++;
    
    // Check if all waypoints are reached
    if (g_context.current_waypoint_index >= g_context.trajectory.num_waypoints) {
      g_context.state = TRAJ_FINAL_FORWARD;
      g_context.state_start_time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
      DEBUG_PRINT("All waypoints reached. Starting final forward movement.\n");
    }
    return;
  }
  
  // Normalize direction
  if (dist_xy > 1e-6f) {
    dx /= dist_xy;
    dy /= dist_xy;
  }
  
  // Set reference velocity proportional to distance
  float speed = fminf(FORWARD_SPEED, dist_xy * 0.5f);
  g_context.ref_vx = dx * speed;
  g_context.ref_vy = dy * speed;
  
  // Set z velocity
  if (dz > 0.01f) {
    g_context.ref_vz = 0.1f;
  } else if (dz < -0.01f) {
    g_context.ref_vz = -0.1f;
  } else {
    g_context.ref_vz = 0.0f;
  }
  
  // Yaw control to face direction of movement
  float desired_yaw = atan2f(dy, dx);
  float yaw_error = wrap_angle(desired_yaw - g_context.drone_yaw);
  g_context.ref_yaw_rate = fminf(fmaxf(yaw_error * 0.5f, -1.0f), 1.0f);
}

static void compute_final_forward_control(void) {
  // Just fly straight forward
  g_context.ref_vx = FORWARD_SPEED;
  g_context.ref_vy = 0.0f;
  g_context.ref_vz = 0.0f;
  g_context.ref_yaw_rate = 0.0f;
}

static void compute_backing_up_control(void) {
  // Back up and prepare to land
  g_context.ref_vx = BACKWARD_SPEED;
  g_context.ref_vy = 0.0f;
  g_context.ref_vz = 0.0f;
  g_context.ref_yaw_rate = 0.0f;
}

static void compute_landing_control(void) {
  // Descend slowly
  g_context.ref_vx = 0.0f;
  g_context.ref_vy = 0.0f;
  g_context.ref_vz = LANDING_SPEED;
  g_context.ref_yaw_rate = 0.0f;
}

// ============================================================
// State Machine
// ============================================================

static void update_state_machine(void) {
  uint32_t current_time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
  
  switch (g_context.state) {
    case TRAJ_IDLE:
      // Wait for trajectory data
      if (g_context.data_received && g_context.trajectory.valid) {
        g_context.state = TRAJ_TAKEOFF;
        g_context.state_start_time_ms = current_time_ms;
        g_context.current_waypoint_index = 0;
        DEBUG_PRINT("Trajectory received. Starting takeoff.\n");
      }
      break;
      
    case TRAJ_TAKEOFF:
      // Move up to first waypoint height
      {
        float target_z = g_context.trajectory.waypoints[0].z;
        float dz = target_z - g_context.drone_z;
        
        g_context.ref_vx = 0.0f;
        g_context.ref_vy = 0.0f;
        g_context.ref_vz = (dz > 0.1f) ? 0.2f : 0.0f;
        g_context.ref_yaw_rate = 0.0f;
        
        if (fabsf(dz) < 0.1f) {
          g_context.state = TRAJ_FOLLOWING;
          g_context.state_start_time_ms = current_time_ms;
          DEBUG_PRINT("Takeoff complete. Starting trajectory following.\n");
        }
      }
      break;
      
    case TRAJ_FOLLOWING:
      detect_collision();
      compute_waypoint_control();
      
      if (g_context.collision_state.collision_detected) {
        g_context.state = TRAJ_COLLISION_DETECTED;
        g_context.state_start_time_ms = current_time_ms;
        DEBUG_PRINT("Collision detected, entering collision handling.\n");
      }
      break;
      
    case TRAJ_FINAL_FORWARD:
      detect_collision();
      compute_final_forward_control();
      
      if (g_context.collision_state.collision_detected) {
        g_context.state = TRAJ_COLLISION_DETECTED;
        g_context.state_start_time_ms = current_time_ms;
        DEBUG_PRINT("Collision detected during final forward, entering collision handling.\n");
      }
      break;
      
    case TRAJ_COLLISION_DETECTED:
      // Stop immediately
      g_context.ref_vx = 0.0f;
      g_context.ref_vy = 0.0f;
      g_context.ref_vz = 0.0f;
      g_context.ref_yaw_rate = 0.0f;
      
      // Move to deploying state after brief pause
      if (current_time_ms - g_context.state_start_time_ms > 200) {
        g_context.state = TRAJ_DEPLOYING;
        g_context.state_start_time_ms = current_time_ms;
      }
      break;
      
    case TRAJ_DEPLOYING:
      activate_end_effector();
      
      // Wait a bit for end-effector to deploy
      if (current_time_ms - g_context.state_start_time_ms > 500) {
        g_context.state = TRAJ_BACKING_UP;
        g_context.state_start_time_ms = current_time_ms;
        DEBUG_PRINT("End-effector deployed. Starting backup.\n");
      }
      break;
      
    case TRAJ_BACKING_UP:
      compute_backing_up_control();
      
      // Back up for a few seconds
      if (current_time_ms - g_context.state_start_time_ms > 3000) {
        g_context.state = TRAJ_LANDING;
        g_context.state_start_time_ms = current_time_ms;
        deactivate_end_effector();
        DEBUG_PRINT("Backing up complete. Starting landing.\n");
      }
      break;
      
    case TRAJ_LANDING:
      compute_landing_control();
      
      // Continue landing (ground contact will stop it)
      DEBUG_PRINT("Landing...\n");
      break;
      
    default:
      g_context.state = TRAJ_IDLE;
      break;
  }
}

// ============================================================
// Public API
// ============================================================

void trajectoryTrackingInit(void) {
  memset(&g_context, 0, sizeof(TrajectoryTrackingContext));
  g_context.state = TRAJ_IDLE;
  g_context.collision_state.collision_detected = false;
  g_context.collision_state.collision_counter = 0;
  
  // Initialize IO_3 if available
  if (deckIsUsingIO3()) {
    pinMode(g_io3_pin, OUTPUT);
    digitalWrite(g_io3_pin, 0);
    DEBUG_PRINT("Trajectory tracking initialized with IO_3 support\n");
  } else {
    DEBUG_PRINT("Trajectory tracking initialized without IO_3 support\n");
  }
  
  g_initialized = true;
}

void trajectoryTrackingUpdateState(float current_x, float current_y, float current_z, float current_yaw) {
  if (!g_initialized) {
    return;
  }
  
  // Update drone state
  g_context.drone_x = current_x;
  g_context.drone_y = current_y;
  g_context.drone_z = current_z;
  g_context.drone_yaw = current_yaw;
  
  // Update state machine
  update_state_machine();
}

void trajectoryTrackingReceiveWaypoints(const uint8_t* data, uint32_t length) {
  if (!g_initialized) {
    return;
  }
  
  // Simple protocol: each waypoint is 5 floats (x, y, z, yaw, duration)
  uint32_t waypoint_size = sizeof(TrajectoryWaypoint);
  uint32_t num_waypoints = length / waypoint_size;
  
  if (num_waypoints > MAX_WAYPOINTS) {
    num_waypoints = MAX_WAYPOINTS;
    DEBUG_PRINT("Warning: Too many waypoints, truncating to %d\n", MAX_WAYPOINTS);
  }
  
  memcpy(g_context.trajectory.waypoints, data, num_waypoints * waypoint_size);
  g_context.trajectory.num_waypoints = num_waypoints;
  g_context.trajectory.valid = true;
  g_context.data_received = true;
  
  DEBUG_PRINT("Received %d waypoints\n", num_waypoints);
}

bool trajectoryTrackingIsActive(void) {
  if (!g_initialized) {
    return false;
  }
  
  return g_context.state != TRAJ_IDLE && g_context.state != TRAJ_LANDING;
}

void trajectoryTrackingGetControl(float* vx, float* vy, float* vz, float* yaw_rate) {
  if (!g_initialized) {
    *vx = *vy = *vz = *yaw_rate = 0.0f;
    return;
  }
  
  *vx = g_context.ref_vx;
  *vy = g_context.ref_vy;
  *vz = g_context.ref_vz;
  *yaw_rate = g_context.ref_yaw_rate;
}

void trajectoryTrackingTask(void) {
  // This function can be called from appMain() at regular intervals
  // Or integrated into the main control loop
  
  if (!g_initialized) {
    trajectoryTrackingInit();
  }
  
  // The actual state updates happen in trajectoryTrackingUpdateState()
  // which should be called with current drone position/orientation
}

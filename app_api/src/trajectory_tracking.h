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
 * trajectory_tracking.h - Trajectory tracking header file
 * Implements waypoint-based trajectory following with collision detection
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

// Trajectory constraints
#define MAX_WAYPOINTS 100
#define TRAJECTORY_TIMEOUT_MS 30000

// Collision detection thresholds
#define COLLISION_THRESHOLD_X 0.2f  // Distance threshold for collision (m)
#define COLLISION_COUNT_THRESHOLD 3 // Consecutive frames before declaring collision

// Waypoint tolerance
#define WAYPOINT_REACH_DISTANCE 0.05f  // Distance to consider waypoint reached (m)
#define WAYPOINT_YAW_TOLERANCE 0.1f    // Yaw angle tolerance (rad)

// Movement speeds
#define FORWARD_SPEED 0.3f      // Forward speed (m/s)
#define BACKWARD_SPEED -0.3f    // Backward speed (m/s)
#define LANDING_SPEED -0.2f     // Landing speed (m/s)

// State machine states
typedef enum {
  TRAJ_IDLE,              // Idle state
  TRAJ_TAKEOFF,           // Taking off
  TRAJ_FOLLOWING,         // Following trajectory
  TRAJ_FINAL_FORWARD,     // Moving forward after final waypoint
  TRAJ_COLLISION_DETECTED, // Collision detected
  TRAJ_DEPLOYING,         // Deploying end-effector (activating IO_3)
  TRAJ_BACKING_UP,        // Backing up after collision
  TRAJ_LANDING            // Landing
} TrajectoryState;

// Waypoint structure
typedef struct {
  float x;
  float y;
  float z;
  float yaw;
  uint32_t duration_ms;  // Time to reach this waypoint
} TrajectoryWaypoint;

// Trajectory data structure
typedef struct {
  TrajectoryWaypoint waypoints[MAX_WAYPOINTS];
  uint16_t num_waypoints;
  uint32_t start_time_ms;
  bool valid;
} TrajectoryData;

// Collision detection state
typedef struct {
  float last_x;
  float last_y;
  float last_z;
  uint32_t collision_counter;
  bool collision_detected;
} CollisionDetectionState;

// Main state machine context
typedef struct {
  TrajectoryState state;
  TrajectoryData trajectory;
  CollisionDetectionState collision_state;
  
  uint16_t current_waypoint_index;
  uint32_t state_start_time_ms;
  
  // Current drone state
  float drone_x;
  float drone_y;
  float drone_z;
  float drone_yaw;
  
  // Reference values for control
  float ref_vx;
  float ref_vy;
  float ref_vz;
  float ref_yaw_rate;
  
  bool data_received;
  uint32_t last_update_time_ms;
} TrajectoryTrackingContext;

// Function declarations
void trajectoryTrackingInit(void);
void trajectoryTrackingTask(void);
void trajectoryTrackingUpdateState(float current_x, float current_y, float current_z, float current_yaw);
void trajectoryTrackingReceiveWaypoints(const uint8_t* data, uint32_t length);
bool trajectoryTrackingIsActive(void);
void trajectoryTrackingGetControl(float* vx, float* vy, float* vz, float* yaw_rate);

#endif // TRAJECTORY_TRACKING_H

#pragma once

#include <stdint.h>
#include <stdbool.h>

#define MAX_WAYPOINTS 100

#define WAYPOINT_REACH_DISTANCE 0.06f
#define WAYPOINT_Z_TOLERANCE    0.08f

#define MAX_XY_SPEED            0.30f
#define MAX_Z_SPEED             0.20f
#define MAX_YAW_RATE            1.00f

#define TAKEOFF_SPEED           0.20f
#define FINAL_FORWARD_SPEED     0.20f
#define BACKWARD_SPEED          0.18f
#define LANDING_SPEED           0.18f

#define FINAL_FORWARD_TIMEOUT_MS 6000U
#define COLLISION_STOP_TIME_MS   250U
#define SERVO_DEPLOY_WAIT_MS     1000U
#define BACKUP_TIME_MS           2500U
#define LANDING_TIMEOUT_MS       5000U

#define COLLISION_MIN_CMD_SPEED  0.12f
#define COLLISION_MIN_REAL_SPEED 0.025f
#define COLLISION_COUNT_THRESHOLD 12U

typedef enum {
  TRAJ_IDLE = 0,
  TRAJ_TAKEOFF,
  TRAJ_FOLLOWING,
  TRAJ_FINAL_FORWARD,
  TRAJ_COLLISION_DETECTED,
  TRAJ_SERVO_CW,
  TRAJ_BACKING_UP,
  TRAJ_LANDING,
  TRAJ_FINISHED
} TrajectoryState;

typedef struct __attribute__((packed)) {
  float x;
  float y;
  float z;
  float yaw;
  uint32_t duration_ms;
} TrajectoryWaypoint;

typedef struct {
  TrajectoryWaypoint waypoints[MAX_WAYPOINTS];
  uint16_t num_waypoints;
  bool valid;
} TrajectoryData;

typedef struct {
  float last_x;
  float last_y;
  float last_z;
  uint32_t last_time_ms;
  uint32_t low_progress_counter;
  bool initialized;
  bool collision_detected;
} CollisionDetectionState;

typedef struct {
  TrajectoryState state;
  TrajectoryData trajectory;
  CollisionDetectionState collision;

  uint16_t current_waypoint_index;
  uint32_t state_start_time_ms;

  float drone_x;
  float drone_y;
  float drone_z;
  float drone_yaw;

  float ref_vx;
  float ref_vy;
  float ref_vz;
  float ref_yaw_rate;

  bool data_received;
  bool servo_cw_sent;
} TrajectoryTrackingContext;

void trajectoryTrackingInit(void);
void trajectoryTrackingUpdateState(float current_x, float current_y, float current_z, float current_yaw);
void trajectoryTrackingClearWaypoints(void);
void trajectoryTrackingReceiveWaypoints(const uint8_t* data, uint32_t length);
void trajectoryTrackingGetControl(float* vx, float* vy, float* vz, float* yaw_rate);
bool trajectoryTrackingIsActive(void);
TrajectoryState trajectoryTrackingGetState(void);

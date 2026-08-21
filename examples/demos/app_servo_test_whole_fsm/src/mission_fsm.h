#pragma once

#include <stdint.h>
#include <stdbool.h>

#define DEFAULT_FLIGHT_HEIGHT      0.3f
#define HOVER_TIME_MS              5000U

#define MISSION_LOOP_DT_S          0.05f      // 50 ms

#define TRAJ_REACH_DISTANCE        0.05f
#define TRAJ_YAW_TOL_RAD           0.08726646f   // 5 deg

#define TRACKING_NOMINAL_SPEED     0.20f
#define TRAJ_MAX_POS_STEP          0.10f

#define TRAJ_MAX_YAW_STEP_RAD  ((float)M_PI / 36.0f)

#define FORWARD_SPEED              0.20f
#define BACKWARD_SPEED             0.4f

#define FINAL_FORWARD_TIMEOUT_MS   2000U
#define CONTACT_HOLD_FORWARD_SPEED     0.10f
#define CONTACT_HOLD_FORWARD_TIME_MS   3000U
#define SERVO_DEPLOY_WAIT_MS       1000U
#define BACKUP_TIME_MS             5000U
#define LANDING_TIMEOUT_MS         6000U

#define CONTACT_MIN_CMD_SPEED      0.10f
#define CONTACT_MIN_REAL_SPEED     0.04f
#define CONTACT_COUNT_THRESHOLD    6U


// Servo 打开前下降高度：5 cm
#define CONTACT_DESCEND_HEIGHT_M 0.05f

// 直接下降到目标高度后，保持 2 秒再打开 servo
#define CONTACT_DESCEND_SETTLE_TIME_MS 2000U

// 最低安全高度，防止 DEFAULT_FLIGHT_HEIGHT 本身较低时降得太低
#define MIN_SAFE_HEIGHT_M 0.10f


typedef enum {
  MISSION_IDLE = 0,
  MISSION_HOVERING,
  MISSION_TRAJECTORY_FOLLOWING,
  MISSION_FLY_FORWARD,
  MISSION_CONTACT,
  MISSION_DESCEND_BEFORE_SERVO,
  MISSION_SERVO_CW,
  MISSION_FLY_BACKWARD,
  MISSION_LANDING,
  MISSION_FINISHED
} MissionState;

typedef enum {
  MISSION_CMD_STOP = 0,
  MISSION_CMD_BODY_VEL_HEIGHT,
  MISSION_CMD_POSITION
} MissionCommandMode;

typedef struct {
  float last_x;
  float last_y;
  uint32_t last_time_ms;
  uint32_t low_progress_counter;
  bool initialized;
  bool contact_detected;
} NominalContactDetector;

typedef struct {
  MissionState state;
  uint32_t state_start_time_ms;

  uint16_t traj_index;
  NominalContactDetector contact;

  float drone_x;
  float drone_y;
  float drone_z;
  float drone_yaw_deg;

  MissionCommandMode cmd_mode;

  // MISSION_CMD_BODY_VEL_HEIGHT
  float cmd_vx_body;
  float cmd_vy_body;
  float cmd_height;
  float cmd_yaw_rate_deg_s;

  // MISSION_CMD_POSITION
  float cmd_x;
  float cmd_y;
  float cmd_z;
  float cmd_yaw_deg;
  float no_dyn_vx;
  float no_dyn_vy;
  float no_dyn_speed;
  bool servo_cw_sent;
} MissionFsmContext;

void missionFsmInit(void);
void missionFsmUpdate(float current_x,
                      float current_y,
                      float current_z,
                      float current_yaw_deg,
                      float no_dyn_vx,
                      float no_dyn_vy);
void missionFsmGetCommand(MissionCommandMode *mode,
                          float *a,
                          float *b,
                          float *c,
                          float *d);
MissionState missionFsmGetState(void);
bool missionFsmIsActive(void);

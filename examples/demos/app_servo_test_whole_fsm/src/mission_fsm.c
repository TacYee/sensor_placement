#include "mission_fsm.h"

#include <string.h>
#include <stdbool.h>
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"
#include "param.h"
#include "log.h"
#include "debug.h"

#include "trajectory.h"
#include "servo_test.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEBUG_MODULE "MISSION"

static MissionFsmContext g_ctx;
static bool g_initialized = false;

// Runtime parameters controlled from Python / cfclient.
static uint8_t startMission = 0;
static uint8_t abortMission = 0;
static uint8_t clearMissionTrajectory = 0;
static uint8_t forceContact = 0;
static float contactMinRealSpeed = CONTACT_MIN_REAL_SPEED;
static uint16_t contactCountThreshold = CONTACT_COUNT_THRESHOLD;

static float wrap_rad(float angle)
{
  while (angle > (float)M_PI) {
    angle -= 2.0f * (float)M_PI;
  }
  while (angle < -(float)M_PI) {
    angle += 2.0f * (float)M_PI;
  }
  return angle;
}

static float wrap_deg(float angle)
{
  while (angle > 180.0f) {
    angle -= 360.0f;
  }
  while (angle < -180.0f) {
    angle += 360.0f;
  }
  return angle;
}

static float clampf_local(float v, float lo, float hi)
{
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static float contact_final_height(void)
{
  return clampf_local(
      DEFAULT_FLIGHT_HEIGHT - CONTACT_DESCEND_HEIGHT_M,
      MIN_SAFE_HEIGHT_M,
      DEFAULT_FLIGHT_HEIGHT
  );
}

static float dist2d(float x1, float y1, float x2, float y2)
{
  const float dx = x2 - x1;
  const float dy = y2 - y1;
  return sqrtf(dx * dx + dy * dy);
}

static float deg_to_rad(float deg)
{
  return deg * ((float)M_PI / 180.0f);
}

static float rad_to_deg(float rad)
{
  return rad * (180.0f / (float)M_PI);
}

static void command_stop(void)
{
  g_ctx.cmd_mode = MISSION_CMD_STOP;
  g_ctx.cmd_vx_body = 0.0f;
  g_ctx.cmd_vy_body = 0.0f;
  g_ctx.cmd_height = DEFAULT_FLIGHT_HEIGHT;
  g_ctx.cmd_yaw_rate_deg_s = 0.0f;
  g_ctx.cmd_x = g_ctx.drone_x;
  g_ctx.cmd_y = g_ctx.drone_y;
  g_ctx.cmd_z = g_ctx.drone_z;
  g_ctx.cmd_yaw_deg = g_ctx.drone_yaw_deg;
}

static void command_body_vel_height(float vx_body, float vy_body, float height, float yaw_rate_deg_s)
{
  g_ctx.cmd_mode = MISSION_CMD_BODY_VEL_HEIGHT;
  g_ctx.cmd_vx_body = vx_body;
  g_ctx.cmd_vy_body = vy_body;
  g_ctx.cmd_height = height;
  g_ctx.cmd_yaw_rate_deg_s = yaw_rate_deg_s;
}

static void command_position(float x, float y, float z, float yaw_deg)
{
  g_ctx.cmd_mode = MISSION_CMD_POSITION;
  g_ctx.cmd_x = x;
  g_ctx.cmd_y = y;
  g_ctx.cmd_z = z;
  g_ctx.cmd_yaw_deg = wrap_deg(yaw_deg);
}

static void reset_contact_detector(void)
{
  memset(&g_ctx.contact, 0, sizeof(g_ctx.contact));
}

static void enter_state(MissionState new_state, uint32_t now_ms)
{
  g_ctx.state = new_state;
  g_ctx.state_start_time_ms = now_ms;

  if (new_state == MISSION_TRAJECTORY_FOLLOWING ||
      new_state == MISSION_FLY_FORWARD ||
      new_state == MISSION_FLY_BACKWARD) {
    reset_contact_detector();
  }

  if (new_state == MISSION_SERVO_CW) {
    g_ctx.servo_cw_sent = false;
  }

  DEBUG_PRINT("State -> %d\n", (int)new_state);
}

/*
 * Contact trigger:
 *
 * This version is still a speed-based detector, but it no longer computes
 * real speed from stateEstimate.x/y difference.
 *
 * Instead, it uses the auxiliary no-dynamics Kalman velocity:
 *
 *   real_speed = sqrt(no_dyn_vx^2 + no_dyn_vy^2)
 *
 * Since statePX_noDyn/statePY_noDyn are body-frame velocity states,
 * using the magnitude is frame-independent for contact triggering.
 */
static void detect_nominal_contact(uint32_t now_ms, float nominal_cmd_speed)
{
  if (forceContact != 0) {
    g_ctx.contact.contact_detected = true;
    forceContact = 0;
    return;
  }

  if (!g_ctx.contact.initialized) {
    g_ctx.contact.last_time_ms = now_ms;
    g_ctx.contact.initialized = true;
    return;
  }

  const uint32_t dt_ms = now_ms - g_ctx.contact.last_time_ms;
  if (dt_ms < 50U) {
    return;
  }

  const float real_speed = sqrtf(
      g_ctx.no_dyn_vx * g_ctx.no_dyn_vx +
      g_ctx.no_dyn_vy * g_ctx.no_dyn_vy
  );

  g_ctx.no_dyn_speed = real_speed;

  if (nominal_cmd_speed > CONTACT_MIN_CMD_SPEED &&
      real_speed < contactMinRealSpeed) {
    g_ctx.contact.low_progress_counter++;
  } else {
    g_ctx.contact.low_progress_counter = 0;
  }

  if (g_ctx.contact.low_progress_counter >= contactCountThreshold) {
    g_ctx.contact.contact_detected = true;

    DEBUG_PRINT("Contact by noDyn V: cmd=%.3f noDynV=%.3f count=%lu/%u\n",
                (double)nominal_cmd_speed,
                (double)real_speed,
                (unsigned long)g_ctx.contact.low_progress_counter,
                (unsigned int)contactCountThreshold);
  }

  g_ctx.contact.last_time_ms = now_ms;
}

static float compute_old_style_trajectory_tracking(void)
{
  if (trajectory_length < 2 || g_ctx.traj_index >= (uint16_t)(trajectory_length - 1)) {
    return 0.0f;
  }

  const TrajectoryPoint p1 = trajectory_buffer[g_ctx.traj_index];
  const TrajectoryPoint p2 = trajectory_buffer[g_ctx.traj_index + 1];

  float dx_path = p2.x - p1.x;
  float dy_path = p2.y - p1.y;

  const float norm = sqrtf(dx_path * dx_path + dy_path * dy_path) + 1e-6f;
  dx_path /= norm;
  dy_path /= norm;

  const float ref_yaw_rad = atan2f(dy_path, dx_path);
  const float drone_yaw_rad = deg_to_rad(g_ctx.drone_yaw_deg);
  const float yaw_diff = wrap_rad(ref_yaw_rad - drone_yaw_rad);

  const float yaw_step = clampf_local(
      yaw_diff,
      -TRAJ_MAX_YAW_STEP_RAD,
      TRAJ_MAX_YAW_STEP_RAD
  );

  const float intermediate_yaw_deg = wrap_deg(
      rad_to_deg(drone_yaw_rad + yaw_step)
  );

  // Keep your current old-style behavior: track p1, not p2.
  const float ref_x = p1.x;
  const float ref_y = p1.y;

  const float diff_x = ref_x - g_ctx.drone_x;
  const float diff_y = ref_y - g_ctx.drone_y;

  const float pos_dist = sqrtf(diff_x * diff_x + diff_y * diff_y);

  const float step_ratio = fminf(
      1.0f,
      TRAJ_MAX_POS_STEP / (pos_dist + 1e-6f)
  );

  const float step_x = g_ctx.drone_x + diff_x * step_ratio;
  const float step_y = g_ctx.drone_y + diff_y * step_ratio;

  command_position(
      step_x,
      step_y,
      DEFAULT_FLIGHT_HEIGHT,
      intermediate_yaw_deg
  );

  const float nominal_step_dist = dist2d(
      g_ctx.drone_x,
      g_ctx.drone_y,
      step_x,
      step_y
  );

  const float nominal_speed = nominal_step_dist / MISSION_LOOP_DT_S;

  if (pos_dist < TRAJ_REACH_DISTANCE &&
      fabsf(yaw_diff) < TRAJ_MAX_YAW_STEP_RAD) {
    g_ctx.traj_index++;

    DEBUG_PRINT("Trajectory segment reached: %u/%d\n",
                (unsigned int)g_ctx.traj_index,
                trajectory_length);
  }

  return nominal_speed;
}

static void update_state_machine(void)
{
  const uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

  if (clearMissionTrajectory != 0) {
    clearMissionTrajectory = 0;
    clearTrajectory();
    g_ctx.traj_index = 0;
    reset_contact_detector();

    if (g_ctx.state == MISSION_IDLE || g_ctx.state == MISSION_FINISHED) {
      enter_state(MISSION_IDLE, now_ms);
    }
  }

  if (abortMission != 0) {
    abortMission = 0;

    if (g_ctx.state != MISSION_IDLE && g_ctx.state != MISSION_FINISHED) {
      enter_state(MISSION_LANDING, now_ms);
    } else {
      enter_state(MISSION_IDLE, now_ms);
    }

    return;
  }

  if (forceContact != 0 &&
      g_ctx.state != MISSION_IDLE &&
      g_ctx.state != MISSION_FINISHED &&
      g_ctx.state != MISSION_CONTACT &&
      g_ctx.state != MISSION_DESCEND_BEFORE_SERVO &&
      g_ctx.state != MISSION_SERVO_CW &&
      g_ctx.state != MISSION_FLY_BACKWARD &&
      g_ctx.state != MISSION_LANDING) {
    forceContact = 0;
    enter_state(MISSION_CONTACT, now_ms);
    return;
  }

  switch (g_ctx.state) {
    case MISSION_IDLE:
      command_stop();

      if (trajectory_received && trajectory_length >= 2 && startMission != 0) {
        startMission = 0;
        g_ctx.traj_index = 0;
        enter_state(MISSION_HOVERING, now_ms);
      }

      break;

    case MISSION_HOVERING:
      command_body_vel_height(0.0f, 0.0f, DEFAULT_FLIGHT_HEIGHT, 0.0f);

      if ((now_ms - g_ctx.state_start_time_ms) >= HOVER_TIME_MS) {
        enter_state(MISSION_TRAJECTORY_FOLLOWING, now_ms);
      }

      break;

    case MISSION_TRAJECTORY_FOLLOWING: {
      if (trajectory_length < 2 || g_ctx.traj_index >= (uint16_t)(trajectory_length - 1)) {
        clearTrajectory();
        g_ctx.traj_index = 0;
        enter_state(MISSION_FLY_FORWARD, now_ms);
        break;
      }

      const float nominal_speed = compute_old_style_trajectory_tracking();

      detect_nominal_contact(now_ms, nominal_speed);

      if (g_ctx.contact.contact_detected) {
        clearTrajectory();
        g_ctx.traj_index = 0;
        enter_state(MISSION_CONTACT, now_ms);
      }

      break;
    }

    case MISSION_FLY_FORWARD:
      command_body_vel_height(FORWARD_SPEED, 0.0f, DEFAULT_FLIGHT_HEIGHT, 0.0f);

      detect_nominal_contact(now_ms, fabsf(FORWARD_SPEED));

      if (g_ctx.contact.contact_detected) {
        enter_state(MISSION_CONTACT, now_ms);
      } else if ((now_ms - g_ctx.state_start_time_ms) > FINAL_FORWARD_TIMEOUT_MS) {
        enter_state(MISSION_CONTACT, now_ms);
      }

      break;

    case MISSION_CONTACT:
      // Contact 后先不下降，保持原高度，继续轻微向前顶住。
      command_body_vel_height(CONTACT_HOLD_FORWARD_SPEED,
                              0.0f,
                              DEFAULT_FLIGHT_HEIGHT,
                              0.0f);

      if ((now_ms - g_ctx.state_start_time_ms) >= CONTACT_HOLD_FORWARD_TIME_MS) {
        enter_state(MISSION_DESCEND_BEFORE_SERVO, now_ms);
      }

      break;

    case MISSION_DESCEND_BEFORE_SERVO: {
      // Servo 打开之前，直接把高度目标设置为低 5 cm。
      // 不做 ramp；保持下降后的高度 2 秒，再进入 servo。
      const float contact_height = contact_final_height();

      command_body_vel_height(CONTACT_HOLD_FORWARD_SPEED,
                              0.0f,
                              contact_height,
                              0.0f);

      if ((now_ms - g_ctx.state_start_time_ms) >= CONTACT_DESCEND_SETTLE_TIME_MS) {
        enter_state(MISSION_SERVO_CW, now_ms);
      }

      break;
    }

    case MISSION_SERVO_CW: {
      // Servo 打开期间保持最终下降后的高度。
      const float contact_height = contact_final_height();

      command_body_vel_height(0.0f, 0.0f, contact_height, 0.0f);

      if (!g_ctx.servo_cw_sent) {
        servoControlMoveCw();
        g_ctx.servo_cw_sent = true;
        DEBUG_PRINT("Servo CW command sent\n");
      }

      if ((now_ms - g_ctx.state_start_time_ms) >= SERVO_DEPLOY_WAIT_MS) {
        enter_state(MISSION_FLY_BACKWARD, now_ms);
      }

      break;
    }

    case MISSION_FLY_BACKWARD: {
      // Servo 后退阶段也保持最终下降后的高度。
      const float contact_height = contact_final_height();

      command_body_vel_height(-BACKWARD_SPEED, 0.0f, contact_height, 0.0f);

      if ((now_ms - g_ctx.state_start_time_ms) >= BACKUP_TIME_MS) {
        enter_state(MISSION_LANDING, now_ms);
      }

      break;
    }

    case MISSION_LANDING:
      command_body_vel_height(0.0f, 0.0f, 0.10f, 0.0f);

      if (g_ctx.drone_z < 0.11f ||
          (now_ms - g_ctx.state_start_time_ms) >= LANDING_TIMEOUT_MS) {
        command_stop();
        clearTrajectory();
        enter_state(MISSION_FINISHED, now_ms);
      }

      break;

    case MISSION_FINISHED:
      command_stop();
      break;

    default:
      command_stop();
      enter_state(MISSION_IDLE, now_ms);
      break;
  }
}

void missionFsmInit(void)
{
  memset(&g_ctx, 0, sizeof(g_ctx));

  g_ctx.state = MISSION_IDLE;
  g_ctx.cmd_height = DEFAULT_FLIGHT_HEIGHT;

  trajectoryInit();

  g_initialized = true;

  DEBUG_PRINT("Mission FSM initialized\n");
}

void missionFsmUpdate(float current_x,
                      float current_y,
                      float current_z,
                      float current_yaw_deg,
                      float no_dyn_vx,
                      float no_dyn_vy)
{
  if (!g_initialized) {
    return;
  }

  g_ctx.drone_x = current_x;
  g_ctx.drone_y = current_y;
  g_ctx.drone_z = current_z;
  g_ctx.drone_yaw_deg = current_yaw_deg;

  g_ctx.no_dyn_vx = no_dyn_vx;
  g_ctx.no_dyn_vy = no_dyn_vy;
  g_ctx.no_dyn_speed = sqrtf(no_dyn_vx * no_dyn_vx + no_dyn_vy * no_dyn_vy);

  update_state_machine();
}

void missionFsmGetCommand(MissionCommandMode *mode, float *a, float *b, float *c, float *d)
{
  if (!g_initialized || mode == NULL || a == NULL || b == NULL || c == NULL || d == NULL) {
    return;
  }

  *mode = g_ctx.cmd_mode;

  if (g_ctx.cmd_mode == MISSION_CMD_POSITION) {
    *a = g_ctx.cmd_x;
    *b = g_ctx.cmd_y;
    *c = g_ctx.cmd_z;
    *d = g_ctx.cmd_yaw_deg;
  } else if (g_ctx.cmd_mode == MISSION_CMD_BODY_VEL_HEIGHT) {
    *a = g_ctx.cmd_vx_body;
    *b = g_ctx.cmd_vy_body;
    *c = g_ctx.cmd_height;
    *d = g_ctx.cmd_yaw_rate_deg_s;
  } else {
    *a = 0.0f;
    *b = 0.0f;
    *c = 0.0f;
    *d = 0.0f;
  }
}

MissionState missionFsmGetState(void)
{
  return g_ctx.state;
}

bool missionFsmIsActive(void)
{
  return g_initialized && g_ctx.state != MISSION_IDLE && g_ctx.state != MISSION_FINISHED;
}

PARAM_GROUP_START(traj)
PARAM_ADD(PARAM_UINT8, start, &startMission)
PARAM_ADD(PARAM_UINT8, abort, &abortMission)
PARAM_ADD(PARAM_UINT8, clear, &clearMissionTrajectory)
PARAM_ADD(PARAM_UINT8, forceCollision, &forceContact)
PARAM_ADD(PARAM_FLOAT, colMinSpeed, &contactMinRealSpeed)
PARAM_ADD(PARAM_UINT16, colCount, &contactCountThreshold)
PARAM_GROUP_STOP(traj)

LOG_GROUP_START(mission)
LOG_ADD(LOG_UINT8, state, &g_ctx.state)
LOG_ADD(LOG_UINT16, trajIndex, &g_ctx.traj_index)
LOG_ADD(LOG_FLOAT, cmdA, &g_ctx.cmd_x)
LOG_ADD(LOG_FLOAT, cmdB, &g_ctx.cmd_y)
LOG_ADD(LOG_FLOAT, cmdHeight, &g_ctx.cmd_height)
LOG_ADD(LOG_UINT8, contact, &g_ctx.contact.contact_detected)
LOG_ADD(LOG_FLOAT, noDynVx, &g_ctx.no_dyn_vx)
LOG_ADD(LOG_FLOAT, noDynVy, &g_ctx.no_dyn_vy)
LOG_ADD(LOG_FLOAT, noDynSpeed, &g_ctx.no_dyn_speed)
LOG_GROUP_STOP(mission)
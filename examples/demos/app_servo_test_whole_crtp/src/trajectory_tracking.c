#include <string.h>
#include <stdbool.h>
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"
#include "param.h"
#include "log.h"
#include "debug.h"
#include "crtp.h"

#include "trajectory_tracking.h"
#include "servo_test.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef CRTP_PORT_TRAJECTORY
#define CRTP_PORT_TRAJECTORY TRAJECTORY_CRTP_PORT
#endif

#define DEBUG_MODULE "TRAJ"

TrajectoryPoint trajectory_buffer[MAX_TRAJECTORY_POINTS];
int trajectory_length = 0;
bool trajectory_received = false;

static TrajectoryTrackingContext g_context;
static bool g_initialized = false;

static uint8_t trajStart = 0;
static uint8_t trajAbort = 0;
static uint8_t forceCollision = 0;
static float collisionMinRealSpeed = COLLISION_MIN_REAL_SPEED;
static uint16_t collisionCountThreshold = COLLISION_COUNT_THRESHOLD;

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
  if (v < lo) {
    return lo;
  }
  if (v > hi) {
    return hi;
  }
  return v;
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
  g_context.cmd_mode = TRAJ_CMD_STOP;
  g_context.cmd_vx_body = 0.0f;
  g_context.cmd_vy_body = 0.0f;
  g_context.cmd_height = DEFAULT_FLIGHT_HEIGHT;
  g_context.cmd_yaw_rate_deg_s = 0.0f;
  g_context.cmd_x = g_context.drone_x;
  g_context.cmd_y = g_context.drone_y;
  g_context.cmd_z = g_context.drone_z;
  g_context.cmd_yaw_deg = g_context.drone_yaw_deg;
}

static void command_body_vel_height(float vx_body, float vy_body, float height, float yaw_rate_deg_s)
{
  g_context.cmd_mode = TRAJ_CMD_BODY_VEL_HEIGHT;
  g_context.cmd_vx_body = vx_body;
  g_context.cmd_vy_body = vy_body;
  g_context.cmd_height = height;
  g_context.cmd_yaw_rate_deg_s = yaw_rate_deg_s;
}

static void command_position(float x, float y, float z, float yaw_deg)
{
  g_context.cmd_mode = TRAJ_CMD_POSITION;
  g_context.cmd_x = x;
  g_context.cmd_y = y;
  g_context.cmd_z = z;
  g_context.cmd_yaw_deg = wrap_deg(yaw_deg);
}

static void reset_collision_detector(void)
{
  memset(&g_context.collision, 0, sizeof(g_context.collision));
}

static void enter_state(TrajectoryState new_state, uint32_t now_ms)
{
  g_context.state = new_state;
  g_context.state_start_time_ms = now_ms;

  if (new_state == TRAJ_FLY_FORWARD || new_state == TRAJ_FLY_BACKWARD || new_state == TRAJ_FOLLOWING) {
    reset_collision_detector();
  }

  if (new_state == TRAJ_SERVO_CW) {
    g_context.servo_cw_sent = false;
  }

  DEBUG_PRINT("State -> %d\n", (int)new_state);
}

void clearTrajectory(void)
{
  trajectory_length = 0;
  trajectory_received = false;
  g_context.traj_index = 0;
  DEBUG_PRINT("Trajectory cleared\n");
}

static void crtpTrajectoryHandler(CRTPPacket *pk)
{
  if (pk->size != sizeof(TrajectoryPoint)) {
    return;
  }

  TrajectoryPoint pt;
  memcpy(&pt, pk->data, sizeof(TrajectoryPoint));

  if (isnan(pt.x) && isnan(pt.y)) {
    trajectory_received = true;
    DEBUG_PRINT("Trajectory received: len=%d\n", trajectory_length);
    return;
  }

  if (trajectory_length >= MAX_TRAJECTORY_POINTS) {
    DEBUG_PRINT("Trajectory buffer full\n");
    return;
  }

  trajectory_buffer[trajectory_length++] = pt;
}

static void detect_nominal_collision(uint32_t now_ms, float nominal_cmd_speed)
{
  if (forceCollision != 0) {
    g_context.collision.collision_detected = true;
    forceCollision = 0;
    return;
  }

  if (!g_context.collision.initialized) {
    g_context.collision.last_x = g_context.drone_x;
    g_context.collision.last_y = g_context.drone_y;
    g_context.collision.last_time_ms = now_ms;
    g_context.collision.initialized = true;
    return;
  }

  const uint32_t dt_ms = now_ms - g_context.collision.last_time_ms;
  if (dt_ms < 40U) {
    return;
  }

  const float moved_xy = dist2d(g_context.collision.last_x,
                                g_context.collision.last_y,
                                g_context.drone_x,
                                g_context.drone_y);
  const float real_speed = moved_xy / ((float)dt_ms * 0.001f);

  if (nominal_cmd_speed > COLLISION_MIN_CMD_SPEED && real_speed < collisionMinRealSpeed) {
    g_context.collision.low_progress_counter++;
  } else {
    g_context.collision.low_progress_counter = 0;
  }

  if (g_context.collision.low_progress_counter >= collisionCountThreshold) {
    g_context.collision.collision_detected = true;
    DEBUG_PRINT("Contact detected: cmd=%.3f real=%.3f count=%lu\n",
                (double)nominal_cmd_speed,
                (double)real_speed,
                (unsigned long)g_context.collision.low_progress_counter);
  }

  g_context.collision.last_x = g_context.drone_x;
  g_context.collision.last_y = g_context.drone_y;
  g_context.collision.last_time_ms = now_ms;
}

static void compute_old_style_trajectory_tracking(void)
{
  if (trajectory_length < 2 || g_context.traj_index >= (uint16_t)(trajectory_length - 1)) {
    enter_state(TRAJ_FLY_FORWARD, xTaskGetTickCount() * portTICK_PERIOD_MS);
    return;
  }

  const TrajectoryPoint p1 = trajectory_buffer[g_context.traj_index];
  const TrajectoryPoint p2 = trajectory_buffer[g_context.traj_index + 1];

  float dx_path = p2.x - p1.x;
  float dy_path = p2.y - p1.y;
  const float norm = sqrtf(dx_path * dx_path + dy_path * dy_path) + 1e-6f;
  dx_path /= norm;
  dy_path /= norm;

  const float ref_yaw_rad = atan2f(dy_path, dx_path);
  const float drone_yaw_rad = deg_to_rad(g_context.drone_yaw_deg);
  const float yaw_diff = wrap_rad(ref_yaw_rad - drone_yaw_rad);

  const float yaw_step = clampf_local(yaw_diff, -TRAJ_MAX_YAW_STEP_RAD, TRAJ_MAX_YAW_STEP_RAD);
  const float intermediate_yaw_deg = wrap_deg(rad_to_deg(drone_yaw_rad + yaw_step));

  const float diff_x = p1.x - g_context.drone_x;
  const float diff_y = p1.y - g_context.drone_y;
  const float pos_dist = sqrtf(diff_x * diff_x + diff_y * diff_y);

  const float step_ratio = fminf(1.0f, TRAJ_MAX_POS_STEP / (pos_dist + 1e-6f));
  const float step_x = g_context.drone_x + diff_x * step_ratio;
  const float step_y = g_context.drone_y + diff_y * step_ratio;

  command_position(step_x, step_y, DEFAULT_FLIGHT_HEIGHT, intermediate_yaw_deg);

  if (pos_dist < TRAJ_REACH_DISTANCE && fabsf(yaw_diff) < TRAJ_YAW_TOL_RAD) {
    g_context.traj_index++;
    DEBUG_PRINT("Trajectory point reached: %u/%d\n", (unsigned int)g_context.traj_index, trajectory_length);
  }
}

static void update_state_machine(void)
{
  const uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

  if (trajAbort != 0) {
    trajAbort = 0;
    if (g_context.state != TRAJ_IDLE && g_context.state != TRAJ_FINISHED) {
      enter_state(TRAJ_LANDING, now_ms);
    } else {
      enter_state(TRAJ_IDLE, now_ms);
    }
    return;
  }

  if (forceCollision != 0 &&
      g_context.state != TRAJ_IDLE &&
      g_context.state != TRAJ_FINISHED &&
      g_context.state != TRAJ_CONTACT &&
      g_context.state != TRAJ_SERVO_CW &&
      g_context.state != TRAJ_FLY_BACKWARD &&
      g_context.state != TRAJ_LANDING) {
    forceCollision = 0;
    enter_state(TRAJ_CONTACT, now_ms);
    return;
  }

  switch (g_context.state) {
    case TRAJ_IDLE:
      command_stop();
      if (trajectory_received && trajectory_length >= 2 && trajStart != 0) {
        trajStart = 0;
        g_context.traj_index = 0;
        enter_state(TRAJ_HOVERING, now_ms);
      }
      break;

    case TRAJ_HOVERING:
      command_body_vel_height(0.0f, 0.0f, DEFAULT_FLIGHT_HEIGHT, 0.0f);
      if ((now_ms - g_context.state_start_time_ms) >= HOVER_TIME_MS) {
        enter_state(TRAJ_FOLLOWING, now_ms);
      }
      break;

    case TRAJ_FOLLOWING:
      compute_old_style_trajectory_tracking();
      break;

    case TRAJ_FLY_FORWARD:
      command_body_vel_height(FORWARD_SPEED, 0.0f, DEFAULT_FLIGHT_HEIGHT, 0.0f);
      detect_nominal_collision(now_ms, fabsf(FORWARD_SPEED));
      if (g_context.collision.collision_detected) {
        enter_state(TRAJ_CONTACT, now_ms);
      } else if ((now_ms - g_context.state_start_time_ms) > FINAL_FORWARD_TIMEOUT_MS) {
        // Safety fallback: do not fly forward forever if nominal collision is not detected.
        enter_state(TRAJ_CONTACT, now_ms);
      }
      break;

    case TRAJ_CONTACT:
      command_body_vel_height(0.0f, 0.0f, DEFAULT_FLIGHT_HEIGHT, 0.0f);
      if ((now_ms - g_context.state_start_time_ms) >= COLLISION_STOP_TIME_MS) {
        enter_state(TRAJ_SERVO_CW, now_ms);
      }
      break;

    case TRAJ_SERVO_CW:
      command_body_vel_height(0.0f, 0.0f, DEFAULT_FLIGHT_HEIGHT, 0.0f);
      if (!g_context.servo_cw_sent) {
        servoControlMoveCw();
        g_context.servo_cw_sent = true;
        DEBUG_PRINT("Servo CW command sent\n");
      }
      if ((now_ms - g_context.state_start_time_ms) >= SERVO_DEPLOY_WAIT_MS) {
        enter_state(TRAJ_FLY_BACKWARD, now_ms);
      }
      break;

    case TRAJ_FLY_BACKWARD:
      command_body_vel_height(-BACKWARD_SPEED, 0.0f, DEFAULT_FLIGHT_HEIGHT, 0.0f);
      if ((now_ms - g_context.state_start_time_ms) >= BACKUP_TIME_MS) {
        enter_state(TRAJ_LANDING, now_ms);
      }
      break;

    case TRAJ_LANDING:
      command_body_vel_height(0.0f, 0.0f, 0.10f, 0.0f);
      if (g_context.drone_z < 0.11f || (now_ms - g_context.state_start_time_ms) >= LANDING_TIMEOUT_MS) {
        command_stop();
        clearTrajectory();
        enter_state(TRAJ_FINISHED, now_ms);
      }
      break;

    case TRAJ_FINISHED:
      command_stop();
      break;

    default:
      command_stop();
      enter_state(TRAJ_IDLE, now_ms);
      break;
  }
}

void trajectoryTrackingInit(void)
{
  memset(&g_context, 0, sizeof(g_context));
  clearTrajectory();
  g_context.state = TRAJ_IDLE;
  g_context.cmd_height = DEFAULT_FLIGHT_HEIGHT;
  g_initialized = true;

  crtpRegisterPortCB(CRTP_PORT_TRAJECTORY, crtpTrajectoryHandler);
  DEBUG_PRINT("Initialized, CRTP trajectory port=0x%02X\n", TRAJECTORY_CRTP_PORT);
}

void trajectoryTrackingUpdateState(float current_x, float current_y, float current_z, float current_yaw_deg)
{
  if (!g_initialized) {
    return;
  }

  g_context.drone_x = current_x;
  g_context.drone_y = current_y;
  g_context.drone_z = current_z;
  g_context.drone_yaw_deg = current_yaw_deg;

  update_state_machine();
}

void trajectoryTrackingGetCommand(TrajectoryCommandMode *mode,
                                  float *a,
                                  float *b,
                                  float *c,
                                  float *d)
{
  if (!g_initialized || mode == NULL || a == NULL || b == NULL || c == NULL || d == NULL) {
    return;
  }

  *mode = g_context.cmd_mode;

  if (g_context.cmd_mode == TRAJ_CMD_POSITION) {
    *a = g_context.cmd_x;
    *b = g_context.cmd_y;
    *c = g_context.cmd_z;
    *d = g_context.cmd_yaw_deg;
  } else if (g_context.cmd_mode == TRAJ_CMD_BODY_VEL_HEIGHT) {
    *a = g_context.cmd_vx_body;
    *b = g_context.cmd_vy_body;
    *c = g_context.cmd_height;
    *d = g_context.cmd_yaw_rate_deg_s;
  } else {
    *a = 0.0f;
    *b = 0.0f;
    *c = 0.0f;
    *d = 0.0f;
  }
}

bool trajectoryTrackingIsActive(void)
{
  return g_initialized &&
         g_context.state != TRAJ_IDLE &&
         g_context.state != TRAJ_FINISHED;
}

TrajectoryState trajectoryTrackingGetState(void)
{
  return g_context.state;
}

PARAM_GROUP_START(traj)
PARAM_ADD(PARAM_UINT8, start, &trajStart)
PARAM_ADD(PARAM_UINT8, abort, &trajAbort)
PARAM_ADD(PARAM_UINT8, forceCollision, &forceCollision)
PARAM_ADD(PARAM_FLOAT, colMinSpeed, &collisionMinRealSpeed)
PARAM_ADD(PARAM_UINT16, colCount, &collisionCountThreshold)
PARAM_GROUP_STOP(traj)

LOG_GROUP_START(traj)
LOG_ADD(LOG_UINT8, state, &g_context.state)
LOG_ADD(LOG_UINT16, wpIndex, &g_context.traj_index)
LOG_ADD(LOG_UINT16, length, &trajectory_length)
LOG_ADD(LOG_UINT8, received, &trajectory_received)
LOG_ADD(LOG_FLOAT, traj_x, &trajectory_buffer[0].x)
LOG_ADD(LOG_FLOAT, traj_y, &trajectory_buffer[0].y)
LOG_ADD(LOG_UINT8, collision, &g_context.collision.collision_detected)
LOG_ADD(LOG_FLOAT, cmdX, &g_context.cmd_x)
LOG_ADD(LOG_FLOAT, cmdY, &g_context.cmd_y)
LOG_ADD(LOG_FLOAT, cmdYaw, &g_context.cmd_yaw_deg)
LOG_GROUP_STOP(traj)

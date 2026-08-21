#include <string.h>
#include <stdbool.h>
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"
#include "param.h"
#include "log.h"
#include "debug.h"

#include "trajectory_tracking.h"
#include "servo_test.h"

#define DEBUG_MODULE "TRAJ"

static TrajectoryTrackingContext g_context;
static bool g_initialized = false;

// Runtime tuning / manual trigger through Crazyflie params.
static uint8_t trajStart = 0;
static uint8_t trajAbort = 0;
static uint8_t forceCollision = 0;
static float collisionMinRealSpeed = COLLISION_MIN_REAL_SPEED;
static uint16_t collisionCountThreshold = COLLISION_COUNT_THRESHOLD;

static float wrap_angle(float angle)
{
  while (angle > (float)M_PI) {
    angle -= 2.0f * (float)M_PI;
  }
  while (angle < -(float)M_PI) {
    angle += 2.0f * (float)M_PI;
  }
  return angle;
}

static float clampf(float v, float lo, float hi)
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

static void zero_control(void)
{
  g_context.ref_vx = 0.0f;
  g_context.ref_vy = 0.0f;
  g_context.ref_vz = 0.0f;
  g_context.ref_yaw_rate = 0.0f;
}

static void reset_collision_detector(void)
{
  memset(&g_context.collision, 0, sizeof(g_context.collision));
  forceCollision = 0;
}

static void enter_state(TrajectoryState new_state, uint32_t now_ms)
{
  g_context.state = new_state;
  g_context.state_start_time_ms = now_ms;

  if (new_state == TRAJ_FOLLOWING || new_state == TRAJ_FINAL_FORWARD || new_state == TRAJ_BACKING_UP) {
    reset_collision_detector();
  }

  if (new_state == TRAJ_SERVO_CW) {
    g_context.servo_cw_sent = false;
  }

  DEBUG_PRINT("State -> %d\n", (int)new_state);
}

static float deg_to_rad(float deg)
{
  return deg * ((float)M_PI / 180.0f);
}

static float get_yaw_rad(void)
{
  return deg_to_rad(g_context.drone_yaw);
}

static void body_forward_to_world(float body_forward_speed, float *vx, float *vy)
{
  const float yaw_rad = get_yaw_rad();
  *vx = cosf(yaw_rad) * body_forward_speed;
  *vy = sinf(yaw_rad) * body_forward_speed;
}

static void detect_collision(uint32_t now_ms)
{
  if (forceCollision != 0) {
    g_context.collision.collision_detected = true;
    return;
  }

  const float cmd_speed = sqrtf(g_context.ref_vx * g_context.ref_vx +
                                g_context.ref_vy * g_context.ref_vy);

  if (!g_context.collision.initialized) {
    g_context.collision.last_x = g_context.drone_x;
    g_context.collision.last_y = g_context.drone_y;
    g_context.collision.last_z = g_context.drone_z;
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

  if (cmd_speed > COLLISION_MIN_CMD_SPEED && real_speed < collisionMinRealSpeed) {
    g_context.collision.low_progress_counter++;
  } else {
    g_context.collision.low_progress_counter = 0;
  }

  if (g_context.collision.low_progress_counter >= collisionCountThreshold) {
    g_context.collision.collision_detected = true;
    DEBUG_PRINT("Collision: cmd=%.3f real=%.3f count=%lu\n",
                (double)cmd_speed,
                (double)real_speed,
                (unsigned long)g_context.collision.low_progress_counter);
  }

  g_context.collision.last_x = g_context.drone_x;
  g_context.collision.last_y = g_context.drone_y;
  g_context.collision.last_z = g_context.drone_z;
  g_context.collision.last_time_ms = now_ms;
}

static void compute_waypoint_control(void)
{
  zero_control();

  if (g_context.current_waypoint_index >= g_context.trajectory.num_waypoints) {
    return;
  }

  const TrajectoryWaypoint *wp = &g_context.trajectory.waypoints[g_context.current_waypoint_index];

  const float dx = wp->x - g_context.drone_x;
  const float dy = wp->y - g_context.drone_y;
  const float dz = wp->z - g_context.drone_z;
  const float dxy = sqrtf(dx * dx + dy * dy);
  const float yaw_error = wrap_angle(wp->yaw - get_yaw_rad());

  if (dxy < WAYPOINT_REACH_DISTANCE && fabsf(dz) < WAYPOINT_Z_TOLERANCE) {
    DEBUG_PRINT("Waypoint %u reached\n", (unsigned int)g_context.current_waypoint_index);
    g_context.current_waypoint_index++;
    return;
  }

  if (dxy > 1e-4f) {
    const float speed = clampf(0.8f * dxy, 0.05f, MAX_XY_SPEED);
    g_context.ref_vx = speed * dx / dxy;
    g_context.ref_vy = speed * dy / dxy;
  }

  g_context.ref_vz = clampf(1.0f * dz, -MAX_Z_SPEED, MAX_Z_SPEED);
  // attitudeRate.yaw in Crazyflie commander setpoints is deg/s.
  const float yaw_rate_rad_s = clampf(1.5f * yaw_error, -MAX_YAW_RATE, MAX_YAW_RATE);
  g_context.ref_yaw_rate = yaw_rate_rad_s * (180.0f / (float)M_PI);
}

static void compute_final_forward_control(void)
{
  body_forward_to_world(FINAL_FORWARD_SPEED, &g_context.ref_vx, &g_context.ref_vy);
  g_context.ref_vz = 0.0f;
  g_context.ref_yaw_rate = 0.0f;
}

static void compute_backing_up_control(void)
{
  body_forward_to_world(-BACKWARD_SPEED, &g_context.ref_vx, &g_context.ref_vy);
  g_context.ref_vz = 0.0f;
  g_context.ref_yaw_rate = 0.0f;
}

static void compute_landing_control(void)
{
  g_context.ref_vx = 0.0f;
  g_context.ref_vy = 0.0f;
  g_context.ref_vz = -LANDING_SPEED;
  g_context.ref_yaw_rate = 0.0f;
}

static void update_state_machine(void)
{
  const uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

  if (trajAbort != 0) {
    trajAbort = 0;
    zero_control();
    enter_state(TRAJ_IDLE, now_ms);
    return;
  }

  switch (g_context.state) {
    case TRAJ_IDLE:
      zero_control();
      if (g_context.data_received && g_context.trajectory.valid && trajStart != 0) {
        trajStart = 0;
        g_context.current_waypoint_index = 0;
        enter_state(TRAJ_TAKEOFF, now_ms);
      }
      break;

    case TRAJ_TAKEOFF: {
      zero_control();
      const float target_z = g_context.trajectory.waypoints[0].z;
      const float dz = target_z - g_context.drone_z;
      g_context.ref_vz = clampf(1.0f * dz, -TAKEOFF_SPEED, TAKEOFF_SPEED);

      if (fabsf(dz) < WAYPOINT_Z_TOLERANCE) {
        enter_state(TRAJ_FOLLOWING, now_ms);
      }
      break;
    }

    case TRAJ_FOLLOWING:
      compute_waypoint_control();
      detect_collision(now_ms);

      if (g_context.collision.collision_detected) {
        enter_state(TRAJ_COLLISION_DETECTED, now_ms);
      } else if (g_context.current_waypoint_index >= g_context.trajectory.num_waypoints) {
        enter_state(TRAJ_FINAL_FORWARD, now_ms);
      }
      break;

    case TRAJ_FINAL_FORWARD:
      compute_final_forward_control();
      detect_collision(now_ms);

      if (g_context.collision.collision_detected) {
        enter_state(TRAJ_COLLISION_DETECTED, now_ms);
      } else if ((now_ms - g_context.state_start_time_ms) > FINAL_FORWARD_TIMEOUT_MS) {
        // Safety fallback: if no collision is detected, still land instead of flying forever.
        enter_state(TRAJ_LANDING, now_ms);
      }
      break;

    case TRAJ_COLLISION_DETECTED:
      zero_control();
      if ((now_ms - g_context.state_start_time_ms) > COLLISION_STOP_TIME_MS) {
        enter_state(TRAJ_SERVO_CW, now_ms);
      }
      break;

    case TRAJ_SERVO_CW:
      zero_control();
      if (!g_context.servo_cw_sent) {
        servoControlMoveCw();
        g_context.servo_cw_sent = true;
        DEBUG_PRINT("Servo CW command sent\n");
      }
      if ((now_ms - g_context.state_start_time_ms) > SERVO_DEPLOY_WAIT_MS) {
        enter_state(TRAJ_BACKING_UP, now_ms);
      }
      break;

    case TRAJ_BACKING_UP:
      compute_backing_up_control();
      if ((now_ms - g_context.state_start_time_ms) > BACKUP_TIME_MS) {
        enter_state(TRAJ_LANDING, now_ms);
      }
      break;

    case TRAJ_LANDING:
      compute_landing_control();
      if (g_context.drone_z < 0.08f || (now_ms - g_context.state_start_time_ms) > LANDING_TIMEOUT_MS) {
        zero_control();
        enter_state(TRAJ_FINISHED, now_ms);
      }
      break;

    case TRAJ_FINISHED:
      zero_control();
      break;

    default:
      zero_control();
      enter_state(TRAJ_IDLE, now_ms);
      break;
  }
}

void trajectoryTrackingInit(void)
{
  memset(&g_context, 0, sizeof(g_context));
  g_context.state = TRAJ_IDLE;
  g_initialized = true;
  DEBUG_PRINT("Initialized\n");
}

void trajectoryTrackingUpdateState(float current_x, float current_y, float current_z, float current_yaw)
{
  if (!g_initialized) {
    return;
  }

  g_context.drone_x = current_x;
  g_context.drone_y = current_y;
  g_context.drone_z = current_z;
  g_context.drone_yaw = current_yaw;

  update_state_machine();
}

void trajectoryTrackingClearWaypoints(void)
{
  if (!g_initialized) {
    return;
  }

  memset(&g_context.trajectory, 0, sizeof(g_context.trajectory));
  g_context.data_received = false;
  g_context.current_waypoint_index = 0;
  reset_collision_detector();

  DEBUG_PRINT("Trajectory buffer cleared\n");
}

void trajectoryTrackingReceiveWaypoints(const uint8_t* data, uint32_t length)
{
  if (!g_initialized || data == NULL) {
    return;
  }

  const uint32_t waypoint_size = sizeof(TrajectoryWaypoint);
  uint32_t num_waypoints = length / waypoint_size;

  if (num_waypoints == 0U) {
    DEBUG_PRINT("No valid waypoint in packet, len=%lu\n", (unsigned long)length);
    return;
  }

  const uint16_t current_count = g_context.trajectory.num_waypoints;
  if (current_count >= MAX_WAYPOINTS) {
    DEBUG_PRINT("Waypoint buffer full\n");
    return;
  }

  uint32_t available = MAX_WAYPOINTS - current_count;
  if (num_waypoints > available) {
    num_waypoints = available;
  }

  memcpy(&g_context.trajectory.waypoints[current_count], data, num_waypoints * waypoint_size);
  g_context.trajectory.num_waypoints = current_count + (uint16_t)num_waypoints;
  g_context.trajectory.valid = true;
  g_context.data_received = true;

  if (g_context.state == TRAJ_IDLE || g_context.state == TRAJ_FINISHED) {
    g_context.current_waypoint_index = 0;
  }

  reset_collision_detector();

  DEBUG_PRINT("Appended %lu waypoint(s), total=%u. Set traj.start=1 to run.\n",
              (unsigned long)num_waypoints,
              (unsigned int)g_context.trajectory.num_waypoints);
}

void trajectoryTrackingGetControl(float* vx, float* vy, float* vz, float* yaw_rate)
{
  if (!g_initialized || vx == NULL || vy == NULL || vz == NULL || yaw_rate == NULL) {
    return;
  }

  *vx = g_context.ref_vx;
  *vy = g_context.ref_vy;
  *vz = g_context.ref_vz;
  *yaw_rate = g_context.ref_yaw_rate;
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
LOG_ADD(LOG_UINT16, wpIndex, &g_context.current_waypoint_index)
LOG_ADD(LOG_FLOAT, refVx, &g_context.ref_vx)
LOG_ADD(LOG_FLOAT, refVy, &g_context.ref_vy)
LOG_ADD(LOG_FLOAT, refVz, &g_context.ref_vz)
LOG_ADD(LOG_FLOAT, yawRate, &g_context.ref_yaw_rate)
LOG_ADD(LOG_UINT8, collision, &g_context.collision.collision_detected)
LOG_GROUP_STOP(traj)

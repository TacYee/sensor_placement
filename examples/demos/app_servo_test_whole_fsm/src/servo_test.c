/**
 * servo_test.c - Non-blocking app-layer servo driver on TX2 / PA2
 *
 * Param:
 *   servotest.cmd = 1  -> counter-clockwise position
 *   servotest.cmd = 2  -> clockwise position
 *
 * Wiring:
 *   Servo signal  -> TX2 / PA2
 *   Servo VCC     -> external 5V
 *   Servo GND     -> external GND
 *   Crazyflie GND -> external GND
 */

#include <stdint.h>
#include <stdbool.h>

#include "app.h"
#include "FreeRTOS.h"
#include "task.h"
#include "deck.h"
#include "param.h"
#include "debug.h"

#include "servo_test.h"

#define DEBUG_MODULE "SERVO"

#define SERVO_PERIOD_US      20000

// Adjust these two values if direction/range is not correct for your mechanism.
#define SERVO_CCW_US         1650
#define SERVO_CW_US          1000

#define SERVO_MIN_LIMIT_US   800
#define SERVO_MAX_LIMIT_US   2400
#define SERVO_MOVE_TIME_MS   800

#define SERVO_TASK_STACK     256
#define SERVO_TASK_PRI       1

static uint8_t servoCmd = SERVO_CMD_IDLE;       // exposed as param
static uint8_t requestedCmd = SERVO_CMD_IDLE;   // internal request from trajectory FSM
static uint8_t lastExecutedCmd = SERVO_CMD_IDLE;

static bool tx2Initialized = false;
static bool dwtInitialized = false;
static bool servoTaskStarted = false;

static void dwtDelayInit(void)
{
  if (!dwtInitialized) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    dwtInitialized = true;
  }
}

static void delayUs(uint32_t us)
{
  const uint32_t start = DWT->CYCCNT;
  const uint32_t cycles = (SystemCoreClock / 1000000UL) * us;

  while ((DWT->CYCCNT - start) < cycles) {
    __asm volatile ("nop");
  }
}

static void initTx2Lazy(void)
{
  if (!tx2Initialized) {
    dwtDelayInit();
    pinMode(DECK_GPIO_TX2, OUTPUT);
    digitalWrite(DECK_GPIO_TX2, 0);
    tx2Initialized = true;
  }
}

static uint16_t clampPulse(uint16_t pulseUs)
{
  if (pulseUs < SERVO_MIN_LIMIT_US) {
    pulseUs = SERVO_MIN_LIMIT_US;
  }
  if (pulseUs > SERVO_MAX_LIMIT_US) {
    pulseUs = SERVO_MAX_LIMIT_US;
  }
  return pulseUs;
}

void servoControlStop(void)
{
  if (tx2Initialized) {
    digitalWrite(DECK_GPIO_TX2, 0);
  }
}

static void servoWriteForMs(uint16_t pulseUs, uint32_t durationMs)
{
  initTx2Lazy();

  const uint16_t highUs = clampPulse(pulseUs);
  const uint32_t cycles = durationMs / 20U;

  for (uint32_t i = 0; i < cycles; i++) {
    vTaskSuspendAll();
    digitalWrite(DECK_GPIO_TX2, 1);
    delayUs(highUs);
    digitalWrite(DECK_GPIO_TX2, 0);
    xTaskResumeAll();

    const uint32_t lowUs = SERVO_PERIOD_US - highUs;
    const uint32_t lowMs = lowUs / 1000U;
    const uint32_t lowRemUs = lowUs % 1000U;

    if (lowMs > 1U) {
      vTaskDelay(M2T(lowMs));
    }
    if (lowRemUs > 0U) {
      delayUs(lowRemUs);
    }
  }

  servoControlStop();
}

static void servoTask(void *param)
{
  (void)param;

  vTaskDelay(M2T(500));

  // Default idle position: CW.
  servoWriteForMs(SERVO_CCW_US, SERVO_MOVE_TIME_MS);
  lastExecutedCmd = SERVO_CMD_CW;
  DEBUG_PRINT("Default position: CW\n");

  while (1) {
    uint8_t cmd = SERVO_CMD_IDLE;

    if (requestedCmd != SERVO_CMD_IDLE) {
      cmd = requestedCmd;
      requestedCmd = SERVO_CMD_IDLE;
    } else if (servoCmd != SERVO_CMD_IDLE) {
      cmd = servoCmd;
      servoCmd = SERVO_CMD_IDLE;
    }

    if (cmd == SERVO_CMD_CCW) {
      DEBUG_PRINT("Move CCW\n");
      servoWriteForMs(SERVO_CCW_US, SERVO_MOVE_TIME_MS);
      lastExecutedCmd = SERVO_CMD_CCW;
    } else if (cmd == SERVO_CMD_CW) {
      DEBUG_PRINT("Move CW\n");
      servoWriteForMs(SERVO_CW_US, SERVO_MOVE_TIME_MS);
      lastExecutedCmd = SERVO_CMD_CW;
    }

    vTaskDelay(M2T(20));
  }
}

void servoControlInit(void)
{
  if (!servoTaskStarted) {
    tx2Initialized = false;
    dwtInitialized = false;
    requestedCmd = SERVO_CMD_IDLE;
    servoCmd = SERVO_CMD_IDLE;
    xTaskCreate(servoTask, "servo", SERVO_TASK_STACK, NULL, SERVO_TASK_PRI, NULL);
    servoTaskStarted = true;
  }
}

void servoControlMoveCw(void)
{
  requestedCmd = SERVO_CMD_CW;
}

void servoControlMoveCcw(void)
{
  requestedCmd = SERVO_CMD_CCW;
}

uint8_t servoControlGetLastCommand(void)
{
  return lastExecutedCmd;
}

PARAM_GROUP_START(servotest)
PARAM_ADD(PARAM_UINT8, cmd, &servoCmd)
PARAM_GROUP_STOP(servotest)

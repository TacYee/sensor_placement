/**
 * servo_test.c - Minimal param-controlled servo test on TX2 / PA2
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

#define SERVO_CMD_IDLE  0
#define SERVO_CMD_CCW   1
#define SERVO_CMD_CW    2

#define SERVO_PERIOD_US      20000

// 这里先用你之前测试过的值。
// 如果方向反了，就交换这两个值。
// 如果两个角度太接近，把 CW 改成 1200 或 1000。
#define SERVO_CCW_US         1650
#define SERVO_CW_US          1500

#define SERVO_MIN_LIMIT_US   800
#define SERVO_MAX_LIMIT_US   2400
#define SERVO_MOVE_TIME_MS   800

static uint8_t servoCmd = SERVO_CMD_IDLE;

static bool tx2Initialized = false;
static bool dwtInitialized = false;

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

static void servoStop(void)
{
  if (tx2Initialized) {
    digitalWrite(DECK_GPIO_TX2, 0);
  }
}

static void servoWriteForMs(uint16_t pulseUs, uint32_t durationMs)
{
  initTx2Lazy();

  const uint16_t highUs = clampPulse(pulseUs);
  const uint32_t cycles = durationMs / 20;

  for (uint32_t i = 0; i < cycles; i++) {
    vTaskSuspendAll();
    digitalWrite(DECK_GPIO_TX2, 1);
    delayUs(highUs);

    digitalWrite(DECK_GPIO_TX2, 0);
    xTaskResumeAll();
    const uint32_t lowUs = SERVO_PERIOD_US - highUs;
    const uint32_t lowMs = lowUs / 1000;
    const uint32_t lowRemUs = lowUs % 1000;

    if (lowMs > 1) {
      vTaskDelay(M2T(lowMs));
    }

    if (lowRemUs > 0) {
      delayUs(lowRemUs);
    }
  }

  servoStop();
}

void appMain(void)
{
  vTaskDelay(M2T(5000));

  servoCmd = SERVO_CMD_IDLE;
  tx2Initialized = false;
  dwtInitialized = false;

  // Move servo to default CW/idle position once at startup
  servoWriteForMs(SERVO_CW_US, SERVO_MOVE_TIME_MS);

  while (1) {
    if (servoCmd != SERVO_CMD_IDLE) {
      const uint8_t cmd = servoCmd;
      servoCmd = SERVO_CMD_IDLE;

      if (cmd == SERVO_CMD_CCW) {
        servoWriteForMs(SERVO_CCW_US, SERVO_MOVE_TIME_MS);
      } else if (cmd == SERVO_CMD_CW) {
        servoWriteForMs(SERVO_CW_US, SERVO_MOVE_TIME_MS);
      }
    }

    vTaskDelay(M2T(50));
  }
}

PARAM_GROUP_START(servotest)
PARAM_ADD(PARAM_UINT8, cmd, &servoCmd)
PARAM_GROUP_STOP(servotest)
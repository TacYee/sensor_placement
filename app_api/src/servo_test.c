//**
 * servo_test.c - Crazyflie IO_3/PB4 Servo Test
 *
 * This app receives app_channel commands from Python and drives
 * a standard RC servo on IO_3 / PB4.
 *
 * Servo signal:
 *   50 Hz PWM
 *   20 ms period
 *   1000 us = one side
 *   1500 us = center
 *   2000 us = other side
 */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"
#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"
#include "app_channel.h"
#include "deck.h"

#define DEBUG_MODULE "SERVO_TEST"

// Command codes from Python
#define TEST_CMD_CENTER  0x01
#define TEST_CMD_TOGGLE  0x02
#define TEST_CMD_SWEEP   0x03
#define TEST_CMD_INFO    0x04

// IO_3 pin
static uint32_t io3_pin = DECK_GPIO_IO3;
static bool io3_enabled = false;

// -----------------------------------------------------------------------------
// Approximate microsecond delay
//
// FreeRTOS vTaskDelay() is millisecond-level, so for the 1-2 ms high pulse
// we use a short busy-wait loop.
//
// If your servo does not move correctly, this constant may need tuning.
// Larger number = longer pulse.
// -----------------------------------------------------------------------------
static void delay_us_approx(uint32_t us)
{
  volatile uint32_t count;

  // Approximate value for STM32F405-class MCU.
  // This is not perfect hardware PWM, but good enough for a first servo test.
  for (count = 0; count < us * 40; count++) {
    __asm volatile ("nop");
  }
}

static void init_io3(void)
{
  io3_pin = DECK_GPIO_IO3;

  // Do not block on deckIsUsingIO3() here. For this test app we directly
  // configure IO_3 as an output.
  pinMode(io3_pin, OUTPUT);
  digitalWrite(io3_pin, 0);

  io3_enabled = true;

  DEBUG_PRINT("IO_3 / PB4 initialized as OUTPUT\n");
  DEBUG_PRINT("Servo signal pin: IO_3 / PB4\n");
}

static void set_io3(bool state)
{
  if (io3_enabled) {
    digitalWrite(io3_pin, state ? 1 : 0);
  }
}

// -----------------------------------------------------------------------------
// Generate servo PWM
//
// A standard servo expects repeated 20 ms frames.
// high_us should normally be:
//   1000 = left / one end
//   1500 = center
//   2000 = right / other end
// -----------------------------------------------------------------------------
static void servo_pwm_us(uint16_t high_us, uint32_t duration_ms)
{
  if (!io3_enabled) {
    DEBUG_PRINT("ERROR: IO_3 not enabled\n");
    return;
  }

  if (high_us < 800) {
    high_us = 800;
  }

  if (high_us > 2200) {
    high_us = 2200;
  }

  uint32_t cycles = duration_ms / 20;

  DEBUG_PRINT("Servo PWM: high=%u us, duration=%lu ms, cycles=%lu\n",
              high_us, duration_ms, cycles);

  for (uint32_t i = 0; i < cycles; i++) {
    // High pulse: 1.0-2.0 ms
    set_io3(true);
    delay_us_approx(high_us);

    // Low part of 20 ms frame
    set_io3(false);

    uint32_t low_ms = 20 - (high_us / 1000);
    if (low_ms < 1) {
      low_ms = 1;
    }

    vTaskDelay(M2T(low_ms));
  }

  set_io3(false);
}

// -----------------------------------------------------------------------------
// Test commands
// -----------------------------------------------------------------------------

static void test_center(void)
{
  DEBUG_PRINT(">>> Test: Servo CENTER <<<\n");
  DEBUG_PRINT("Sending 1.5 ms PWM for 2 seconds\n");

  servo_pwm_us(1500, 2000);

  DEBUG_PRINT("Center test complete\n\n");
}

static void test_toggle(void)
{
  DEBUG_PRINT(">>> Test: Servo LEFT / RIGHT / CENTER <<<\n");

  DEBUG_PRINT("Move to side A: 1.0 ms\n");
  servo_pwm_us(1000, 1500);

  vTaskDelay(M2T(500));

  DEBUG_PRINT("Move to side B: 2.0 ms\n");
  servo_pwm_us(2000, 1500);

  vTaskDelay(M2T(500));

  DEBUG_PRINT("Move to center: 1.5 ms\n");
  servo_pwm_us(1500, 1500);

  DEBUG_PRINT("Toggle test complete\n\n");
}

static void test_sweep(void)
{
  DEBUG_PRINT(">>> Test: Servo SWEEP <<<\n");

  for (int cycle = 0; cycle < 3; cycle++) {
    DEBUG_PRINT("Sweep cycle %d\n", cycle + 1);

    servo_pwm_us(1000, 700);
    servo_pwm_us(1250, 700);
    servo_pwm_us(1500, 700);
    servo_pwm_us(1750, 700);
    servo_pwm_us(2000, 700);
    servo_pwm_us(1750, 700);
    servo_pwm_us(1500, 700);
    servo_pwm_us(1250, 700);
  }

  servo_pwm_us(1500, 1000);

  DEBUG_PRINT("Sweep test complete\n\n");
}

static void print_status(void)
{
  DEBUG_PRINT(">>> IO_3 Servo Test Status <<<\n");
  DEBUG_PRINT("IO_3 enabled: %s\n", io3_enabled ? "YES" : "NO");
  DEBUG_PRINT("Pin: IO_3 / PB4\n");
  DEBUG_PRINT("Signal voltage: 3.3V logic\n");
  DEBUG_PRINT("Servo PWM: 50Hz, 1000-2000us pulse\n");
  DEBUG_PRINT("Commands:\n");
  DEBUG_PRINT("  0x01 = Center, 1.5ms PWM\n");
  DEBUG_PRINT("  0x02 = Left / Right / Center\n");
  DEBUG_PRINT("  0x03 = Sweep\n");
  DEBUG_PRINT("  0x04 = Status\n");
  DEBUG_PRINT("\n");
}

// -----------------------------------------------------------------------------
// App main
// -----------------------------------------------------------------------------

void appMain()
{
  DEBUG_PRINT("\n");
  DEBUG_PRINT("========================================\n");
  DEBUG_PRINT(" Crazyflie Servo Test on IO_3 / PB4\n");
  DEBUG_PRINT("========================================\n\n");

  init_io3();
  print_status();

  DEBUG_PRINT("IMPORTANT WIRING:\n");
  DEBUG_PRINT("  Servo signal -> IO_3 / PB4\n");
  DEBUG_PRINT("  Servo VCC    -> external 5V supply\n");
  DEBUG_PRINT("  Servo GND    -> external supply GND\n");
  DEBUG_PRINT("  Crazyflie GND must connect to external supply GND\n\n");

  DEBUG_PRINT("Waiting for commands from Python app_channel...\n\n");

  while (1) {
    uint8_t data[16];
    uint32_t length = 0;

    if (appchannelReceiveDataPacket(data, &length)) {
      if (length > 0) {
        uint8_t cmd = data[0];

        DEBUG_PRINT("Received command: 0x%02X\n", cmd);

        switch (cmd) {
          case TEST_CMD_CENTER:
            test_center();
            break;

          case TEST_CMD_TOGGLE:
            test_toggle();
            break;

          case TEST_CMD_SWEEP:
            test_sweep();
            break;

          case TEST_CMD_INFO:
            print_status();
            break;

          default:
            DEBUG_PRINT("Unknown command: 0x%02X\n", cmd);
            break;
        }
      }
    }

    vTaskDelay(M2T(20));
  }
}
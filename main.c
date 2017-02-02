/*
 * Copyright (C) 2016 https://www.brobwind.com
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"

#include "shell.h"
#include "chprintf.h"


#define WS2812B_PORT                    GPIOA
#define WS2812B_PIN                     GPIOA_PIN9
#define WS2812B_NUMBER                  7

#define EVENT_WS2812B_UPDATE            1
event_source_t upd_event;

struct Ws2812bCtx {
    mutex_t mtx;
    uint32_t fb[WS2812B_NUMBER];

    stm32_gpio_t *port;
    uint16_t pin;
    uint16_t MASK;
    uint16_t ctl[WS2812B_NUMBER * 24];
    uint8_t demo;
} ledCtx;

static void pwmc1cb(PWMDriver *pwmp) {
  pwmp->tim->CR1 &= ~TIM_CR1_CEN;
}

/*
 * WS2812B thread, times are in milliseconds.
 * Steal from: https://github.com/omriiluz/WS2812B-LED-Driver-ChibiOS
 */
static THD_WORKING_AREA(waWs2812b, 256);
static __attribute__((noreturn)) THD_FUNCTION(Ws2812b, arg) {
  (void)arg;
  // Configure pwm timers:
  // - timer 3 as master, active for data transmission and inactive to disable transmission during reset period (50ms)
  // - timer 1 as slave, during active time creates a 1.25 us signal, with duty cycle controlled by frame buffer values
  const PWMConfig pwmd3cfg = {
    48000000 / 60, /* 800Khz PWM clock frequency. 1/60 of PWMC3 */
    (48000000 / 60) * 0.05, /* Total period is 50ms (20FPS), including leds cycles + reset length for ws2812b and FB writes */
    NULL,
    {
      { PWM_OUTPUT_ACTIVE_HIGH, NULL },
      { PWM_OUTPUT_DISABLED, NULL },
      { PWM_OUTPUT_ACTIVE_HIGH, pwmc1cb },
      { PWM_OUTPUT_DISABLED, NULL }
    },
    TIM_CR2_MMS_2, /* master mode selection */
    0,
  };
  const PWMConfig pwmd1cfg = {
    48000000, /* 48Mhz PWM clock frequency. */
    60, /* 60 cycles period (0.25 us per period @48Mhz */
    NULL,
    {
      { PWM_OUTPUT_ACTIVE_HIGH, NULL },
      { PWM_OUTPUT_ACTIVE_HIGH, NULL },
      { PWM_OUTPUT_DISABLED, NULL },
      { PWM_OUTPUT_DISABLED, NULL }
    },
    0,
    0,
  };

  chRegSetThreadName("Ws2812b");

  event_listener_t el;
  chEvtRegister(&upd_event, &el, EVENT_WS2812B_UPDATE);

  /* WS2812B framebuffer initialize */
  ledCtx.MASK = 1 << ledCtx.pin;
  memset(ledCtx.fb, 0x00, sizeof(ledCtx.fb));

  /*
   * WS2812B pins setup.
   */
  palSetPadMode(ledCtx.port, ledCtx.pin, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
  palClearPad(ledCtx.port, ledCtx.pin);

  /* Send reset code: should > 50us */
  chThdSleepMilliseconds(1);

  // DMA stream 2, triggered by TIM1_CH1 pwm signal. if FB indicates, reset output value early to indicate "0" bit to ws2812
  dmaStreamAllocate(STM32_DMA1_STREAM2, 10, NULL, NULL);
  dmaStreamSetPeripheral(STM32_DMA1_STREAM2, &(ledCtx.port->BSRR.H.clear));
  dmaStreamSetMemory0(STM32_DMA1_STREAM2, ledCtx.ctl);
  dmaStreamSetTransactionSize(STM32_DMA1_STREAM2, sizeof(ledCtx.ctl) / sizeof(ledCtx.ctl[0]));
  dmaStreamSetMode(STM32_DMA1_STREAM2,
      STM32_DMA_CR_DIR_M2P | STM32_DMA_CR_MINC | STM32_DMA_CR_PSIZE_HWORD |
      STM32_DMA_CR_MSIZE_HWORD | STM32_DMA_CR_CIRC);

  // DMA stream 5, triggered by TIM1_UP event. output high at beginning of signal
  dmaStreamAllocate(STM32_DMA1_STREAM5, 10, NULL, NULL);
  dmaStreamSetPeripheral(STM32_DMA1_STREAM5, &(ledCtx.port->BSRR.H.set));
  dmaStreamSetMemory0(STM32_DMA1_STREAM5, &ledCtx.MASK);
  dmaStreamSetTransactionSize(STM32_DMA1_STREAM5, 1);
  dmaStreamSetMode(STM32_DMA1_STREAM5, STM32_DMA_CR_TEIE |
      STM32_DMA_CR_DIR_M2P | STM32_DMA_CR_PSIZE_HWORD |
      STM32_DMA_CR_MSIZE_HWORD | STM32_DMA_CR_CIRC);

  // DMA stream 3, triggered by TIM1_CH2 event. reset output value late to indicate "1" bit to ws2812.
  // always triggers but no affect if dma stream 2 already change output value to 0
  dmaStreamAllocate(STM32_DMA1_STREAM3, 10, NULL, NULL);
  dmaStreamSetPeripheral(STM32_DMA1_STREAM3, &(ledCtx.port->BSRR.H.clear));
  dmaStreamSetMemory0(STM32_DMA1_STREAM3, &ledCtx.MASK);
  dmaStreamSetTransactionSize(STM32_DMA1_STREAM3, 1);
  dmaStreamSetMode(STM32_DMA1_STREAM3,
      STM32_DMA_CR_DIR_M2P | STM32_DMA_CR_PSIZE_HWORD |
      STM32_DMA_CR_MSIZE_HWORD | STM32_DMA_CR_CIRC);

  pwmStart(&PWMD3, &pwmd3cfg);
  pwmStart(&PWMD1, &pwmd1cfg);
  // set pwm1 as slave, triggerd by pwm3 oc1 event. disables pwmd3 for synchronization.
  PWMD1.tim->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_2 | TIM_SMCR_TS_1; // ITR2(TS=010) TIM3
  PWMD3.tim->CR1 &= ~TIM_CR1_CEN;

  // set pwm values.
  // 17 (duty in ticks) / 60 (period in ticks) * 1.25us (period in S) = 0.354 us
  pwmEnableChannel(&PWMD1, 0, 17);
  // 43 (duty in ticks) / 60 (period in ticks) * 1.25us (period in S) = 0.896 us
  pwmEnableChannel(&PWMD1, 1, 43);

  // active during transfer of leds * 24 bytes
  pwmEnableChannel(&PWMD3, 0, WS2812B_NUMBER * 24);

  pwmEnableChannelNotification(&PWMD3, 2);
  pwmEnableChannel(&PWMD3, 2, (WS2812B_NUMBER + 2) * 24);

  // stop and reset counters for synchronization
  PWMD3.tim->CNT = 0;
  // Slave (TIM1) needs to "update" immediately after master (TIM3) start in order to start in sync.
  // this initial sync is crucial for the stability of the run
  PWMD1.tim->CNT = 59;
  PWMD1.tim->DIER |= TIM_DIER_CC1DE | TIM_DIER_CC2DE | TIM_DIER_UDE;
  dmaStreamEnable(STM32_DMA1_STREAM2);
  dmaStreamEnable(STM32_DMA1_STREAM5);
  dmaStreamEnable(STM32_DMA1_STREAM3);

  while (TRUE) {
    uint32_t idx;

    chMtxLock(&ledCtx.mtx);
    for (idx = 0; idx < WS2812B_NUMBER * 24; idx++) {
      if (ledCtx.fb[idx / 24] & (1 << (23 - idx % 24))) {
        ledCtx.ctl[idx] = 0;
      } else {
        ledCtx.ctl[idx] = ledCtx.MASK;
      }
    }
    chMtxUnlock(&ledCtx.mtx);

    // all systems go! both timers and all channels are configured to resonate
    // in complete sync without any need for CPU cycles (only DMA and timers)
    // start pwm3 for system to start resonating
    PWMD3.tim->CR1 |= TIM_CR1_CEN;
    while (PWMD3.tim->CR1 & TIM_CR1_CEN) { }

    chEvtWaitAnyTimeout(ALL_EVENTS, TIME_INFINITE);
  }
}

/*===========================================================================*/
/* DemoCtrl thread related.                                                  */
/*===========================================================================*/

#define MAX_BRIGHTNESS                    15

// http://citeseerx.ist.psu.edu/viewdoc/summary?doi=10.1.1.43.3639 (modified)
/* Generates numbers between 0 and 1. */
static float lfsr113(void) {
    static uint32_t z1 = 127, z2 = 127, z3 = 127, z4 = 127;
    uint32_t b;

    b = (((z1 << 6) ^ z1) >> 13);
    z1 = (((z1 & 4294967294U) << 18) ^ b);
    b = (((z2 << 2) ^ z2) >> 27);
    z2 = (((z2 & 4294967288U) << 2) ^ b);
    b = (((z3 << 13) ^ z3) >> 21);
    z3 = (((z3 & 4294967280U) << 7) ^ b);
    b = (((z4 << 3) ^ z4) >> 12);
    z4 = (((z4 & 4294967168U) << 13) ^ b);

    return ((z1 ^ z2 ^ z3 ^ z4) * 2.3283064365387e-10);
}

/*
 * Demo thread, times are in milliseconds.
 */
static THD_WORKING_AREA(waDemoCtrl, 96);
static __attribute__((noreturn)) THD_FUNCTION(DemoCtrl, arg) {
  (void)arg;

  chRegSetThreadName("DemoCtrl");

  while (TRUE) {
    chThdSleepMilliseconds(50);

    if (ledCtx.demo) {
      uint32_t idx, r, g, b;

      chMtxLock(&ledCtx.mtx);
      for (idx = 0; idx < WS2812B_NUMBER; idx++) {
        r = lfsr113() * MAX_BRIGHTNESS;
        g = lfsr113() * MAX_BRIGHTNESS;
        b = lfsr113() * MAX_BRIGHTNESS;
        ledCtx.fb[idx] = g << 16 | r << 8 | b;
      }
      chMtxUnlock(&ledCtx.mtx);

      chEvtBroadcastFlags(&upd_event, EVENT_WS2812B_UPDATE);
    }
  }
}

/*===========================================================================*/
/* Command line related.                                                     */
/*===========================================================================*/

#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(512)

static int hexToInt(const char *p) {
  int idx, val;
  for (idx = 0, val = 0; idx < 2; idx++) {
    if (p[idx] >= 'a' && p[idx] <= 'f') {
      val = val * 16 + p[idx] - 'a' + 10;
    } else if (p[idx] >= 'A' && p[idx] <= 'F') {
      val = val * 16 + p[idx] - 'A' + 10;
    } else if (p[idx] >= '0' && p[idx] <= '9') {
      val = val * 16 + p[idx] - '0';
    }
  }

  return val;
}

static void cmd_led(BaseSequentialStream *chp, int argc, char *argv[]) {
  if (argc != 3) return;

  uint8_t r, g, b;
  uint32_t idx;

  r = hexToInt(argv[0]);
  g = hexToInt(argv[1]);
  b = hexToInt(argv[2]);

  /* Disable WS2812B demo controller */
  ledCtx.demo = 0;

  chMtxLock(&ledCtx.mtx);
  for (idx = 0; idx < WS2812B_NUMBER; idx++) {
    ledCtx.fb[idx] = g << 16 | r << 8 | b;
  }
  chMtxUnlock(&ledCtx.mtx);

  chprintf(chp, "led: r=%d, g=%d, b=%d\r\n", r, g, b);

  chEvtBroadcastFlags(&upd_event, EVENT_WS2812B_UPDATE);
}

static const ShellCommand commands[] = {
  { "led", cmd_led },
  { NULL, NULL }
};

static const ShellConfig shellcfg = {
  (BaseSequentialStream *)&SD1,
  commands
};

/*===========================================================================*/
/* Main                                                                      */
/*===========================================================================*/

/*
 * Application entry point.
 */
int __attribute__((noreturn)) main(void)
{
  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  chEvtObjectInit(&upd_event);
  chMtxObjectInit(&ledCtx.mtx);

  /*
   * Activates the serial driver 1 using the driver default configuration.
   */
  sdStart(&SD1, NULL);

  /*
   * Shell manager initialization.
   */
  shellInit();

  ledCtx.port = WS2812B_PORT;
  ledCtx.pin = WS2812B_PIN;
  ledCtx.demo = 1;

  /*
   * Creates the WS2812B thread.
   */
  chThdCreateStatic(waWs2812b, sizeof(waWs2812b), NORMALPRIO, Ws2812b, NULL);

  /*
   * Creates the DemoCtrl thread.
   */
  chThdCreateStatic(waDemoCtrl, sizeof(waDemoCtrl), NORMALPRIO, DemoCtrl, NULL);

  while(TRUE){
    thread_t *shelltp = chThdCreateFromHeap(NULL, SHELL_WA_SIZE,
        "shell", NORMALPRIO + 1, shellThread, (void *)&shellcfg);
    chThdWait(shelltp); /* Waiting termination. */

    chThdSleepMilliseconds(500);
  }
}

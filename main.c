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
} ledCtx;

static void pwmc1cb(PWMDriver *pwmp) {
  pwmp->tim->CR1 &= ~TIM_CR1_CEN;
}

/*
 * WS2812B thread, times are in milliseconds.
 */
static THD_WORKING_AREA(waWs2812b, 256);
static __attribute__((noreturn)) THD_FUNCTION(Ws2812b, arg) {
  (void)arg;
  // Configure pwm timers:
  // - timer 1 as master, active for data transmission and inactive to disable transmission during reset period (50ms)
  // - timer 3 as slave, during active time creates a 1.25 us signal, with duty cycle controlled by frame buffer values
  const PWMConfig pwmc1 = {
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
  /* master mode selection */
  const PWMConfig pwmc3 = {
    48000000, /* 48Mhz PWM clock frequency. */
    60, /* 60 cycles period (0.25 us per period @48Mhz */
    NULL,
    {
      { PWM_OUTPUT_ACTIVE_HIGH, NULL },
      { PWM_OUTPUT_DISABLED, NULL },
      { PWM_OUTPUT_ACTIVE_HIGH, NULL },
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

  // DMA stream 2, triggered by channel3 pwm signal. if FB indicates, reset output value early to indicate "0" bit to ws2812
  dmaStreamAllocate(STM32_DMA1_STREAM2, 10, NULL, NULL);
  dmaStreamSetPeripheral(STM32_DMA1_STREAM2, &(ledCtx.port->BSRR.H.clear));
  dmaStreamSetMemory0(STM32_DMA1_STREAM2, ledCtx.ctl);
  dmaStreamSetTransactionSize(STM32_DMA1_STREAM2, sizeof(ledCtx.ctl) / sizeof(ledCtx.ctl[0]));
  dmaStreamSetMode(
      STM32_DMA1_STREAM2,
      STM32_DMA_CR_DIR_M2P | STM32_DMA_CR_MINC | STM32_DMA_CR_PSIZE_HWORD |
      STM32_DMA_CR_MSIZE_HWORD | STM32_DMA_CR_CIRC | STM32_DMA_CR_PL(2));

  // DMA stream 3, triggered by pwm update event. output high at beginning of signal
  dmaStreamAllocate(STM32_DMA1_STREAM3, 10, NULL, NULL);
  dmaStreamSetPeripheral(STM32_DMA1_STREAM3, &(ledCtx.port->BSRR.H.set));
  dmaStreamSetMemory0(STM32_DMA1_STREAM3, &ledCtx.MASK);
  dmaStreamSetTransactionSize(STM32_DMA1_STREAM3, 1);
  dmaStreamSetMode(
      STM32_DMA1_STREAM3, STM32_DMA_CR_TEIE |
      STM32_DMA_CR_DIR_M2P | STM32_DMA_CR_PSIZE_HWORD |
      STM32_DMA_CR_MSIZE_HWORD | STM32_DMA_CR_CIRC | STM32_DMA_CR_PL(3));

  // DMA stream 4, triggered by channel1 update event. reset output value late to indicate "1" bit to ws2812.
  // always triggers but no affect if dma stream 2 already change output value to 0
  dmaStreamAllocate(STM32_DMA1_STREAM4, 10, NULL, NULL);
  dmaStreamSetPeripheral(STM32_DMA1_STREAM4, &(ledCtx.port->BSRR.H.clear));
  dmaStreamSetMemory0(STM32_DMA1_STREAM4, &ledCtx.MASK);
  dmaStreamSetTransactionSize(STM32_DMA1_STREAM4, 1);
  dmaStreamSetMode(
      STM32_DMA1_STREAM4,
      STM32_DMA_CR_DIR_M2P | STM32_DMA_CR_PSIZE_HWORD |
      STM32_DMA_CR_MSIZE_HWORD | STM32_DMA_CR_CIRC | STM32_DMA_CR_PL(3));

  pwmStart(&PWMD1, &pwmc1);
  pwmStart(&PWMD3, &pwmc3);
  // set pwm3 as slave, triggerd by pwm1 oc1 event. disables pwmd1 for synchronization.
  PWMD3.tim->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_2; // ITR0(TS=000)
  PWMD1.tim->CR1 &= ~TIM_CR1_CEN;

  // set pwm values.
  // 17 (duty in ticks) / 60 (period in ticks) * 1.25us (period in S) = 0.354 us
  pwmEnableChannel(&PWMD3, 2, 17);
  // 43 (duty in ticks) / 60 (period in ticks) * 1.25us (period in S) = 0.896 us
  pwmEnableChannel(&PWMD3, 0, 43);

  // active during transfer of leds * 24 bytes
  pwmEnableChannel(&PWMD1, 0, WS2812B_NUMBER * 24);

  pwmEnableChannelNotification(&PWMD1, 2);
  pwmEnableChannel(&PWMD1, 2, (WS2812B_NUMBER + 2) * 24);

  // stop and reset counters for synchronization
  PWMD1.tim->CNT = 0;
  // Slave (TIM3) needs to "update" immediately after master (TIM1) start in order to start in sync.
  // this initial sync is crucial for the stability of the run
  PWMD3.tim->CNT = 59;
  PWMD3.tim->DIER |= TIM_DIER_CC3DE | TIM_DIER_CC1DE | TIM_DIER_UDE;
  dmaStreamEnable(STM32_DMA1_STREAM3);
  dmaStreamEnable(STM32_DMA1_STREAM4);
  dmaStreamEnable(STM32_DMA1_STREAM2);

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
    // start pwm1 for system to start resonating
    PWMD1.tim->CR1 |= TIM_CR1_CEN;
    while (PWMD1.tim->CR1 & TIM_CR1_CEN) {}

    chEvtWaitAnyTimeout(ALL_EVENTS, TIME_INFINITE);
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
  /*
   * Creates the WS2812B thread.
   */
  chThdCreateStatic(waWs2812b, sizeof(waWs2812b), NORMALPRIO, Ws2812b, NULL);

  while(TRUE){
    thread_t *shelltp = chThdCreateFromHeap(NULL, SHELL_WA_SIZE,
        "shell", NORMALPRIO + 1, shellThread, (void *)&shellcfg);
    chThdWait(shelltp); /* Waiting termination. */

    chThdSleepMilliseconds(500);
  }
}

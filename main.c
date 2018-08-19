/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

// Adapted by Ruvim Kondratyev from sample code by Giovanni Di Sirio.
// From 6.8.18 to 11.8.18;

#include "ch.h"
#include "hal.h"
#include "rt_test_root.h"
#include "oslib_test_root.h"

#include "string.h"
#include "math.h"

#define USART_CR1_9BIT_WORD (1<<12)
#define USART_CR1_PARITY_SET (1<<10)
#define USART_CR1_EVEN_PARITY (0<<9)

// Serial stuff:
static SerialConfig sd2cfg = {
9600, //115200,
USART_CR1_9BIT_WORD | USART_CR1_PARITY_SET | USART_CR1_EVEN_PARITY,
USART_CR2_STOP1_BITS | USART_CR2_LINEN,
0
};

/*
 * Green LED blinker thread, times are in milliseconds.
 */
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  chRegSetThreadName("blinker");
  while (true) {
    palClearPad(GPIOA, GPIOA_LED_GREEN);
    chThdSleepMilliseconds(1);
    palSetPad(GPIOA, GPIOA_LED_GREEN);
    chThdSleepMilliseconds(1);
  }
}

// A thread to test the output on one of the pins. Also a blinker.
static THD_WORKING_AREA(waThread2, 128);
static THD_FUNCTION(Thread2, arg) {
  (void) arg;
  chRegSetThreadName("VoltageTest");
  palSetPadMode(GPIOA, GPIOA_ARD_D8, PAL_MODE_OUTPUT_PUSHPULL);
  while (true) {
    palClearPad(GPIOA, GPIOA_ARD_D8);
    chThdSleepMilliseconds(10);
    palSetPad(GPIOA, GPIOA_ARD_D8);
    chThdSleepMilliseconds(10);
  }
}


// The following has been copied from the sample code under (testhal/STM32/STM32F4xx/ADC/main.c)
// It is for reading analog voltages.

#define ADC_GRP1_NUM_CHANNELS   2
#define ADC_GRP1_BUF_DEPTH      1

static adcsample_t samples1[ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH];

static const ADCConversionGroup adcgrpcfg1 = {
  FALSE,
  ADC_GRP1_NUM_CHANNELS,
  NULL,
  NULL,
  ~ADC_CR1_RES,                        /* CR1 */
  ADC_CR2_SWSTART,          /* CR2 */
  ADC_SMPR1_SMP_AN11(ADC_SAMPLE_56) | ADC_SMPR1_SMP_AN10 (ADC_SAMPLE_56),
  0,                        /* SMPR2 */
  0,                        /* SQR1 */
  0,                        /* SQR2 */
  ADC_SQR3_SQ2_N(ADC_CHANNEL_IN11) | ADC_SQR3_SQ1_N (ADC_CHANNEL_IN10)
};

// The following is code by Ruvim Kondratyev for quickly calculating mean and variance of a data set:
#define STD_CALC_EXTRA_BITS 2
uint32_t calc_mean_sum (uint16_t buf [], uint8_t log2_of_count, uint16_t * dest_min, uint16_t * dest_max) {
  uint32_t sum = buf[0];
  size_t count = 1 << log2_of_count;
  size_t i;
  *dest_min = buf[0];
  *dest_max = buf[0];
  for (i = 1; i < count; i++) {
    if (buf[i] > *dest_max)
      *dest_max = buf[i];
    if (buf[i] < *dest_min)
      *dest_min = buf[i];
    sum += buf[i];
  }
  return sum;
}
uint16_t calc_var_sum (uint16_t buf [], uint8_t log2_of_count, uint32_t mean_sum) {
  uint32_t std_sum = 0;
  uint32_t mean = mean_sum >> (log2_of_count - STD_CALC_EXTRA_BITS);
  size_t count = 1 << log2_of_count;
  size_t i;
  for (i = 0; i < count; i++) {
    uint32_t x = (buf[i] << STD_CALC_EXTRA_BITS) - mean;
    std_sum += (x * x) >> (STD_CALC_EXTRA_BITS * 2);
  }
  return std_sum;
}

// Structure for holding mean, variance, and min/max of voltage data:
typedef struct {
  uint16_t v_mean;
  uint16_t v_var;
  uint16_t v_min;
  uint16_t v_max;
} VOLTAGE_STAT;

// Structure for organizing multiple sets of voltage statistics data:
typedef struct {
  VOLTAGE_STAT v1; // Driver voltage. Gets amplified and sent to the circuit component.
  VOLTAGE_STAT v2; // Response voltage. For reading some parameter about how the component responds.
} V12_STATS;

// Variables relating to calculating voltage statistics:
mutex_t vstat_mtx; // Control thread access to vstat_obj.
V12_STATS vstat_obj; // Holds actual voltage statistics data.
uint8_t vstat_update_count = 0; // Counts updates. Mainly for checking if there's any new stats available.

// Buffer size, or number of samples to collect before doing statistics on the samples (log2, so 12 means buffer is 2^12):
#define V_BUF_SZ_LOG2 12
// Extra bits to keep. We have 12-bit resolution and 16-bit half-words, so it makes averages more accurate to keep 4 extra:
#define V_RESULT_KEEP_LSB_EXTRA_BITS 4
// Thread3 is the one that does the ADC conversion and saves statistics results to vstat_obj:
static THD_WORKING_AREA(waThread3, 128);
static THD_FUNCTION(Thread3, arg) {
  (void) arg;
  chRegSetThreadName("AnalogReader");
  static uint16_t v_a [1 << V_BUF_SZ_LOG2];
  static uint16_t v_b [1 << V_BUF_SZ_LOG2];
  size_t i;
  while (true) {
    chThdSleepMicroseconds (1);
    adcConvert(&ADCD1, &adcgrpcfg1, samples1, ADC_GRP1_BUF_DEPTH);
    v_a[0] = samples1[0];
    v_b[1] = samples1[1];
    for (i = 1; i < (1 << V_BUF_SZ_LOG2); i++) {
      chThdSleepMicroseconds (1);
      adcConvert(&ADCD1, &adcgrpcfg1, samples1, ADC_GRP1_BUF_DEPTH);
      v_a[i] = samples1[0];
      v_b[i] = samples1[1];
    }
    uint16_t a_min = 0, a_max = 0, b_min = 0, b_max = 0;
    uint32_t a_mean_sum = calc_mean_sum (v_a, V_BUF_SZ_LOG2, &a_min, &a_max);
    uint32_t a_var_sum = calc_var_sum (v_a, V_BUF_SZ_LOG2, a_mean_sum);
    uint32_t b_mean_sum = calc_mean_sum (v_b, V_BUF_SZ_LOG2, &b_min, &b_max);
    uint32_t b_var_sum = calc_var_sum (v_b, V_BUF_SZ_LOG2, b_mean_sum);
    chMtxLock (&vstat_mtx);
    vstat_obj.v1.v_mean = a_mean_sum >> (V_BUF_SZ_LOG2 - V_RESULT_KEEP_LSB_EXTRA_BITS);
    vstat_obj.v1.v_var = a_var_sum >> (V_BUF_SZ_LOG2 - V_RESULT_KEEP_LSB_EXTRA_BITS);
    vstat_obj.v1.v_min = a_min;
    vstat_obj.v1.v_max = a_max;
    vstat_obj.v2.v_mean = b_mean_sum >> (V_BUF_SZ_LOG2 - V_RESULT_KEEP_LSB_EXTRA_BITS);
    vstat_obj.v2.v_var = b_var_sum >> (V_BUF_SZ_LOG2 - V_RESULT_KEEP_LSB_EXTRA_BITS);
    vstat_obj.v2.v_min = b_min;
    vstat_obj.v2.v_max = b_max;
    vstat_update_count++; // It's okay if it rolls over on overflow.
    chMtxUnlock (&vstat_mtx);
  }
}

// PWM stuff. Taken from a code sample, and adapted for the experiment:

#define MY_PWM5_CLOCKFREQ 8000000
#define MY_PWM5_PERIOD_CYCLES 100
static PWMConfig pwmcfg = {
  MY_PWM5_CLOCKFREQ,                                    /* 8000kHz PWM clock frequency.     */
  MY_PWM5_PERIOD_CYCLES,                                    /* Initial PWM period .1ms.         */
  NULL,                                     /* Period callback.               */
  {
   {PWM_OUTPUT_ACTIVE_HIGH, NULL},          /* CH1 mode and callback.         */
   {PWM_OUTPUT_ACTIVE_HIGH, NULL},             /* CH2 mode and callback.         */
   {PWM_OUTPUT_DISABLED, NULL},             /* CH3 mode and callback.         */
   {PWM_OUTPUT_DISABLED, NULL}              /* CH4 mode and callback.         */
  },
  0,                                        /* Control Register 2.            */
  0                                         /* DMA/Interrupt Enable Register. */
};
static PWMConfig pwmslowcfg = {
  100000, // 100 kHz
  100, // 1ms initial.
  NULL,
  {
   {PWM_OUTPUT_ACTIVE_HIGH, NULL},
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_DISABLED, NULL}
  },
  0,
  0
};

// A semaphore to control access to the ADC parts and the serial writing:
binary_semaphore_t experiment_sem; // To keep track of when the experiment is running.

// Serial signal functions written by Ruvim Kondratyev:

// Sends voltage statistic results to the serial:
uint8_t sendSerialData () {
  V12_STATS my_copy;
  static uint8_t last_update_count = (uint8_t) (-1);
  uint8_t now = vstat_update_count; // It's an atomic operation, so this is OK to do without a mutex.
  if (now != last_update_count) {
    chMtxLock (&vstat_mtx);
    my_copy = vstat_obj;
    chMtxUnlock (&vstat_mtx);
    sdWrite (&SD2, (uint8_t *) &my_copy, sizeof (V12_STATS));
    last_update_count = now;
    return TRUE;
  }
  return FALSE;
}
// Sends some signal byte in a sync packet to the serial (useful for telling the computer when experiment starts/stops):
void sendSyncSignal (uint8_t signal) {
  V12_STATS signal_data;
  memset (&signal_data, 0b01010101, sizeof (V12_STATS));
  ((uint8_t *) &signal_data) [7] = signal;
  sdWrite (&SD2, (uint8_t *) &signal_data, sizeof (V12_STATS));
}

// Functions by Ruvim Kondratyev, written for working with the ADC code synchronously:

// readVoltageStats (): If you access vstat_obj directly, it can change while you're accessing it, so
// this function is just for convenience to not have to do the mutex locking/unlocking yourself:
void readVoltageStats (V12_STATS * dest_stats) {
  V12_STATS my_copy;
  chMtxLock (&vstat_mtx);
  my_copy = vstat_obj;
  chMtxUnlock (&vstat_mtx);
  *dest_stats = my_copy;
}
// Apparently abs () is not defined in math.h, so here we go:
#define abs(x) (x < 0 ? -x : x)
// Waits for the voltages on both inputs to not fluctuate so much, and then returns:
void waitForStableVoltage (uint16_t wait_amounts_ms) {
  V12_STATS prev;
  V12_STATS now;
  readVoltageStats (&prev);
  chThdSleepMilliseconds (wait_amounts_ms);
  readVoltageStats (&now);
  while (abs (prev.v1.v_mean - now.v1.v_mean) > abs (prev.v1.v_var + now.v1.v_var) ||
      abs (prev.v2.v_mean - now.v2.v_mean) > abs (prev.v2.v_var + now.v2.v_var)) {
    prev = now;
    chThdSleepMilliseconds (wait_amounts_ms);
    readVoltageStats (&now);
  }
}

// Thread4 is the director part of the experiment: it controls how and when the
// output pin gets its PWM signal.

// The denominator to use in the duty cycle calculation fraction:
#define MY_MAX_DUTY 100
// A convenience macro to set the PWM duty cycle (between 0 and MY_MAX_DUTY, inclusive):
#define SET_DRIVER_VOLTAGE_NUMBER(x) pwmEnableChannel (&PWMD5, 0, PWM_FRACTION_TO_WIDTH (&PWMD5, MY_MAX_DUTY, x))
THD_WORKING_AREA(waThread4, 128);
static THD_FUNCTION(Thread4, arg) {
  (void) arg;
  size_t i;
  chRegSetThreadName("VoltageSweeper");
  while (true) {
    if (!palReadPad(GPIOC, GPIOC_BUTTON)) {
      // The user button was pressed. We need to start an experiment.
      chBSemWait (&experiment_sem);
      SET_DRIVER_VOLTAGE_NUMBER (0);
      waitForStableVoltage (2000); // 2 seconds seems like about enough for us.
      sendSyncSignal (1); // Let Python know that the experiment is starting.
      while (!sendSerialData ())
        chThdSleepMilliseconds (1);
      for (i = 1; i < MY_MAX_DUTY; i++) {
        SET_DRIVER_VOLTAGE_NUMBER (i);
        waitForStableVoltage (10);
        while (!sendSerialData ())
          chThdSleepMilliseconds (1);
      }
      sendSyncSignal (2); // Let Python know that the experiment is done.
      SET_DRIVER_VOLTAGE_NUMBER (MY_MAX_DUTY / 5);
      chBSemSignal (&experiment_sem);
    }
    chThdSleepMicroseconds (100);
  }
}


/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  // Start PWM driver 5:
  pwmStart (&PWMD5, &pwmcfg);
  palSetLineMode (LINE_ARD_A0, PAL_MODE_ALTERNATE(2));// Route timer TIM5 to this pin.
  palSetLineMode (LINE_ARD_A1, PAL_MODE_ALTERNATE(2));
  pwmEnableChannel (&PWMD5, 0, PWM_FRACTION_TO_WIDTH (&PWMD5, 100, 20)); // Our channel of interest.
  pwmEnableChannel (&PWMD5, 1, PWM_FRACTION_TO_WIDTH (&PWMD5, 100, 25));

  // Start PWM driver 3:
  pwmStart (&PWMD3, &pwmslowcfg);
  palSetLineMode (LINE_ARD_D5, PAL_MODE_ALTERNATE(2));
  pwmEnableChannel (&PWMD3, 0, PWM_FRACTION_TO_WIDTH (&PWMD3, 100, 75));

  /*
   * Activates the serial driver 2 using our configuration.
   */
  sdStart(&SD2, &sd2cfg);
  palSetPadMode (GPIOA, 2, PAL_MODE_ALTERNATE (7));
  palSetPadMode (GPIOA, 3, PAL_MODE_ALTERNATE (7));

  // Set up lines 10 and 11 to be designated for analog voltage input:
  palSetGroupMode(GPIOC, PAL_PORT_BIT(1) | PAL_PORT_BIT(0),
                    0, PAL_MODE_INPUT_ANALOG);
  adcStart(&ADCD1, NULL);

  // Initialize mutex and semaphore:
  chMtxObjectInit (&vstat_mtx); // So we can control access to voltage statistics.
  chBSemObjectInit (&experiment_sem, FALSE);

  // Create all threads. We don't actually need all the threads, but we'll leave them in for now.
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO + 1, Thread1, NULL);
  chThdCreateStatic(waThread2, sizeof (waThread2), NORMALPRIO + 1, Thread2, NULL);
  chThdCreateStatic(waThread3, sizeof(waThread3), NORMALPRIO + 3, Thread3, NULL);
  chThdCreateStatic(waThread4, sizeof(waThread4), NORMALPRIO + 2, Thread4, NULL);

  // Main loop:
  while (true) {
    // Our main loop waits for the experiment to be over before just sending voltage data to the computer.
    // If the experiment is running, this loop freezes until the experiment is done.
    chBSemWait (&experiment_sem);
    sendSerialData ();
    chBSemSignal (&experiment_sem);
    chThdSleepMilliseconds(200);
  }
}



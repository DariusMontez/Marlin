/**
 * Marlin 3D Printer Firmware
 *
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 * Copyright (c) 2016 Bob Cousins bobcousins42@googlemail.com
 * Copyright (c) 2015-2016 Nico Tonnhofer wurstnase.reprap@gmail.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * Description: HAL for Arduino Due and compatible (SAM3X8E)
 *
 * For ARDUINO_ARCH_SAM
 */

#ifdef ARDUINO_ARCH_ESP32

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------

#include "../HAL.h"

#include "HAL_timers_ESP.h"

// --------------------------------------------------------------------------
// Externals
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Local defines
// --------------------------------------------------------------------------

#define NUM_HARDWARE_TIMERS 4
//
// #define PRESCALER 2
// // --------------------------------------------------------------------------
// // Types
// // --------------------------------------------------------------------------
//
//
// // --------------------------------------------------------------------------
// // Public Variables
// // --------------------------------------------------------------------------
//
// // --------------------------------------------------------------------------
// // Private Variables
// // --------------------------------------------------------------------------
//
// const tTimerConfig TimerConfig [NUM_HARDWARE_TIMERS] = {
//   { TC0, 0, TC0_IRQn, 0},  // 0 - [servo timer5]
//   { TC0, 1, TC1_IRQn, 0},  // 1
//   { TC0, 2, TC2_IRQn, 0},  // 2
//   { TC1, 0, TC3_IRQn, 2},  // 3 - stepper
//   { TC1, 1, TC4_IRQn, 15}, // 4 - temperature
//   { TC1, 2, TC5_IRQn, 0},  // 5 - [servo timer3]
//   { TC2, 0, TC6_IRQn, 0},  // 6
//   { TC2, 1, TC7_IRQn, 0},  // 7
//   { TC2, 2, TC8_IRQn, 0},  // 8
// };
//
hw_timer_t *timers[NUM_HARDWARE_TIMERS];
//
// // --------------------------------------------------------------------------
// // Function prototypes
// // --------------------------------------------------------------------------
//
// // --------------------------------------------------------------------------
// // Private functions
// // --------------------------------------------------------------------------
//
// // --------------------------------------------------------------------------
// // Public functions
// // --------------------------------------------------------------------------
//
// /*
//   Timer_clock1: Prescaler 2 -> 42MHz
//   Timer_clock2: Prescaler 8 -> 10.5MHz
//   Timer_clock3: Prescaler 32 -> 2.625MHz
//   Timer_clock4: Prescaler 128 -> 656.25kHz
// */
//
#include "Marlin.h"
//
void HAL_timer_start (uint8_t timer_num, uint32_t frequency) {
  // SERIAL_ECHO("HAL_TIMER_START\n");
  uint32_t period = 1000000/frequency;
  timers[timer_num] = timerBegin(timer_num, HAL_TICKS_PER_US, true);
  timerAlarmWrite(timers[timer_num], period, true);
}

// void IRAM_ATTR onStepTimer() {
//   Step_Handler();
// }
//
// void IRAM_ATTR onTempTimer() {
//   Temp_Handler();
// }

#include "stepper.h"
#include "temperature.h"

//
void HAL_timer_enable_interrupt (uint8_t timer_num) {
  // SERIAL_ECHO("HAL_timer_enable_interrupt\n");

  void (*fn)(void);
  switch (timer_num) {
    case STEP_TIMER_NUM:
      fn = &Stepper::isr;
      break;
    case TEMP_TIMER_NUM:
      fn = &Temperature::isr;
      break;
  }

  timerAttachInterrupt(timers[timer_num], fn, true);
  timerAlarmEnable(timers[timer_num]);
}
//
void HAL_timer_disable_interrupt (uint8_t timer_num) {
  // SERIAL_ECHO("HAL_timer_disable_interrupt\n");

  timerAlarmDisable(timers[timer_num]);
  // timerDetachInterrupt(timers[timer_num]);
}

HAL_TIMER_TYPE HAL_timer_get_count (uint8_t timer_num) {
  // SERIAL_ECHO("HAL_timer_get_count\n");
  timerGetCountUp(timers[timer_num]);
}

uint32_t HAL_timer_get_current_count(uint8_t timer_num) {
  // SERIAL_ECHO("HAL_timer_get_current_count\n");
  timerRead(timers[timer_num]);
}

#endif // ARDUINO_ARCH_SAM

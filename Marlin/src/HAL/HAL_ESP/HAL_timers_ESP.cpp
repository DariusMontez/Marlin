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

#include "../../../Marlin.h"
#include "../HAL.h"

#include "HAL_timers_ESP.h"
#include "../../../stepper.h"
#include "../../../temperature.h"


// --------------------------------------------------------------------------
// Externals
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Local defines
// --------------------------------------------------------------------------

#define NUM_HARDWARE_TIMERS 4

// --------------------------------------------------------------------------
// Types
// --------------------------------------------------------------------------


// --------------------------------------------------------------------------
// Public Variables
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Private Variables
// --------------------------------------------------------------------------

hw_timer_t *timers[NUM_HARDWARE_TIMERS];

// --------------------------------------------------------------------------
// Function prototypes
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Private functions
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Public functions
// --------------------------------------------------------------------------

//
void HAL_timer_start (uint8_t timer_num, uint32_t frequency) {
  switch (timer_num) {
    case STEP_TIMER_NUM:
      timers[timer_num] = timerBegin(timer_num, STEP_TIMER_PRESCALE, true);
      timerAlarmWrite(timers[timer_num], STEP_TIMER_RATE / frequency, true);
      timerAttachInterrupt(timers[timer_num], &Step_Handler, true);
      break;
    case TEMP_TIMER_NUM:
      timers[timer_num] = timerBegin(timer_num, TEMP_TIMER_PRESCALE, true);
      timerAlarmWrite(timers[timer_num], TEMP_TIMER_RATE / frequency, true);
      timerAttachInterrupt(timers[timer_num], &Temp_Handler, true);
      break;
  }
}

void HAL_timer_set_count(uint8_t timer_num, HAL_TIMER_TYPE count) {
  timerWrite(timers[timer_num], count);
}

//
void HAL_timer_enable_interrupt (uint8_t timer_num) {
  timerAlarmEnable(timers[timer_num]);
}
//
void HAL_timer_disable_interrupt (uint8_t timer_num) {
  timerAlarmDisable(timers[timer_num]);
}

HAL_TIMER_TYPE HAL_timer_get_count (uint8_t timer_num) {
  return timerAlarmRead(timers[timer_num]);
}

HAL_TIMER_TYPE HAL_timer_get_current_count(uint8_t timer_num) {
  return timerRead(timers[timer_num]);
}

void HAL_timer_isr_prologue(uint8_t timer_num) {
  timerWrite(timers[timer_num], 0);
}

#endif // ARDUINO_ARCH_ESP32

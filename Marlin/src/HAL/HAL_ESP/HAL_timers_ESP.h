/**
 * Marlin 3D Printer Firmware
 *
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 * Copyright (c) 2016 Bob Cousins bobcousins42@googlemail.com
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

#ifndef _HAL_TIMERS_ESP_H
#define _HAL_TIMERS_ESP_H

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------

#include <stdint.h>

// --------------------------------------------------------------------------
// Defines
// --------------------------------------------------------------------------
//
#define FORCE_INLINE __attribute__((always_inline)) inline

#define HAL_TIMER_TYPE uint64_t

#define STEP_TIMER_NUM 0  // index of timer to use for stepper
#define TEMP_TIMER_NUM 1  // index of timer to use for temperature

#define STEP_TIMER_PRESCALE 80
#define TEMP_TIMER_PRESCALE 80

#define STEP_TIMER_RATE APB_CLK_FREQ/STEP_TIMER_PRESCALE //
#define TEMP_TIMER_RATE APB_CLK_FREQ/TEMP_TIMER_PRESCALE //

#define HAL_TIMER_RATE (STEP_TIMER_RATE)
#define HAL_STEPPER_TIMER_RATE HAL_TIMER_RATE
#define HAL_TICKS_PER_US ((HAL_STEPPER_TIMER_RATE) / 1000000)


#define TEMP_TIMER_FREQUENCY   1000 // temperature interrupt frequency

#define ENABLE_STEPPER_DRIVER_INTERRUPT() HAL_timer_enable_interrupt (STEP_TIMER_NUM)
#define DISABLE_STEPPER_DRIVER_INTERRUPT()  HAL_timer_disable_interrupt (STEP_TIMER_NUM)

#define ENABLE_TEMPERATURE_INTERRUPT()  HAL_timer_enable_interrupt (TEMP_TIMER_NUM)
#define DISABLE_TEMPERATURE_INTERRUPT() HAL_timer_disable_interrupt (TEMP_TIMER_NUM)

#define HAL_ENABLE_ISRs() do { cli(); if (thermalManager.in_temp_isr)DISABLE_TEMPERATURE_INTERRUPT(); else ENABLE_TEMPERATURE_INTERRUPT(); ENABLE_STEPPER_DRIVER_INTERRUPT(); } while(0)


//
#define HAL_STEP_TIMER_ISR  void Step_Handler()
#define HAL_TEMP_TIMER_ISR  void Temp_Handler()

// --------------------------------------------------------------------------
// Types
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Public Variables
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Public functions
// --------------------------------------------------------------------------

void HAL_timer_start (uint8_t timer_num, uint32_t frequency);

void HAL_timer_set_count(uint8_t timer_num, HAL_TIMER_TYPE count);
HAL_TIMER_TYPE HAL_timer_get_count (uint8_t timer_num);
HAL_TIMER_TYPE HAL_timer_get_current_count(uint8_t timer_num);

void HAL_timer_enable_interrupt(uint8_t timer_num);
void HAL_timer_disable_interrupt(uint8_t timer_num);
void HAL_timer_isr_prologue(uint8_t timer_num);

#endif // _HAL_TIMERS_DUE_H

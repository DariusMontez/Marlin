/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
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
  This code contributed by Triffid_Hunter and modified by Kliment
  why double up on these macros? see http://gcc.gnu.org/onlinedocs/cpp/Stringification.html
*/

/**
 * Description: Fast IO functions for Arduino Due and compatible (SAM3X8E)
 *
 * For ARDUINO_ARCH_SAM
 */

#ifndef	_FASTIO_ESP_H
#define	_FASTIO_ESP_H

// /**
//   utility functions
// */
//
// #ifndef MASK
//   #define MASK(PIN)  (1 << PIN)
// #endif
//
// /**
//   magic I/O routines
//   now you can simply SET_OUTPUT(STEP); WRITE(STEP, 1); WRITE(STEP, 0);
// */
//
// /// Read a pin
// #define _READ(IO) ((bool)(DIO ## IO ## _WPORT -> PIO_PDSR & (MASK(DIO ## IO ## _PIN))))
//
// /// Write to a pin
// #define _WRITE_VAR(IO, v) do {  if (v) {g_APinDescription[IO].pPort->PIO_SODR = g_APinDescription[IO].ulPin; } \
//                                     else {g_APinDescription[IO].pPort->PIO_CODR = g_APinDescription[IO].ulPin; } \
//                                  } while (0)
//
// #define _WRITE(IO, v) do {  if (v) {DIO ## IO ## _WPORT -> PIO_SODR = MASK(DIO ## IO ##_PIN); } \
//                                 else {DIO ##  IO ## _WPORT -> PIO_CODR = MASK(DIO ## IO ## _PIN); }; \
//                              } while (0)
//
// /// toggle a pin
// #define _TOGGLE(IO)  _WRITE(IO, !READ(IO))
//
/// set pin as input
#define _SET_INPUT(IO)      pinMode(IO, INPUT);


/// set pin as output
#define _SET_OUTPUT(IO)     pinMode(IO, OUTPUT);
//
/// set pin as input with pullup mode
#define _PULLUP(IO, v)      pinMode(IO, v ? INPUT_PULLUP : INPUT);
//
// /// check if pin is an input
// #define _GET_INPUT(IO)
// /// check if pin is an output
// #define _GET_OUTPUT(IO)
//
// /// check if pin is an timer
// #define _GET_TIMER(IO)
//
// //  why double up on these macros? see http://gcc.gnu.org/onlinedocs/cpp/Stringification.html
//
/// Read a pin wrapper
#define READ(IO)  digitalRead(IO)
//
// /// Write to a pin wrapper
// #define WRITE_VAR(IO, v)  _WRITE_VAR(IO, v)
// #define WRITE(IO, v)  _WRITE(IO, v)
#define WRITE(IO, v) digitalWrite(IO, v)
//
// /// toggle a pin wrapper
// #define TOGGLE(IO)  _TOGGLE(IO)
//
/// set pin as input wrapper
#define SET_INPUT(IO)  _SET_INPUT(IO)
/// set pin as input with pullup wrapper
#define SET_INPUT_PULLUP(IO) do{ _SET_INPUT(IO); _PULLUP(IO, HIGH); }while(0)
/// set pin as output wrapper
#define SET_OUTPUT(IO)  do{ _SET_OUTPUT(IO); WRITE(IO, LOW); }while(0)
//
// /// check if pin is an input wrapper
// #define GET_INPUT(IO)  _GET_INPUT(IO)
// /// check if pin is an output wrapper
// #define GET_OUTPUT(IO)  _GET_OUTPUT(IO)
//
// /// check if pin is an timer wrapper
// #define GET_TIMER(IO)  _GET_TIMER(IO)
//
// // Shorthand
// #define OUT_WRITE(IO, v) { SET_OUTPUT(IO); WRITE(IO, v); }

/**
  ports and functions

  added as necessary or if I feel like it- not a comprehensive list!
*/

// UART
#define RXD        3
#define TXD        1

// TWI (I2C)
#define SCL        5
#define SDA        4

/**
pins
*/


#endif	/* _FASTIO_ESP_H */

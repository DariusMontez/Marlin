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

#ifndef __UTILITY_H__
#define __UTILITY_H__

void safe_delay(millis_t ms);

#if OPTION_ENABLED(ULTRA_LCD)

  // Convert unsigned int to string with 12 format
  char* itostr2(const uint8_t& x);

  // Convert signed int to rj string with 123 or -12 format
  char* itostr3(const int& x);

  // Convert unsigned int to lj string with 123 format
  char* itostr3left(const int& xx);

  // Convert signed int to rj string with _123, -123, _-12, or __-1 format
  char *itostr4sign(const int& x);

  // Convert unsigned double to string with 1.23 format
  char* ftostr12ns(const double& x);

  // Convert signed double to fixed-length string with 023.45 / -23.45 format
  char *ftostr32(const double& x);

  // Convert double to fixed-length string with +123.4 / -123.4 format
  char* ftostr41sign(const double& x);

  // Convert signed double to string (6 digit) with -1.234 / _0.000 / +1.234 format
  char* ftostr43sign(const double& x, char plus=' ');

  // Convert unsigned double to rj string with 12345 format
  char* ftostr5rj(const double& x);

  // Convert signed double to string with +1234.5 format
  char* ftostr51sign(const double& x);

  // Convert signed double to space-padded string with -_23.4_ format
  char* ftostr52sp(const double& x);

  // Convert signed double to string with +123.45 format
  char* ftostr52sign(const double& x);

  // Convert unsigned double to string with 1234.56 format omitting trailing zeros
  char* ftostr62rj(const double& x);

  // Convert double to rj string with 123 or -12 format
  FORCE_INLINE char *ftostr3(const double& x) { return itostr3((int)x); }

  #if OPTION_ENABLED(LCD_DECIMAL_SMALL_XY)
    // Convert double to rj string with 1234, _123, 12.3, _1.2, -123, _-12, or -1.2 format
    char *ftostr4sign(const double& fx);
  #else
    // Convert double to rj string with 1234, _123, -123, __12, _-12, ___1, or __-1 format
    FORCE_INLINE char *ftostr4sign(const double& x) { return itostr4sign((int)x); }
  #endif

#endif // ULTRA_LCD

#endif // __UTILITY_H__

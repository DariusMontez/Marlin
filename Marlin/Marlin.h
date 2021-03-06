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
#ifndef MARLIN_H
#define MARLIN_H

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "MarlinConfig.h"

#include "src/HAL/HAL.h"

#include "enum.h"
#include "types.h"
#include "utility.h"
#include "serial.h"

#include "WString.h"

#if OPTION_ENABLED(PRINTCOUNTER)
  #include "printcounter.h"
#else
  #include "stopwatch.h"
#endif

void idle(
  #if OPTION_ENABLED(FILAMENT_CHANGE_FEATURE)
    bool no_stepper_sleep = false  // pass true to keep steppers from disabling on timeout
  #endif
);

void manage_inactivity(bool ignore_stepper_queue = false);

#if OPTION_ENABLED(DUAL_X_CARRIAGE) || OPTION_ENABLED(DUAL_NOZZLE_DUPLICATION_MODE)
  extern bool extruder_duplication_enabled;
#endif

#if HAS_X2_ENABLE
  #define  enable_x() do{ X_ENABLE_WRITE( X_ENABLE_ON); X2_ENABLE_WRITE( X_ENABLE_ON); }while(0)
  #define disable_x() do{ X_ENABLE_WRITE(!X_ENABLE_ON); X2_ENABLE_WRITE(!X_ENABLE_ON); axis_known_position[X_AXIS] = false; }while(0)
#elif HAS_X_ENABLE
  #define  enable_x() X_ENABLE_WRITE( X_ENABLE_ON)
  #define disable_x() do{ X_ENABLE_WRITE(!X_ENABLE_ON); axis_known_position[X_AXIS] = false; }while(0)
#else
  #define  enable_x() NOOP
  #define disable_x() NOOP
#endif

#if HAS_Y2_ENABLE
  #define  enable_y() do{ Y_ENABLE_WRITE( Y_ENABLE_ON); Y2_ENABLE_WRITE(Y_ENABLE_ON); }while(0)
  #define disable_y() do{ Y_ENABLE_WRITE(!Y_ENABLE_ON); Y2_ENABLE_WRITE(!Y_ENABLE_ON); axis_known_position[Y_AXIS] = false; }while(0)
#elif HAS_Y_ENABLE
  #define  enable_y() Y_ENABLE_WRITE( Y_ENABLE_ON)
  #define disable_y() do{ Y_ENABLE_WRITE(!Y_ENABLE_ON); axis_known_position[Y_AXIS] = false; }while(0)
#else
  #define  enable_y() NOOP
  #define disable_y() NOOP
#endif

#if HAS_Z2_ENABLE
  #define  enable_z() do{ Z_ENABLE_WRITE( Z_ENABLE_ON); Z2_ENABLE_WRITE(Z_ENABLE_ON); }while(0)
  #define disable_z() do{ Z_ENABLE_WRITE(!Z_ENABLE_ON); Z2_ENABLE_WRITE(!Z_ENABLE_ON); axis_known_position[Z_AXIS] = false; }while(0)
#elif HAS_Z_ENABLE
  #define  enable_z() Z_ENABLE_WRITE( Z_ENABLE_ON)
  #define disable_z() do{ Z_ENABLE_WRITE(!Z_ENABLE_ON); axis_known_position[Z_AXIS] = false; }while(0)
#else
  #define  enable_z() NOOP
  #define disable_z() NOOP
#endif

#if OPTION_ENABLED(MIXING_EXTRUDER)

  /**
   * Mixing steppers synchronize their enable (and direction) together
   */
  #if MIXING_STEPPERS > 3
    #define  enable_e0() { E0_ENABLE_WRITE( E_ENABLE_ON); E1_ENABLE_WRITE( E_ENABLE_ON); E2_ENABLE_WRITE( E_ENABLE_ON); E3_ENABLE_WRITE( E_ENABLE_ON); }
    #define disable_e0() { E0_ENABLE_WRITE(!E_ENABLE_ON); E1_ENABLE_WRITE(!E_ENABLE_ON); E2_ENABLE_WRITE(!E_ENABLE_ON); E3_ENABLE_WRITE(!E_ENABLE_ON); }
  #elif MIXING_STEPPERS > 2
    #define  enable_e0() { E0_ENABLE_WRITE( E_ENABLE_ON); E1_ENABLE_WRITE( E_ENABLE_ON); E2_ENABLE_WRITE( E_ENABLE_ON); }
    #define disable_e0() { E0_ENABLE_WRITE(!E_ENABLE_ON); E1_ENABLE_WRITE(!E_ENABLE_ON); E2_ENABLE_WRITE(!E_ENABLE_ON); }
  #else
    #define  enable_e0() { E0_ENABLE_WRITE( E_ENABLE_ON); E1_ENABLE_WRITE( E_ENABLE_ON); }
    #define disable_e0() { E0_ENABLE_WRITE(!E_ENABLE_ON); E1_ENABLE_WRITE(!E_ENABLE_ON); }
  #endif
  #define  enable_e1() NOOP
  #define disable_e1() NOOP
  #define  enable_e2() NOOP
  #define disable_e2() NOOP
  #define  enable_e3() NOOP
  #define disable_e3() NOOP

#else // !MIXING_EXTRUDER

  #if HAS_E0_ENABLE
    #define  enable_e0() E0_ENABLE_WRITE( E_ENABLE_ON)
    #define disable_e0() E0_ENABLE_WRITE(!E_ENABLE_ON)
  #else
    #define  enable_e0() NOOP
    #define disable_e0() NOOP
  #endif

  #if E_STEPPERS > 1 && HAS_E1_ENABLE
    #define  enable_e1() E1_ENABLE_WRITE( E_ENABLE_ON)
    #define disable_e1() E1_ENABLE_WRITE(!E_ENABLE_ON)
  #else
    #define  enable_e1() NOOP
    #define disable_e1() NOOP
  #endif

  #if E_STEPPERS > 2 && HAS_E2_ENABLE
    #define  enable_e2() E2_ENABLE_WRITE( E_ENABLE_ON)
    #define disable_e2() E2_ENABLE_WRITE(!E_ENABLE_ON)
  #else
    #define  enable_e2() NOOP
    #define disable_e2() NOOP
  #endif

  #if E_STEPPERS > 3 && HAS_E3_ENABLE
    #define  enable_e3() E3_ENABLE_WRITE( E_ENABLE_ON)
    #define disable_e3() E3_ENABLE_WRITE(!E_ENABLE_ON)
  #else
    #define  enable_e3() NOOP
    #define disable_e3() NOOP
  #endif

#endif // !MIXING_EXTRUDER

#if OPTION_ENABLED(G38_PROBE_TARGET)
  extern bool G38_move,        // flag to tell the interrupt handler that a G38 command is being run
              G38_endstop_hit; // flag from the interrupt handler to indicate if the endstop went active
#endif

/**
 * The axis order in all axis related arrays is X, Y, Z, E
 */
#define _AXIS(AXIS) AXIS ##_AXIS

void enable_all_steppers();
void disable_e_steppers();
void disable_all_steppers();

void FlushSerialRequestResend();
void ok_to_send();

void kill(const char*);

void quickstop_stepper();

#if OPTION_ENABLED(FILAMENT_RUNOUT_SENSOR)
  void handle_filament_runout();
#endif

extern uint8_t marlin_debug_flags;
#define DEBUGGING(F) (marlin_debug_flags & (DEBUG_## F))

extern bool Running;
inline bool IsRunning() { return  Running; }
inline bool IsStopped() { return !Running; }

bool enqueue_and_echo_command(const char* cmd, bool say_ok=false); // Add a single command to the end of the buffer. Return false on failure.
void enqueue_and_echo_commands_P(const char * const cmd);          // Set one or more commands to be prioritized over the next Serial/SD command.
void clear_command_queue();

extern millis_t previous_cmd_ms;
inline void refresh_cmd_timeout() { previous_cmd_ms = millis(); }

#if OPTION_ENABLED(FAST_PWM_FAN)
  void setPwmFrequency(uint8_t pin, int val);
#endif

/**
 * Feedrate scaling and conversion
 */
extern int feedrate_percentage;

#define MMM_TO_MMS(MM_M) ((MM_M)/60.0)
#define MMS_TO_MMM(MM_S) ((MM_S)*60.0)
#define MMS_SCALED(MM_S) ((MM_S)*feedrate_percentage*0.01)

extern bool axis_relative_modes[];
extern bool volumetric_enabled;
extern int flow_percentage[EXTRUDERS]; // Extrusion factor for each extruder
extern double filament_size[EXTRUDERS]; // cross-sectional area of filament (in millimeters), typically around 1.75 or 2.85, 0 disables the volumetric calculations for the extruder.
extern double volumetric_multiplier[EXTRUDERS]; // reciprocal of cross-sectional area of filament (in square millimeters), stored this way to reduce computational burden in planner
extern bool axis_known_position[XYZ]; // axis[n].is_known
extern bool axis_homed[XYZ]; // axis[n].is_homed
extern volatile bool wait_for_heatup;

#if OPTION_ENABLED(EMERGENCY_PARSER) || OPTION_ENABLED(ULTIPANEL)
  extern volatile bool wait_for_user;
#endif

extern double current_position[NUM_AXIS];

// Workspace offsets
#if OPTION_DISABLED(NO_WORKSPACE_OFFSETS)
  extern double position_shift[XYZ],
               home_offset[XYZ],
               workspace_offset[XYZ];
  #define LOGICAL_POSITION(POS, AXIS) ((POS) + workspace_offset[AXIS])
  #define RAW_POSITION(POS, AXIS)     ((POS) - workspace_offset[AXIS])
#else
  #define LOGICAL_POSITION(POS, AXIS) (POS)
  #define RAW_POSITION(POS, AXIS)     (POS)
#endif

#define LOGICAL_X_POSITION(POS)     LOGICAL_POSITION(POS, X_AXIS)
#define LOGICAL_Y_POSITION(POS)     LOGICAL_POSITION(POS, Y_AXIS)
#define LOGICAL_Z_POSITION(POS)     LOGICAL_POSITION(POS, Z_AXIS)
#define RAW_X_POSITION(POS)         RAW_POSITION(POS, X_AXIS)
#define RAW_Y_POSITION(POS)         RAW_POSITION(POS, Y_AXIS)
#define RAW_Z_POSITION(POS)         RAW_POSITION(POS, Z_AXIS)
#define RAW_CURRENT_POSITION(AXIS)  RAW_POSITION(current_position[AXIS], AXIS)

#if HOTENDS > 1
  extern double hotend_offset[XYZ][HOTENDS];
#endif

// Software Endstops
extern double soft_endstop_min[XYZ];
extern double soft_endstop_max[XYZ];

#if HAS_SOFTWARE_ENDSTOPS
  extern bool soft_endstops_enabled;
  void clamp_to_software_endstops(double target[XYZ]);
#else
  #define soft_endstops_enabled false
  #define clamp_to_software_endstops(x) NOOP
#endif

#if OPTION_DISABLED(NO_WORKSPACE_OFFSETS) || OPTION_ENABLED(DUAL_X_CARRIAGE) || OPTION_ENABLED(DELTA)
  void update_software_endstops(const AxisEnum axis);
#endif

// GCode support for external objects
bool code_seen(char);
int code_value_int();
double code_value_temp_abs();
double code_value_temp_diff();

#if IS_KINEMATIC
  extern double delta[ABC];
  void inverse_kinematics(const double logical[XYZ]);
#endif

#if OPTION_ENABLED(DELTA)
  extern double endstop_adj[ABC],
               delta_radius,
               delta_diagonal_rod,
               delta_segments_per_second,
               delta_diagonal_rod_trim[ABC],
               delta_tower_angle_trim[ABC],
               delta_clip_start_height;
  void recalc_delta_settings(double radius, double diagonal_rod);
#elif IS_SCARA
  void forward_kinematics_SCARA(const double &a, const double &b);
#endif

#if OPTION_ENABLED(AUTO_BED_LEVELING_BILINEAR)
  extern int bilinear_grid_spacing[2], bilinear_start[2];
  extern double bed_level_grid[ABL_GRID_MAX_POINTS_X][ABL_GRID_MAX_POINTS_Y];
  double bilinear_z_offset(double logical[XYZ]);
  void set_bed_leveling_enabled(bool enable=true);
#endif

#if PLANNER_LEVELING
  void reset_bed_level();
#endif

#if OPTION_ENABLED(Z_DUAL_ENDSTOPS)
  extern double z_endstop_adj;
#endif

#if HAS_BED_PROBE
  extern double zprobe_zoffset;
#endif

#if OPTION_ENABLED(HOST_KEEPALIVE_FEATURE)
  extern MarlinBusyState busy_state;
  #define KEEPALIVE_STATE(n) do{ busy_state = n; }while(0)
#else
  #define KEEPALIVE_STATE(n) NOOP
#endif

#if FAN_COUNT > 0
  extern int fanSpeeds[FAN_COUNT];
#endif

#if OPTION_ENABLED(BARICUDA)
  extern int baricuda_valve_pressure;
  extern int baricuda_e_to_p_pressure;
#endif

#if OPTION_ENABLED(FILAMENT_WIDTH_SENSOR)
  extern bool filament_sensor;         // Flag that filament sensor readings should control extrusion
  extern double filament_width_nominal, // Theoretical filament diameter i.e., 3.00 or 1.75
               filament_width_meas;    // Measured filament diameter
  extern int8_t measurement_delay[];   // Ring buffer to delay measurement
  extern int filwidth_delay_index[2];  // Ring buffer indexes. Used by planner, temperature, and main code
  extern int meas_delay_cm;            // Delay distance
#endif

#if OPTION_ENABLED(FILAMENT_CHANGE_FEATURE)
  extern FilamentChangeMenuResponse filament_change_menu_response;
#endif

#if OPTION_ENABLED(PID_EXTRUSION_SCALING)
  extern int lpq_len;
#endif

#if OPTION_ENABLED(FWRETRACT)
  extern bool autoretract_enabled;
  extern bool retracted[EXTRUDERS]; // extruder[n].retracted
  extern double retract_length, retract_length_swap, retract_feedrate_mm_s, retract_zlift;
  extern double retract_recover_length, retract_recover_length_swap, retract_recover_feedrate_mm_s;
#endif

// Print job timer
#if OPTION_ENABLED(PRINTCOUNTER)
  extern PrintCounter print_job_timer;
#else
  extern Stopwatch print_job_timer;
#endif

// Handling multiple extruders pins
extern uint8_t active_extruder;

#if HAS_TEMP_HOTEND || HAS_TEMP_BED
  void print_heaterstates();
#endif

#if OPTION_ENABLED(MIXING_EXTRUDER)
  extern double mixing_factor[MIXING_STEPPERS];
#endif

void calculate_volumetric_multipliers();

/**
 * Blocking movement and shorthand functions
 */
void do_blocking_move_to(const double &x, const double &y, const double &z, const double &fr_mm_s=0.0);
void do_blocking_move_to_x(const double &x, const double &fr_mm_s=0.0);
void do_blocking_move_to_z(const double &z, const double &fr_mm_s=0.0);
void do_blocking_move_to_xy(const double &x, const double &y, const double &fr_mm_s=0.0);

#if OPTION_ENABLED(Z_PROBE_ALLEN_KEY) || OPTION_ENABLED(Z_PROBE_SLED) || HAS_PROBING_PROCEDURE || HOTENDS > 1 || OPTION_ENABLED(NOZZLE_CLEAN_FEATURE) || OPTION_ENABLED(NOZZLE_PARK_FEATURE)
  bool axis_unhomed_error(const bool x, const bool y, const bool z);
#endif

#endif //MARLIN_H

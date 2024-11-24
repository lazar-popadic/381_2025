/*
 * lib.h
 *
 *  Created on: Aug 13, 2024
 *      Author: lazar
 */

#ifndef LIB_LIB_H_
#define LIB_LIB_H_

#include "structs.h"

// pwm.h
void
pwm_right_dc (int16_t duty_cycle);
void
pwm_left_dc (int16_t duty_cycle);
void
pwm_start ();
void
pwm_stop ();

// time.h
void
time_ISR ();
uint8_t
delay_nonblocking (uint32_t delay_ms);
void
time_start ();
void
time_stop ();
void
start_match ();
void
stop_match ();
void
set_time_ms (uint32_t time);
uint32_t
get_time_ms ();

// ax12a.h
void
ax_move (uint8_t id, uint16_t angle, uint16_t speed);

// odometry.h
void
odometry_init ();
void
update_odom ();

//signal.h
void
wrap180_ptr (float*);
void
wrapPi_ptr (float*);
float
wrap180 (float signal);
float
wrap360 (float signal);
int8_t
get_sign (float num);
char
get_sign_char (int16_t num);
int8_t
interpret_sign (char character);

// rpi.h
void
rpi_init ();
void
update_transmit_buffer ();
void
edit_recieved_odom ();

// input.h
uint8_t
read_cinc();
uint8_t
read_switch_S();
uint8_t
read_switch_2();
uint8_t
read_switch_1();
void
choose_tactic (struct_tactic_num *tactic);
uint8_t
read_sensors_front();
uint8_t
read_sensors_back();

// output.h
void
motor_l_forw ();
void
motor_l_back ();
void
motor_r_forw ();
void
motor_r_back ();
void
vacuum_0 (uint8_t on);
void
vacuum_1 (uint8_t on);
void
vacuum_2 (uint8_t on);
void
vacuum_3 (uint8_t on);

#endif /* LIB_LIB_H_ */

/*
 * lib.h
 *
 *  Created on: Aug 13, 2024
 *      Author: lazar
 */

#ifndef LIB_LIB_H_
#define LIB_LIB_H_

#include "structs.h"

void
pwm_right_dc (int16_t duty_cycle);
void
pwm_left_dc (int16_t duty_cycle);
void
pwm_start ();
void
pwm_stop ();

void
time_ISR ();
uint8_t
delay_nonblocking (uint32_t delay_ms);
void
time_start ();
void
time_stop ();
void
start_match();
void
stop_match();
void
set_time_ms (uint32_t time);
uint32_t
get_time_ms ();

void
ax_move (uint8_t id, uint16_t angle, uint16_t speed);

void
odometry_init ();
void
update_odom (volatile struct_robot_base *base);

void
wrapTo180_ptr(float*);
void
wrapToPi_ptr(float*);
float
wrapTo180(float signal);

#endif /* LIB_LIB_H_ */

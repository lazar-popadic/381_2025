/*
 * lib.h
 *
 *  Created on: Aug 13, 2024
 *      Author: lazar
 */

#ifndef LIB_LIB_H_
#define LIB_LIB_H_

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

#endif /* LIB_LIB_H_ */

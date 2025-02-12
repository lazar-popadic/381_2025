/*
 * lib.h
 *
 *  Created on: Aug 13, 2024
 *      Author: lazar
 */

#ifndef LIB_LIB_H_
#define LIB_LIB_H_

#define V_MOVING_MIN	0.01	// [m/s]
#define W_MOVING_MIN	1.8		// [deg/s]

#define V_MAX_DEF		2.0		// [m/s]
#define W_MAX_DEF		360.0	// [deg/s]
#define A_MAX_DEF		0.02	// [m/s * 1/10ms]			// za 1000ms dodje do 2m/s
#define ALPHA_MAX_DEF	36		// [deg/s * 1/ms]			// za 100ms dodje do 360deg/s
#define J_MAX_DEF		0.0016	// [mm/ms * 1/10ms^2]		// za 1000ms dodje do 2m/s i sve vreme je jerk-limited
#define J_ROT_MAX_DEF	2.88	// [deg/s * 1/10ms^2]		// za 100ms dodje do 360deg/s i sve vreme je jerk-limited
#define CTRL_MAX		4200	// [inc]
#define MAX_PWM_CHANGE	420		// [inc/ms]					// za 10ms dodje do 4200
#define POS_LOOP_PSC	10

#define TASK_RUNNING	0
#define TASK_SUCCESS	-1
#define TASK_FAIL		1

#define FORWARD		1
#define BACKWARD	-1

#define TRAP_VEL_PROFILE	0
#define S_CURVE_VEL_PROFILE	1

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
delay_nb (uint32_t delay_ms);
uint8_t
delay_nb_2 (uint32_t *start_time, uint32_t delay_ms);
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
wrap180_ptr (volatile float*);
void
wrapPi_ptr (volatile float*);
float
wrap180 (float signal);
int8_t
get_sign (float num);
void
saturation (volatile float *signal, float max, float min);
void
scale_vel_ref (volatile float *ref_1, volatile float *ref_2, float limit);
float
abs_max (float a, float b);
float
abs_min (float a, float b);
uint32_t
uint_min (uint32_t a, uint32_t b);
void
vel_ramp_up_ptr (float *signal, float reference, float acc);
float
vel_ramp_up (float signal, float reference, float acc_max);
float
vel_s_curve_up_webots (float *vel, float prev_vel, float vel_ref, float jerk_slope);
float
vel_s_curve_up (float vel, float accel, float vel_ref, float jerk);
float
min3 (float a, float b, float c);

// rpi.h
void
rpi_init ();
void
update_transmit_buffer ();
void
edit_recieved_odom ();

// input.h
uint8_t
read_cinc ();
uint8_t
cinc_db ();
uint8_t
read_switch_S ();
uint8_t
read_switch_2 ();
uint8_t
read_switch_1 ();
void
choose_tactic (struct_tactic_num *tactic);
uint8_t
read_sensors_front ();
uint8_t
read_sensors_back ();

// output.h
void
set_motor_l_dir (int8_t dir);
void
set_motor_r_dir (int8_t dir);
void
vacuum_0 (uint8_t on);
void
vacuum_1 (uint8_t on);
void
vacuum_2 (uint8_t on);
void
vacuum_3 (uint8_t on);

// prediction.h
uint16_t
get_pts ();
void
set_pts (uint16_t pts);
void
add_pts (uint16_t pts);

// pid.h
float
calc_pid (volatile struct_pid *pid_ptr, float err);
float
calc_pid_2 (volatile struct_pid *pid_ptr, float ref, float val);
void
init_pid (volatile struct_pid *pid_ptr, float p, float i, float d, float limit, float sum_limit);

// base.h
void
base_init ();
volatile struct_robot_base*
get_robot_base ();
void
set_v_max (float multiplier);
void
reset_v_max ();
void
set_w_max (float multiplier);
void
reset_w_max ();
void
update_base_status ();
void
check_sensors ();
uint8_t
get_regulation_status ();
void
set_regulation_status (uint8_t status);
uint8_t
get_obstacle_detected ();
void
stop_robot ();

// regulation.h
void
regulation_init ();
void
position_loop ();
void
velocity_loop ();
void
set_reg_type (int8_t type);
int8_t
move_to_xy (float x, float y, int8_t dir, float v_max, float w_max);
int8_t
rot_to_phi (float phi, float w_max);
int8_t
move_on_dir (float distance, int8_t dir, float v_max);
int8_t
rot_to_xy (float x, float y, int dir, float w_max);

// mechanism.h
void
mechanism_init ();
void
lift_front_up ();
void
lift_front_down ();
void
lift_front_drop ();
void
lift_back_up ();
void
lift_back_down ();
void
lift_back_drop ();
void
grtl_front_open ();
void
grtl_back_open ();
void
grtl_front_close ();
void
grtl_back_close ();
void
grtl_front_grip_all ();
void
grtl_back_grip_all ();
void
grtl_front_grip_inside ();
void
grtl_back_grip_inside ();
void
grtl_front_grip_outside ();
void
grtl_back_grip_outside ();
void
grtl_front_open_inside ();
void
grtl_back_open_inside ();
void
grtl_front_open_outside ();
void
grtl_back_open_outside ();
void
ruc_front_down ();
void
ruc_back_down ();
void
ruc_front_mid ();
void
ruc_back_mid ();
void
ruc_front_up ();
void
ruc_back_up ();
void
gurl_front ();
void
gurl_back ();
void
gurl_mid ();
void
bnr_start ();
void
bnr_drop_1 ();
void
bnr_drop_2 ();
void
bnr_drop_3 ();
void
vacuum_front (uint8_t on);
void
vacuum_back (uint8_t on);

#endif /* LIB_LIB_H_ */

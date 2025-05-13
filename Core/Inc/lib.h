/*
 * lib.h
 *
 *  Created on: Aug 13, 2024
 *      Author: lazar
 */

#ifndef LIB_LIB_H_
#define LIB_LIB_H_

#define YELLOW	0
#define BLUE		1

#define V_MOVING_MIN		0.05		// [m/s]
#define W_MOVING_MIN		18			// [deg/s]
#define V_MAX_DEF				0.95		// [m/s]
#define W_MAX_DEF				360.0		// [deg/s]
#define A_MAX_DEF				0.25		// [m/s * 1/10ms]				// za 40ms dodje do 1m/s
#define ALPHA_MAX_DEF		90			// [deg/s * 1/10ms]			// za 40ms dodje do 360deg/s
#define J_MAX_DEF				0.1			// [mm/ms * 1/10ms^2]		// za 40ms dodje do 1m/s i sve vreme je jerk-limited
#define J_ROT_MAX_DEF		36			// [deg/s * 1/10ms^2]		// za 40ms dodje do 360deg/s i sve vreme je jerk-limited
#define CTRL_MAX				4200		// [inc]
#define MAX_PWM_CHANGE	840			// [inc/ms]					// za 5ms dodje do 4200
#define POS_LOOP_PSC		10
#define V_MAX_DEF_PATH	0.95		// [m/s]

#define TASK_RUNNING	0
#define TASK_SUCCESS	-1
#define TASK_FAIL			1

#define FORWARD		1
#define BACKWARD	-1
#define ALL_SENS	-2
#define NO_SENS		0

#define TO_CENTER	1
#define TO_OUT		-1

#define IN_GRTL		-1
#define OUT_GRTL	1

#define TRAP_VEL_PROFILE		0
#define S_CURVE_VEL_PROFILE	1
#define STOPPING						2

#define POINT_DISTANCE 		25
#define BEZIER_RESOLUTION 500
#define MAX_EQU_PTS				220
#define OFFS_ROBOT 		360
#define OFFS_DESIRED	420
#define PAD_NUM				20

#define HOME_TIME			96

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
void
sg90_init ();
void
sg90_1_move (float us);
void
sg90_2_move (float us);
void
sg90_3_move (float us);
void
sg90_4_move (float us);

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
uint16_t
get_time_s ();

// ax12a.h
void
torque_enable (uint8_t id);
void
ax_move (uint8_t id, uint16_t angle, uint16_t speed, UART_HandleTypeDef huart);

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
float
snap_phi (float phi);

// rpi.h
void
rpi_init ();
void
update_transmit_buffer ();
void
update_recieve_buffer ();

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
choose_tactic (tactic_num *tactic);
uint8_t
read_sensors_front ();
uint8_t
read_sensors_back ();
uint8_t
check_front ();
uint8_t
check_back ();

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
calc_pid (volatile pid *pid_ptr, float err);
float
calc_pid_2 (volatile pid *pid_ptr, float ref, float val);
void
init_pid (volatile pid *pid_ptr, float p, float i, float d, float limit, float sum_limit);
void
reset_pid (volatile pid *pid_ptr);

// base.h
void
base_init ();
volatile robot_base*
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
uint8_t
check_sensors ();
uint8_t
get_regulation_status ();
void
set_regulation_status (uint8_t status);
uint8_t
get_obstacle_detected ();
void
motors_off ();

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
move_to_xy (float x, float y, int8_t dir, float v_max, float w_max, int8_t check_sensors);
int8_t
rot_to_phi (float phi, float w_max, int8_t check_sensors);
int8_t
move_on_dir_ortho (float distance, int8_t dir, float v_max, int8_t check_sensors);
int8_t
move_on_dir (float distance, int8_t dir, float v_max, int8_t check_sensors);
int8_t
rot_to_xy (float x, float y, int dir, float w_max, int8_t check_sensors);
int8_t
rot_relative (float angle, float w_max, int8_t check_sensors);
int8_t
move_on_path (float x, float y, float phi, int8_t dir, int8_t cont, float v_max, int8_t avoid, int8_t check_sensors);
void
stop_moving ();
void
continue_moving ();
void
reset_movement ();

// mechanism.h
mech_states
get_mech_states ();
void
ax_init ();
void
mechanism_init ();
void
prepare_front ();
void
prepare_back ();
void
lift_front_up ();
void
lift_front_down ();
void
lift_front_drop ();
void
lift_front_carry ();
void
lift_front_leave ();
void
lift_back_up ();
void
lift_back_down ();
void
lift_back_drop ();
void
lift_back_carry ();
void
lift_back_leave ();
void
grtl_front_open ();
void
grtl_back_open ();
void
grtl_front_close ();
void
grtl_back_close ();
void
lift_back_down_bnr ();
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
ruc_front_full_down ();
void
ruc_back_full_down ();
void
ruc_front_mid ();
void
ruc_back_mid ();
void
ruc_front_up ();
void
ruc_back_up ();
void
ruc_front_carry ();
void
ruc_back_carry ();
void
gurl_front ();
void
gurl_back ();
void
gurl_mid ();
void
vacuum_front (uint8_t on);
void
vacuum_back (uint8_t on);
void
bnr_close ();
void
bnr_1 ();
void
bnr_2 ();
void
bnr_3 ();
void
bnr_4 ();
void
grtl_front_open_outside_s1 ();
void
grtl_back_open_outside_s1 ();

// curve.h
void
create_curve (curve *curve_ptr, float x_ref, float y_ref, float phi_ref, int dir, int avoid_obst);
void
cubic_bezier_pts (curve *curve_ptr, float p0_x, float p0_y, float p1_x, float p1_y, float p2_x, float p2_y, float p3_x, float p3_y);
void
equ_coords (curve *curve_ptr);
int8_t
get_curve_ready ();
void
set_curve_ready (int8_t ready);
void
pad_curve (curve *curve_ptr, int8_t dir);

// tactics.h
uint8_t
get_points ();
void
set_points (uint8_t pts);
void
add_points (uint8_t pts);
float
x_side (float x);
float
phi_side (float phi);
int8_t
bnr_side (int8_t bnr_dir);
void
reset_all_delays ();
tactic_num*
get_tact_num_ptr ();
uint8_t
timeout (uint32_t time);
uint8_t
detected_timeout (uint32_t time);
int8_t
tact_0 ();
int8_t
tact_1 ();
int8_t
tact_2 ();
void
tact_3 ();
int8_t
tact_homologation ();
int8_t
tact_dev ();
int8_t
tact_dev_2 ();
int8_t
tact_dev_3 ();

// tasks.h
int8_t
task_sprat_1 (int8_t side, int8_t in_out);
int8_t
task_sprat_12 (int8_t side);
int8_t
task_sprat_3 (int8_t side);
int8_t
task_sprat_3_full (int8_t side);
int8_t
task_sprat_3_half (int8_t side);
int8_t
task_sprat_3_1_full (int8_t side);
int8_t
task_sprat_3_2_full (int8_t side);
int8_t
task_banner (int8_t bnr_dir);

#endif /* LIB_LIB_H_ */

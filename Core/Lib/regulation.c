/*
 * regulation.c
 *
 *  Created on: Dec 4, 2024
 *      Author: lazar
 */

// Tolerancije za distancu i ugao kada smatram da je zadovoljen uslov
#define D_PROJ_TOL	5
#define D_TOL				20
#define D_2_TOL			20
#define D_3_TOL			50
#define PHI_TOL			1
#define PHI_PRIM_TOL	2

#include "main.h"

static void
rotate ();
static void
go_to_xy ();
static void
pure_pursuit (uint8_t lookahead_pnt_num, uint8_t lookahead_pnt_num_2);
static void
pos_hold ();

static uint8_t vel_profile = S_CURVE_VEL_PROFILE;

static int8_t reg_type = 0;
static int8_t prev_reg_type = 0;
static int8_t phase = 0;
static int8_t direction;
static uint8_t position_cnt = 1;

//static curve curve_var;
static curve *curve_ptr;
static int16_t curve_cnt = 0;

static float v_err = 0;							// [ms/ms]
static float w_err = 0;							// [deg/ms]
static float x_err = 0;							// [mm]
static float y_err = 0;							// [mm]
static float phi_err = 0;						// [deg]
static float d = 0;									// [mm]
static float d_proj = 0;						// [mm]
static float y_err_next = 0;				// [mm]
static float x_err_next = 0;				// [mm]
static float phi_prim_err = 0;			// [deg]
static float phi_prim_1_err = 0;		// [deg]
static int8_t cont_move = 0;
static float d_total = 0;						// [mm]

static volatile robot_base *base_ptr;

static volatile pid d_loop;
static volatile pid phi_loop;
static volatile pid phi_curve_loop;
static volatile pid v_loop;
static volatile pid w_loop;

int8_t move_status = TASK_RUNNING;

void
regulation_init ()
{
	base_ptr = get_robot_base ();
	init_pid (&d_loop, 0.01, 0.00, 0.264, V_MAX_DEF, 0.0);
	init_pid (&phi_loop, 4.0, 0.02, 0.2, W_MAX_DEF, W_MAX_DEF * 0.25);
	init_pid (&phi_curve_loop, 6.0, 0.02, 0.1, W_MAX_DEF, W_MAX_DEF * 0.25);
	init_pid (&v_loop, 6800, 32, 800, CTRL_MAX, 840);
	init_pid (&w_loop, 72, 0.36, 3, CTRL_MAX, 2100);
	curve_ptr = (curve*) malloc (sizeof(curve));
	for (int i = 0; i < BEZIER_RESOLUTION; i++)
		{
			curve_ptr->pts_x[i] = 0;
			curve_ptr->pts_y[i] = 0;
			if (i < MAX_EQU_PTS)
				{
					curve_ptr->equ_pts_x[i] = 0;
					curve_ptr->equ_pts_y[i] = 0;
				}
		}
}

void
velocity_loop ()
{
// ulazi su reference za brzinu
// izlazi su upravljacki signali za brzinu: motor_left_ctrl, motor_right_ctrl
	v_err = base_ptr->v_ref - base_ptr->v;
	w_err = base_ptr->w_ref - base_ptr->w;
	base_ptr->v_ctrl = calc_pid (&v_loop, v_err);
	base_ptr->w_ctrl = calc_pid (&w_loop, w_err);

	base_ptr->motor_r_ctrl = base_ptr->v_ctrl + base_ptr->w_ctrl;
	base_ptr->motor_l_ctrl = base_ptr->v_ctrl - base_ptr->w_ctrl;
	scale_vel_ref (&base_ptr->motor_r_ctrl, &base_ptr->motor_l_ctrl, CTRL_MAX);

	base_ptr->motor_r_dir = get_sign (base_ptr->motor_r_ctrl);
	base_ptr->motor_l_dir = get_sign (base_ptr->motor_l_ctrl);
	// u uint su idalje stare vrednosti, u float su nove tj. reference
	// u motor_*_ctrl upisuje apsolutnu vrednost
	base_ptr->motor_r_ctrl = abs_min (
			base_ptr->motor_r_ctrl,
			vel_ramp_up ((float) (base_ptr->motor_r_ctrl_uint * base_ptr->motor_r_dir), base_ptr->motor_r_ctrl, MAX_PWM_CHANGE));
	base_ptr->motor_l_ctrl = abs_min (
			base_ptr->motor_l_ctrl,
			vel_ramp_up ((float) (base_ptr->motor_l_ctrl_uint * base_ptr->motor_l_dir), base_ptr->motor_l_ctrl, MAX_PWM_CHANGE));
	base_ptr->motor_r_ctrl_uint = (uint16_t) base_ptr->motor_r_ctrl;
	base_ptr->motor_l_ctrl_uint = (uint16_t) base_ptr->motor_l_ctrl;

	set_motor_r_dir (base_ptr->motor_r_dir);
	set_motor_l_dir (base_ptr->motor_l_dir);
	pwm_right_dc (base_ptr->motor_r_ctrl_uint);
	pwm_left_dc (base_ptr->motor_l_ctrl_uint);

}

void
position_loop ()
{
// ulazi su reference za poziciju
// izlazi su reference za brzinu: base_ptr->v_ref, base_ptr->w_ref
	position_cnt++;

	if (!(position_cnt % POS_LOOP_PSC))
		{
			position_cnt = 1;

			switch (reg_type)
				{
				case -2:
					pos_hold ();
					break;
				case -1:
					rotate ();
					break;
				case 0:
					base_ptr->v_ref = 0;
					base_ptr->w_ref = 0;
					break;
				case 1:
					go_to_xy ();
					break;
				case 2:
					if (get_curve_ready ())
						pure_pursuit (9, 4);
					else
						{
							base_ptr->v_ref = 0;
							base_ptr->w_ref = 0;
						}
					break;
				}

			// obrada referenci
			switch (vel_profile)
				{
				case S_CURVE_VEL_PROFILE:
					base_ptr->v_ref = get_sign (base_ptr->v_ref)
							* min3 (fabs (base_ptr->v_ref), base_ptr->v_max,
											fabs (vel_s_curve_up (base_ptr->v, base_ptr->a, base_ptr->v_ref, base_ptr->j_max)));
					base_ptr->w_ref = get_sign (base_ptr->w_ref)
							* min3 (fabs (base_ptr->w_ref), base_ptr->w_max,
											fabs (vel_s_curve_up (base_ptr->w, base_ptr->alpha, base_ptr->w_ref, base_ptr->j_rot_max)));
					break;
				case TRAP_VEL_PROFILE:
					base_ptr->v_ref = get_sign (base_ptr->v_ref)
							* min3 (fabs (base_ptr->v_ref), base_ptr->v_max, fabs (vel_ramp_up (base_ptr->v, base_ptr->v_ref, base_ptr->a_max)));
					base_ptr->w_ref = get_sign (base_ptr->w_ref)
							* min3 (fabs (base_ptr->w_ref), base_ptr->w_max, fabs (vel_ramp_up (base_ptr->w, base_ptr->w_ref, base_ptr->alpha_max)));
					break;
				case STOPPING:
					base_ptr->v_ref = 0;
					base_ptr->w_ref = 0;
					break;
				}
		}
}

static void
rotate ()
{
	phi_err = base_ptr->phi_ref - base_ptr->phi;
	wrap180_ptr (&phi_err);
	base_ptr->v_ref = 0;
	base_ptr->w_ref = calc_pid (&phi_loop, phi_err);
	if (fabs (phi_err) < PHI_TOL)
		{
			base_ptr->on_target = 1;
			base_ptr->movement_finished = 1;
			base_ptr->v_ref = 0;
			base_ptr->w_ref = 0;
			set_reg_type (0);
		}
}

static void
go_to_xy ()
{
	x_err = base_ptr->x_ref - base_ptr->x;
	y_err = base_ptr->y_ref - base_ptr->y;
	phi_err = atan2 (y_err, x_err) * 180 / M_PI + (direction - 1) * 90 - base_ptr->phi;
	wrap180_ptr (&phi_err);

	switch (phase)
		{
		case 0:    // rot2pos
			base_ptr->v_ref = 0;
			base_ptr->w_ref = calc_pid (&phi_loop, phi_err);
			if (fabs (phi_err) < PHI_PRIM_TOL && !(base_ptr->moving))
				phase = 1;
			break;

		case 1:    // tran
			d = sqrt (x_err * x_err + y_err * y_err);
			d_proj = d * cos (phi_err * M_PI / 180);

			base_ptr->v_ref = calc_pid (&d_loop, d_proj * direction);

			if (fabs (d) > D_3_TOL)
				base_ptr->w_ref = calc_pid (&phi_loop, phi_err);
			else
				base_ptr->w_ref = 0;

			if (d_proj < D_PROJ_TOL && fabs (d) < D_2_TOL && !(base_ptr->moving))
				{
					base_ptr->movement_finished = 1;
					base_ptr->v_ref = 0;
					base_ptr->w_ref = 0;
					phase = 0;
					set_reg_type (0);
					if (fabs (d) < D_TOL)
						base_ptr->on_target = 1;
					else
						base_ptr->on_target = 0;
				}
			break;
		}
}

static void
pos_hold ()
{
	x_err = base_ptr->x_ref - base_ptr->x;
	y_err = base_ptr->y_ref - base_ptr->y;
	phi_err = atan2 (y_err, x_err) * 180 / M_PI - base_ptr->phi;
	wrap180_ptr (&phi_err);

	d = sqrt (x_err * x_err + y_err * y_err);
	d_proj = d * cos (phi_err * M_PI / 180);

	if (fabs (d_proj) > D_TOL)
		{
			base_ptr->w_ref = calc_pid (&phi_loop, phi_err);
			base_ptr->v_ref = calc_pid (&d_loop, d_proj);
		}
	else
		{
			base_ptr->w_ref = 0;
			base_ptr->v_ref = 0;
		}
}

static void
pure_pursuit (uint8_t lookahead_pnt_num, uint8_t lookahead_pnt_num_2)
{
	x_err = curve_ptr->equ_pts_x[curve_cnt] - base_ptr->x;
	y_err = curve_ptr->equ_pts_y[curve_cnt] - base_ptr->y;
	phi_prim_1_err = atan2 (y_err, x_err) * 180 / M_PI + (direction - 1) * 90 - base_ptr->phi;
	wrap180_ptr (&phi_prim_1_err);
	d = sqrt (x_err * x_err + y_err * y_err);
	d_proj = d * cos (phi_prim_1_err * M_PI / 180);
	// projektovana do sledece tacke + sve preostale tacke * POINT_DISTANCE
	d_total = d_proj + curve_ptr->dis - (curve_ptr->num_equ_pts - curve_cnt - 1) * POINT_DISTANCE;

//	if (curve_cnt < curve_ptr->num_equ_pts - lookahead_pnt_num)	// ako ima vise od lookahead_pnt_num
//		{
	x_err_next = curve_ptr->equ_pts_x[curve_cnt + lookahead_pnt_num] - base_ptr->x;
	y_err_next = curve_ptr->equ_pts_y[curve_cnt + lookahead_pnt_num] - base_ptr->y;
	phi_prim_err = atan2 (y_err_next, x_err_next) * 180 / M_PI + (direction - 1) * 90 - base_ptr->phi;
//		}
//	else if (curve_cnt < curve_ptr->num_equ_pts - lookahead_pnt_num_2)	// ako ima manje od lookahead_pnt_num, a vise od lookahead_pnt_num_2
//		{
//			x_err_next = curve_ptr->equ_pts_x[curve_cnt + lookahead_pnt_num_2] - base_ptr->x;
//			y_err_next = curve_ptr->equ_pts_y[curve_cnt + lookahead_pnt_num_2] - base_ptr->y;
//			phi_prim_err = atan2 (y_err_next, x_err_next) * 180 / M_PI + (direction - 1) * 90 - base_ptr->phi;
//		}
//	else	// ako ima manje od lookahead_pnt_num_2
//		{
//			phi_prim_err = curve_ptr->goal_phi - base_ptr->phi;
//		}

	wrap180_ptr (&phi_prim_err);
	base_ptr->w_ref = calc_pid (&phi_curve_loop, phi_prim_err);
	if (cont_move)
		base_ptr->v_ref = direction * base_ptr->v_max;
	else
		base_ptr->v_ref = direction * calc_pid (&d_loop, d_total);

	if (d_proj < D_PROJ_TOL)
		{
			curve_cnt++;
			if (curve_cnt > curve_ptr->num_equ_pts)
				{
					if (fabs (d) < D_2_TOL)
						base_ptr->on_target = 1;
					else
						base_ptr->on_target = 0;
					base_ptr->movement_finished = 1;
					curve_cnt = 0;
					base_ptr->v_ref = 0;
					base_ptr->w_ref = 0;
					set_reg_type (0);

// TODO: zaobilazenje
//		  if (get_avoid_obst_glb ())
//			reset_push_pts_loop ();
					set_curve_ready (0);
				}
		}
}

void
set_reg_type (int8_t type)
{
	reg_type = type;
	prev_reg_type = type;
}
int8_t
move_to_xy (float x, float y, int8_t dir, float v_max, float w_max, int8_t check_sensors)
{
	if (!base_ptr->movement_started)					// ako nije zapoceta kretnja
		{
			move_status = TASK_RUNNING;
			base_ptr->movement_finished = 0;				// i nije zavrsena
			base_ptr->x_ref = x;
			base_ptr->y_ref = y;
			direction = dir;
			base_ptr->v_max = v_max;
			base_ptr->w_max = w_max;
			base_ptr->obstacle_dir = check_sensors;
			base_ptr->movement_started = 1;	// kretnja zapoceta
			set_reg_type (1);
		}
	if (base_ptr->movement_finished)					// ako je zavrsio task kretnje
		{
			move_status = base_ptr->on_target * (-2) + 1; // mapiraj on_target u task_status:  1 (na meti) -> -1 (success); 0 (nije na meti -> 1 (fail)
			reset_v_max ();
			reset_w_max ();
			base_ptr->obstacle_dir = 0;
			base_ptr->movement_started = 0;				// resetuj da je zapoceta kretnja
		}

	return move_status;
}

int8_t
rot_to_phi (float phi, float w_max, int8_t check_sensors)
{
	if (!base_ptr->movement_started)					// ako nije zapoceta kretnja
		{
			move_status = TASK_RUNNING;
			base_ptr->movement_finished = 0;				// i nije zavrsena
			base_ptr->phi_ref = wrap180 (phi);
			base_ptr->w_max = w_max;
			base_ptr->obstacle_dir = check_sensors;
			base_ptr->movement_started = 1;				// kretnja zapoceta
			set_reg_type (-1);
		}
	if (base_ptr->movement_finished)					// ako je zavrsio task kretnje
		{
			move_status = base_ptr->on_target * (-2) + 1; // mapiraj on_target u task_status:  1 (na meti) -> -1 (success); 0 (nije na meti -> 1 (fail)
			reset_v_max ();
			reset_w_max ();
			base_ptr->obstacle_dir = 0;
			base_ptr->movement_started = 0;				// resetuj da je zapoceta kretnja
		}

	return move_status;
}

int8_t
move_on_dir (float distance, int8_t dir, float v_max, int8_t check_sensors)
{
	if (!base_ptr->movement_started)					// ako nije zapoceta kretnja
		{
			move_status = TASK_RUNNING;
			base_ptr->movement_finished = 0;				// i nije zavrsena
			direction = dir;
			base_ptr->v_max = v_max;
			base_ptr->x_ref = base_ptr->x + dir * distance * cos (base_ptr->phi * M_PI / 180);
			base_ptr->y_ref = base_ptr->y + dir * distance * sin (base_ptr->phi * M_PI / 180);
			base_ptr->phi_ref = base_ptr->phi;
			base_ptr->obstacle_dir = check_sensors;
			base_ptr->movement_started = 1;				// kretnja zapoceta
			set_reg_type (1);
		}
	if (base_ptr->movement_finished)					// ako je zavrsio task kretnje
		{
			move_status = base_ptr->on_target * (-2) + 1;	// mapiraj on_target u task_status:  1 (na meti) -> -1 (success); 0 (nije na meti -> 1 (fail)
			reset_v_max ();
			reset_w_max ();
			base_ptr->obstacle_dir = 0;
			base_ptr->movement_started = 0;				// resetuj da je zapoceta kretnja
		}

	return move_status;
}

int8_t
rot_to_xy (float x, float y, int dir, float w_max, int8_t check_sensors)
{
	return rot_to_phi (atan2 (y - base_ptr->y, x - base_ptr->x) * 180 / M_PI + (dir - 1) * 90, w_max, check_sensors);
}

int8_t
rot_relative (float angle, float w_max, int8_t check_sensors)
{
	return rot_to_phi (base_ptr->phi + angle, w_max, check_sensors);
}

int8_t
move_on_path (float x, float y, float phi, int8_t dir, int8_t cont, float v_max, int8_t avoid, int8_t check_sensors)
{

	if (!base_ptr->movement_started)					// ako nije zapoceta kretnja
		{
			move_status = TASK_RUNNING;
			base_ptr->movement_finished = 0;				// i nije zavrsena
			direction = dir;
			create_curve (curve_ptr, x, y, phi, dir, avoid);
			cont_move = cont;
			base_ptr->v_max = v_max;
			base_ptr->w_max = W_MAX_DEF;
			base_ptr->x_ref = base_ptr->x;
			base_ptr->y_ref = base_ptr->y;
			base_ptr->phi_ref = base_ptr->phi;
			base_ptr->obstacle_dir = check_sensors;
			base_ptr->movement_started = 1;				// kretnja zapoceta
			set_reg_type (2);
		}
	if (base_ptr->movement_finished)					// ako je zavrsio task kretnje
		{
			move_status = base_ptr->on_target * (-2) + 1; // mapiraj on_target u task_status:  1 (na meti) -> -1 (success); 0 (nije na meti -> 1 (fail)
			reset_v_max ();
			reset_w_max ();
			base_ptr->obstacle_dir = 0;
			base_ptr->movement_started = 0;				// resetuj da je zapoceta kretnja
		}

	return move_status;
}

void
continue_moving ()
{
	if (prev_reg_type)	// ovde teba da udje samo nakon sto je prethodno bio pozvan stop, odnosno ako je prev_reg_type != 0
		{
			reg_type = prev_reg_type;
			prev_reg_type = 0;
			vel_profile = S_CURVE_VEL_PROFILE;
		}
}

void
stop_moving ()
{
	if (!prev_reg_type)	// ovde treba da udje u ovo samo jednom, kada je prev_reg_type jednak 0, ili ako je reg_type razlicit od 0
		{
			prev_reg_type = reg_type;
			reg_type = 0;
			phase = 0;
			vel_profile = STOPPING;
			reset_pid (&d_loop);
			reset_pid (&phi_loop);
			reset_pid (&phi_curve_loop);
			reset_pid (&v_loop);
			reset_pid (&w_loop);
		}
	base_ptr->v_ref = 0;
	base_ptr->w_ref = 0;
	base_ptr->v_ctrl = 0;
	base_ptr->w_ctrl = 0;
	base_ptr->motor_r_ctrl = 0;
	base_ptr->motor_l_ctrl = 0;
	base_ptr->motor_r_ctrl_uint = 0;
	base_ptr->motor_l_ctrl_uint = 0;
	pwm_right_dc (0);
	pwm_left_dc (0);

}

void
reset_movement ()
{
	reset_v_max ();
	reset_w_max ();
	set_reg_type (0);
	prev_reg_type = 10;
	base_ptr->obstacle_dir = 0;
	base_ptr->movement_finished = 0;
	base_ptr->x_ref = base_ptr->x;
	base_ptr->y_ref = base_ptr->y;
	base_ptr->phi_ref = base_ptr->phi;
	base_ptr->v_ref = 0;
	base_ptr->w_ref = 0;
	base_ptr->v_ctrl = 0;
	base_ptr->w_ctrl = 0;
	base_ptr->motor_r_ctrl = 0;
	base_ptr->motor_l_ctrl = 0;
	base_ptr->motor_r_ctrl_uint = 0;
	base_ptr->motor_l_ctrl_uint = 0;
	reset_pid (&d_loop);
	reset_pid (&phi_loop);
	reset_pid (&phi_curve_loop);
	reset_pid (&v_loop);
	reset_pid (&w_loop);
	pwm_right_dc (0);
	pwm_left_dc (0);
	set_curve_ready (0);
	base_ptr->movement_started = 0;
}

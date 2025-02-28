/*
 * regulation.c
 *
 *  Created on: Dec 4, 2024
 *      Author: lazar
 */

// Tolerancije za distancu i ugao kada smatram da je zadovoljen uslov
#define D_TOL			20
#define D_2_TOL			75
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

void
regulation_init ()
{
	base_ptr = get_robot_base ();
	init_pid (&d_loop, 0.0024, 0.002, 0, V_MAX_DEF, V_MAX_DEF * 0.2);
	init_pid (&phi_loop, 3.2, 0.02, 0.2, W_MAX_DEF, W_MAX_DEF * 0.2);
	// TODO: jos ovo
	init_pid (&phi_curve_loop, 1.2, 0.1, 0, W_MAX_DEF, W_MAX_DEF * 0.2);
	init_pid (&v_loop, 6000, 30, 0, CTRL_MAX, 1600);
	init_pid (&w_loop, 64, 0.32, 6, CTRL_MAX, 1600);
	curve_ptr = (curve*) malloc (sizeof(curve));
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
				case -1:
					rotate ();
					break;
				case 0:
					base_ptr->v_ref = 0;
					base_ptr->w_ref = 0;
//		  pos_hold ();
					break;
				case 1:
					go_to_xy ();
					break;
				case 2:
					pure_pursuit (10, 5);
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
			base_ptr->v_ref = 0;
			base_ptr->w_ref = 0;
			set_reg_type (0);
			base_ptr->on_target = 1;
			base_ptr->movement_finished = 1;
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

			base_ptr->v_ref = calc_pid (&d_loop, d_proj);

			if (fabs (d) > D_2_TOL)
				base_ptr->w_ref = calc_pid (&phi_loop, phi_err);
			else
				base_ptr->w_ref = 0;

			if (d_proj < 0 && !(base_ptr->moving))
				{
					base_ptr->v_ref = 0;
					base_ptr->w_ref = 0;
					phase = 0;
					set_reg_type (0);
					base_ptr->movement_finished = 1;
// TODO: testiraj
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

	if (curve_cnt < curve_ptr->num_equ_pts - lookahead_pnt_num)	// ako ima vise od lookahead_pnt_num
		{
			x_err_next = curve_ptr->equ_pts_x[curve_cnt + lookahead_pnt_num] - base_ptr->x;
			y_err_next = curve_ptr->equ_pts_y[curve_cnt + lookahead_pnt_num] - base_ptr->y;
			phi_prim_err = atan2 (y_err_next, x_err_next) * 180 / M_PI + (direction - 1) * 90 - base_ptr->phi;
		}
	else if (curve_cnt < curve_ptr->num_equ_pts - lookahead_pnt_num_2)	// ako ima manje od lookahead_pnt_num, a vise od lookahead_pnt_num_2
		{
			x_err_next = curve_ptr->equ_pts_x[curve_cnt + lookahead_pnt_num_2] - base_ptr->x;
			y_err_next = curve_ptr->equ_pts_y[curve_cnt + lookahead_pnt_num_2] - base_ptr->y;
			phi_prim_err = atan2 (y_err_next, x_err_next) * 180 / M_PI + (direction - 1) * 90 - base_ptr->phi;
		}
	else	// ako ima manje od lookahead_pnt_num_2
		{
			phi_prim_err = curve_ptr->goal_phi - base_ptr->phi;
		}

	wrap180_ptr (&phi_prim_err);
	base_ptr->w_ref = calc_pid (&phi_curve_loop, phi_prim_err);
	// TODO: testiraj novu
	if (cont_move)
		base_ptr->v_ref = direction * base_ptr->v_max;
	else
		base_ptr->v_ref = direction * calc_pid (&d_loop, d_total);

	if (d_proj < 0)
		{
			curve_cnt++;
			if (curve_cnt > curve_ptr->num_equ_pts)
				{
					curve_cnt = 0;
					base_ptr->v_ref = 0;
					base_ptr->w_ref = 0;
					set_reg_type (0);
					base_ptr->movement_finished = 1;
// TODO: testiraj
					if (fabs (d) < D_TOL)
						base_ptr->on_target = 1;
					else
						base_ptr->on_target = 0;
// TODO: zaobilazenje
//		  if (get_avoid_obst_glb ())
//			reset_push_pts_loop ();
				}
		}
}

void
set_reg_type (int8_t type)
{
	reg_type = type;
}

int8_t
move_to_xy (float x, float y, int8_t dir, float v_max, float w_max)
{
	int8_t move_status = TASK_RUNNING;
	if (!base_ptr->movement_started)					// ako nije zapoceta kretnja
		{
			base_ptr->movement_started = 1;				// kretnja zapoceta
			base_ptr->movement_finished = 0;				// i nije zavrsena
			set_reg_type (1);
			base_ptr->x_ref = x;
			base_ptr->y_ref = y;
			direction = dir;
			base_ptr->v_max = v_max;
			base_ptr->w_max = w_max;
		}
	if (base_ptr->movement_finished)					// ako je zavrsio task kretnje
		{
			base_ptr->movement_started = 0;				// resetuj da je zapoceta kretnja
			move_status = base_ptr->on_target * (-2) + 1; // mapiraj on_target u task_status:  1 (na meti) -> -1 (success); 0 (nije na meti -> 1 (fail)
			reset_v_max ();
			reset_w_max ();
			base_ptr->x_ref = base_ptr->x;
			base_ptr->y_ref = base_ptr->y;
			base_ptr->phi_ref = base_ptr->phi;
		}

	return move_status;
}

int8_t
rot_to_phi (float phi, float w_max)
{
	int8_t move_status = TASK_RUNNING;
	if (!base_ptr->movement_started)					// ako nije zapoceta kretnja
		{
			base_ptr->movement_started = 1;				// kretnja zapoceta
			base_ptr->movement_finished = 0;				// i nije zavrsena
			set_reg_type (-1);
			base_ptr->phi_ref = phi;
			base_ptr->w_max = w_max;
		}
	if (base_ptr->movement_finished)					// ako je zavrsio task kretnje
		{
			base_ptr->movement_started = 0;				// resetuj da je zapoceta kretnja
			move_status = base_ptr->on_target * (-2) + 1; // mapiraj on_target u task_status:  1 (na meti) -> -1 (success); 0 (nije na meti -> 1 (fail)
			reset_v_max ();
			reset_w_max ();
			base_ptr->x_ref = base_ptr->x;
			base_ptr->y_ref = base_ptr->y;
			base_ptr->phi_ref = base_ptr->phi;
		}

	return move_status;
}

int8_t
move_on_dir (float distance, int8_t dir, float v_max)
{
	int8_t move_status = TASK_RUNNING;
	if (!base_ptr->movement_started)					// ako nije zapoceta kretnja
		{
			base_ptr->movement_started = 1;				// kretnja zapoceta
			base_ptr->movement_finished = 0;				// i nije zavrsena
			set_reg_type (1);
			direction = dir;
			base_ptr->v_max = v_max;
			base_ptr->phi_ref = base_ptr->phi;
		}
	if (base_ptr->movement_finished)					// ako je zavrsio task kretnje
		{
			base_ptr->movement_started = 0;				// resetuj da je zapoceta kretnja
			move_status = base_ptr->on_target * (-2) + 1;	// mapiraj on_target u task_status:  1 (na meti) -> -1 (success); 0 (nije na meti -> 1 (fail)
			reset_v_max ();
			reset_w_max ();
			base_ptr->x_ref = base_ptr->x;
			base_ptr->y_ref = base_ptr->y;
			base_ptr->phi_ref = base_ptr->phi;
		}

	return move_status;
}

int8_t
rot_to_xy (float x, float y, int dir, float w_max)
{
	return rot_to_phi (atan2 (y - base_ptr->y, x - base_ptr->x) * 180 / M_PI + (dir - 1) * 90, w_max);
}

int8_t
move_on_path (float x, float y, float phi, int8_t dir, int cont, float v_max, int avoid)
{
	int8_t move_status = TASK_RUNNING;
	if (!base_ptr->movement_started)					// ako nije zapoceta kretnja
		{
			base_ptr->movement_started = 1;				// kretnja zapoceta
			base_ptr->movement_finished = 0;				// i nije zavrsena
			set_reg_type (2);
			direction = dir;
			base_ptr->v_max = v_max;
			base_ptr->w_max = W_MAX_DEF;
			create_curve (curve_ptr, x, y, phi, dir, avoid);
		}
	if (base_ptr->movement_finished)					// ako je zavrsio task kretnje
		{
			base_ptr->movement_started = 0;				// resetuj da je zapoceta kretnja
			move_status = base_ptr->on_target * (-2) + 1; // mapiraj on_target u task_status:  1 (na meti) -> -1 (success); 0 (nije na meti -> 1 (fail)
			reset_v_max ();
			reset_w_max ();
			base_ptr->x_ref = base_ptr->x;
			base_ptr->y_ref = base_ptr->y;
			base_ptr->phi_ref = base_ptr->phi;
		}

	return move_status;
}

// void move_on_path(float x, float y, float phi, int dir, bool cont, float cruising_vel)
// {
// movement_started();
// set_reg_type(2);
// set_curve_ptr((curve *)malloc(sizeof(curve)));
// create_curve(get_curve_ptr(), create_target(x, y, phi), dir);
// set_dir(dir);
// cont_move = cont;
// set_cruising_vel(cruising_vel);
// }


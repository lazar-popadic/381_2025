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
follow_curve ();

static int8_t reg_type = 0;
static int8_t phase = 0;
static int8_t direction;

static float x_err = 0;				// [mm]
static float y_err = 0;				// [mm]
static float phi_err = 0;			// [deg]
static float phi_prim_err = 0;		// [deg]
static float d = 0;					// [mm]
static float d_proj = 0;			// [mm]
static float phi_prim_1_err = 0;	// [deg]
static float phi_prim_2_err = 0;	// [deg]
static float phi_final_err = 0;		// [deg]

static volatile float motor_left_pid;      	// pid duty cycle
static volatile float motor_right_pid;   		// pid duty cycle
static volatile uint16_t motor_left_ctrl;    	// control duty cycle
static volatile uint16_t motor_right_ctrl; 	// control duty cycle

static volatile struct_robot_base *base_ptr;
static volatile struct_pid d_loop;
static volatile struct_pid phi_loop;
static volatile struct_pid phi_curve_loop;

void
regulation_init ()
{
  base_ptr = get_robot_base ();
  init_pid (&d_loop, 1, 0, 0, 4200, 1);
  init_pid (&phi_loop, 1, 0, 0, 4200, 1);
  init_pid (&phi_curve_loop, 1, 0, 0, 4200, 1);
}

void
position_loop ()
{

}

void
move ()
{
  switch (reg_type)
	{
	case -1:
	  rotate ();
	  break;
	case 1:
	  go_to_xy ();
	  break;
	case 2:
	  follow_curve ();
	  break;
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
	  base_ptr->movement_finished = 1;
	}
}

static void
go_to_xy ()
{
  x_err = base_ptr->x_ref - base_ptr->x;
  y_err = base_ptr->y_ref - base_ptr->y;
  phi_prim_err = atan2 (y_err, x_err) * 180 / M_PI + direction * 180 - base_ptr->phi;
  wrap180_ptr (&phi_prim_err);

  switch (phase)
	{
	case 0: // rot2pos
	  base_ptr->v_ref = 0;
	  base_ptr->w_ref = calc_pid (&phi_loop, phi_prim_err);
	  if (fabs (phi_prim_err) < PHI_PRIM_TOL)
		phase = 1;
	  break;

	case 1: // tran
	  d = (direction * (-2) + 1) * sqrt (x_err * x_err + y_err * y_err);
	  d_proj = d * cos (phi_prim_err * M_PI / 180);

	  if (fabs (phi_prim_err) > 90)
		base_ptr->v_ref = -calc_pid (&d_loop, d_proj);				// d_proj ili d
	  else
		base_ptr->v_ref = calc_pid (&d_loop, d_proj);				// d_proj ili d

	  if (fabs (d) > D_2_TOL)
		base_ptr->w_ref = calc_pid (&phi_loop, phi_prim_err);
	  else
		base_ptr->w_ref = 0;

	  if (fabs (d_proj) < 0)
		{
		  base_ptr->v_ref = 0;
		  base_ptr->w_ref = 0;
		  phase = 0;
		  base_ptr->movement_finished = 1;
		}
	  break;
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
  static int8_t move_status = TASK_RUNNING;
  if (!base_ptr->movement_started)						// ako nije zapoceta kretnja
	{
	  base_ptr->movement_started = 1;					// kretnja zapoceta
	  base_ptr->movement_finished = 0;					// i nije zavrsena
	  set_reg_type (1);
	  base_ptr->x_ref = x;
	  base_ptr->y_ref = y;
	  direction = dir;
	  base_ptr->v_max = v_max;
	  base_ptr->w_max = w_max;
	}
  if (base_ptr->movement_finished)						// ako je zavrsio task kretnje
	{
	  base_ptr->movement_started = 0;					// resetuj da je zapoceta kretnja
	  move_status = base_ptr->on_target * (-2) + 1;		// mapiraj on_target u task_status:  1 (na meti) -> -1 (success); 0 (nije na meti -> 1 (fail)
	  reset_v_max ();
	  reset_w_max ();
	}

  return move_status;
}

int8_t
rot_to_phi (float phi, float w_max)
{
  static int8_t move_status = TASK_RUNNING;
  if (!base_ptr->movement_started)						// ako nije zapoceta kretnja
	{
	  base_ptr->movement_started = 1;					// kretnja zapoceta
	  base_ptr->movement_finished = 0;					// i nije zavrsena
	  set_reg_type (-1);
	  base_ptr->phi_ref = phi;
	  base_ptr->w_max = w_max;
	}
  if (base_ptr->movement_finished)						// ako je zavrsio task kretnje
	{
	  base_ptr->movement_started = 0;					// resetuj da je zapoceta kretnja
	  move_status = base_ptr->on_target * (-2) + 1;		// mapiraj on_target u task_status:  1 (na meti) -> -1 (success); 0 (nije na meti -> 1 (fail)
	  reset_w_max ();
	}

  return move_status;
}

int8_t
move_on_dir (float distance, int8_t dir, float v_max)
{
  static int8_t move_status = TASK_RUNNING;
  if (!base_ptr->movement_started)						// ako nije zapoceta kretnja
	{
	  base_ptr->movement_started = 1;					// kretnja zapoceta
	  base_ptr->movement_finished = 0;					// i nije zavrsena
	  set_reg_type (1);
	  base_ptr->x_ref = base_ptr->x + (dir * (-2) + 1) * distance * cos(base_ptr->phi * M_PI / 180);
	  base_ptr->y_ref = base_ptr->y + (dir * (-2) + 1) * distance * sin(base_ptr->phi * M_PI / 180);
	  direction = dir;
	  base_ptr->v_max = v_max;
	}
  if (base_ptr->movement_finished)						// ako je zavrsio task kretnje
	{
	  base_ptr->movement_started = 0;					// resetuj da je zapoceta kretnja
	  move_status = base_ptr->on_target * (-2) + 1;		// mapiraj on_target u task_status:  1 (na meti) -> -1 (success); 0 (nije na meti -> 1 (fail)
	  reset_v_max ();
	}

  return move_status;
}

int8_t
rot_to_xy(float x, float y, int dir, float w_max)
{
  return rot_to_phi (atan2(y - base_ptr->y, x - base_ptr->x) * 180 / M_PI + dir * 180, w_max);
}

/*

 void move_on_path(double x, double y, double phi, int dir, bool cont, double cruising_vel)
 {
 movement_started();
 set_reg_type(2);
 set_curve_ptr((curve *)malloc(sizeof(curve)));
 create_curve(get_curve_ptr(), create_target(x, y, phi), dir);
 // if (create_curve(get_curve_ptr(), create_target(x, y, phi), dir))
 // return 1;    // TODO: ovde uradi testiranje drugih krivih
 set_dir(dir);
 cont_move = cont;
 set_cruising_vel(cruising_vel);
 }

 */


/*
 * structs.h
 *
 *  Created on: Nov 9, 2024
 *      Author: lazar
 */

#ifndef INC_STRUCTS_H_
#define INC_STRUCTS_H_

typedef struct st_robot_base
{
	// position, velocity, acceleration of base
	float x;						// [mm]
	float y;						// [mm]
	float phi;					// [deg]
	float v;						// [mm/ms = m/s]
	float w;						// [deg/s]
	float a;						// [mm/ms * 1/ms]
	float alpha;					// [deg/s * 1/ms]
	float v_left;					// [mm/ms = m/s]
	float v_right;				// [mm/ms = m/s]
	float v_prev;					// [mm/ms = m/s]
	float w_prev;					// [deg/s]

	// references for position, velocity of base
	float v_ref;					// [mm/ms = m/s]
	float w_ref;					// [deg/s]
	float x_ref;					// [mm]
	float y_ref;					// [mm]
	float phi_ref;				// [deg]

	// limits for velocity, acceleration, jerk of base
	float v_max;					// [mm/ms = m/s]
	float w_max;					// [deg/s]
	float a_max;					// [mm/ms * 1/ms]
	float alpha_max;				// [deg/s * 1/ms]
	float j_max;					// [mm/ms * 1/ms^2]
	float j_rot_max;				// [deg/s * 1/ms^2]

	// control signals
	float v_ctrl;
	float w_ctrl;
	float motor_l_ctrl;
	float motor_r_ctrl;
	int8_t motor_l_dir :2;
	int8_t motor_r_dir :2;
	uint16_t motor_l_ctrl_uint;
	uint16_t motor_r_ctrl_uint;

	// status
	uint8_t moving :1;			// statusi po poziciji i brzini baze
	uint8_t on_target :1;			// uspesno odradjen task kretnje
	uint8_t movement_started :1;	// zapocet task kretnje
	uint8_t movement_finished :1;	// zavrsen task kretnje
	uint8_t obstacle_detected :1;	// prepreka u pravcu kretnje sa upaljenim senzorima
	int8_t obstacle_dir;			// koji senzori su upaljeni
	uint8_t regulation_status :1;	// status regulacije

} robot_base;

typedef struct st_mechanism_states
{
	uint8_t lift_front :3;	// 5 pozicija: down = 0, carry = 1, drop = 2, up = 3, leave = 4
	uint8_t lift_back :3;

	uint8_t grtl_fol :2;		// 3 pozicije: close = 0, grip = 1, open = 2
	uint8_t grtl_fil :2;
	uint8_t grtl_for :2;
	uint8_t grtl_fir :2;
	uint8_t grtl_bol :2;
	uint8_t grtl_bil :2;
	uint8_t grtl_bor :2;
	uint8_t grtl_bir :2;

	uint8_t ruc_front :2;		// 3 pozicije: up = 0, mid = 1, down = 2
	uint8_t ruc_back :2;

	uint8_t gurl :2;				// 3 pozicije: mid = 0, front = 1, back = 2

	uint8_t bnr :3; 				// 5 poz: closed = 0, 1 = 1, ...

} mech_states;

typedef struct st_pid
{
	float p;
	float i;
	float d;
	float lmt;
	float ctrl;
	float ctrl_p;
	float ctrl_pp;
	float err_p;
	float err_sum;
	float err_dif;
	float sum_lmt;

} pid;

typedef struct st_tactic_num
{
	uint8_t side :1;
	uint8_t num :2;

} tactic_num;

typedef struct curve
{
	float pts_x[BEZIER_RESOLUTION];
	float pts_y[BEZIER_RESOLUTION];
	float equ_pts_x[MAX_EQU_PTS];
	float equ_pts_y[MAX_EQU_PTS];
	uint16_t num_equ_pts;
	float dis;
	float goal_phi;
} curve;

#endif /* INC_STRUCTS_H_ */

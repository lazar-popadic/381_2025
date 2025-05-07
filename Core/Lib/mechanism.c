/*
 * mechanism.c
 *
 *  Created on: Jan 25, 2025
 *      Author: lazar
 */

#include "main.h"
#include "usart.h"

#define OFFS 0

#define GRTL_FOL_ID 1	// grtalica front outside left
#define GRTL_FIL_ID 2
#define GRTL_FIR_ID 3
#define GRTL_FOR_ID 4
#define GRTL_BOL_ID 9
#define GRTL_BIL_ID 8
#define GRTL_BIR_ID	7	// grtalica back inside right
#define GRTL_BOR_ID 6
#define LIFT_F_ID	5	// lift front
#define LIFT_B_ID	10	// lift back
#define RUC_FL_ID	13	// rucica front left
#define RUC_FR_ID	14
#define RUC_BL_ID	12
#define RUC_BR_ID	11	// rucica back right
#define GURL_L_ID	15	// guralica left
#define GURL_R_ID	16	// guralica right

#define GRTL_OL_OPEN		551
#define GRTL_OL_OPEN_S1	511
#define GRTL_OL_GRIP		476
#define GRTL_OL_CLOSE		181
#define GRTL_IL_OPEN		511
#define GRTL_IL_GRIP		451
#define GRTL_IL_CLOSE		251
#define GRTL_OR_OPEN		471
#define GRTL_OR_OPEN_S1	511
#define GRTL_OR_GRIP		556
#define GRTL_OR_CLOSE		841
#define GRTL_IR_OPEN		511
#define GRTL_IR_GRIP		571
#define GRTL_IR_CLOSE		781
#define GRTL_SPEED_FAST		500
#define GRTL_SPEED_SLOW		100

#define LIFT_DOWN		0
#define LIFT_CARRY	100
#define LIFT_UP			1023
#define LIFT_DROP		850
#define LIFT_LEAVE	750
#define LIFT_BNR		400
#define LIFT_SPEED_FAST	500
#define LIFT_SPEED_SLOW	300

#define RUC_L_UP				531
#define RUC_L_MID				320
#define RUC_L_CARRY			215
#define RUC_L_DOWN			200
#define RUC_L_FULL_DOWN	160
#define RUC_R_UP				491
#define RUC_R_MID				705
#define RUC_R_CARRY			800
#define RUC_R_DOWN			815
#define RUC_R_FULL_DOWN	865
#define RUC_SPEED_FAST		500
#define RUC_SPEED_SLOW		200

#define GURL_FULL_LEFT		973
#define GURL_MID					511
#define GURL_FULL_RIGHT		50
#define GURL_FAST					500
#define GURL_SLOW					250

#define BNR_ID		18
#define BNR_0			426 //start
#define BNR_1			266 //donji levi
#define BNR_2			41 //gornji levi
#define BNR_3			635 //donji desni
#define BNR_4			850 //gornji desni
#define BNR_SPEED_FAST	500

int16_t ax_1_offs = 58;

mech_states mechanism_states;

mech_states
get_mech_states ()
{
	return mechanism_states;
}

void
ax_init ()
{
	torque_enable (1);
	torque_enable (2);
	torque_enable (3);
	torque_enable (4);
	torque_enable (5);
	torque_enable (6);
	torque_enable (7);
	torque_enable (8);
	torque_enable (9);
	torque_enable (10);
	torque_enable (11);
	torque_enable (12);
	torque_enable (13);
	torque_enable (14);
	torque_enable (15);
	torque_enable (16);
	torque_enable (18);
}

void
mechanism_init ()
{
	bnr_close ();
	gurl_mid ();
	HAL_Delay (200);
	ruc_front_up ();
	HAL_Delay (50);
	ruc_back_up ();
	HAL_Delay (200);
	grtl_front_close ();
	HAL_Delay (250);
	grtl_back_close ();
	HAL_Delay (250);
	lift_front_down ();
	HAL_Delay (30);
	lift_back_drop ();
}

void
prepare_front ()
{
	grtl_front_open ();
	ruc_front_mid ();
}

void
prepare_back ()
{
	grtl_back_open ();
	ruc_back_mid ();
}

void
lift_front_up ()
{
	ax_move (LIFT_F_ID, LIFT_UP, LIFT_SPEED_FAST, huart6);
	mechanism_states.lift_front = 3;
}

void
lift_front_down ()
{
	ax_move (LIFT_F_ID, LIFT_DOWN, LIFT_SPEED_FAST, huart6);
	mechanism_states.lift_front = 0;
}

void
lift_front_drop ()
{
	ax_move (LIFT_F_ID, LIFT_DROP, LIFT_SPEED_SLOW, huart6);
	mechanism_states.lift_front = 2;
}

void
lift_front_carry ()
{
	ax_move (LIFT_F_ID, LIFT_CARRY, LIFT_SPEED_SLOW, huart6);
	mechanism_states.lift_front = 1;
}

void
lift_front_leave ()
{
	ax_move (LIFT_F_ID, LIFT_LEAVE, LIFT_SPEED_SLOW, huart6);
	mechanism_states.lift_front = 4;
}

void
lift_back_up ()
{
	ax_move (LIFT_B_ID, LIFT_UP, LIFT_SPEED_FAST, huart6);
	mechanism_states.lift_back = 3;
}

void
lift_back_down ()
{
	ax_move (LIFT_B_ID, LIFT_DOWN, LIFT_SPEED_FAST, huart6);
	mechanism_states.lift_back = 0;
}

void
lift_back_drop ()
{
	ax_move (LIFT_B_ID, LIFT_DROP, LIFT_SPEED_SLOW, huart6);
	mechanism_states.lift_back = 2;
}

void
lift_back_carry ()
{
	ax_move (LIFT_B_ID, LIFT_CARRY, LIFT_SPEED_SLOW, huart6);
	mechanism_states.lift_back = 1;
}

void
lift_back_leave ()
{
	ax_move (LIFT_B_ID, LIFT_LEAVE, LIFT_SPEED_SLOW, huart6);
	mechanism_states.lift_back = 4;
}

void
lift_back_down_bnr ()
{
	ax_move (LIFT_B_ID, LIFT_BNR, LIFT_SPEED_SLOW, huart6);
	mechanism_states.lift_back = 4;
}

void
grtl_front_open ()
{
	ax_move (GRTL_FOR_ID, GRTL_OR_OPEN, GRTL_SPEED_FAST, huart6);
	ax_move (GRTL_FOL_ID, GRTL_OL_OPEN + ax_1_offs, GRTL_SPEED_FAST, huart6);
	ax_move (GRTL_FIR_ID, GRTL_IR_OPEN, GRTL_SPEED_FAST, huart6);
	ax_move (GRTL_FIL_ID, GRTL_IL_OPEN, GRTL_SPEED_FAST, huart6);
	mechanism_states.grtl_fil = 2;
	mechanism_states.grtl_fol = 2;
	mechanism_states.grtl_fir = 2;
	mechanism_states.grtl_for = 2;
}

void
grtl_back_open ()
{
	ax_move (GRTL_BOR_ID, GRTL_OR_OPEN, GRTL_SPEED_FAST, huart6);
	ax_move (GRTL_BOL_ID, GRTL_OL_OPEN, GRTL_SPEED_FAST, huart6);
	ax_move (GRTL_BIR_ID, GRTL_IR_OPEN, GRTL_SPEED_FAST, huart6);
	ax_move (GRTL_BIL_ID, GRTL_IL_OPEN, GRTL_SPEED_FAST, huart6);
	mechanism_states.grtl_bil = 2;
	mechanism_states.grtl_bol = 2;
	mechanism_states.grtl_bir = 2;
	mechanism_states.grtl_bor = 2;
}

void
grtl_front_close ()
{
	ax_move (GRTL_FOR_ID, GRTL_OR_CLOSE, GRTL_SPEED_FAST, huart6);
	ax_move (GRTL_FOL_ID, GRTL_OL_CLOSE + ax_1_offs, GRTL_SPEED_FAST, huart6);
	ax_move (GRTL_FIR_ID, GRTL_IR_CLOSE, GRTL_SPEED_FAST, huart6);
	ax_move (GRTL_FIL_ID, GRTL_IL_CLOSE, GRTL_SPEED_FAST, huart6);
	mechanism_states.grtl_fil = 0;
	mechanism_states.grtl_fol = 0;
	mechanism_states.grtl_fir = 0;
	mechanism_states.grtl_for = 0;
}

void
grtl_back_close ()
{
	ax_move (GRTL_BOR_ID, GRTL_OR_CLOSE, GRTL_SPEED_FAST, huart6);
	ax_move (GRTL_BOL_ID, GRTL_OL_CLOSE, GRTL_SPEED_FAST, huart6);
	ax_move (GRTL_BIR_ID, GRTL_IR_CLOSE, GRTL_SPEED_FAST, huart6);
	ax_move (GRTL_BIL_ID, GRTL_IL_CLOSE, GRTL_SPEED_FAST, huart6);
	mechanism_states.grtl_bil = 0;
	mechanism_states.grtl_bol = 0;
	mechanism_states.grtl_bir = 0;
	mechanism_states.grtl_bor = 0;
}

void
grtl_front_grip_all ()
{
	ax_move (GRTL_FOR_ID, GRTL_OR_GRIP, GRTL_SPEED_SLOW, huart6);
	ax_move (GRTL_FOL_ID, GRTL_OL_GRIP - OFFS + ax_1_offs, GRTL_SPEED_SLOW, huart6);
	ax_move (GRTL_FIR_ID, GRTL_IR_GRIP, GRTL_SPEED_SLOW, huart6);
	ax_move (GRTL_FIL_ID, GRTL_IL_GRIP, GRTL_SPEED_SLOW, huart6);
	mechanism_states.grtl_fil = 1;
	mechanism_states.grtl_fol = 1;
	mechanism_states.grtl_fir = 1;
	mechanism_states.grtl_for = 1;
}

void
grtl_back_grip_all ()
{
	ax_move (GRTL_BOR_ID, GRTL_OR_GRIP, GRTL_SPEED_SLOW, huart6);
	ax_move (GRTL_BOL_ID, GRTL_OL_GRIP, GRTL_SPEED_SLOW, huart6);
	ax_move (GRTL_BIR_ID, GRTL_IR_GRIP, GRTL_SPEED_SLOW, huart6);
	ax_move (GRTL_BIL_ID, GRTL_IL_GRIP, GRTL_SPEED_SLOW, huart6);
	mechanism_states.grtl_bil = 1;
	mechanism_states.grtl_bol = 1;
	mechanism_states.grtl_bir = 1;
	mechanism_states.grtl_bor = 1;
}

void
grtl_front_grip_inside ()
{
	ax_move (GRTL_FIR_ID, GRTL_IR_GRIP, GRTL_SPEED_SLOW, huart6);
	ax_move (GRTL_FIL_ID, GRTL_IL_GRIP, GRTL_SPEED_SLOW, huart6);
	mechanism_states.grtl_fil = 1;
	mechanism_states.grtl_fir = 1;
}

void
grtl_back_grip_inside ()
{
	ax_move (GRTL_BIR_ID, GRTL_IR_GRIP, GRTL_SPEED_SLOW, huart6);
	ax_move (GRTL_BIL_ID, GRTL_IL_GRIP, GRTL_SPEED_SLOW, huart6);
	mechanism_states.grtl_bil = 1;
	mechanism_states.grtl_bir = 1;
}

void
grtl_front_grip_outside ()
{
	ax_move (GRTL_FOR_ID, GRTL_OR_GRIP, GRTL_SPEED_SLOW, huart6);
	ax_move (GRTL_FOL_ID, GRTL_OL_GRIP - OFFS + ax_1_offs, GRTL_SPEED_SLOW, huart6);
	mechanism_states.grtl_fol = 1;
	mechanism_states.grtl_for = 1;
}

void
grtl_back_grip_outside ()
{
	ax_move (GRTL_BOR_ID, GRTL_OR_GRIP, GRTL_SPEED_SLOW, huart6);
	ax_move (GRTL_BOL_ID, GRTL_OL_GRIP, GRTL_SPEED_SLOW, huart6);
	mechanism_states.grtl_bol = 1;
	mechanism_states.grtl_bor = 1;
}

void
grtl_front_open_inside ()
{
	ax_move (GRTL_FIR_ID, GRTL_IR_OPEN, GRTL_SPEED_FAST, huart6);
	ax_move (GRTL_FIL_ID, GRTL_IL_OPEN, GRTL_SPEED_FAST, huart6);
	mechanism_states.grtl_fil = 2;
	mechanism_states.grtl_fir = 2;
}

void
grtl_back_open_inside ()
{
	ax_move (GRTL_BIR_ID, GRTL_IR_OPEN, GRTL_SPEED_FAST, huart6);
	ax_move (GRTL_BIL_ID, GRTL_IL_OPEN, GRTL_SPEED_FAST, huart6);
	mechanism_states.grtl_bil = 2;
	mechanism_states.grtl_bir = 2;
}

void
grtl_front_open_outside ()
{
	ax_move (GRTL_FOR_ID, GRTL_OR_OPEN, GRTL_SPEED_FAST, huart6);
	ax_move (GRTL_FOL_ID, GRTL_OL_OPEN + ax_1_offs, GRTL_SPEED_FAST, huart6);
	mechanism_states.grtl_fol = 2;
	mechanism_states.grtl_for = 2;
}

void
grtl_back_open_outside ()
{
	ax_move (GRTL_BOR_ID, GRTL_OR_OPEN, GRTL_SPEED_FAST, huart6);
	ax_move (GRTL_BOL_ID, GRTL_OL_OPEN, GRTL_SPEED_FAST, huart6);
	mechanism_states.grtl_bol = 2;
	mechanism_states.grtl_bor = 2;
}

void
grtl_front_open_outside_s1 ()
{
	ax_move (GRTL_FOR_ID, GRTL_OR_OPEN_S1, GRTL_SPEED_SLOW, huart6);
	ax_move (GRTL_FOL_ID, GRTL_OL_OPEN_S1 + ax_1_offs, GRTL_SPEED_SLOW, huart6);
	mechanism_states.grtl_fol = 2;
	mechanism_states.grtl_for = 2;
}

void
grtl_back_open_outside_s1 ()
{
	ax_move (GRTL_BOR_ID, GRTL_OR_OPEN_S1, GRTL_SPEED_SLOW, huart6);
	ax_move (GRTL_BOL_ID, GRTL_OL_OPEN_S1, GRTL_SPEED_SLOW, huart6);
	mechanism_states.grtl_bol = 2;
	mechanism_states.grtl_bor = 2;
}

void
ruc_front_down ()
{
	ax_move (RUC_FL_ID, RUC_L_DOWN, RUC_SPEED_FAST, huart6);
	ax_move (RUC_FR_ID, RUC_R_DOWN, RUC_SPEED_FAST, huart6);
	mechanism_states.ruc_front = 2;
}

void
ruc_back_down ()
{
	ax_move (RUC_BL_ID, RUC_L_DOWN, RUC_SPEED_FAST, huart6);
	ax_move (RUC_BR_ID, RUC_R_DOWN, RUC_SPEED_FAST, huart6);
	mechanism_states.ruc_back = 2;
}

void
ruc_front_full_down ()
{
	ax_move (RUC_FL_ID, RUC_L_FULL_DOWN, RUC_SPEED_FAST, huart6);
	ax_move (RUC_FR_ID, RUC_R_FULL_DOWN, RUC_SPEED_FAST, huart6);
	mechanism_states.ruc_front = 4;
}

void
ruc_back_full_down ()
{
	ax_move (RUC_BL_ID, RUC_L_FULL_DOWN, RUC_SPEED_FAST, huart6);
	ax_move (RUC_BR_ID, RUC_R_FULL_DOWN, RUC_SPEED_FAST, huart6);
	mechanism_states.ruc_back = 4;
}

void
ruc_front_mid ()
{
	ax_move (RUC_FL_ID, RUC_L_MID, RUC_SPEED_SLOW, huart6);
	ax_move (RUC_FR_ID, RUC_R_MID, RUC_SPEED_SLOW, huart6);
	mechanism_states.ruc_front = 1;
}

void
ruc_back_mid ()
{
	ax_move (RUC_BL_ID, RUC_L_MID, RUC_SPEED_SLOW, huart6);
	ax_move (RUC_BR_ID, RUC_R_MID, RUC_SPEED_SLOW, huart6);
	mechanism_states.ruc_back = 1;
}

void
ruc_front_up ()
{

	ax_move (RUC_FL_ID, RUC_L_UP, RUC_SPEED_SLOW, huart6);
	ax_move (RUC_FR_ID, RUC_R_UP, RUC_SPEED_SLOW, huart6);
	mechanism_states.ruc_front = 0;
}

void
ruc_back_up ()
{

	ax_move (RUC_BL_ID, RUC_L_UP, RUC_SPEED_SLOW, huart6);
	ax_move (RUC_BR_ID, RUC_R_UP, RUC_SPEED_SLOW, huart6);
	mechanism_states.ruc_back = 3;
}

void
ruc_front_carry ()
{
	ax_move (RUC_FL_ID, RUC_L_CARRY, RUC_SPEED_FAST, huart6);
	ax_move (RUC_FR_ID, RUC_R_CARRY, RUC_SPEED_FAST, huart6);
	mechanism_states.ruc_front = 3;
}

void
ruc_back_carry ()
{
	ax_move (RUC_BL_ID, RUC_L_CARRY, RUC_SPEED_FAST, huart6);
	ax_move (RUC_BR_ID, RUC_R_CARRY, RUC_SPEED_FAST, huart6);
	mechanism_states.ruc_back = 0;
}

void
gurl_front ()
{
	ax_move (GURL_L_ID, GURL_FULL_LEFT, GURL_SLOW, huart6);
	ax_move (GURL_R_ID, GURL_FULL_RIGHT, GURL_SLOW, huart6);
	mechanism_states.gurl = 1;
}

void
gurl_back ()
{
	ax_move (GURL_L_ID, GURL_FULL_RIGHT, GURL_SLOW, huart6);
	ax_move (GURL_R_ID, GURL_FULL_LEFT, GURL_SLOW, huart6);
	mechanism_states.gurl = 2;
}

void
gurl_mid ()
{
	ax_move (GURL_L_ID, GURL_MID, GURL_FAST, huart6);
	ax_move (GURL_R_ID, GURL_MID, GURL_FAST, huart6);
	mechanism_states.gurl = 0;
}

void
vacuum_back (uint8_t on)
{
	HAL_Delay (120);
	vacuum_0 (on);
	HAL_Delay (120);
	vacuum_1 (on);
	HAL_Delay (120);
}

void
vacuum_front (uint8_t on)
{
	HAL_Delay (120);
	vacuum_2 (on);
	HAL_Delay (120);
	vacuum_3 (on);
	HAL_Delay (120);
}

void
bnr_close ()
{
	ax_move (BNR_ID, BNR_0, BNR_SPEED_FAST, huart6);
	mechanism_states.bnr = 0;
}

void
bnr_1 ()
{
	ax_move (BNR_ID, BNR_1, BNR_SPEED_FAST, huart6);
	mechanism_states.bnr = 1;
}

void
bnr_2 ()
{
	ax_move (BNR_ID, BNR_2, BNR_SPEED_FAST, huart6);
	mechanism_states.bnr = 2;
}

void
bnr_3 ()
{
	ax_move (BNR_ID, BNR_3, BNR_SPEED_FAST, huart6);
	mechanism_states.bnr = 3;
}

void
bnr_4 ()
{
	ax_move (BNR_ID, BNR_4, BNR_SPEED_FAST, huart6);
	mechanism_states.bnr = 4;
}

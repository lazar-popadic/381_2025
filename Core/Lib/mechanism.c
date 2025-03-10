/*
 * mechanism.c
 *
 *  Created on: Jan 25, 2025
 *      Author: lazar
 */

#include "main.h"
#include "usart.h"

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
#define RUC_BL_ID	11
#define RUC_BR_ID	12	// rucica back right
#define GURL_L_ID	15	// guralica left
#define GURL_R_ID	16	// guralica right

#define GRTL_OL_OPEN		550
#define GRTL_OL_GRIP		430
#define GRTL_OL_CLOSE		180
#define GRTL_IL_OPEN		490
#define GRTL_IL_GRIP		430
#define GRTL_IL_CLOSE		280
#define GRTL_OR_OPEN		472
#define GRTL_OR_GRIP		592
#define GRTL_OR_CLOSE		842
#define GRTL_IR_OPEN		532
#define GRTL_IR_GRIP		592
#define GRTL_IR_CLOSE		742
#define GRTL_SPEED_FAST		500
#define GRTL_SPEED_SLOW		100

#define LIFT_DOWN		0
#define LIFT_UP			1023
#define LIFT_DROP		900
#define LIFT_SPEED_FAST	400
#define LIFT_SPEED_SLOW	200

#define RUC_L_UP			511
#define RUC_L_MID			320
#define RUC_L_DOWN		180
#define RUC_R_UP			511
#define RUC_R_MID			705
#define RUC_R_DOWN		845
#define RUC_SPEED_FAST		500
#define RUC_SPEED_SLOW		300

#define GURL_FULL_LEFT		973
#define GURL_MID					511
#define GURL_FULL_RIGHT		50
#define GURL_FAST					500
#define GURL_SLOW					150

mech_states mechanism_states;

mech_states
get_mech_states ()
{
	return mechanism_states;
}

void
mechanism_init ()
{
	lift_front_down ();
	lift_back_down ();
	grtl_front_close ();
	grtl_back_close ();
	ruc_front_up ();
	ruc_back_up ();
	gurl_mid ();
}

void
prepare_front ()
{
	grtl_front_open ();
	ruc_front_mid ();
}

void
lift_front_up ()
{
	ax_move (LIFT_F_ID, LIFT_UP, LIFT_SPEED_FAST, huart6);
	mechanism_states.lift_front = 2;
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
	mechanism_states.lift_front = 1;
}

void
lift_back_up ()
{
	ax_move (LIFT_B_ID, LIFT_UP, LIFT_SPEED_FAST, huart6);
	mechanism_states.lift_back = 2;
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
	mechanism_states.lift_back = 1;
}

void
grtl_front_open ()
{
	ax_move (GRTL_FOL_ID, GRTL_OL_OPEN, GRTL_SPEED_FAST, huart6);
	ax_move (GRTL_FOR_ID, GRTL_OR_OPEN, GRTL_SPEED_FAST, huart6);
	HAL_Delay (20);
	ax_move (GRTL_FIL_ID, GRTL_IL_OPEN, GRTL_SPEED_FAST, huart6);
	ax_move (GRTL_FIR_ID, GRTL_IR_OPEN, GRTL_SPEED_FAST, huart6);
	mechanism_states.grtl_fil = 2;
	mechanism_states.grtl_fol = 2;
	mechanism_states.grtl_fir = 2;
	mechanism_states.grtl_for = 2;
}

void
grtl_back_open ()
{
	ax_move (GRTL_BIL_ID, GRTL_IL_OPEN, GRTL_SPEED_FAST, huart6);
	ax_move (GRTL_BOL_ID, GRTL_OL_OPEN, GRTL_SPEED_FAST, huart6);
	ax_move (GRTL_BIR_ID, GRTL_IR_OPEN, GRTL_SPEED_FAST, huart6);
	ax_move (GRTL_BOR_ID, GRTL_OR_OPEN, GRTL_SPEED_FAST, huart6);
	mechanism_states.grtl_bil = 2;
	mechanism_states.grtl_bol = 2;
	mechanism_states.grtl_bir = 2;
	mechanism_states.grtl_bor = 2;
}

void
grtl_front_close ()
{
	ax_move (GRTL_FIL_ID, GRTL_IL_CLOSE, GRTL_SPEED_FAST, huart6);
	ax_move (GRTL_FOL_ID, GRTL_OL_CLOSE, GRTL_SPEED_FAST, huart6);
	ax_move (GRTL_FIR_ID, GRTL_IR_CLOSE, GRTL_SPEED_FAST, huart6);
	ax_move (GRTL_FOR_ID, GRTL_OR_CLOSE, GRTL_SPEED_FAST, huart6);
	mechanism_states.grtl_fil = 0;
	mechanism_states.grtl_fol = 0;
	mechanism_states.grtl_fir = 0;
	mechanism_states.grtl_for = 0;
}

void
grtl_back_close ()
{
	ax_move (GRTL_BIL_ID, GRTL_IL_CLOSE, GRTL_SPEED_FAST, huart6);
	ax_move (GRTL_BOL_ID, GRTL_OL_CLOSE, GRTL_SPEED_FAST, huart6);
	ax_move (GRTL_BIR_ID, GRTL_IR_CLOSE, GRTL_SPEED_FAST, huart6);
	ax_move (GRTL_BOR_ID, GRTL_OR_CLOSE, GRTL_SPEED_FAST, huart6);
	mechanism_states.grtl_bil = 0;
	mechanism_states.grtl_bol = 0;
	mechanism_states.grtl_bir = 0;
	mechanism_states.grtl_bor = 0;
}

void
grtl_front_grip_all ()
{
	ax_move (GRTL_FIL_ID, GRTL_IL_GRIP, GRTL_SPEED_SLOW, huart6);
	ax_move (GRTL_FOL_ID, GRTL_OL_GRIP, GRTL_SPEED_SLOW, huart6);
	ax_move (GRTL_FIR_ID, GRTL_IR_GRIP, GRTL_SPEED_SLOW, huart6);
	ax_move (GRTL_FOR_ID, GRTL_OR_GRIP, GRTL_SPEED_SLOW, huart6);
	mechanism_states.grtl_fil = 1;
	mechanism_states.grtl_fol = 1;
	mechanism_states.grtl_fir = 1;
	mechanism_states.grtl_for = 1;
}

void
grtl_back_grip_all ()
{
	ax_move (GRTL_BIL_ID, GRTL_IL_GRIP, GRTL_SPEED_SLOW, huart6);
	ax_move (GRTL_BOL_ID, GRTL_OL_GRIP, GRTL_SPEED_SLOW, huart6);
	ax_move (GRTL_BIR_ID, GRTL_IR_GRIP, GRTL_SPEED_SLOW, huart6);
	ax_move (GRTL_BOR_ID, GRTL_OR_GRIP, GRTL_SPEED_SLOW, huart6);
	mechanism_states.grtl_bil = 1;
	mechanism_states.grtl_bol = 1;
	mechanism_states.grtl_bir = 1;
	mechanism_states.grtl_bor = 1;
}

void
grtl_front_grip_inside ()
{
	ax_move (GRTL_FIL_ID, GRTL_IL_GRIP, GRTL_SPEED_SLOW, huart6);
	ax_move (GRTL_FIR_ID, GRTL_IR_GRIP, GRTL_SPEED_SLOW, huart6);
	mechanism_states.grtl_fil = 1;
	mechanism_states.grtl_fir = 1;
}

void
grtl_back_grip_inside ()
{
	ax_move (GRTL_BIL_ID, GRTL_IL_GRIP, GRTL_SPEED_SLOW, huart6);
	ax_move (GRTL_BIR_ID, GRTL_IR_GRIP, GRTL_SPEED_SLOW, huart6);
	mechanism_states.grtl_bil = 1;
	mechanism_states.grtl_bir = 1;
}

void
grtl_front_grip_outside ()
{
	ax_move (GRTL_FOL_ID, GRTL_OL_GRIP, GRTL_SPEED_SLOW, huart6);
	ax_move (GRTL_FOR_ID, GRTL_OR_GRIP, GRTL_SPEED_SLOW, huart6);
	mechanism_states.grtl_fol = 1;
	mechanism_states.grtl_for = 1;
}

void
grtl_back_grip_outside ()
{
	ax_move (GRTL_BOL_ID, GRTL_OL_GRIP, GRTL_SPEED_SLOW, huart6);
	ax_move (GRTL_BOR_ID, GRTL_OR_GRIP, GRTL_SPEED_SLOW, huart6);
	mechanism_states.grtl_bol = 1;
	mechanism_states.grtl_bor = 1;
}

void
grtl_front_open_inside ()
{
	ax_move (GRTL_FIL_ID, GRTL_IL_OPEN, GRTL_SPEED_SLOW, huart6);
	ax_move (GRTL_FIR_ID, GRTL_IR_OPEN, GRTL_SPEED_SLOW, huart6);
	mechanism_states.grtl_fil = 2;
	mechanism_states.grtl_fir = 2;
}

void
grtl_back_open_inside ()
{
	ax_move (GRTL_BIL_ID, GRTL_IL_OPEN, GRTL_SPEED_SLOW, huart6);
	ax_move (GRTL_BIR_ID, GRTL_IR_OPEN, GRTL_SPEED_SLOW, huart6);
	mechanism_states.grtl_bil = 2;
	mechanism_states.grtl_bir = 2;
}

void
grtl_front_open_outside ()
{
	ax_move (GRTL_FOL_ID, GRTL_OL_OPEN, GRTL_SPEED_SLOW, huart6);
	ax_move (GRTL_FOR_ID, GRTL_OR_OPEN, GRTL_SPEED_SLOW, huart6);
	mechanism_states.grtl_fol = 2;
	mechanism_states.grtl_for = 2;
}

void
grtl_back_open_outside ()
{
	ax_move (GRTL_BOL_ID, GRTL_OL_OPEN, GRTL_SPEED_SLOW, huart6);
	ax_move (GRTL_BOR_ID, GRTL_OR_OPEN, GRTL_SPEED_SLOW, huart6);
	mechanism_states.grtl_bol = 2;
	mechanism_states.grtl_bor = 2;
}

void
ruc_front_down ()
{
	ax_move (RUC_FL_ID, RUC_L_DOWN, RUC_SPEED_SLOW, huart6);
	ax_move (RUC_FR_ID, RUC_R_DOWN, RUC_SPEED_SLOW, huart6);
	mechanism_states.ruc_front = 2;
}

void
ruc_back_down ()
{
	ax_move (RUC_BL_ID, RUC_L_DOWN, RUC_SPEED_SLOW, huart6);
	ax_move (RUC_BR_ID, RUC_R_DOWN, RUC_SPEED_SLOW, huart6);
	mechanism_states.ruc_back = 2;
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
	ax_move (RUC_FL_ID, RUC_L_UP, RUC_SPEED_FAST, huart6);
	ax_move (RUC_FR_ID, RUC_R_UP, RUC_SPEED_FAST, huart6);
	mechanism_states.ruc_front = 0;
}

void
ruc_back_up ()
{
	ax_move (RUC_BL_ID, RUC_L_UP, RUC_SPEED_FAST, huart6);
	ax_move (RUC_BR_ID, RUC_R_UP, RUC_SPEED_FAST, huart6);
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
	vacuum_0 (on);
	HAL_Delay (20);
	vacuum_1 (on);
	HAL_Delay (20);

}

void
vacuum_front (uint8_t on)
{
	vacuum_2 (on);
	HAL_Delay (20);
	vacuum_3 (on);
	HAL_Delay (20);
}

void
bnr_close ()
{
	sg90_1_move (0);
	sg90_2_move (0);
	sg90_3_move (0);
	sg90_4_move (0);
	mechanism_states.bnr = 0;
}

void
bnr_1 ()
{
	sg90_1_move (90);
	sg90_2_move (0);
	sg90_3_move (0);
	sg90_4_move (0);
	mechanism_states.bnr = 1;
}

void
bnr_2 ()
{
	sg90_1_move (90);
	sg90_2_move (90);
	sg90_3_move (0);
	sg90_4_move (0);
	mechanism_states.bnr = 2;
}

void
bnr_3 ()
{
	sg90_1_move (90);
	sg90_2_move (90);
	sg90_3_move (90);
	sg90_4_move (0);
	mechanism_states.bnr = 3;
}

void
bnr_4 ()
{
	sg90_1_move (90);
	sg90_2_move (90);
	sg90_3_move (90);
	sg90_4_move (90);
	mechanism_states.bnr = 4;
}

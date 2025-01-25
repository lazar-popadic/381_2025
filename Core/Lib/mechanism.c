/*
 * mechanism.c
 *
 *  Created on: Jan 25, 2025
 *      Author: lazar
 */

#include "main.h"

#define GRTL_FOL_ID 1	// grtalica front outside left
#define GRTL_FIL_ID 1
#define GRTL_FOR_ID 1
#define GRTL_FIR_ID 1
#define GRTL_BOL_ID 1
#define GRTL_BIL_ID 1
#define GRTL_BOR_ID 1
#define GRTL_BIR_ID	1	// grtalica back inside right
#define LIFT_F_ID	13	// lift front
#define LIFT_B_ID	13	// lift back
#define RUC_FL_ID	10	// rucica front left
#define RUC_FR_ID	10
#define RUC_BL_ID	10
#define RUC_BR_ID	10	// rucica back right
#define GURL_L_ID	12	// guralica left
#define GURL_R_ID	12	// guralica right
#define BNR_ID		11	// banner

#define GRTL_OL_OPEN	0
#define GRTL_OL_GRIP	0
#define GRTL_OL_CLOSE	0
#define GRTL_IL_OPEN	0
#define GRTL_IL_GRIP	0
#define GRTL_IL_CLOSE	0
#define GRTL_OR_OPEN	0
#define GRTL_OR_GRIP	0
#define GRTL_OR_CLOSE	0
#define GRTL_IR_OPEN	0
#define GRTL_IR_GRIP	0
#define GRTL_IR_CLOSE	0
#define GRTL_SPEED_FAST	500
#define GRTL_SPEED_SLOW	100

#define LIFT_DOWN		950
#define LIFT_UP			0
#define LIFT_DROP		100
#define LIFT_SPEED_FAST	300
#define LIFT_SPEED_SLOW	100

#define RUC_L_UP		0
#define RUC_L_MID		0
#define RUC_L_DOWN		0
#define RUC_R_UP		0
#define RUC_R_MID		0
#define RUC_R_DOWN		0

#define GURL_L_FRONT	0
#define GURL_L_MID		0
#define GURL_L_BACK		0
#define GURL_R_FRONT	0
#define GURL_R_MID		0
#define GURL_R_BACK		0

#define BNR_START		0
#define BNR_DROP_1		0
#define BNR_DROP_2		0
#define BNR_DROP_3		0

struct_mechanism_states mechanism_states;

void
mechanism_init ()
{
  // ovde pozovi da se postavi u pocetnu poziciju
}

void
lift_front_up ()
{
  ax_move (LIFT_F_ID, LIFT_UP, LIFT_SPEED_FAST);
  mechanism_states.lift_front = 2;
}

void
lift_front_down ()
{
  ax_move (LIFT_F_ID, LIFT_DOWN, LIFT_SPEED_FAST);
  mechanism_states.lift_front = 0;
}

void
lift_front_drop ()
{
  ax_move (LIFT_F_ID, LIFT_DROP, LIFT_SPEED_SLOW);
  mechanism_states.lift_front = 1;
}

void
lift_back_up ()
{
  ax_move (LIFT_B_ID, LIFT_UP, LIFT_SPEED_FAST);
  mechanism_states.lift_back = 2;
}

void
lift_back_down ()
{
  ax_move (LIFT_B_ID, LIFT_DOWN, LIFT_SPEED_FAST);
  mechanism_states.lift_back = 0;
}

void
lift_back_drop ()
{
  ax_move (LIFT_B_ID, LIFT_DROP, LIFT_SPEED_SLOW);
  mechanism_states.lift_back = 1;
}




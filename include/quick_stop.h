/*
 * quick_stop.h
 *
 *  Created on: Sep 24, 2019
 *      Author: user
 */

#ifndef QUICK_STOP_H_
#define QUICK_STOP_H_
#include "motion.h"
#include "amc.h"
#include "dc_bldc_current.h"
#include "position_mode.h"
#include "velocity_mode.h"


#define INIT_POS_MODE 		0x01
#define INIT_VEL_MODE 		0x02
#define INIT_ASSISTED_MODE	0x04



void quick_stop_init( motion_state_struct *, int *);

/*****************************************************************/
/*! 	Makes necessary initializations for quick stop (set target velocity = 0)
	\param state Cinematic state of the system
	\param mode quick stop mode
*/
/*****************************************************************/
void quick_stop_initialization( motion_state_struct *, int *);



/*****************************************************************/
/*! 	Control velocity for quick stop
	\param state Cinematic state of the system
	\param mode quick stop mode
*/
/*****************************************************************/
void quick_stop_control( motion_state_struct *, int);

extern int reload_change_time;
extern int test_feedforward_current;

extern _iq factor_current_down ;
extern _iq test_current_ref;

#endif /* QUICK_STOP_H_ */

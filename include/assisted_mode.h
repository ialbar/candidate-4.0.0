/*!	\file assisted_mode.h
	\brief Header file containing declarations for Assisted Mode
*/

#ifndef _ASSISTED_MODE_H_
#define _ASSISTED_MODE_H_

/* variables declarations */
extern long int velocity_demand_filtered;
extern _iq fb_vel;

/* function declarations */
void assisted_mode_operation(motion_state_struct *state, char init );
int am_check_params(void);
void am_initialize(motion_state_struct *state);
void am_status_flags( long target, unsigned long v_max );
int am_position_limits(void);
void am_set_max_vel(long int*);
_iq am_position_controller(long, long, _iq , _iq);
void am_detent_status_flags(manual_state_t manual_state, long int detent, motion_state_struct *state, short reset);

_iq am_velocity_controller_mms(long int demand, long int velocity);

#endif  /* end _ASSISTED_MODE_H_ definition */

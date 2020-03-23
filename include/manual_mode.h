/*!	\file manual_mode.h
	\brief Header file containing declarations for Manual Mode (with software detents)
*/

#ifndef _MANUAL_MODE_H_
#define _MANUAL_MODE_H_

#include "stdint.h"

/*! manual move states */
typedef enum {
	DETENTS_INACTIVE,		/*!< No active detent (we are far from detent points) */
	DETENTS_ACTIVE,			/*!< There is a detent active (we are close to a detent point) */
	DETENTS_LEAVING			/*!< Detent leave order received */
} manual_state_t;

#define NESTED_LOOPS_MANUAL_MODE 1

extern manual_state_t manual_state;

/*! Maximum numbers of detents */
#define MAX_DETENTS 16

extern unsigned int ntimes;

/* function declarations */
void manual_mode_operation(motion_state_struct *state, char init);
void manual_initialize(motion_state_struct *state);
void manual_status_flags(manual_state_t manual_state, long int detent, motion_state_struct *state, int reset);
unsigned int check_EndManual_delay(void);
void pending_end_manual(motion_state_struct *state);
extern _iq position_controller_manual(long int demand, long int position);

extern _iq control_effort_manual_profile;
extern _iq control_effort_manual_velocity;
extern _iq control_effort_manual_position;

extern PIDREG manual_vel_control_pid;	/*!< PID struct for manual velocity control */
extern PIDREG manual_pos_control_pid;
extern long int Position_demand_value_trajectory;

extern unsigned int state_machine_manual_mode_dbg;
extern unsigned int manual_mode_trans_dbg;
extern unsigned int target_reach_dbg;
extern long int position_last_detent;
extern unsigned int avoid_last_detent;
extern unsigned int move_away_output_distance;
#if 1
extern void set_Ki_vel_manual(void);
#endif

#endif  /* end _MANUAL_MODE_H_ definition */

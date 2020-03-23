/*!	\file position_mode.h
	\brief Header file containing declarations for Profile Position Mode
*/

#ifndef _POSITION_MODE_H_
#define _POSITION_MODE_H_

/* function declarations */
int profile_position_mode_operation(motion_state_struct *state, char init);
long pp_trajectory_point(position_trajectory_struct *trajectory, long long current_time);
void pp_trajectory_generator(long new_target, position_trajectory_struct *trajectory, motion_state_struct *state);
void pp_trajectory_from_stop(position_trajectory_struct *trajectory);
void pp_stop_trajectory(unsigned long dec, position_trajectory_struct *trajectory, motion_state_struct *state);
void pp_trajectory_2decs(position_trajectory_struct *trajectory);
void pp_limits(long new_target, long *limited_target, long *target_pos, long *vel, long *acc, long *dec);
_iq position_controller_mm(long int demand, long int position);
_iq pos_pv_velocity_controller_mms(_iq demand, long int velocity);
void pp_status_flags(long int position, long int demand, long int target, long int halt, motion_state_struct *state, int reset);
void pp_initialize(motion_state_struct *state);
long pp_trajectory_vel(position_trajectory_struct *trajectory, long long current_time);
void pp_trajectory_set_stop_point( unsigned long dec, position_trajectory_struct *trajectory, motion_state_struct *state );
void pp_trajectory_reset( position_trajectory_struct *trajectory, motion_state_struct *state );

extern int set_position_control_dataset( void );
extern int set_position_velocity_control_dataset( void );
extern int set_position_current_control_dataset (void);
extern long pp_get_max_travel(void);
#if 1 // TEst Dentent, arreglar
extern void pp_set_pid_tunning( void );
#endif

extern int position_error_mms;
extern unsigned int debug_trajectory;
extern long vel_ref;



#endif  /* end of_POSITION_MODE_H_ definition */

/*!	\file velocity_mode.h
	\brief Header file containing declarations for Profile Velocity Mode
*/

#ifndef _VELOCITY_MODE_H_
#define _VELOCITY_MODE_H_

/* function declarations */
int  profile_velocity_mode_operation(motion_state_struct *state, char init);
long int pv_trajectory_point(velocity_trajectory_struct *target, long long current_time);
void pv_newVelocityTarget(long new_target, velocity_trajectory_struct *target, motion_state_struct *state);
void pv_stop_trajectory( long deceleration, velocity_trajectory_struct *target, motion_state_struct *state );
_iq pv_velocity_controller(long int demand, long int velocity);
_iq pv_velocity_controller_mms(long int demand, long int velocity);
void pv_status_flags(long int velocity, long int demand, long int target, motion_state_struct *state, int reset);
void pv_initialize(motion_state_struct *state);
int pv_position_limits(void);
void pv_trajectory_set_stop_point( unsigned long dec, velocity_trajectory_struct *trajectory, motion_state_struct *state );
void pv_trajectory_reset( velocity_trajectory_struct *trajectory, motion_state_struct *state );

extern int set_velocity_control_dataset( void );
extern int set_velocity_current_control_dataset (void);
extern int velocity_error_mms;
extern unsigned int velocity_demand_test;
extern long int pv_max_travel;


extern unsigned long Position_actual_value_prev;


extern unsigned int debug_detent;

#endif  /* end _VELOCITY_MODE_H_ definition */

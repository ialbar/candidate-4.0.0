/*!	\file circle_mode.h
	\brief Header file containing declarations for Circle Mode
*/

#ifndef _CIRCLE_MODE_H_
#define _CIRCLE_MODE_H_


/* function declarations */
void circle_mode_operation(motion_state_struct *statei, char init);
void circle_initialize(circle_trajectory_struct *trajectory, motion_state_struct *state);
void circle_start_trajectory(circle_trajectory_struct *trajectory, motion_state_struct *state);
void circle_stop_trajectory(circle_trajectory_struct *trajectory, motion_state_struct *state);
long circle_trajectory_point(circle_trajectory_struct *trajectory, long long current_time);
long circle_get_circular_point(circle_trajectory_struct *trajectory, long long current_time);

#endif  /* end _CIRCLE_MODE_H_ definition */

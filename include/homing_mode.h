/*!	\file homing_mode.h
	\brief Header file containing declarations for Homing Mode
*/

#ifndef _HOMING_MODE_H_
#define _HOMING_MODE_H_

/* function declarations */
int homing_mode_operation(motion_state_struct *state, char init);

extern long absolute_sensor;

#endif  /* end _HOMING_MODE_H_ definition */

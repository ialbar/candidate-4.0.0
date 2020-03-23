/*
 * redundancy.h
 *
 *  Created on: Nov 14, 2019
 *      Author: user
 */

#ifndef REDUNDANCY_H_
#define REDUNDANCY_H_


int redundancy_test(int init, motion_state_struct *state);
int hybrid_redundancy_test(int init, motion_state_struct *state);
void new_redundancy(motion_state_struct *state );
void check_redundancy( motion_state_struct *state );
void hybrid_check_redundancy( motion_state_struct *state );



#endif /* REDUNDANCY_H_ */

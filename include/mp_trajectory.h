/*
 * mp_trajectory.h
 *
 *  Created on: Oct 15, 2019
 *      Author: user
 */

#ifndef MP_TRAJECTORY_H_
#define MP_TRAJECTORY_H_

#include "amc.h"



extern void mp_trajectory_generator(long , position_trajectory_struct *, motion_state_struct *);

extern position_trajectory_struct mp_trajectory;		/*!< Struct that keeps position trajectory in manual position mode */


#endif /* MP_TRAJECTORY_H_ */

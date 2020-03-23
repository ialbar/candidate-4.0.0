/*!	\file motion.h
	\brief Header file containing  motion definitions.

\verbatim
* File: fir.h
* Device: TMS320F28235
* Author: Advanced Embeeded Control (AEC)  Texas Instruments Inc.
* Description: Header file containing  motion definitions.
\endverbatim
*/

#ifndef _MOTION_H_
#define _MOTION_H_

/*! struct that includes the motion state of the drive (in ) */
typedef struct{
	long position;        /*!< current position [internal units] */
	long velocity;        /*!< current velocity [internal units] */
	long long time;       /*!< current time */
	long pos_cntr;        /*!< position counter [units may not be either internal or external] */
	long vel_cntr;        /*!< position read from velocity counter [units may not be either internal or external] */
	long abs_pos_cntr;    /*!< absolute position counter [external units] */
	long hyb_pos_cntr;	  /*!< hybrid position counter [units may not be either internal or external] */
	long red_pos_cntr;	  /*!< redundancy position counter [units may not be either internal or external] */
	long pos_prev;        /*!< position [units may not be either internal or external] */
	long vel_prev;        /*!< velocity [units may not be either internal or external] */
	} motion_state_struct;


/*! struct that defines trajectories in Profile Velocity Mode

It is meant for trapezodal velocity trajectories that can have different accelerations and
decelerations and any initial velocity.
Variables are expressed in internal units.
*/
typedef  struct {
	long long init_time;	/*!< Init time, when the trajectory is calculated */
	long long end_time;		/*!< End time, when it will reach target velocity */
	long long stop_time;	/*!< Stop time, will be different from 0 if there is a stop in the middle of the trajectory */
	long target_v;			/*!< Target velocity */
	long init_v;			/*!< Initial velocity */
	} velocity_trajectory_struct;


/*! struct that defines trajectories in Profile Position Mode

It is meant for trajectories with trapezodal velocity that can have different accelerations and
decelerations and any initial velocity and position.
In the most generic situation with initial velocity there will be a decceleration period, an acceleration
period, a period with constant velocity and a final deceleration period.
Variables are expressed in internal units (except \a target).

If initial speed is in the right direccion stop time is calculated as a hypothetic past time when the moment
began considering there has been an acceleration equal to the profile acceleration
*/
typedef struct {
	long target;		/*!< Target position (limitated) in position units */
	long long t0;		/*!< Initial time */
	long long t1;		/*!< Stop time or hypothetical previous stop time */
	long long t2;		/*!< Time when constant velocity is reached */
	long long t3;		/*!< Time when decceleration begins */
	long long t4;		/*!< Time when target is reached with velocity = 0 */
	long s0;			/*!< Initial position */
	long s1;			/*!< Postion in the stop time or hypothetical previous stop time */
	long s2;			/*!< Position when constant velocity begins */
	long s3;			/*!< Position when decceleration begins */
	long s4;			/*!< Target position (limitated) in increments */
	long v0;			/*!< Initial speed */
	long v1;			/*!< In most trajectories it is 0 (intermediate stop), will be !=0 when no real or hipothetical stop point is used (vel > v2) */
	long v2;			/*!< Velocity in constant velocity period */
	long v3;
	long v4;
	long a0;			/*!< Initial acceleration */
	long a1;			/*!< Acceleration */
	long a3;			/*!< Deceleration */
	long next_target;	/*!< Next target */
	int next_target_active;		/*!< Determines whether next target will be active when current target is reached or not */
	} position_trajectory_struct;


/*! struct that defines trajectories in Circle Mode

It only considers linear interpolation trajectories between 2 points.
The struc defines 2 trajectories, a very simple one between 2 points, defined in internal units, that represents the position of my axis for a short time. And a more general one
that represents the global velocity of the system (that follows a circle). The simple trajectory (t_first, t_second, s_first and s_second) is calculated for every time slice out of
this trajectory.
*/
typedef  struct {
	long start_position;	/*!< position when the circle is started [internal units] */
	long long t_first;	/*!< initial time (of the current time slice) */
	long long t_second;	/*!< end time (of the current time slice) */
	long s_first;			/*!< initial position (of the current time slice) [internal units] */
	long s_second;			/*!< final position (of the current time slice) [internal units] */
	long long t1;		/*!< initial stop time */
	long long t2;		/*!< Time when constant velocity is reached */
	long long t3;		/*!< Time when decceleration begins */
	long long t4;		/*!< Stop time */
	long s1;			/*!< Postion in the initial stop time [revolutions/2^16] */
	long s2;			/*!< Position when constant velocity begins [revolutions/2^16] */
	long s3;			/*!< Position when decceleration begins [revolutions/2^16] */
	long s4;			/*!< Target position (limitated) in increments [revolutions/2^16] */
	long v2;			/*!< Velocity in constant velocity period [revolutions/2^16/s] */
	long a1;			/*!< Acceleration [revolutions/2^16/s^2] */
	long a3;			/*!< Deceleration [revolutions/2^16/s^2] */
	long radius;			/*!< Radius of the circle */
	unsigned int time_incr;	/*!< time increment between two points of the trajectory */
	unsigned int axis;		/*!< axis of the circle (sine or consine) */
	} circle_trajectory_struct;

#endif

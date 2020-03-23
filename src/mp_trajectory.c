/*
 * mp_trajectory.c
 *
 *  Created on: Oct 15, 2019
 *      Author: user
 */


#include "mp_trajectory.h"

long mp_trajectory_point(position_trajectory_struct *trajectory, long long current_time);
void mp_trajectory_from_stop(position_trajectory_struct *trajectory);
void mp_stop_trajectory(unsigned long dec, position_trajectory_struct *trajectory, motion_state_struct *state);
void mp_trajectory_2decs(position_trajectory_struct *trajectory);
void mp_limits(long new_target, long *limited_target, long *target_pos, long *vel, long *acc, long *dec);
long mp_trajectory_vel(position_trajectory_struct *trajectory, long long current_time);
void mp_trajectory_set_stop_point( unsigned long dec, position_trajectory_struct *trajectory, motion_state_struct *state );
void mp_trajectory_reset( position_trajectory_struct *trajectory, motion_state_struct *state );

position_trajectory_struct mp_trajectory;


/*****************************************************************/
/*!	 Calculates main points of the trajectory to follow

	It doesn't use the current state as initial point but the "demanded" state
	\param new_target New position target (in velocity units)
	\param trajectory Position trajectory to be created
	\param state Cinematic state of the system
*/
/*****************************************************************/
void mp_trajectory_generator(long new_target, position_trajectory_struct *trajectory, motion_state_struct *state)
{
	long limited_target;				/* in position units */
	long target_pos, vel, acc, dec;		/* in [inc] [inc/s] and [inc/s^2] */
	long delta_s;
	long long stop_length;
	motion_state_struct demand_state;
	long tmp;

	demand_state.time = state->time;
	demand_state.position = mp_trajectory_point(trajectory, demand_state.time);
	demand_state.velocity = mp_trajectory_vel(trajectory, demand_state.time);

	/* limit movements on one side if any limit has been reached */
	if (position_flags & MIN_POS_MASKBIT)
	{
		tmp = int2ext_pos(demand_state.position, Home_offset);
		if(new_target < tmp) new_target = tmp;
	}
	if (position_flags & MAX_POS_MASKBIT)
	{
		tmp = int2ext_pos(demand_state.position, Home_offset);
		if(new_target > tmp) new_target = tmp;
	}

	acc = Profile_acceleration;		/* will be limitated and converted into [inc/s^2] */
	dec = Profile_deceleration;		/* will be limitated and converted into [inc/s^2] */
	vel = Profile_velocity;			/* will be limitated and converted into [inc/s] */
	mp_limits(new_target, &limited_target, &target_pos, &vel, &acc, &dec);	/* calculate limited parameters in internal units */

	trajectory->target = limited_target;
	trajectory->t0 = demand_state.time;
	trajectory->s0 = demand_state.position;
	trajectory->s4 = target_pos;
	trajectory->a0 = (-1) * sign(demand_state.velocity) * dec;
	trajectory->v0 = demand_state.velocity;
	trajectory->v2 = vel;	/* the sign will be checked */
	trajectory->v1 = 0;		/* unless no stop point and vel>v2 */
	trajectory->v4 = 0;

	/* calculate the length it would take it to stop */
	stop_length = (long long)demand_state.velocity * demand_state.velocity;
	stop_length = -stop_length / (trajectory->a0 * 2);	/*  stop_length = -1/2 * (v0^2/a0)  (can be negative) */

	delta_s = (target_pos - demand_state.position);

	if((sign(delta_s) != sign(demand_state.velocity)) || (labs(delta_s) < llabs(stop_length)))
	{		/* there will be an intermediate stop */
		long long st;		/* stop time */

		/* set the stop time */
		st = (long long)demand_state.velocity * lcounts_p_s;
		st = -st / trajectory->a0;

		trajectory->t1 = trajectory->t0 + st;		/* st is in low resolution clock time units */

		/* set the stop position */
		trajectory->s1 = trajectory->s0 + stop_length;

		/* set the trajectory accelerations depending on the direction of movement */
		trajectory->a1 = sign(trajectory->s4 - trajectory->s1) * acc;
		trajectory->a3 = -sign(trajectory->s4 - trajectory->s1) * dec;

		/* calculate the rest of the trajectory as if beginning stopped */
		mp_trajectory_from_stop(trajectory);
	}
	else
	{		/* there will NOT be an intermediate stop */
		long long tmp;
		if(labs(demand_state.velocity) > vel)
		{	/* v1 will not be zero, deceleration before v2 */
			/* set the initial velocity of the first deceleration section */
			trajectory->v1 = demand_state.velocity;

			/* point 1 is now */
			trajectory->t1 = demand_state.time;
			trajectory->s1 = demand_state.position;

			/* set accelerations */
			trajectory->a1 = -sign(demand_state.velocity) * dec;
			trajectory->a3 = trajectory->a1;

			/* calculate the rest of the trajectory */
			mp_trajectory_2decs(trajectory);
		}
		else
		{		/* there will NOT be an intermediate stop (but I will consider a previous one) */
			/* set the trajectory accelerations depending on the direction of movement */
			trajectory->a1 = sign(delta_s) * acc;
			trajectory->a3 = -sign(delta_s) * dec;

			/* t1: set the stop time from a hypothetical previous stop */
			tmp = (long long)demand_state.velocity * lcounts_p_s;
			trajectory->t1 = trajectory->t0 - (tmp / trajectory->a1);

			/* s1: set the hypothetical previous stop position */
			tmp = (long long)demand_state.velocity * demand_state.velocity;
			trajectory->s1 = trajectory->s0 - (tmp / (trajectory->a1 * 2));	/*  s1 = s0 - 1/2 * v0^2/a1  */

			/* calculate the rest of the trajectory as if beginning stopped */
			mp_trajectory_from_stop(trajectory);
		}
	}
}

/*****************************************************************/
/*!	 Calculates the position demand for current control cycle
	\param trajectory Position trajectory being followed
	\param current_time Current time
	\return Position demand
*/
/*****************************************************************/
long mp_trajectory_point(position_trajectory_struct *trajectory, long long current_time)
{
	long long tmp;
	long ret;

	/* It depends on the motion profile type (by now only linear ramp mode used) */
	if(current_time >= trajectory->t4)
	{
		DINT;
		p_current_limit = &current_limit;	/* set current limit to use */
		EINT;
		/* S4 */
		ret = trajectory->s4;
	}
	else if(current_time >= trajectory->t3)
	{
		DINT;
		p_current_limit = &current_limit;	/* set current limit to use */
		EINT;
		/* S3 + v2(t-t3)+ 1/2 * a3 * (t - t3)^2 */
		tmp = (long long)((long)(current_time - trajectory->t3)) * trajectory->v2;
		tmp = trajectory->s3 + (tmp / lcounts_p_s);
		ret = tmp + half_acc_t2(trajectory->a3, (unsigned long)(current_time - trajectory->t3));

	}
	else if(current_time >= trajectory->t2)
	{
		DINT;
		p_current_limit = &current_limit;	/* set current limit to use */
		EINT;
		/* S2 + v2 * (t - t2) */
		tmp = (long long)((long)(current_time - trajectory->t2)) * trajectory->v2;
		ret = trajectory->s2 + (tmp / lcounts_p_s);

	}
	else if(current_time >= trajectory->t1)
	{
		/* set current limit to use (acceleration limit only if v1==0) */
		DINT;
		p_current_limit = (trajectory->v1 == 0)? &current_limit_acc : &current_limit;
		EINT;
		/* S1 + v1 * (t - t1) + 1/2 * a1 * (t - t1)^2 */
		tmp = (long long)((long)(current_time - trajectory->t1)) * trajectory->v1;
		tmp = trajectory->s1 + (tmp / lcounts_p_s);
		ret = tmp + half_acc_t2(trajectory->a1, (unsigned long)(current_time - trajectory->t1));

	}
	else
	{
		DINT;
		p_current_limit = &current_limit;	/* set current limit to use */
		EINT;

		/* S0 + v0(t-t0) + 1/2 * a0 * (t - t0)^2 */
		tmp = (long long)((long)(current_time - trajectory->t0)) * trajectory->v0;
		tmp = trajectory->s0 + (tmp / lcounts_p_s);
		ret = tmp + half_acc_t2(trajectory->a0, (unsigned long)(current_time - trajectory->t0));
	}

	ret = CLAMP( ext2int_pos(Software_position_limit_Min_position_limit, Home_offset),
		     ret,
		     ext2int_pos(Software_position_limit_Max_position_limit, Home_offset) );
	return ret;
}

/*****************************************************************/
/*!	 Calculates a position trajectory starting from a stopped position (s1,t1)
	\param trajectory Position trajectory to be created
*/
/*****************************************************************/
void mp_trajectory_from_stop(position_trajectory_struct *trajectory)
{
	long long tmp;		/* 64-bit auxiliar variable */

	/* calculate the critical length for which Profile_velocity is not reached */
	tmp = (long long)(trajectory->a3 - trajectory->a1) * trajectory->v2;
	tmp /= trajectory->a1;
	tmp = (tmp * trajectory->v2) / trajectory->a3;
	tmp = llabs(tmp / 2);		/* dist = v^2/2 * |(a3-a1)/(a1*a3)| */

	if(labs(trajectory->s4 - trajectory->s1) <= tmp)
	{	/* Profile_velocity is not reached */
		/* calculate total time */
		tmp = (long long)(trajectory->s4 - trajectory->s1) * (trajectory->a3 - trajectory->a1);
		tmp /= trajectory->a1;
		tmp *= ((long)lcounts_p_s * lcounts_p_s * 2);
		tmp /= trajectory->a3;
		tmp = root(tmp);
		trajectory->t4 = trajectory->t1 + tmp;

		/* calculate intermediate time */
		tmp = (long long)(trajectory->s4 - trajectory->s1) * trajectory->a3;
		tmp /= trajectory->a1;
		tmp *= ((long)lcounts_p_s * lcounts_p_s * 2);
		tmp /= (trajectory->a3 - trajectory->a1);
		tmp = root(tmp);
		trajectory->t2 = trajectory->t1 + tmp;
		trajectory->t3 = trajectory->t2;

		/* calculate intermediate position */
		tmp = trajectory->s4 - trajectory->s1;
		trajectory->s2 = trajectory->s1 + (tmp/2);
		trajectory->s3 = trajectory->s2;

		/* calculate Profile velocity */
		tmp = (long long)((long)(trajectory->t3 - trajectory->t1)) * trajectory->a1;
		tmp = (tmp/lcounts_p_s) + trajectory->v1;
		trajectory->v2 = tmp;
	}
	else
	{
		trajectory->v2 = sign(trajectory->s4 - trajectory->s1) * trajectory->v2;	/* correct the sign of v2 */

		/* set the time to reach constant velocity */
		tmp = (long long)trajectory->v2 * lcounts_p_s;
		trajectory->t2 = trajectory->t1 + (tmp / trajectory->a1);

		/* set the position to reach constant velocity */
		tmp = (long long)trajectory->v2 * trajectory->v2;
		trajectory->s2 = trajectory->s1 + (tmp / (2 * trajectory->a1));	/*  s2 = s1 + 1/2 * v2^2/a1  */

		/* set the position to begin deceleration */
		tmp = (long long)trajectory->v2 * trajectory->v2;
		trajectory->s3 = trajectory->s4 + (tmp / (2 * trajectory->a3)); 		/*  s3 = s4 + 1/2 * v2^2/a3  */

		/* set the time to begin deceleration */
		tmp = (long long)(trajectory->s3 - trajectory->s2) * lcounts_p_s;
		trajectory->t3 = trajectory->t2 + (tmp / trajectory->v2);

		/* set the time to stop */
		tmp = (long long)trajectory->v2 * lcounts_p_s;
		trajectory->t4 = trajectory->t3 - (tmp / trajectory->a3);		/*  t4 = t3 - (v2/a3)  */
	}
}


/*****************************************************************/
/*!	 Calculates a position trajectory to stop the drive at a given deceleration
	\param dec Deceleration in [position units]
	\param trajectory Position trajectory to be created
	\param state Cinematic state of the system
*/
/*****************************************************************/
void mp_stop_trajectory(unsigned long dec, position_trajectory_struct *trajectory, motion_state_struct *state)
{
	unsigned long dec_incs;
	long long stop_length;
	long long stop_time;		/* stop time */
	long long tmp;

	tmp = (long long)minimum(dec, Max_deceleration) * Acceleration_factor_Numerator;
	dec_incs = tmp / Acceleration_factor_Divisor;		/* limited and converted into [incs/s^2] */

	/* set trajectory initial values */
	trajectory->t0 = state->time;
	trajectory->s0 = state->position;
	trajectory->a0 = (-1) * sign(state->velocity) * dec_incs;

	/* calculate the length it will take it to stop */
	stop_length = (long long)state->velocity * state->velocity;
	stop_length = -stop_length / (trajectory->a0 * 2);	/*  stop_length = -1/2 * (v0^2/a0)  (can be negative) */

	/* set the stop time */
	stop_time = (long long)state->velocity * lcounts_p_s;
	stop_time = -stop_time / trajectory->a0;

	trajectory->v0 = state->velocity;
	trajectory->v1 = 0;
	trajectory->v2 = 0;
	trajectory->t1 = trajectory->t0 + stop_time;

	/* set the stop position */
	trajectory->s1 = trajectory->s0 + stop_length;

	/* set the rest of the trajectory */
	trajectory->s2 = trajectory->s1;
	trajectory->s3 = trajectory->s1;
	trajectory->s4 = trajectory->s1;
	trajectory->t2 = trajectory->t1;
	trajectory->t3 = trajectory->t1;
	trajectory->t4 = trajectory->t1;

	/* calculate target in position units */
	//trajectory->target = int2ext_pos(trajectory->s1, Home_offset);		/* [position units] */
}





/*****************************************************************/
/*!	 Calculates a position trajectory with 2 deceleration sections (starting from a velocity above profile_velocity)
	\param trajectory Position trajectory to be created
*/
/*****************************************************************/
void mp_trajectory_2decs(position_trajectory_struct *trajectory)
{
	long long tmp;

	/* correct the sign of v2 */
	trajectory->v2 = sign(trajectory->v1) * trajectory->v2;

	/* set the time to reach constant velocity */
	tmp = (long long)(trajectory->v2 - trajectory->v1) * lcounts_p_s;
	trajectory->t2 = trajectory->t1 + (tmp / trajectory->a1);

	/* set the position to reach constant velocity */
	/* s2 = s1 + (v2-v1)/a1 * (v1+v2)/2 */
	tmp = (long long)(trajectory->v2 - trajectory->v1) * (trajectory->v1 + trajectory->v2);
	trajectory->s2 = trajectory->s1 + (tmp / (2 * trajectory->a1));

	/* set the position to begin deceleration */
	tmp = (long long)trajectory->v2 * trajectory->v2;
	trajectory->s3 = trajectory->s4 + (tmp / (2 * trajectory->a3)); 		/*  s3 = s4 + 1/2 * v2^2/a3  */

	/* set the time to begin deceleration */
	tmp = (long long)(trajectory->s3 - trajectory->s2) * lcounts_p_s;
	trajectory->t3 = trajectory->t2 + (tmp / trajectory->v2);

	/* set the time to stop */
	tmp = (long long)trajectory->v2 * lcounts_p_s;
	trajectory->t4 = trajectory->t3 - (tmp / trajectory->a3);		/*  t4 = t3 - (v2/a3)  */
}


/*****************************************************************/
/*!	 Check if target postion, velocity and accelerations meet their limits
	\param new_target New target position [position units]
	\param limited_target Returns new target position after being limitated [position units]
	\param target_pos Returns new target position after being limitated [increments]
	\param vel Receives a velocity in [velocity units] that is limitated and returned converted into [inc/s]
	\param acc Receives an acceleration in [acceleration units] that is limitated and returned converted into [inc/s^2]
	\param dec Receives a deceleration in [acceleration units] that is limitated and returned converted into [inc/s^2]
*/
/*****************************************************************/
void mp_limits(long new_target, long *limited_target, long *target_pos, long *vel, long *acc, long *dec)
{
	long long tmp;

	/* calculate limited accelerations in [inc/s^2] */
	tmp = (long long)minimum(*acc, Max_acceleration) * Acceleration_factor_Numerator;
	tmp /= Acceleration_factor_Divisor;
	*acc = tmp;

	tmp = (long long)minimum(*dec, Max_deceleration) * Acceleration_factor_Numerator;
	tmp /= Acceleration_factor_Divisor;
	*dec = tmp;

	/* calculate limited target position in position units */
	if(new_target > (long)Software_position_limit_Max_position_limit) tmp = (long)Software_position_limit_Max_position_limit;
	else if (new_target < (long)Software_position_limit_Min_position_limit) tmp = (long)Software_position_limit_Min_position_limit;
	else tmp = new_target;
	*limited_target = tmp;

	/* calculate limited target position in [inc] */
	*target_pos = ext2int_pos(tmp, Home_offset);

	/* calculate limited profile velocity in [inc/s] */
	tmp = (long long)Max_motor_speed * Velocity_factor_1_Numerator;
	tmp /= Velocity_factor_1_Divisor;					/* Max_motor_speed limit */
	tmp = minimum(tmp,Max_profile_velocity);			/* minimum of Max_motor_speed and Max_profile_velocity */
	tmp = minimum(*vel, tmp);
	*vel = labs( ext2int_vel( tmp ) );
}

/*****************************************************************/
/*!	 Calculates the velocity demand for current control cycle
	\param trajectory Position trajectory being followed
	\param current_time Current time
	\return Velocity demand
*/
/*****************************************************************/
long mp_trajectory_vel(position_trajectory_struct *trajectory, long long current_time)
{
	long long tmp;
	long ret;

	/* It depends on the motion profile type (by now only linear ramp mode used) */
	if(current_time >= trajectory->t4)
	{
		/* v = v4 */
		ret = trajectory->v4;
	}
	else if(current_time >= trajectory->t3)
	{
		/* v = v2 + a3 * (t - t3) */
		tmp = (long long)((long)(current_time - trajectory->t3)) * trajectory->a3;
		tmp = (tmp/lcounts_p_s) + trajectory->v2;
		ret = tmp;
	}
	else if(current_time >= trajectory->t2)
	{
		/* v = v2 */
		ret = trajectory->v2;
	}
	else if(current_time >= trajectory->t1)
	{
		/* v =  v1 + a1 * (t - t1) */
		tmp = (long long)((long)(current_time - trajectory->t1)) * trajectory->a1;
		tmp = (tmp/lcounts_p_s) + trajectory->v1;
		ret = tmp;
	}
	else
	{
		/* v = v0 + a0 * (t - t0) */
		tmp = (long long)((long)(current_time - trajectory->t0)) * trajectory->a0;
		tmp = (tmp/lcounts_p_s) + trajectory->v0;
		ret = tmp;
	}
	return ret;
}


void mp_trajectory_set_stop_point( unsigned long dec, position_trajectory_struct *trajectory, motion_state_struct *state )
{
	unsigned long dec_incs;
	long target;
	long vel;
	long long stop_length;
	long long tmp;
	long long sim_time;

	//Already braked or braking
	if(state->time >= trajectory->t3)
		return;

	tmp = (long long)minimum(dec, Max_deceleration) * Acceleration_factor_Numerator;
	dec_incs = tmp / Acceleration_factor_Divisor;		/* limited and converted into [incs/s^2] */

	trajectory->a3 = sign(trajectory->a3) * dec_incs;

	/* The profile velocity is used instead of current one,
	 * because the current one could have noise (gearbox gap) */
	//vel = state->velocity;
	vel = mp_trajectory_vel(trajectory, state->time);

	trajectory->v2 = vel;
	sim_time = state->time + (5000/lcounts_p_s);
	trajectory->s2 = mp_trajectory_point(trajectory, sim_time);
	trajectory->t2 = state->time;

	sim_time = state->time + (10000/lcounts_p_s);
	trajectory->s3 = mp_trajectory_point(trajectory, sim_time);
	trajectory->t3 = sim_time;
	stop_length = (long long)vel * vel;
	stop_length = stop_length / (trajectory->a3 * 2);	/*  stop_length = -1/2 * (v0^2/a0)  (can be negative) */
	stop_length = sign(vel)*labs(stop_length);
	target = trajectory->s3 + stop_length;

	tmp = (long long)trajectory->v2 * lcounts_p_s;
	trajectory->t4 = trajectory->t3 - (tmp / trajectory->a3);		/*  t4 = t3 - (v2/a3)  */
	trajectory->s4 = target;
}


void mp_trajectory_reset( position_trajectory_struct *trajectory, motion_state_struct *state )
{
	trajectory->t4 = state->time;
	trajectory->s4 = state->position;
}

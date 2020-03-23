/*!	\file circle_mode.c
	\brief Functions for Circle Mode
*/

#include "amc.h"


/*****************************************************************/
/*!	 Main function of Circle Mode
	\param state Cinematic state of the system
	\param init will be 1 if the mode has to be initialized (0 if not)
*/
/*****************************************************************/
void circle_mode_operation(motion_state_struct *state, char init)
{
	long Position_demand;
	unsigned int active;
	static unsigned int old_active = 0;

	if(init) {
		circle_initialize(&circle_trajectory, state);		/* initialize the mode if necessary */
		old_active = Device_control_word & MODE_SPECIFIC_BIT4_MASKBIT;
	}

	/* Check if the circle mode has been started */
	active = Device_control_word & MODE_SPECIFIC_BIT4_MASKBIT;

	/* update general trajectory if active flag changed */
	if(active && !old_active) {
		circle_start_trajectory(&circle_trajectory, state);		/* it has just been activated */

		/* set the first point of the trajectory */
		circle_trajectory.t_first = circle_trajectory.t1;
		circle_trajectory.s_first = circle_trajectory.start_position;

		circle_trajectory.t_second = circle_trajectory.t_first + Circle_Step_time * 2;
		circle_trajectory.s_second = circle_get_circular_point(&circle_trajectory, circle_trajectory.t_second);

		send_SWord = 1;
	}
	if(!active && old_active) circle_stop_trajectory(&circle_trajectory, state);		/* it has just been disabled */

	old_active = active;

	/* get position demand according to current trajectory */
	Position_demand = circle_trajectory_point(&circle_trajectory, state->time);

	/* update Position_demand_value and Position_demand_value_in_increments */
	Position_demand_value_in_increments = Position_demand;			/* [inc] */
	Position_demand_value = int2ext_pos( Position_demand, Home_offset );		/* [position units] */

//	control_effort = position_controller(Position_demand, state->position);

}


/*****************************************************************/
/*!	 Function that makes necessary initializations when entering circle mode
	\param trajectory Circle trajectory
	\param state Cinematic state of the system
*/
/*****************************************************************/
void circle_initialize(circle_trajectory_struct *trajectory, motion_state_struct *state)
{
	unsigned long ul_tmp;

	/* reset mode specific bits of Statusword */
	Device_status_word &= ~(TARGET_REACHED_MASKBIT | SET_POINT_ACKNOWLEDGE_MASKBIT | FOLLOWING_ERROR_MASKBIT);
	send_SWord = 1;

	/* clear trajectory */
	memset(trajectory, 0, sizeof(*trajectory));
	/* it will try to keep initial position */
	trajectory->t_second = 9223372036854775807LL;		/* 2^63 -1 is the max value of a long long */
	trajectory->s_first = state->position;
	trajectory->s_second = state->position;

	/* initialize PID */
	position_control_pid.Err = 0;
	position_control_pid.Up = 0;
	position_control_pid.Ui = 0;
	position_control_pid.Ud = 0;
	position_control_pid.OutPreSat = 0;
	position_control_pid.Out = 0;
	position_control_pid.SatErr = 0;
	position_control_pid.Err1 = 0;

	ul_tmp = Position_control_parameter_set[0];		/* Kp (has to be divided by Divisor) */
	ul_tmp = ul_tmp << 16;							/* multiply by 2^16 */
	ul_tmp /= Position_control_parameter_set[4];			/* divide by Divisor */
	// position_control_pid.Kp = _IQmpyI32(_IQ(0.0000152587890625),ul_tmp);	/* divide by 2^16 */
	position_control_pid.Kp = ul_tmp << (GLOBAL_Q - 16);		/* convert to IQ and divide by 2^16 */

	ul_tmp = Position_control_parameter_set[1];		/* Ki (has to be divided by Divisor) */
	ul_tmp = ul_tmp << 16;						/* multiply by 2^16 */
	ul_tmp /= Position_control_parameter_set[4];			/* divide by Divisor */
	position_control_pid.Ki = _IQmpyI32(_IQ(0.0000152587890625),ul_tmp);	/* divide by 2^16 */

	ul_tmp = Position_control_parameter_set[2];		/* Kd (has to be divided by Divisor) */
	ul_tmp = ul_tmp << 16;						/* multiply by 2^16 */
	ul_tmp /= Position_control_parameter_set[4];			/* divide by Divisor */
	position_control_pid.Kd = _IQmpyI32(_IQ(0.0000152587890625),ul_tmp);	/* divide by 2^16 */

	ul_tmp = Position_control_parameter_set[3];		/* Kc (has to be divided by Divisor) */
	ul_tmp = ul_tmp << 16;						/* multiply by 2^16 */
	ul_tmp /= Position_control_parameter_set[4];			/* divide by Divisor */
	position_control_pid.Kc = _IQmpyI32(_IQ(0.0000152587890625),ul_tmp);	/* divide by 2^16 */

	DINT;
	p_current_limit = &current_limit;	/* set current limit to use */
	EINT;

	ATM_seti(&ready_to_power_on, 1);
}


/*****************************************************************/
/*!	 Function that calculates circle general trajectory when active bit has been set
	\param trajectory Circle trajectory
	\param state Cinematic state of the system
*/
/*****************************************************************/
void circle_start_trajectory(circle_trajectory_struct *trajectory, motion_state_struct *state)
{
	unsigned long acc;
	long long temp;
	long limit;

	/* set the internal position at the start of the circle */
	trajectory->start_position = state->position;

	/* check that position limits are not overpassed */
	if(Position_actual_value + Circle_Radius > Software_position_limit_Max_position_limit
		|| Position_actual_value + Circle_Radius < Software_position_limit_Min_position_limit)
	{
		trajectory->v2  = 0;
		trajectory->t1 = state->time;
		trajectory->t2 = state->time;
		trajectory->t3 = 9223372036854775807LL;		/* 2^63 -1 is the max value of a long long */
		trajectory->t4 = 9223372036854775807LL;		/* 2^63 -1 is the max value of a long long */
		trajectory->s1 = 0;
		trajectory->s2 = 0;
		trajectory->s3 = 0;
		trajectory->s4 = 0;
		return;
	}

	/* define the circle */
	temp = (long long)Circle_Radius * Position_factor_Numerator;
	trajectory->radius = temp / Position_factor_Feed_constant;
	trajectory->time_incr = Circle_Step_time;
	trajectory->axis = Circle_Axis;

	/* set initial point */
	trajectory->t1 = state->time;
	trajectory->s1 = 0;		/* the circle is always started from zero */

	/* calculate velocity in [revolutions/2^16/s] */
	limit = (Max_motor_speed * Velocity_factor_1_Numerator) / Velocity_factor_1_Divisor;	/* Max_motor_speed limit */
	limit = minimum(limit,Max_profile_velocity);			/* minimum of Max_motor_speed and Max_profile_velocity */
	temp = (long long)(minimum(Profile_velocity, limit)) * 10430;
	temp /= Circle_Radius;	/* convert to cirle units ([revolutions/2^16/s])  */
	trajectory->v2  = temp;

	/* calculate limited accelerations in [revolutions/2^16/s^2] */
	acc = minimum(Profile_acceleration, Max_acceleration);
	if(acc == 0)
	{
		ATM_seti(&ready_to_power_on, 0);
		_ERRORmessage(0xff03, 0x01, 0x0000, "Null acceleration value", 0, 0)
		setFault(FAULT_NULL_ACC);
	}
	temp = (long long)acc * 10430;
	acc = temp / Circle_Radius;
	trajectory->a1 = sign(trajectory->v2) * acc;

	/* set time to reach constant velocity */
	temp = (long long)(labs(trajectory->v2)) * lcounts_p_s;
	trajectory->t2 = trajectory->t1 + temp/acc;

	/* set the position to reach constant velocity */
	temp = (long long)trajectory->v2 * trajectory->v2;
	trajectory->s2 = trajectory->s1 + (temp / (2 * trajectory->a1));	/*  s2 = s1 + 1/2 * v2^2/a1  */

	/* set stop time and time to start deceleration as maximum */
	trajectory->t3 = 9223372036854775807LL;		/* 2^63 -1 is the max value of a long long */
	trajectory->t4 = 9223372036854775807LL;		/* 2^63 -1 is the max value of a long long */
}


/*****************************************************************/
/*!	 Function that calculates circle general trajectory when active bit has been cleared
	\param trajectory Circle trajectory
	\param state Cinematic state of the system
*/
/*****************************************************************/
void circle_stop_trajectory(circle_trajectory_struct *trajectory, motion_state_struct *state)
{
	unsigned long dec;
	long long temp;

	/* set time to start deceleration */
	trajectory->t3 = state->time;
	temp = (long long)(long)(trajectory->t3 - trajectory->t2) * trajectory->v2;
	trajectory->s3 = trajectory->s2 + (temp / lcounts_p_s);

	/* calculate limited accelerations in [revolutions/2^16/s^2] */
	dec = minimum(Profile_deceleration, Max_deceleration);
	if(dec == 0)
	{
		ATM_seti(&ready_to_power_on, 0);
		_ERRORmessage(0xff03, 0x01, 0x0000, "Null acceleration value", 0, 0)
		setFault(FAULT_NULL_ACC);
	}
	temp = (long long)dec * 10430;
	dec = temp / Circle_Radius;
	trajectory->a3 = (-1) * sign(trajectory->v2) * dec;

	/* set time to stop */
	temp = (long long)(labs(trajectory->v2)) * lcounts_p_s;
	trajectory->t4 = trajectory->t3 + temp/dec;

	/* set the position to reach constant velocity */
	temp = (long long)trajectory->v2 * trajectory->v2;
	trajectory->s4 = trajectory->s3 - (temp / (2 * trajectory->a3));	/*  s2 = s1 + 1/2 * v2^2/a1  */
}


/*****************************************************************/
/*!	 Calculates the position demand for current control cycle in Circle Mode
	\param trajectory Circlen trajectory being followed
	\param current_time Current time
	\return Position demand
*/
/*****************************************************************/
long circle_trajectory_point(circle_trajectory_struct *trajectory, long long current_time)
{
	if(current_time > trajectory->t_second) {
		/* set the first point of the trajectory */
		trajectory->t_first = trajectory->t_second;
		trajectory->s_first = trajectory->s_second;

		trajectory->t_second = trajectory->t_first + Circle_Step_time * lcounts_p_s / 1000;
		trajectory->s_second = circle_get_circular_point(trajectory, trajectory->t_second);
	}
	return linterpolate(current_time, trajectory->t_first, trajectory->t_second, trajectory->s_first, trajectory->s_second);
}


/*****************************************************************/
/*!	 Calculates the position in the circle for a given moment
	\param trajectory Circlen trajectory being followed
	\param time time to calculate the point
	\return next slice point
*/
/*****************************************************************/
long circle_get_circular_point(circle_trajectory_struct *trajectory, long long time)
{
	long long tmp;
	long circular_pos;
	int one_turn_pos;
	_iq angle, factor;
	long pos_from_centre;

	/* the first step is to calculate the position in the circunference */
	if(time >= trajectory->t4) circular_pos = trajectory->s4;
	else if(time >= trajectory->t3)
		{
			/* S4 + 1/2 * a3 * (t4 - t)^2 */
			circular_pos = trajectory->s4 + half_acc_t2(trajectory->a3, trajectory->t4 - time);
		} else if(time >= trajectory->t2)
			{
				tmp = (long long)(long)(time - trajectory->t2) * trajectory->v2;
				circular_pos = trajectory->s2 + (tmp / lcounts_p_s);	/* S2 + v2 * (t - t2) */
			} else if(time >= trajectory->t1)
				{
					/* S1 + 1/2 * a1 * (t - t1)^2 */
					circular_pos = trajectory->s1 + half_acc_t2(trajectory->a1, time - trajectory->t1);
				} else
				{
					;	/* should not reach this point */ /* TODO: implement an error for this situation */
				}
	/* This position is limited to only one turn */
	one_turn_pos = (int)circular_pos;
	circular_pos = (long)one_turn_pos;

	/* convert the angle to radians */
	angle = _IQmpy(circular_pos << (GLOBAL_Q - 16), _IQ(2 * 3.14159));

	/* Calculate the sine or the cosine */
	factor = (trajectory->axis)? _IQsin(angle) : _IQcos(angle);

	/* calculate the increment from the centre of the circle */
	pos_from_centre = _IQmpyI32int(factor, trajectory->radius);

	/* Return the position */
	return (trajectory->axis)? trajectory->start_position + pos_from_centre : trajectory->start_position + trajectory->radius - pos_from_centre;
}

/*!	\file interpolated_mode.c
	\brief Functions for Interpolated Position Mode
*/

#include "amc.h"

/* Define the IP buffer and place it in "ipdata" section */
#pragma DATA_SECTION(ip_buffer, "ipdata");
ip_point ip_buffer[IP_BUFFER_SIZE] = {{0,0},};	/*!< buffer for fixed points in the Interpolated position mode */

unsigned int ip_iread = 0;		/*!< Index of the next element in the ip_buffer to be read */
unsigned int ip_iwrite = 0;	/*!< Index of the next element in the ip_buffer to be written */
unsigned int ip_available = 0;	/*!< Number of points available in the interpolation buffer */
unsigned int ip_period = 0;	/*!< Interpolation time in internal units */

ip_state_t ip_state = IP_INITIALIZING;	/*!< state of the interpolated mode */

int ip_following_traj = 0; /*< indicates if we were following a trajectory to set TARGET_REACHED */

/*! Function pointer to select the function that calculates the trajectory depending on the interpolation mode selected */
void (*ip_trajectory_interval)(interpolated_trajectory_struct *trajectory, unsigned int time, ip_point *point_a, ip_point *point_b) = NULL;
/*! Function pointer to select the function that checks the trajectory points depending on the interpolation mode selected */
char (*ip_traj_check)(char start, unsigned int time, ip_point *point_a, ip_point *point_b) = NULL;

/*****************************************************************/
/*!	 Main function of Interpolated Position Mode
	\param init will be 1 if the mode has to be initialized (0 if not)
*/
/*****************************************************************/
void ip_mode_operation(motion_state_struct *state, char init)
{
	long Position_demand;
	unsigned int active;
	static unsigned int old_active = 0;
	int reset_timers = 0;

	if (init) {
		ip_initialize(&ip_trajectory, state);		/* initialize the mode if necessary */
		old_active = Device_control_word & MODE_SPECIFIC_BIT4_MASKBIT;
		reset_timers = 1;
	}

	/* Check if interpolation has been activated */
	active = Device_control_word & MODE_SPECIFIC_BIT4_MASKBIT;

	switch (ip_state) {
		case IP_INITIALIZING:
			if (state->time > ip_trajectory.t1) {		/* if stopped keep current position for ever */
				/* get the final point of the stop trajectory before modifying it */
				ip_trajectory.d = ip_trajectory_point(&ip_trajectory, ip_trajectory.t1);
				ip_trajectory.a = 0;
				ip_trajectory.b = 0;
				ip_trajectory.c = 0;
				ip_trajectory.t0 = ip_trajectory.t1;
				ip_trajectory.t1 = LONGLONG_MAX_VAL;		/* 2^63 -1 is the max value of a long long */

				ip_state = IP_READY;		/* set stat as ready */
				_LOGmessage(0x0039,"Interpolated mode ready",0 ,0);
			}
			break;
		case IP_READY:
			if (active && !old_active && !(Device_control_word & HALT_MASKBIT)) {
				/* if not an error in interpolated mode configuration and trajectory is OK */
				if (!ip_configure() &&
						(ip_trajectory_generator(IP_INITIAL_TRAJECTORY, &ip_trajectory, state->time, state) == IP_TRAJ_OK)) {
					ip_state = IP_ACTIVE;
					ip_following_traj = 1;
					_LOGmessage(0x003A,"Interpolated mode active",0 ,0);
				}
				send_SWord = 1;
				reset_timers = 1;
			}
			break;
		case IP_ACTIVE:
			if (state->time > ip_trajectory.t1) {		/* if current time slice has finished */
				if (ip_trajectory_generator(IP_NORMAL_TRAJECTORY, &ip_trajectory, state->time, state) != IP_TRAJ_OK) {
					ip_state = IP_INITIALIZING;		/* set the state as IP_INITIALIZING */
					ip_stop_trajectory(&ip_trajectory, state, IP_START_FROM_DEMAND);	/* set stop trajectory */
					_LOGmessage(0x0038,"Interpolated mode initialization",0 ,0);
				}
			}
			if (!active) {
				ip_state = IP_INITIALIZING;		/* set the state as IP_INITIALIZING */
				ip_stop_trajectory(&ip_trajectory, state, IP_START_FROM_DEMAND);	/* set stop trajectory */
				ip_following_traj = 0;
				_LOGmessage(0x0038,"Interpolated mode initialization",0 ,0);
			}
			break;
		default:		/* should not happen */
			ip_state = IP_INITIALIZING;		/* set the state as IP_INITIALIZING */
			ip_stop_trajectory(&ip_trajectory, state, IP_START_FROM_CURRENT);	/* set stop trajectory */
			_WARNINGmessage(0xffe2, 0x80, 0, "Error in Interpolation state machine", 0, 0);
			_LOGmessage(0x0038,"Interpolated mode initialization",0 ,0);
			break;
	}

	/* save active flag for next cycle */
	old_active = active;

	/* set the size of the available data buffer */
	Interpolation_data_configuration_Actual_buffer_size = ip_available;

	/* get position demand according to current trajectory */
	Position_demand = ip_trajectory_point(&ip_trajectory, state->time);

	/* update Position_demand_value and Position_demand_value_in_increments */
	Position_demand_value_in_increments = Position_demand;			/* [inc] */
	Position_demand_value = int2ext_pos( Position_demand, Home_offset );		/* [position units] */

	/* update Following error */
	Following_error_actual_value = Position_actual_value - Position_demand_value;

//	control_effort = position_controller(Position_demand, state->position);

	/* check status flags using position units (not increments) */
	ip_status_flags((long)Position_actual_value, (long)Position_demand_value, ip_state, state, reset_timers);
}


/*****************************************************************/
/*!	 Function that makes necessary initializations when entering interpolated position mode
	\param trajectory Circle trajectory
	\param state Cinematic state of the system
*/
/*****************************************************************/
void ip_initialize(interpolated_trajectory_struct *trajectory, motion_state_struct *state)
{
	unsigned long ul_tmp;

	/* set max buffer size in the OD */
	Interpolation_data_configuration_Maximum_buffer_size = IP_BUFFER_SIZE;
	/* clear the buffer */
	ip_iwrite = 0;
	ip_iread = 0;
	ip_available = 0;

	/* reset mode specific bits of Statusword */
	Device_status_word &= ~(TARGET_REACHED_MASKBIT | IP_ACTIVE_MASKBIT | BIT13_MASKBIT);
	send_SWord = 1;

	/* we are not following a trajectory */
	ip_following_traj = 0;

	/* set the state as IP_INITIALIZING */
	ip_state = IP_INITIALIZING;
	_LOGmessage(0x0038,"Interpolated mode initialization",0 ,0);

	/* set stop trajectory */
	ip_stop_trajectory(trajectory, state, IP_START_FROM_CURRENT);
	ATM_seti(&current_limit_ip, current_limit);

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
	p_current_limit = &current_limit_ip;	/* set current limit to use */
	EINT;

	ATM_seti(&ready_to_power_on, 1);		/* drive operation enabled */
}


/*****************************************************************/
/*!	 Function that calculates interpolated trajectory when active bit has been cleared ore mode is initialized
	\param trajectory Interpolated position mode trajectory
	\param state Cinematic state of the system
*/
/*****************************************************************/
void ip_stop_trajectory(interpolated_trajectory_struct *trajectory, motion_state_struct *state, char start)
{
	long long tmp;
	unsigned long dec;
	motion_state_struct demand_state;

	if (start == IP_START_FROM_DEMAND) {
		demand_state.time = state->time;
		demand_state.position = ip_trajectory_point(trajectory, demand_state.time);
		demand_state.velocity = ip_trajectory_vel(trajectory, demand_state.time);
	} else {
		demand_state.time = state->time;
		demand_state.position = state->position;
		demand_state.velocity = state->velocity;
	}

	/* set start time */
	trajectory->t0 = demand_state.time;

	/* clear the jerk (a = jerk/6) */
	trajectory-> a = 0;

	/* calculate limited deceleration in [inc/s^2] */
	tmp = (long long)minimum(Profile_deceleration, Max_deceleration) * Acceleration_factor_Numerator;
	tmp /= Acceleration_factor_Divisor;
	dec = tmp;
	trajectory->b = -(tmp >> 1) * sign(demand_state.velocity);		/* b = 1/2*acceleration */

	/* set initial velocity */
	trajectory->c = demand_state.velocity;

	/* set initial position */
	trajectory->d = demand_state.position;

	/* set time to stop */
	tmp = (long long)(labs(demand_state.velocity)) * lcounts_p_s;
	trajectory->t1 = trajectory->t0 + tmp/dec;
}


/*****************************************************************/
/*!	 Calculates the position demand for current control cycle in Interpolated Position Mode
	\param trajectory Interpolated position mode trajectory being followed
	\param current_time Current time
	\return Position demand
*/
/*****************************************************************/
long ip_trajectory_point(interpolated_trajectory_struct *trajectory, long long current_time)
{
	long long tmp;
	unsigned long aux;
	long pos;
	long dt;

	/* get time increment */
	dt = (long)(current_time - trajectory->t0);

	/* calculate x = a*(t-t0)^3 + b*(t-t0)^2 + c*(t-t0) + d */
	pos = trajectory->d;

	tmp = (long long)(trajectory->c) * dt;
	pos += tmp/lcounts_p_s;

	tmp = (long long)dt * dt;
	aux = (unsigned long)lcounts_p_s * lcounts_p_s;
	tmp = (tmp * trajectory->b) / aux;
	pos += tmp;

	tmp = (long long)dt * dt;
	tmp *= dt;
	tmp /= lcounts_p_s;
	aux = (unsigned long)lcounts_p_s * lcounts_p_s;
	tmp = (tmp * trajectory->a) / aux;
	pos += tmp;

	return pos;
}


/*****************************************************************/
/*!	 Calculates the interpolation period in internal time units
	\param units Time value (time = units*10^index)
	\param index Index of the time value. Only values -2 and -3 are allowed (hundreths and mills)
	\return Interpolation time in internal time units. 0 if units or index is invalid
*/
/*****************************************************************/
unsigned int ip_get_period_time(unsigned char units, char index)
{
	unsigned long tmp;
	char index_local;

	tmp = (unsigned long)units * lcounts_p_s;

	/* As char variables are 16 bits I make manually 1's all 8 most significant bits of negative numbers */
	index_local = (unsigned char)(((index + 128)%256) - 128);

	switch(index_local) {
		case -2:
			return tmp / 100;
		case -3:
			return tmp / 1000;
		default:
			return 0;
	}
}


/*****************************************************************/
/*!	 Calculates the trajectory for the next time slice
	\param trajectory Interpolated position mode trajectory being followed
	\param current_time Current time
	\return 0 if right (IP_TRAJ_OK) or a negative value if there is an error
*/
/*****************************************************************/
char ip_trajectory_generator(unsigned char initial, interpolated_trajectory_struct *trajectory, long long current_time, motion_state_struct *state)
{
	ip_point point_a;
	static ip_point point_b;

	if (initial == IP_INITIAL_TRAJECTORY) {
		if(ip_available >= 2) {
			trajectory->t0 = current_time;
			trajectory->t1 = trajectory->t0 + ip_period;
			point_a = ip_buffer[ip_iread];
			ip_iread = (ip_iread + 1)%IP_BUFFER_SIZE;
			point_b = ip_buffer[ip_iread];
			ip_iread = (ip_iread + 1)%IP_BUFFER_SIZE;
			ip_available = ip_available - 2;

			if(ip_traj_check != NULL) {	/* call ip_traj_check if it is a valid pointer */
				char check;
				check = ip_traj_check(IP_INITIAL_TRAJECTORY, ip_period, &point_a, &point_b);
				if (check != IP_TRAJ_OK) return check;
			} else {
				_WARNINGmessage(0xffde, 0x80, 0, "Error in IP configuration", 0, 0);
				return IP_TRAJ_CONF_ERR;
			}

			if(ip_trajectory_interval != NULL) {	/* call ip_trajectory_interval if it is a valid pointer */
				ip_trajectory_interval(trajectory, ip_period, &point_a, &point_b);
				ip_current_limit(trajectory);
				return IP_TRAJ_OK;
			} else {
				_WARNINGmessage(0xffde, 0x80, 0, "Error in IP configuration", 0, 0);
				return IP_TRAJ_CONF_ERR;
			}
		} else {
			_WARNINGmessage(0xffe3, 0x80, 0, "Not enough points for interpolation", 0, 0);
			return IP_TRAJ_NOPOINTS_ERR;
		}
	} else {
		if (ip_available) {
			trajectory->t0 = trajectory->t1;
			trajectory->t1 = trajectory->t0 + ip_period;
			point_a = point_b;
			point_b = ip_buffer[ip_iread];
			ip_iread = (ip_iread + 1)%IP_BUFFER_SIZE;
			--ip_available;

			if(ip_traj_check != NULL) {	/* call ip_traj_check if it is a valid pointer */
				char check;
				check = ip_traj_check(IP_NORMAL_TRAJECTORY, ip_period, &point_a, &point_b);
				if (check != IP_TRAJ_OK) return check;
			} else {
				_WARNINGmessage(0xffde, 0x80, 0, "Error in IP configuration", 0, 0);
				return IP_TRAJ_CONF_ERR;
			}

			if(ip_trajectory_interval != NULL) {	/* call ip_trajectory_interval if it is a valid pointer */
				ip_trajectory_interval(trajectory, ip_period, &point_a, &point_b);
				ip_current_limit(trajectory);
				return IP_TRAJ_OK;
			} else {
				_WARNINGmessage(0xffde, 0x80, 0, "Error in IP configuration", 0, 0);
				return IP_TRAJ_CONF_ERR;
			}
		} else {		/* no more points, stop */
			return IP_TRAJ_END;
		}
	}
}


/*****************************************************************/
/*!	 Calculates the trajectory for an interval in cubic interpolation
	\param trajectory Trajectory to write
	\param time Time interval between points
	\param point_a Initial point
	\param point_b End point
*/
/*****************************************************************/
void ip_trajectory_interval_cubic(interpolated_trajectory_struct *trajectory, unsigned int time, ip_point *point_a, ip_point *point_b)
{
	long long tmp;
	unsigned long ultmp;
	long a_tmp, b_tmp;

	/* set initial position */
	/*! d = x(i) */
	trajectory->d = ext2int_pos(point_a->pos, Home_offset);

	/* set velocity */
	/*! c = v(i) */
	trajectory->c = (long long)ext2int_vel(point_a->vel);

	/* set acceleration */
	/*! b = 3*(x(i+1)-x(i))/t^2 - (2*v(i)+v(i+1))/t */
	tmp = (long long)ext2int_pos(point_b->pos - point_a->pos, 0) * lcounts_p_s;
	tmp *= lcounts_p_s * 3;
	ultmp = (unsigned long)time * time;
	b_tmp = tmp / ultmp;
	tmp = (long long)b_tmp * lcounts_p_s;
	tmp *= -2;
	a_tmp = tmp / (3*time);

	tmp = (long long)ext2int_vel(2*point_a->vel + point_b->vel) * lcounts_p_s;
	trajectory->b = b_tmp - tmp/time;

	/* set jerk */
	/*! a = 2*(x(i)-x(i+1))/t^3 + (v(i)+v(i+1))/t^2 */
	tmp = (long long)ext2int_vel(point_a->vel + point_b->vel) * lcounts_p_s;
	tmp *= lcounts_p_s;
	ultmp = (unsigned long)time * time;
	trajectory->a = a_tmp + tmp/ultmp;
}


/*****************************************************************/
/*!	 Calculates the trajectory for an interval in linear interpolation
	\param trajectory Trajectory to write
	\param time Time interval between points
	\param point_a Initial point
	\param point_b End point
*/
/*****************************************************************/
void ip_trajectory_interval_linear(interpolated_trajectory_struct *trajectory, unsigned int time, ip_point *point_a, ip_point *point_b)
{
	long long tmp;

	/* clear the jerk (a = jerk/6) */
	trajectory-> a = 0;

	/* clear the acceleration (b = acc/2) */
	trajectory->b = 0;

	/* set velocity */
	tmp = (long long)ext2int_pos(point_b->pos - point_a->pos, 0) * lcounts_p_s;
	trajectory->c = tmp / time;

	/* set initial position */
	trajectory->d = ext2int_pos(point_a->pos, Home_offset);
}


/*****************************************************************/
/*!	 Function that checks the conditions of the points of an interpolated trajectory for cubic interpolation mode

	The initial position  has be closer than Position_window from current position, and velocity has to be lower than Velocity_threshold
	\param start IP_INITIAL_TRAJECTORY if trajectory start, IP_NORMAL_TRAJECTORY if not.
	\param time Time interval between points
	\param point_a Initial point
	\param point_b End point
	\return IP_TRAJ_OK if it's OK or a negative value if there is an error
*/
/*****************************************************************/
char ip_traj_check_cubic(char start, unsigned int time, ip_point *point_a, ip_point *point_b)
{
	long long tmp;

	/* special checks for trajectory start */
	if (start == IP_INITIAL_TRAJECTORY) {
		/* check initial position = current position */
		if ((labs(point_a->pos - Position_actual_value) > Position_window)) {
			_WARNINGmessage(0xffe1, 0x80, 0, "Error in the initial point of the trajectory", 0, 0);
			return IP_TRAJ_START_ERR;
		}

		/* check initial velocity = 0 */
		if (labs(point_a->vel) > Velocity_threshold) {
			_WARNINGmessage(0xffe1, 0x80, 0, "Error in the initial point of the trajectory", 0, 0);
			return IP_TRAJ_START_ERR;
		}
	}

	/* check software position limits */
	if ((point_b->pos > Software_position_limit_Max_position_limit) ||
		(point_b->pos < Software_position_limit_Min_position_limit)) {
		_WARNINGmessage(0xffdc, 0x80, 0, "Position out of limits in IP trajectory", 0, 0);
		return IP_TRAJ_POS_ERR;
	}

	/* check position flags */
	if (position_flags & MIN_POS_MASKBIT) {
		if (point_b->pos < point_a->pos) {
			_WARNINGmessage(0xffdc, 0x80, 0, "Position out of limits in IP trajectory", 0, 0);
			return IP_TRAJ_POS_ERR;
		}
	}
	if (position_flags & MAX_POS_MASKBIT) {
		if (point_b->pos > point_a->pos) {
			_WARNINGmessage(0xffdc, 0x80, 0, "Position out of limits in IP trajectory", 0, 0);
			return IP_TRAJ_POS_ERR;
		}
	}

	/* check max velocity */
	if ((labs(point_a->vel) > Max_profile_velocity) || (labs(point_b->vel) > Max_profile_velocity)) {
		_WARNINGmessage(0xffe0, 0x80, 0, "Velocity over limits in IP trajectory", 0, 0);
		return IP_TRAJ_VEL_ERR;
	}

	/* check mean acceleration <= max acceleration */
	tmp = (long long)labs(point_b->vel - point_a->vel) * lcounts_p_s;
	if (tmp/time > Max_acceleration) {
		_WARNINGmessage(0xffdf, 0x80, 0, "Acceleration over limits in IP trajectory", 0, 0);
		return IP_TRAJ_ACC_ERR;
	}

	/* return IP_TRAJ_OK if all checks are right */
	return IP_TRAJ_OK;
}


/*****************************************************************/
/*!	 Function that checks the conditions of the points of an interpolated trajectory for linear interpolation mode
	\param start IP_INITIAL_TRAJECTORY if trajectory start, IP_NORMAL_TRAJECTORY if not.
	\param time Time interval between points
	\param point_a Initial point
	\param point_b End point
	\return IP_TRAJ_OK if it's OK or a negative value if there is an error
*/
/*****************************************************************/
char ip_traj_check_linear(char start, unsigned int time, ip_point *point_a, ip_point *point_b)
{
	long long tmp;
	static long prev_mean_vel = 0;
	long mean_vel;

	/* special checks for trajectory start */
	if (start == IP_INITIAL_TRAJECTORY) {
		/* check initial position = current position */
		if ((labs(point_a->pos - Position_actual_value) > Position_window)) {
			_WARNINGmessage(0xffe1, 0x80, 0, "Error in the initial point of the trajectory", 0, 0);
			return IP_TRAJ_START_ERR;
		}

		/* clear previous value of mean velocity */
		prev_mean_vel = 0;
	}

	/* check software position limits */
	if ((point_b->pos > Software_position_limit_Max_position_limit) ||
		(point_b->pos < Software_position_limit_Min_position_limit)) {
		_WARNINGmessage(0xffdc, 0x80, 0, "Position out of limits in IP trajectory", 0, 0);
		return IP_TRAJ_POS_ERR;
	}

	/* check position flags */
	if (position_flags & MIN_POS_MASKBIT) {
		if (point_b->pos < point_a->pos) {
			_WARNINGmessage(0xffdc, 0x80, 0, "Position out of limits in IP trajectory", 0, 0);
			return IP_TRAJ_POS_ERR;
		}
	}
	if (position_flags & MAX_POS_MASKBIT) {
		if (point_b->pos > point_a->pos) {
			_WARNINGmessage(0xffdc, 0x80, 0, "Position out of limits in IP trajectory", 0, 0);
			return IP_TRAJ_POS_ERR;
		}
	}

	/* check mean velocity <= max velocity */
	tmp = (long long)labs(point_b->pos - point_a->pos) * lcounts_p_s;
	mean_vel = tmp/time;
	if (mean_vel > Max_profile_velocity) {
		_WARNINGmessage(0xffe0, 0x80, 0, "Velocity over limits in IP trajectory", 0, 0);
		return IP_TRAJ_VEL_ERR;
	}

	/* check mean acceleration <= max acceleration */
	tmp = (long long)labs(mean_vel - prev_mean_vel) * lcounts_p_s;
	if (tmp/time > Max_acceleration) {
		_WARNINGmessage(0xffdf, 0x80, 0, "Acceleration over limits in IP trajectory", 0, 0);
		return IP_TRAJ_ACC_ERR;
	}

	/* save mean velocity value */
	prev_mean_vel = mean_vel;

	/* return IP_TRAJ_OK if all checks are right */
	return IP_TRAJ_OK;
}


/*****************************************************************/
/*!	 Configures the period time and the function pointer for the trajectory calculation
	\return 0 if ok, 1 if error
*/
/*****************************************************************/
char ip_configure(void)
{
	char error = 0;

	/* set period time */
	if(!(ip_period = ip_get_period_time(Interpolation_time_period_Interpolation_time_units, Interpolation_time_period_Interpolation_time_index))) {
		error = 1;
		_WARNINGmessage(0xffee, 0x20, 0x60C2, "Wrong value in OD, index: %x", 0x60C2, 0);
	}

	/* set ip_trajectory_interval function pointer */
	switch(Interpolation_sub_mode_select) {
		case 0:		/* linear interpolation */
			ip_trajectory_interval = ip_trajectory_interval_linear;
			ip_traj_check = ip_traj_check_linear;
			break;
		case -1:		/* cubic interpolation */
			ip_trajectory_interval = ip_trajectory_interval_cubic;
			ip_traj_check = ip_traj_check_cubic;
			break;
		default:
			error = 1;
			Interpolation_sub_mode_select = 0;		/* set the default value, linear interpolation */
			_WARNINGmessage(0xffee, 0x20, 0x60C0, "Wrong value in OD, index: %x", 0x60C2, 0);
			break;
	}

	return error;		/* return the error, if any */
}


/*****************************************************************/
/*!	 Manages the flags in Statusword
	\param position Position in [position units]
	\param demand Position demand [position units]
	\param state State of the Interpolated mode state machine
	\param state Cynematic state (position, velocity and time)
	\param reset If reset = 1, timers will be restarted
*/
/*****************************************************************/
void ip_status_flags(long int position, long int demand, ip_state_t ip_state, motion_state_struct *state, int reset)
{
	static long long p_error_started = 0;
	unsigned long long tmp;
	long following_error;

	if( reset ) p_error_started = 0;

	switch (ip_state) {
		case IP_INITIALIZING:
			Device_status_word &= ~TARGET_REACHED_MASKBIT;	/* clear target reached maskbit */
			Device_status_word &= ~IP_ACTIVE_MASKBIT;			/* clear interpolation active maskbit */
			break;
		case IP_READY:
			if ( ip_following_traj )
				Device_status_word |= TARGET_REACHED_MASKBIT;	/* trajectory finished */
			else
				Device_status_word &= ~TARGET_REACHED_MASKBIT;	/* no target reached after initialization */
			Device_status_word &= ~IP_ACTIVE_MASKBIT;			/* clear interpolation active maskbit */
			break;
		case IP_ACTIVE:
			Device_status_word |= IP_ACTIVE_MASKBIT;			/* Interpolated position mode active */
			Device_status_word &= ~TARGET_REACHED_MASKBIT;	/* clear target reached maskbit */

			/* Check if following error is too big */
			following_error = position - demand;
			if(labs(following_error) > Following_error_window)
			{
				if(!p_error_started) p_error_started = state->time;		/* Start time counting if not started */
				tmp = (long long)(state->time - p_error_started) * 1000;
				tmp /= lcounts_p_s;
				if(tmp > Following_error_time_out)
				{
					Device_status_word |= FOLLOWING_ERROR_MASKBIT;

					/* set the position following error */
					if(!(isFaultActive(FAULT_POS_FOLLOWING)))
					{
						_ERRORmessage(0x8611, 0x20, 0x0000, "position following error", 0, 0);
						setFault(FAULT_POS_FOLLOWING);
					}
				} else
					Device_status_word &= ~FOLLOWING_ERROR_MASKBIT;
			} else
			{
				p_error_started = 0;			/* stop time counting */
				Device_status_word &= ~FOLLOWING_ERROR_MASKBIT;			/* Following error bit = 0 */
			}
			break;
		default:
			ip_state = IP_INITIALIZING;		/* set the state as IP_INITIALIZING */
			ip_stop_trajectory(&ip_trajectory, state, IP_START_FROM_CURRENT);	/* set stop trajectory */
			_WARNINGmessage(0xffe2, 0x80, 0, "Error in Interpolation state machine", 0, 0);
			_LOGmessage(0x0038,"Interpolated mode initialization",0 ,0);
			break;
	}
}


/*****************************************************************/
/*!	 Calculates the velocity demand for current control cycle in Interpolated Position Mode
	\param trajectory Interpolated position mode trajectory being followed
	\param current_time Current time
	\return Velocity demand
*/
/*****************************************************************/
long ip_trajectory_vel(interpolated_trajectory_struct *trajectory, long long current_time)
{
	long long tmp;
	unsigned long aux;
	long vel;
	long dt;

	/* get time increment */
	dt = (long)(current_time - trajectory->t0);

	/* calculate vel = 3*a*(t-t0)^2 + 2*b*(t-t0) + c */
	vel = trajectory->c;

	tmp = (long long)(trajectory->b) * (2*dt);
	vel += tmp/lcounts_p_s;

	tmp = (long long)dt * dt;
	aux = (unsigned long)lcounts_p_s * lcounts_p_s;
	tmp = (tmp * trajectory->b * 2) / aux;
	vel += tmp;

	return vel;
}


/*****************************************************************/
/*!	 Sets the current limit for IP mode according to max acceleration in the interval
	\param trajectory Interpolated position mode trajectory being followed
*/
/*****************************************************************/
void ip_current_limit(interpolated_trajectory_struct *trajectory)
{
	long long tmp;
	long max_acc;
	int c_limit;

	tmp = (long long)((long)(trajectory->t1 - trajectory->t0)) * trajectory->a;
	tmp = trajectory->b + tmp/lcounts_p_s;

	max_acc = maximum(labs(trajectory->b), labs(tmp));

	if(max_acc > Max_acceleration) c_limit = current_limit_acc;	/* maximum current limit */
	else {
		/* interpolate between current_limit and current_limit_acc */
		tmp = max_acc * (current_limit_acc - current_limit);
		c_limit = current_limit + tmp / Max_acceleration;
	}

	ATM_seti(&current_limit_ip, c_limit);
}

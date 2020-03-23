/*!	\file velocity_mode.c
	\brief Functions for Profile Velocity Mode
 */

#include "amc.h"
#include "fpga.h"
#include "debug_sci.h"
#include "dc_bldc_current.h"

#define TIMER_SLIPPAGE	0x01
#define TIMER_HALT		0x02

#define FREQ_VELOCITY_LOOP 10 // 10*1ms

int velocity_error_mms =0;
int pv_following_target = 0;	/*!< Indicates wether we are going to a target or just in initial stop trajectory */


static manual_state_t vel_state = DETENTS_INACTIVE;	/*!< state of the assisted mode */

static long pv_get_max_travel(void);
static long int detent_pos_loop (motion_state_struct *state, short reset);
static _iq pv_position_controller( long pos_fb, long pos_dem, _iq Sat_output, _iq Feedforward);
static void pv_detent_status_flags(manual_state_t manual_state, long int detent, motion_state_struct *state, short reset);
void static control_effort_testbench_manage (void);

static _iq pv_position_controller_test( long pos_fb, long pos_dem, _iq Sat_output, _iq Feedforward);
unsigned int debug_detent=0;
unsigned int velocity_demand_test=0;

long int pv_max_travel;



static int pv_set_pid_tunning( void )
{
	unsigned long ul_tmp;

	ul_tmp = Velocity_control_parameter_set[0];
	ul_tmp = ul_tmp << 16;							/* multiply by 2^16 */
	ul_tmp /= Velocity_control_parameter_set[5];			/* divide by Divisor */
	// pv_control_pid.Kp = _IQmpyI32(_IQ(0.0000152587890625),ul_tmp);	/* divide by 2^16 */
	pv_control_pid.Kp = ul_tmp << (GLOBAL_Q - 16);		/* convert to IQ and divide by 2^16 */
#if 1
	pv_control_pid_mms.Kp = ul_tmp << (GLOBAL_Q - 16);		/* convert to IQ and divide by 2^16 */
#endif

	ul_tmp = Velocity_control_parameter_set[2];
	ul_tmp = ul_tmp << 16;						/* multiply by 2^16 */
	ul_tmp /= Velocity_control_parameter_set[5];			/* divide by Divisor */
#if 0
	pv_control_pid.Ki = _IQmpyI32(_IQ(0.0000152587890625),ul_tmp);	/* divide by 2^16 */
#else
	pv_control_pid.Ki 			= ul_tmp << (GLOBAL_Q - 16);		/* convert to IQ and divide by 2^16 */
	pv_control_pid_mms.Ki 		= ul_tmp << (GLOBAL_Q - 16);		/* convert to IQ and divide by 2^16 */
#endif
	ul_tmp = Velocity_control_parameter_set[3];
	ul_tmp = ul_tmp << 16;						/* multiply by 2^16 */
	ul_tmp /= Velocity_control_parameter_set[5];			/* divide by Divisor */
#if 0
	pv_control_pid.Kd = _IQmpyI32(_IQ(0.0000152587890625),ul_tmp);	/* divide by 2^16 */
#else
	pv_control_pid.Kd  			= ul_tmp << (GLOBAL_Q - 16);		/* convert to IQ and divide by 2^16 */
	pv_control_pid_mms.Kd = ul_tmp << (GLOBAL_Q - 16);		/* convert to IQ and divide by 2^16 */
#endif


	ul_tmp = Velocity_control_parameter_set[4];
	ul_tmp = ul_tmp << 16;						/* multiply by 2^16 */
	ul_tmp /= Velocity_control_parameter_set[5];			/* divide by Divisor */
#if 0
	pv_control_pid.Kc = _IQmpyI32(_IQ(0.0000152587890625),ul_tmp);	/* divide by 2^16 */
#else
	pv_control_pid.Kc  			= ul_tmp << (GLOBAL_Q - 16);		/* convert to IQ and divide by 2^16 */
	pv_control_pid_mms.Kc = ul_tmp << (GLOBAL_Q - 16);		/* convert to IQ and divide by 2^16 */
#endif

	vel_normFactor = Velocity_control_margin;


	set_Kp_current_control_dc_bldc(Velocity_Current_control_parameter_set[0],
								   Velocity_Current_control_parameter_set[4]);

	set_Ki_current_control_dc_bldc(Velocity_Current_control_parameter_set[1],
								   Velocity_Current_control_parameter_set[4]);

	set_Kd_current_control_dc_bldc(Velocity_Current_control_parameter_set[2],
								   Velocity_Current_control_parameter_set[4]);

	set_Kc_current_control_dc_bldc(Velocity_Current_control_parameter_set[3],
								   Velocity_Current_control_parameter_set[4]);



	ul_tmp = Velocity_Detent_control_parameters[0];
	ul_tmp = ul_tmp << 16;							/* multiply by 2^16 */
	ul_tmp /= Velocity_Detent_control_parameters[5];			/* divide by Divisor */
	pv_position_control_pid.Kp = ul_tmp << (GLOBAL_Q - 16);		/* convert to IQ and divide by 2^16 */

	ul_tmp = Velocity_Detent_control_parameters[1];
	ul_tmp = ul_tmp << 16;							/* multiply by 2^16 */
	ul_tmp /= Velocity_Detent_control_parameters[5];			/* divide by Divisor */
	pv_position_control_pid.Ki = ul_tmp << (GLOBAL_Q - 16);		/* convert to IQ and divide by 2^16 */

	ul_tmp = Velocity_Detent_control_parameters[2];
	ul_tmp = ul_tmp << 16;							/* multiply by 2^16 */
	ul_tmp /= Velocity_Detent_control_parameters[5];			/* divide by Divisor */
	pv_position_control_pid.Kd = ul_tmp << (GLOBAL_Q - 16);		/* convert to IQ and divide by 2^16 */

	ul_tmp = Velocity_Detent_control_parameters[3];
	ul_tmp = ul_tmp << 16;							/* multiply by 2^16 */
	ul_tmp /= Velocity_Detent_control_parameters[5];			/* divide by Divisor */
	pv_position_control_pid.Kc = ul_tmp << (GLOBAL_Q - 16);		/* convert to IQ and divide by 2^16 */

	ul_tmp = Velocity_Detent_control_parameters[4];
	ul_tmp = ul_tmp << 16;							/* multiply by 2^16 */
	ul_tmp /= Velocity_Detent_control_parameters[5];			/* divide by Divisor */
	pv_position_control_pid.Kff = ul_tmp << (GLOBAL_Q - 16);		/* convert to IQ and divide by 2^16 */

	return 0;
}


/*****************************************************************/
/*!	 Main function of Profile Velocity Mode
	\param state Cinematic state of the system
	\param state Cynematic state (position, velocity and time)
 */
/*****************************************************************/
int profile_velocity_mode_operation(motion_state_struct *state, char init)
{

	long new_target;		/* for new target mailbox recetion */
	int reset_timers = 0;
	static long long pv_init_timer=0;
	unsigned long long tmp;
	int ret = 1;
	short reset_detent_timer = 0;



	if(init)
	{
		pv_initialize(state);		/* initialize the mode if necessary */
		reset_timers = TIMER_SLIPPAGE; /* all timers will be reset */
		reset_detent_timer = 1;
		if( Device_status_word & BRAKE_MASKBIT )
			pv_init_timer = state->time;
	}
	else
	{
		tmp = (long long)(state->time - pv_init_timer) * 1000;
		tmp /= lcounts_p_s;
		if( tmp > pv_init_time )
		{
			ret = 0;
			/* modify trajectory if a new target arrived */
			if (MBX_pend(&pv_newtarget_mbox, &new_target, 0) && !(Device_control_word & HALT_MASKBIT))
			{
				pv_newVelocityTarget(new_target, &pv_trajectory, state);
				pv_following_target = 1;
				send_SWord = 1;
				reset_timers = TIMER_SLIPPAGE; /* Target reached and max slippage timers will be reset */
			}
		}
	}

	/* if we are going out of limits and velocity target is not null, set null velocity as target */
	if (pv_trajectory.target_v && pv_position_limits())
	{
		/* set new trajectory with null velocity as target */
		pv_newVelocityTarget(0, &pv_trajectory, state);
	}

	//velocity_demand = pv_trajectory_point(&pv_trajectory, state->time);
	Velocity_demand_value = detent_pos_loop(state, reset_detent_timer);
	Velocity_demand_value_in_increments = ext2int_vel(Velocity_demand_value);


	pv_set_pid_tunning();

	/* update velocity error */
	Velocity_following_error = Velocity_actual_value - Velocity_demand_value;

//	control_effort = pv_velocity_controller(velocity_demand, state->velocity);

	control_effort_vel_mms =  pv_velocity_controller_mms(Velocity_demand_value, Velocity_actual_value);

	control_effort_testbench_manage();

	pv_status_flags((long)Velocity_actual_value, (long)Velocity_demand_value, (long)Target_velocity, state, reset_timers);	/* [velocity units] */

#ifdef DEBUG_VEL_LOOP
	TickDebugSci14bytesVelocityMode();
#endif

	return ret;
}

static long int detent_pos_loop (motion_state_struct *state, short reset_detent_timer)
{
	_iq position_effort = 0, pos_velocity_max = 0;//, actual_effort = 0;
	static long int detent_position = 0;
	static _iq last_dem = 0;
	long norm_factor = 0;
	long int velocity_demand =0;
	long int detent_velocity_demand=0;

	/* Use Max_profile_velocity to normalize the position PID */
	norm_factor = ext2int_vel(Max_profile_velocity);

	velocity_demand = pv_trajectory_point(&pv_trajectory, state->time);
	velocity_demand = int2ext_vel(velocity_demand);			/* [velocity units] */

	/* Detent position calculator */
	switch(vel_state)
	{
		case DETENTS_INACTIVE:
			/* Reset detent leave */
			Detents_config_Leave = 0;
			/* Debug position control loop */
/*
			if(Velocity_mode_dentent_active)
			{
				vel_state = DETENTS_ACTIVE;
				detent_position = ext2int_pos(Velocity_mode_debug_position,Home_offset); // Unit conversion of target position
				break;
			}
*/

			if (detent_nearby(int2ext_pos(state->position, Home_offset), &Detents_config_Active_detent) &&
			   (int2ext_vel(labs(state->velocity)) <= Detents_config_Max_velocity))
			{
				vel_state = DETENTS_ACTIVE;
				detent_position = ext2int_pos(Detents_config_Active_detent, Home_offset);	/* converto to internal units */

				reset_detent_timer = 1;

				/* update Position_demand_value and Position_demand_value_in_increments */
				Position_demand_value_in_increments = detent_position;	/* [inc] */
				Position_demand_value = Detents_config_Active_detent;		/* [position units] */
				break;
			}
			break;
		case DETENTS_ACTIVE:
#if 1
			/* Maximum velocity to achieve position target */
			pos_velocity_max = (((long long)ext2int_vel(labs(Max_demand_velocity))) << GLOBAL_Q) / norm_factor;
			/* Position controller */
			position_effort = pv_position_controller(state->position, detent_position, last_dem, 0);
			/* Saturate the output with the max. velocity */
			position_effort = _IQsat(position_effort,pos_velocity_max,-pos_velocity_max);
			/* Convert from IQ to velocity external units */
			detent_velocity_demand = int2ext_vel(_IQmpyI32int(position_effort, norm_factor));

			/* Set velocity demand to velocity controller */
			velocity_demand = detent_velocity_demand;

			velocity_demand_test = pv_position_controller_test(Position_actual_value, Position_demand_value,0,0);
#endif
			/* Leave the detent when active */
			if (Detents_config_Leave)
			{
				vel_state = DETENTS_LEAVING;
			}
			else if (int2ext_pos(labs(state->position - detent_position), 0) > Detents_config_Skip_distance)
			{
				/* Became inactive if position is too far from detent point */
				vel_state = DETENTS_INACTIVE;
				break;
			}
			break;
		case DETENTS_LEAVING:
			if (!detent_nearby(int2ext_pos(state->position, Home_offset), &detent_position))
				vel_state = DETENTS_INACTIVE;
			break;
		default:		/* should never happen */
			vel_state = DETENTS_INACTIVE;
			break;
	}
	/* Check and set the status flags */
	pv_detent_status_flags(vel_state, detent_position, state, reset_detent_timer);
	return velocity_demand;
}

/*****************************************************************/
/*!	Position Mode position controller
	\param pos_fb Actual position
	\param pos_dem Demanded position
	\param Sat_output Input for anti-windup calculation
	\param Feedforward input
*/
/*****************************************************************/

_iq pv_position_controller( long pos_fb, long pos_dem, _iq Sat_output, _iq Feedforward)
{
	long long tmp;
#if 1
	if(!pos_error_normFactor) return 0; /* Cannot divide by 0 */

	tmp = (long long)(pos_fb - pos_dem) * Position_factor_Feed_constant;
	tmp = tmp << 5;	/* multiply by 32 to avoid resolution losses */
	tmp /= Position_factor_Numerator;					/* tmp = 32 * error [position units] */
	if(llabs(tmp) > (long long)(pos_error_normFactor<<5))
		tmp = sign(tmp) * (long long)(pos_error_normFactor<<5); /* limit the error to 'Position_control_margin' */

	/* reference position is always '0' and current position is set as the relative error */
	pv_position_control_pid.Ref = _IQ(0.0);

	/* Divide by normFactor and later convert to IQ and divide by 2^5 */
	pv_position_control_pid.Fdb = (tmp << (GLOBAL_Q - 5)) / pos_error_normFactor ;	/* position error related to 'Position_control_margin' */

	/* Set the anti-windup and feed-forward */
	pv_position_control_pid.AntiWindup = _IQsat(Sat_output,_IQ(1.0),_IQ(-1.0));
	pv_position_control_pid.FeedForward = _IQsat(Feedforward,_IQ(1.0),_IQ(-1.0));

	pv_position_control_pid.calc(&pv_position_control_pid);		/* calculate PID output */

	return pv_position_control_pid.Out;
#else
	pv_position_control_pid.Ref = _IQsat(_IQdiv(pos_dem	,max_travel),_IQ(1.0),_IQ(-1.0));
	pv_position_control_pid.Fdb = _IQdiv(pos_fb	,max_travel);//_IQsat(_IQdiv(position	,max_travel),_IQ(1.0),_IQ(-1.0));
	pv_position_control_pid.calc(&pv_position_control_pid);		/* calculate PID output */
	return pv_position_control_pid.Out;
#endif
}


_iq pv_position_controller_test( long pos_fb, long pos_dem, _iq Sat_output, _iq Feedforward)
{
	pv_position_control_pid_test.Ref = _IQsat(_IQdiv(pos_dem	,pv_max_travel),_IQ(1.0),_IQ(-1.0));
	pv_position_control_pid_test.Fdb = _IQdiv(pos_fb	,pv_max_travel);//_IQsat(_IQdiv(position	,max_travel),_IQ(1.0),_IQ(-1.0));
	pv_position_control_pid_test.calc(&pv_position_control_pid_test);		/* calculate PID output */
	return pv_position_control_pid_test.Out;
}



/*****************************************************************/
/*!	Calculates the PID output in Profile Velocity Mode
	\param demand Velocity demand
	\param velocity Current velocity
	\return PID output [-1,1]
 */
/*****************************************************************/
_iq pv_velocity_controller(long int demand, long int velocity)
{
	long norm_factor;
	_iq dem, vel;

	/* I use Max_profile_velocity to normalize inputs to the PID */
	norm_factor = labs( ext2int_vel( vel_normFactor ) );
	dem = (((long long)demand) << GLOBAL_Q) / norm_factor;
	vel = (((long long)velocity) << GLOBAL_Q) / norm_factor;

	pv_control_pid.Ref = dem;
	pv_control_pid.Fdb = vel;

	pv_control_pid.calc(&pv_control_pid);		/* calculate PID output */

	return pv_control_pid.Out;
}


/*****************************************************************/
/*!	Calculates the PID output in Profile Velocity Mode
	\param demand Velocity demand
	\param velocity Current velocity
	\return PID output [-1,1]
 */
/*****************************************************************/
_iq pv_velocity_controller_mms(long int demand, long int velocity)
{
	velocity_error_mms = (long long)(demand - velocity);

	pv_control_pid_mms.Ref = _IQsat(_IQdiv(demand,Max_demand_velocity),_IQ(1.0),_IQ(-1.0));
	pv_control_pid_mms.Fdb = _IQdiv(velocity,Max_demand_velocity);//_IQsat(_IQdiv(velocity,Max_demand_velocity),_IQ(1.0),_IQ(-1.0));//
	pv_control_pid_mms.calc(&pv_control_pid_mms);		/* calculate PID output */

	return pv_control_pid_mms.Out;
}



/*****************************************************************/
/*!	 Calculates the velocity demand for current control cycle
	\param target Velocity trajectory being followed
	\param current_time Current time
	\return Velocity demand
 */
/*****************************************************************/
long int pv_trajectory_point(velocity_trajectory_struct *target, long long current_time)
{
	/* It depends on the motion profile type (by now only interpolated mode used) */
	if(current_time > (*target).end_time)
	{
		DINT;		/* not accelerating */
		p_current_limit = &current_limit;
		EINT;
		return (*target).target_v;
	}
	else
	{
		long long tmp;

		if((*target).stop_time == 0)			/* no intermediate stop */
		{
			DINT;		/* check if accelerating to set current limit */
			p_current_limit = (labs(target->target_v) > labs(target->init_v))? &current_limit_acc : &current_limit;
			EINT;

			tmp = ((*target).target_v - (*target).init_v) * (current_time - (*target).init_time);
			tmp /= ((*target).end_time - (*target).init_time);
			return (*target).init_v + tmp;
		}
		else if(current_time < (*target).stop_time)		/* still decelerating */
		{
			DINT;		/* not accelerating */
			p_current_limit = &current_limit;
			EINT;

			tmp = (current_time - (*target).init_time) * (-(*target).init_v);
			tmp /= ((*target).stop_time - (*target).init_time);
			return (*target).init_v + tmp;
		}
		else
		{
			DINT;		/* accelerating */
			p_current_limit = &current_limit_acc;
			EINT;

			tmp = (current_time - (*target).stop_time) * ((*target).target_v);
			tmp /= ((*target).end_time - (*target).stop_time);
			return tmp;
		}
	}
}


/*****************************************************************/
/*!	 Calculates main points of the trajectory to follow
  It starts from current velocity demand, not from current status
	\param new_target New velocity target (in velocity units)
	\param target Velocity trajectory to be created
	\param state Cinematic state of the system
 */
/*****************************************************************/
void pv_newVelocityTarget(long new_target, velocity_trajectory_struct *target, motion_state_struct *state)
{
	long limit_p, limit_n;
	unsigned long acc, dec, quick_dec;
	long long temp;
	//long vel = state->velocity;
	long vel = pv_trajectory_point( target, state->time );

	/* initial point is current demand, not current velocity */
	(*target).init_v = pv_trajectory_point( target, state->time );

	/* calculate limited profile velocity */
	temp = (long long)Max_motor_speed * Velocity_factor_1_Numerator;
	temp /= Velocity_factor_1_Divisor;					/* Max_motor_speed limit */
	limit_p = minimum(temp,Max_profile_velocity);			/* minimum of Max_motor_speed and Max_profile_velocity */
	limit_n = -limit_p;

	/* if any position limit flag is active allow only velocity in the opposite sense */
	if((Polarity & 0x40) == ((Polarity & 0x80) >> 1))
	{
		if (position_flags & MIN_POS_MASKBIT) limit_n = 0;
		if (position_flags & MAX_POS_MASKBIT) limit_p = 0;
	}
	else
	{
		if (position_flags & MIN_POS_MASKBIT) limit_p = 0;
		if (position_flags & MAX_POS_MASKBIT) limit_n = 0;
	}

	/* limit target and convert into [inc/s] */
	new_target = (new_target >= 0) ? minimum(new_target, limit_p) : maximum(new_target, limit_n);
	(*target).target_v = ext2int_vel(new_target);		/* convert to inc/s */

	(*target).init_time = state->time;

	/* calculate limited accelerations in [inc/s2] */
	acc = minimum(Profile_acceleration, Max_acceleration);
	dec = minimum(Profile_deceleration, Max_deceleration);
	quick_dec = minimum(Quick_stop_deceleration, Max_deceleration);
	if((acc == 0) || (dec == 0) || (quick_dec == 0))
	{
		ATM_seti(&ready_to_power_on, 0);
		if(!(isFaultActive(FAULT_NULL_ACC)))
		{
			_ERRORmessage(0xff03, 0x01, 0x0000, "Null acceleration value", 0, 0);
			setFault(FAULT_NULL_ACC);
		}
		QueueFault(FAULT_NULL_ACC);
	}
	else
	{
		DeQueueFault(FAULT_NULL_ACC);
	}
	temp = (long long)acc * Acceleration_factor_Numerator;
	acc = temp / Acceleration_factor_Divisor;
	temp = (long long)dec * Acceleration_factor_Numerator;
	dec = temp / Acceleration_factor_Divisor;
	temp = (long long)quick_dec * Acceleration_factor_Numerator;
	quick_dec = temp / Acceleration_factor_Divisor;

	/* If fault reaction active or quick stop at quick stop deceleration */
	if ((getDriveState(Device_status_word) == FAULT_REACTION_ACTIVE) ||
			(getDriveState(Device_status_word) == QUICK_STOP_ACTIVE) && ((Quick_stop_option_code == 2) ))
	{
		temp = (long long)labs(vel) * lcounts_p_s;
		temp /= quick_dec;
		(*target).end_time = (*target).init_time + temp;
		(*target).stop_time = 0;
	}
	else
	{
		if(vel >= 0)
		{
			if((*target).target_v < 0)		/* There will be an intermediate stop */
			{
				temp = (long long)vel * lcounts_p_s;
				temp /= dec;
				(*target).stop_time = (*target).init_time + temp;		/* set the stop time */
				temp = (long long)(-(*target).target_v) * lcounts_p_s;
				temp /= acc;
				(*target).end_time = (*target).stop_time + temp;		/* set the end time */
			}
			else if((*target).target_v >= vel)		/* acceleration */
			{
				temp = (long long)((*target).target_v - vel) * lcounts_p_s;
				temp /= acc;
				(*target).end_time = (*target).init_time + temp;
				(*target).stop_time = 0;
			}
			else					/* deceleration */
			{
				temp = (long long)(vel - (*target).target_v) * lcounts_p_s;
				temp /= dec;
				(*target).end_time = (*target).init_time + temp;
				(*target).stop_time = 0;
			}
		}
		else
		{
			if((*target).target_v > 0)		/* There will be an intermediate stop */
			{
				temp = (long long)(-vel) * lcounts_p_s;
				temp /= dec;
				(*target).stop_time = (*target).init_time + temp;		/* set the stop time */
				temp = (*target).target_v;
				temp = (temp * lcounts_p_s) / acc;
				(*target).end_time = (*target).stop_time + temp;		/* set the end time */
			}
			else if((*target).target_v <= vel)		/* acceleration */
			{
				temp = (long long)(vel - (*target).target_v) * lcounts_p_s;
				temp /= acc;
				(*target).end_time = (*target).init_time + temp;
				(*target).stop_time = 0;
			}
			else					/* deceleration */
			{
				temp = (long long)((*target).target_v - vel) * lcounts_p_s;
				temp /= dec;
				(*target).end_time = (*target).init_time + temp;
				(*target).stop_time = 0;
			}
		}
	}
}


/*****************************************************************/
/*!	 Calculates main points of a trajectory from current position to stop
	\param deceleration deceleration used to calculate trajectory
	\param target Velocity trajectory to be created
	\param state Cinematic state of the system
 */
/*****************************************************************/
void pv_stop_trajectory( long deceleration, velocity_trajectory_struct *target, motion_state_struct *state )
{
	unsigned long dec;
	long long temp;
	//long vel = state->velocity;
	long vel = pv_trajectory_point( target, state->time );

	(*target).target_v = 0;		/* convert to inc/s */

	(*target).init_v = vel;

	(*target).init_time = state->time;

	/* calculate limited decelerations in [inc/s2] */
	dec = minimum(deceleration, Max_deceleration);
	if( dec == 0 )
	{
		ATM_seti(&ready_to_power_on, 0);
		if(!(isFaultActive(FAULT_NULL_ACC)))
		{
			_ERRORmessage(0xff03, 0x01, 0x0000, "Null acceleration value", 0, 0);
			setFault(FAULT_NULL_ACC);
		}
		QueueFault(FAULT_NULL_ACC);
	}
	else
	{
		DeQueueFault(FAULT_NULL_ACC);
	}
	temp = (long long)dec * Acceleration_factor_Numerator;
	dec = temp / Acceleration_factor_Divisor;

	temp = (long long)labs(vel) * lcounts_p_s;
	temp /= dec;
	(*target).end_time = (*target).init_time + temp;
	(*target).stop_time = 0;
}




/*****************************************************************/
/*!	 Sets the status flags in Profile Velocity Mode (Target Reached, Zero Speed and Max Slippage
	\param velocity Current velocity [velocity units]
	\param demand Current Demand velocity [velocity units]
	\param target Target velocity [velocity units]
	\param state Cinematic state of the system
	\param reset If reset != 0, some timers will be restarted
 */
/*****************************************************************/
void pv_status_flags(long int velocity, long int demand, long int target, motion_state_struct *state, int reset)
{

	static long long v_slippage_started = 0;
	//static long long v_halt_reached_started = 0;
	long long tmp;

	if(reset & TIMER_SLIPPAGE) v_slippage_started = 0;
	//if(reset & TIMER_HALT) v_halt_reached_started = 0;

	/* Check if slippage is too large */
	if(((demand - velocity >= 0)? demand - velocity : velocity - demand ) >= Max_slippage)
	{
		if(!v_slippage_started)
		{
			v_slippage_started = state->time;		/* Start time counting if not started */
		}
		tmp = state->time - v_slippage_started;
		tmp = (tmp * 1000) / lcounts_p_s;
		if(tmp >= Velocity_threshold_time)
		{
			if( pv_following_target )
			{
				Device_status_word |= MAX_SLIPPAGE_MASKBIT;		/* Max slippage reached bit = 1 */

				/* set the position following error */
				if(!(isFaultActive(FAULT_VEL_FOLLOWING)))
				{
					_ERRORmessage(0x8400, 0x20, 0x0000, "velocity following error", 0, 0);
					setFault(FAULT_VEL_FOLLOWING);
				}
				QueueFault(FAULT_VEL_FOLLOWING);
				//disable the checking because the system enters in OPERATION_ENABLE before mode is initialized
				pv_following_target = 0;
			}
		}
		else
		{
			Device_status_word &= ~MAX_SLIPPAGE_MASKBIT;
			DeQueueFault(FAULT_VEL_FOLLOWING);
		}
	}
	else
	{
		v_slippage_started = 0;			/* stop time counting */
		Device_status_word &= ~MAX_SLIPPAGE_MASKBIT;		/* Max slippage reached bit = 0 */
		DeQueueFault(FAULT_VEL_FOLLOWING);
	}


}


/*****************************************************************/
/*!	 Function that makes necessary initializations when entering profile velocity mode
	\param state Cinematic state of the system
 */
/*****************************************************************/
void pv_initialize(motion_state_struct *state)
{
	int target;

	while(MBX_pend(&pv_newtarget_mbox, &target, 0));	/* empty pv_newtarget_mbox */

	/* set new trajectory with null velocity as target */
	pv_stop_trajectory( Profile_deceleration, &pv_trajectory, state );	/* velocity target = 0 (from current state) */

	/* reset mode specific bits of Statusword */
	Device_status_word &= ~(TARGET_REACHED_MASKBIT | MAX_SLIPPAGE_MASKBIT | HALT_REACHED_MASKBIT);
	send_SWord = 1;

	/* initialize variables */
	pv_following_target = 0;

	/* Reset current error BLAC current control*/
	Ui_d=0;
	Up_d=0;
	SatErr_d=0;
	Ui_q=0;
	Up_q=0;
	SatErr_q=0;

	vel_state = DETENTS_LEAVING;

	/* initialize PID */ //Velocidad viejo
	pv_control_pid.Err = 0;
	pv_control_pid.Up = 0;
	pv_control_pid.Ui = 0;
	pv_control_pid.Ud = 0;
	pv_control_pid.OutPreSat = 0;
	pv_control_pid.Out = 0;
	pv_control_pid.SatErr = 0;
	pv_control_pid.Err1 = 0;
	pv_control_pid.OutMax = MAX_DUTY_CYCLE;
	pv_control_pid.OutMin = MIN_DUTY_CYCLE;

	pv_control_pid.Period = _IQ(0.001);
	pv_control_pid.Frequency = 1000;


	//Velocidad nuevo
	pv_control_pid_mms.Err = 0;
	pv_control_pid_mms.Up = 0;
	pv_control_pid_mms.Ui = 0;
	pv_control_pid_mms.Ud = 0;
	pv_control_pid_mms.OutPreSat = 0;
	pv_control_pid_mms.Out = 0;
	pv_control_pid_mms.SatErr = 0;
	pv_control_pid_mms.Err1 = 0;
	pv_control_pid_mms.OutMax = MAX_DUTY_CYCLE;
	pv_control_pid_mms.OutMin = MIN_DUTY_CYCLE;

	pv_control_pid_mms.Period = _IQ(0.001);
	pv_control_pid_mms.Frequency = 1000;

	/* initialize assisted mode position PID */
	//Detenes velocidad
	pv_position_control_pid.Err1 = 0;
	pv_position_control_pid.SatError = 0;
	pv_position_control_pid.Ui = 0;
	pv_position_control_pid.Ud = 0;
	pv_position_control_pid.Period = _IQ(0.01);
	pv_position_control_pid.Frequency = 100;

	init_current_controller_dc_bldc();

	pv_set_pid_tunning();
#if 1
	pv_max_travel=pp_get_max_travel();
	pv_position_control_pid_test = pv_position_control_pid;
#endif

	DINT;
	p_current_limit = &current_limit;	/* set current limit to use */
	EINT;

	ATM_seti(&ready_to_power_on, 1);
}


/*****************************************************************/
/*!	 Checks if the current speed would make the drive go further position limits
	\return 0 if drive can stop between limits, 1 if not.
 */
/*****************************************************************/
int pv_position_limits(void)
{
	long long stop_length;
	long dec;

	/* calculate deceleration */
	dec = minimum(Profile_deceleration, Max_deceleration);

	/* calculate stop length (absolute value) */
	stop_length = (long long)Velocity_actual_value * Velocity_actual_value;
	stop_length /= dec;
	stop_length = stop_length / 2;

	/* multiply stop_length by 1,125 for some margin */
	stop_length += (stop_length >> 3);

	/* check limits */
	if((Polarity & 0x40) == ((Polarity & 0x80) >> 1))	/* if position polarity == velocity polarity (most usual) */
	{
		if ((Velocity_actual_value > 0) && (Target_velocity > 0)) {
			if ((Position_actual_value + (long)stop_length) >= Software_position_limit_Max_position_limit) return 1;
			else return 0;
		}
		if ((Velocity_actual_value < 0) && (Target_velocity < 0)) {
			if ((Position_actual_value - (long)stop_length) <= Software_position_limit_Min_position_limit) return 1;
			else return 0;
		}
		return 0;		/* Velocity_actual_value = 0 */
	}
	else
	{
		if ((Velocity_actual_value < 0) && (Target_velocity < 0)) {
			if ((Position_actual_value + (long)stop_length) >= Software_position_limit_Max_position_limit) return 1;
			else return 0;
		}
		if ((Velocity_actual_value > 0) && (Target_velocity > 0)) {
			if ((Position_actual_value - (long)stop_length) <= Software_position_limit_Min_position_limit) return 1;
			else return 0;
		}
		return 0;		/* Velocity_actual_value = 0 */
	}
}



void pv_trajectory_set_stop_point( unsigned long dec, velocity_trajectory_struct *trajectory, motion_state_struct *state )
{
	static long target = 0;
	MBX_post(&pv_newtarget_mbox, &target, 0);
}


void pv_trajectory_reset( velocity_trajectory_struct *trajectory, motion_state_struct *state )
{
	trajectory->end_time = state->time;
	trajectory->target_v = 0;
}


/*****************************************************************/
/*!	 Sets Assisted Mode status flags (Target Reached and Detent Active)
	\param manual_state State of the Assisted Mode state machine
	\param detent Position of the active detent (if any) [inc]
	\param state Cynematic state (position, velocity and time)
	\param reset If reset = 1, timers will be restarted
*/
/*****************************************************************/
static void pv_detent_status_flags(manual_state_t manual_state, long int detent, motion_state_struct *state, short reset)
{
	static long long t_reached_started = 0;
	unsigned long long tmp;

	if( reset ) t_reached_started = 0;

	/* manage Detent Active status flag */
	if (manual_state == DETENTS_ACTIVE)
	{
		Device_status_word |= DETENT_ACTIVE_MASKBIT;		/* set Detent Active flag */
		/* manage Target Reached status flag */
		if (labs(int2ext_pos(state->position - detent, 0)) <= Position_window)
		{
			if(!t_reached_started) t_reached_started = state->time;		/* Start time counting if not started */
			tmp = (long long)(state->time - t_reached_started) * 1000;
			tmp /= lcounts_p_s;
			if(tmp > Position_window_time)
				Device_status_word |= TARGET_REACHED_MASKBIT;			/* set Target Reached flag */
		}
		else
		{
			t_reached_started = 0;			/* stop time counting */
		}
	}
	else
	{
		Device_status_word &= ~DETENT_ACTIVE_MASKBIT;		/* clear Detent Active flag */
		Device_status_word &= ~TARGET_REACHED_MASKBIT;		/* clear Target Reached flag */
	}
}

int set_velocity_control_dataset( void )
{
	int i;
	int set = Velocity_control_dataset[0];
	set=0;

	if( Sensors_configuration_active )
	{
		for(i=0;i<6;i++)
		{
			Velocity_control_parameter_set[i] = Velocity_control_dataset[1+(set*7)+i];
		}
		Velocity_control_margin = (long long)Velocity_control_dataset[1+(set*7)+6];
	}
	return 0;
}

int set_velocity_current_control_dataset( void )
{
	int i;
	int set = Velocity_Current_control_dataset[0];
	set=0;

	if( Sensors_configuration_active )
	{
		for(i=0;i<5;i++)
		{
			Velocity_Current_control_parameter_set[i] = Velocity_Current_control_dataset[1+(set*6)+i];
		}
	}
	return 0;
}

void static control_effort_testbench_manage (void)
{
	const unsigned int ENABLE_TEST_PULSE_CURRENT = 0;
	const unsigned int ENABLE_TEST_SHORT_PULSE_CURRENT = 1;
	const unsigned int ENABLE_TEST_HALF_PULSE_CURRENT = 0;
	static unsigned short int ntimes_vel_loop=FREQ_VELOCITY_LOOP-1;

	if (!ENABLE_TEST_PULSE_CURRENT)
		control_effort = control_effort_vel_mms;
	else
	{
		if (ENABLE_TEST_SHORT_PULSE_CURRENT)
		{
			if (ntimes_vel_loop==FREQ_VELOCITY_LOOP-1)
			{
				control_effort = _IQdiv(_IQ(Control_effort_current_debug),_IQ(100.0));
				ntimes_vel_loop = 0;
			}
			else
			{
				control_effort = _IQ(0.0);
				ntimes_vel_loop++;
			}
		}
		if (ENABLE_TEST_HALF_PULSE_CURRENT)
		{
			if (ntimes_vel_loop<FREQ_VELOCITY_LOOP/2)
			{
				control_effort = _IQdiv(_IQ(Control_effort_current_debug),_IQ(100.0));
				ntimes_vel_loop++;
			}
			else
			{
				control_effort = _IQ(0.0);
				if (ntimes_vel_loop==FREQ_VELOCITY_LOOP-1)
					ntimes_vel_loop=0;
				else
					ntimes_vel_loop++;
			}
		}
	}
}
#if 0
	control_effort = control_effort_vel_mms;
#endif
#if 0
	if (ntimes_vel_loop==FREQ_VELOCITY_LOOP-1)
	{
		control_effort = _IQdiv(_IQ(Control_effort_current_debug),_IQ(100.0));
		ntimes_vel_loop = 0;
		}
		else
		{
		control_effort = _IQ(0.0);
		ntimes_vel_loop++;
		}
#endif
#if 0
	if (ntimes_vel_loop<FREQ_VELOCITY_LOOP/2)
	{
		control_effort = _IQdiv(_IQ(Control_effort_current_debug),_IQ(100.0));
		ntimes_vel_loop++;
	}
	else
	{
		control_effort = _IQ(0.0);
		if (ntimes_vel_loop==FREQ_VELOCITY_LOOP-1){
			ntimes_vel_loop=0;
		}
		else{
		ntimes_vel_loop++;
		}
	}
#endif

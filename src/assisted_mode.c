/*!	\file assisted_mode.c
	\brief Functions for Assisted Mode

\verbatim
*********************************************************************
* File: assisted_mode.c
* Devices: TMS320F28XXX
* Author: Luis Jimenez.
* History:
*   28/04/2009 - original
*********************************************************************
\endverbatim
*/

#include "DSP2833x_Device.h"

#include "amc.h"
#include "fpga.h"
#include "vectorial_current_mode.h"
#include "dc_bldc_current.h"
#include "debug_sci.h"

#define AM_REF_NULL 0
#define AM_REF_POS  1
#define AM_REF_NEG  2

#define iq2dacval(x) (unsigned int)((int)((x)+0x07FF))

int am_config_err = 0;	/* !* used to check an error in configuration */
int am_following_target = 0;	/*!< Indicates wether we are going to a target or just in initial stop trajectory */
PIDREG am_control_pid = PIDREG_DEFAULTS;		/*!< PID struct for assisted mode velocity control */
PIDREG slow_vel_am_control_pid = PIDREG_DEFAULTS;		/*!< PID struct for assisted mode velocity control */
PIDCONTROLLER am_poscontrol_pid = PIDCONTROLLER_DEFAULTS; /*!< PID struct for assisted mode position control */


long int velocity_demand_filtered = 0, velocity_demand_filtered2 = 0, vel_err = 0, slow_vel_err = 0;
int am_current_limit = 0;
long vel_error_normFactor = 1, slow_vel_error_normFactor = 1;
_iq30 error_filter_factor = _IQ30(1.0);
_iq feedforward = 0;
_iq fb_vel = 0;
///////////////
_iq Ki_max = 0, Ki_min = 0;
_iq Ki_factor = 0;
/////////////

static manual_state_t am_state = DETENTS_INACTIVE;	/*!< state of the assisted mode */

//static long int am_detent_pos_loop (motion_state_struct *state, short reset);
_iq am_detent_pos_loop (motion_state_struct *state, short reset);
long int am_trajectory_point(velocity_trajectory_struct *target, long long current_time);

/*****************************************************************/
/*!	 Main function of Profile Assisted Mode (current control implementation)
	\param state Cinematic state of the system
	\param init will be 1 if the mode has to be initialized (0 if not)
*/
/*****************************************************************/
void assisted_mode_operation(motion_state_struct *state, char init )
{
	short reset_detent_timer = 0;
	_iq current_effort = _IQ(0.0);

	if(init) {
		am_initialize(state);		/* initialize the mode if necessary */
		reset_detent_timer = 1;
	}
	else
	{

	Velocity_demand_value = am_detent_pos_loop(state, reset_detent_timer);
	Velocity_demand_value_in_increments = ext2int_vel(Velocity_demand_value);

	Velocity_following_error = Velocity_actual_value - Velocity_demand_value;

	control_effort_vel_mms =  am_velocity_controller_mms(Velocity_demand_value, Velocity_actual_value);

	switch(am_state)
	{
		case DETENTS_INACTIVE:
		case DETENTS_LEAVING:
		default:
				current_effort = _IQsat(_IQ( Assisted_mode_current_demand / 1000.0),_IQ(1.0),_IQ(-1.0));
				break;
			case DETENTS_ACTIVE:
				current_effort = control_effort_vel_mms;
			break;
	}

	control_effort = current_effort;

    #ifdef DEBUG_ASSISTED_LOOP
	TickDebugSci14bytesAssistedMode();
#endif
	}

}

/*****************************************************************/
/*!	Sets the status flags in Assisted Mode
	\param *vel pointer to assisted mode velocity demand in external units
*/
/*****************************************************************/
void am_set_max_vel(long int *vel)
{
	long int max_vel_permited = 0;
	int stop_time = 1;
	if(*vel > 0)
	{
		max_vel_permited = (Software_position_limit_Max_position_limit - Position_actual_value) / stop_time;
		if (*vel > max_vel_permited)
		{
			*vel = max_vel_permited;
		}
		return;
	}
	if(*vel < 0)
	{
		max_vel_permited = (Software_position_limit_Min_position_limit - Position_actual_value) / stop_time;
		if (*vel < max_vel_permited)
		{
			*vel = max_vel_permited;
		}
		return;
	}
	return;
}

/*****************************************************************/
/*!	Function that checks the configuration parameters of the assited mode
	\return 0 if ok, -1 if there is an error
*/
/*****************************************************************/
int am_check_params(void)
{
	int err = 0;

	/* check if analog input is ok */
	if( ( Assisted_mode_conf_Analog_input < 1 ) ||
	    ( Assisted_mode_conf_Analog_input > 4 ) ) {
		err = -1;
		am_config_err = 1;
		_WARNINGmessage(0xffee, 0x20, 0x2120, "Wrong value in OD, index: %x", 0x2120, 0);
	}

	/* gain numerator or divisor can't be zero */
	if( !Assisted_mode_conf_Gain_numerator || !Assisted_mode_conf_Gain_divisor ) {
		err = -1;
		am_config_err = 1;
		_WARNINGmessage(0xffee, 0x20, 0x2120, "Wrong value in OD, index: %x", 0x2120, 0);
	}

	return err;
}


/*****************************************************************/
/*!	 Function that makes necessary initializations when Entering assisted mode
	\param state Cinematic state of the system
*/
/*****************************************************************/
void am_initialize(motion_state_struct *state)
{
	unsigned long ul_tmp = 0;
	_iq tmp = 0;

	/* set new trajectory with current velocity as target */
//	pv_stop_trajectory( Profile_deceleration, &am_trajectory, state );	/* velocity target = 0 */

	/* reset mode specific bits of Statusword */
	Device_status_word &= ~(TARGET_REACHED_MASKBIT | AM_OPERATION_MASKBIT  | AM_MAX_VEL_MASKBIT );
	send_SWord = 1;

	/* initialize assisted mode position PID */
	am_poscontrol_pid.Err1 = 0;
	am_poscontrol_pid.SatError = 0;
	am_poscontrol_pid.Ui = 0;
	am_poscontrol_pid.Ud = 0;
	am_poscontrol_pid.Period = _IQ(0.01);
	am_poscontrol_pid.Frequency = 100;

	ul_tmp = Assisted_mode_pos_Kp;
	ul_tmp = ul_tmp << 16;							/* multiply by 2^16 */
	ul_tmp /= Assisted_mode_pos_control_parameter_Divisor;			/* divide by Divisor */
	am_poscontrol_pid.Kp = ul_tmp << (GLOBAL_Q - 16);		/* convert to IQ and divide by 2^16 */

	ul_tmp = Assisted_mode_pos_Ki;
	ul_tmp = ul_tmp << 16;							/* multiply by 2^16 */
	ul_tmp /= Assisted_mode_pos_control_parameter_Divisor;			/* divide by Divisor */
	am_poscontrol_pid.Ki = ul_tmp << (GLOBAL_Q - 16);		/* convert to IQ and divide by 2^16 */

	ul_tmp = Assisted_mode_pos_Kd;
	ul_tmp = ul_tmp << 16;							/* multiply by 2^16 */
	ul_tmp /= Assisted_mode_pos_control_parameter_Divisor;			/* divide by Divisor */
	am_poscontrol_pid.Kd = ul_tmp << (GLOBAL_Q - 16);		/* convert to IQ and divide by 2^16 */

	ul_tmp = Assisted_mode_pos_Kc;
	ul_tmp = ul_tmp << 16;							/* multiply by 2^16 */
	ul_tmp /= Assisted_mode_pos_control_parameter_Divisor;			/* divide by Divisor */
	am_poscontrol_pid.Kc = ul_tmp << (GLOBAL_Q - 16);		/* convert to IQ and divide by 2^16 */

	ul_tmp = Assisted_mode_pos_Kff;
	ul_tmp = ul_tmp << 16;							/* multiply by 2^16 */
	ul_tmp /= Assisted_mode_pos_control_parameter_Divisor;			/* divide by Divisor */
	am_poscontrol_pid.Kff = ul_tmp << (GLOBAL_Q - 16);		/* convert to IQ and divide by 2^16 */

	/* Init detent state */
	am_state = DETENTS_LEAVING;

	//Velocidad nuevo
	am_control_pid_mms.Err = 0;
	am_control_pid_mms.Up = 0;
	am_control_pid_mms.Ui = 0;
	am_control_pid_mms.Ud = 0;
	am_control_pid_mms.OutPreSat = 0;
	am_control_pid_mms.Out = 0;
	am_control_pid_mms.SatErr = 0;
	am_control_pid_mms.Err1 = 0;
	am_control_pid_mms.OutMax = MAX_DUTY_CYCLE;
	am_control_pid_mms.OutMin = MIN_DUTY_CYCLE;

	am_control_pid_mms.Period = _IQ(0.001);
	am_control_pid_mms.Frequency = 1000;

	init_current_controller_dc_bldc();
	am_set_pid_tunning();

	DINT;
	p_current_limit = &current_limit;	/* set current limit to use */
	EINT;

	ATM_seti(&ready_to_power_on, 1);
}

/*****************************************************************/
/*!	Sets the status flags in Assisted Mode
	\param target Target velocity [velocity units]
	\param v_max Assisted max velocity [velocity units]
*/
/*****************************************************************/
void am_status_flags( long target, unsigned long v_max )
{
	/* set AM_OPERATION_MASKBIT if target != 0 */
	if( target ) Device_status_word |= AM_OPERATION_MASKBIT;	/* AM operation = 1 */
	else Device_status_word &= ~AM_OPERATION_MASKBIT;	/* AM operation = 0 */

	/* set AM_MAX_VEL_MASKBIT if target = +/-limit */
	if( ( target == (long)v_max ) || ( target == -(long)v_max ) )
		Device_status_word |= AM_MAX_VEL_MASKBIT;	/* AM max vel = 1 */
	else
		Device_status_word &= ~AM_MAX_VEL_MASKBIT;	/* AM max vel = 0 */
}

/*****************************************************************/
/*!	 Checks position limits
	\return 1 for stop 0 if not.
*/
/*****************************************************************/
int am_position_limits(void)
{
	long long stop_length;
	long dec;

	/* calculate deceleration */
	dec = minimum(Profile_deceleration, Max_deceleration);

	/* calculate stop length (absolute value) */
	//stop_length = (long long)Velocity_actual_value * Velocity_actual_value;
	stop_length = (long long)(Max_profile_velocity)*(Max_profile_velocity);
	stop_length /= dec;
	stop_length = stop_length / 2;

	/* multiply stop_length by 1,125 for some margin */
	stop_length += (stop_length >> 3);

	/* check limits */
	if((Polarity & 0x40) == ((Polarity & 0x80) >> 1))	/* if position polarity == velocity polarity (most usual) */
	{
		if ((Velocity_actual_value > 0) && (Assisted_mode_current_demand > 0)) {
			if ((Position_actual_value + (long)stop_length) >= Software_position_limit_Max_position_limit) return 1;
			else return 0;
		}
		if ((Velocity_actual_value < 0) && (Assisted_mode_current_demand < 0)) {
			if ((Position_actual_value - (long)stop_length) <= Software_position_limit_Min_position_limit) return 1;
			else return 0;
		}
		return 0;		/* Velocity_actual_value = 0 */
	}

	else
	{
		if ((Velocity_actual_value < 0) && (Assisted_mode_current_demand < 0)) {
			if ((Position_actual_value + (long)stop_length) >= Software_position_limit_Max_position_limit) return 1;
			else return 0;
		}
		if ((Velocity_actual_value > 0) && (Assisted_mode_current_demand > 0)) {
			if ((Position_actual_value - (long)stop_length) <= Software_position_limit_Min_position_limit) return 1;
			else return 0;
		}
		return 0;		/* Velocity_actual_value = 0 */
	}
}

/*****************************************************************/
/*!	Assisted Mode position controller
	\param pos_fb Actual position
	\param pos_dem Demanded position
	\param Sat_output Input for anti-windup calculation
	\param Feedforward input
*/
/*****************************************************************/

_iq am_position_controller( long pos_fb, long pos_dem, _iq Sat_output, _iq Feedforward)
{
	long long tmp;

	if(!pos_error_normFactor) return 0; /* Cannot divide by 0 */

	tmp = (long long)(pos_fb - pos_dem) * Position_factor_Feed_constant;
	tmp = tmp << 5;	/* multiply by 32 to avoid resolution losses */
	tmp /= Position_factor_Numerator;					/* tmp = 32 * error [position units] */
	if(llabs(tmp) > (long long)(pos_error_normFactor<<5))
		tmp = sign(tmp) * (long long)(pos_error_normFactor<<5); /* limit the error to 'Position_control_margin' */

	/* reference position is always '0' and current position is set as the relative error */
	am_poscontrol_pid.Ref = _IQ(0.0);

	/* Divide by normFactor and later convert to IQ and divide by 2^5 */
	am_poscontrol_pid.Fdb = (tmp << (GLOBAL_Q - 5)) / pos_error_normFactor ;	/* position error related to 'Position_control_margin' */

	/* Set the anti-windup and feed-forward */
	am_poscontrol_pid.AntiWindup = _IQsat(Sat_output,_IQ(1.0),_IQ(-1.0));
	am_poscontrol_pid.FeedForward = _IQsat(Feedforward,_IQ(1.0),_IQ(-1.0));

	am_poscontrol_pid.calc(&am_poscontrol_pid);		/* calculate PID output */

	//Debug_word_1 = am_poscontrol_pid.Fdb >> (GLOBAL_Q-10); /* Show in IQ10 [-1024, 1024]*/
	//Debug_word_2 = am_poscontrol_pid.Err >> (GLOBAL_Q-10); /* Show in IQ10 [-1024, 1024]*/

	return am_poscontrol_pid.Out;
}

/*****************************************************************/
/*!	 Sets Assisted Mode status flags (Target Reached and Detent Active)
	\param manual_state State of the Assisted Mode state machine
	\param detent Position of the active detent (if any) [inc]
	\param state Cynematic state (position, velocity and time)
	\param reset If reset = 1, timers will be restarted
*/
/*****************************************************************/
void am_detent_status_flags(manual_state_t manual_state, long int detent, motion_state_struct *state, short reset)
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

static long int am_detent_pos_loop (motion_state_struct *state, short reset_detent_timer)
//_iq am_detent_pos_loop (motion_state_struct *state, short reset_detent_timer)
{
	_iq position_effort = 0, pos_velocity_max = 0;//, actual_effort = 0;
	static long int detent_position = 0;
	static _iq last_dem = 0;
	long norm_factor = 0;
	long int velocity_demand =0;
	long int detent_velocity_demand=0;
	static int flag_detent = 0;
	static long int last_detent = 0;

	/* Use Max_profile_velocity to normalize the position PID */
	norm_factor = ext2int_vel(Max_profile_velocity);

	velocity_demand = am_trajectory_point(&am_trajectory, state->time);
	velocity_demand = int2ext_vel(velocity_demand);			/* [velocity units] */

	/* Detent position calculator */
	switch(am_state)
	{
		case DETENTS_INACTIVE:
			Debug_long5 = 0;
			/* Reset detent leave */
			Detents_config_Leave = 0;
			/* Debug position control loop */
/*
			if(Velocity_mode_dentent_active)
			{
				am_state = DETENTS_ACTIVE;
				detent_position = ext2int_pos(Velocity_mode_debug_position,Home_offset); // Unit conversion of target position
				break;
			}
*/

			if (detent_nearby(int2ext_pos(state->position, Home_offset), &Detents_config_Active_detent) &&
			   (int2ext_vel(labs(state->velocity)) <= Detents_config_Max_velocity))// && (flag_detent == 0))
			{
				am_state = DETENTS_ACTIVE;
				detent_position = ext2int_pos(Detents_config_Active_detent, Home_offset);	/* converto to internal units */

				reset_detent_timer = 1;

				/* update Position_demand_value and Position_demand_value_in_increments */
				Position_demand_value_in_increments = detent_position;	/* [inc] */
				Position_demand_value = Detents_config_Active_detent;		/* [position units] */
				//break;
			}
			break;
		case DETENTS_ACTIVE:
			Debug_long5 = 1000;
			/* Maximum velocity to achieve position target */
			pos_velocity_max = (((long long)ext2int_vel(labs(Max_detent_velocity))) << GLOBAL_Q) / norm_factor;
			/* Position controller */
			position_effort = am_position_controller(state->position, detent_position, last_dem, 0);
			/* Saturate the output with the max. velocity */
			position_effort = _IQsat(position_effort,pos_velocity_max,-pos_velocity_max);
			/* Convert from IQ to velocity external units */
			detent_velocity_demand = int2ext_vel(_IQmpyI32int(position_effort, norm_factor));

			/* Set velocity demand to velocity controller */
			velocity_demand = detent_velocity_demand;

			/* Leave the detent when active */
			if (Detents_config_Leave)
			{
				am_state = DETENTS_LEAVING;
			}
			else if (int2ext_pos(labs(state->position - detent_position), 0) > Detents_config_Skip_distance)
			{
				/* Became inactive if position is too far from detent point */
				am_state = DETENTS_INACTIVE;
				//break;
			}
			break;
		case DETENTS_LEAVING:
			Debug_long5 = 2000;
			if (!detent_nearby(int2ext_pos(state->position, Home_offset), &detent_position))
				am_state = DETENTS_INACTIVE;
			break;
		default:		/* should never happen */
			Debug_long5 = 3000;
			am_state = DETENTS_INACTIVE;
			break;
	}
	/* Check and set the status flags */
	am_detent_status_flags(am_state, detent_position, state, reset_detent_timer);
	return velocity_demand;
	//return position_effort;
}

/*****************************************************************/
/*!	 Calculates the velocity demand for current control cycle
	\param target Velocity trajectory being followed
	\param current_time Current time
	\return Velocity demand
 */
/*****************************************************************/
long int am_trajectory_point(velocity_trajectory_struct *target, long long current_time)
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

static int am_set_pid_tunning( void )
{
	unsigned long ul_tmp;

	ul_tmp = Velocity_control_parameter_set[0];
	ul_tmp = ul_tmp << 16;							/* multiply by 2^16 */
	ul_tmp /= Velocity_control_parameter_set[5];			/* divide by Divisor */
	am_control_pid_mms.Kp = ul_tmp << (GLOBAL_Q - 16);		/* convert to IQ and divide by 2^16 */

	ul_tmp = Velocity_control_parameter_set[2];
	ul_tmp = ul_tmp << 16;						/* multiply by 2^16 */
	ul_tmp /= Velocity_control_parameter_set[5];			/* divide by Divisor */
	am_control_pid_mms.Ki 		= ul_tmp << (GLOBAL_Q - 16);		/* convert to IQ and divide by 2^16 */

	ul_tmp = Velocity_control_parameter_set[3];
	ul_tmp = ul_tmp << 16;						/* multiply by 2^16 */
	ul_tmp /= Velocity_control_parameter_set[5];			/* divide by Divisor */
	am_control_pid_mms.Kd = ul_tmp << (GLOBAL_Q - 16);		/* convert to IQ and divide by 2^16 */

	ul_tmp = Velocity_control_parameter_set[4];
	ul_tmp = ul_tmp << 16;						/* multiply by 2^16 */
	ul_tmp /= Velocity_control_parameter_set[5];			/* divide by Divisor */
	am_control_pid_mms.Kc = ul_tmp << (GLOBAL_Q - 16);		/* convert to IQ and divide by 2^16 */

	set_Kp_current_control_dc_bldc(Velocity_Current_control_parameter_set[0],
								   Velocity_Current_control_parameter_set[4]);

	set_Ki_current_control_dc_bldc(Velocity_Current_control_parameter_set[1],
								   Velocity_Current_control_parameter_set[4]);

	set_Kd_current_control_dc_bldc(Velocity_Current_control_parameter_set[2],
								   Velocity_Current_control_parameter_set[4]);

	set_Kc_current_control_dc_bldc(Velocity_Current_control_parameter_set[3],
								   Velocity_Current_control_parameter_set[4]);

	return 0;
}

/*****************************************************************/
/*!	Calculates the PID output in Profile Velocity Mode
	\param demand Velocity demand
	\param velocity Current velocity
	\return PID output [-1,1]
 */
/*****************************************************************/
_iq am_velocity_controller_mms(long int demand, long int velocity)
{

	am_control_pid_mms.Ref = _IQsat(_IQdiv(demand,Max_demand_velocity),_IQ(1.0),_IQ(-1.0));
	am_control_pid_mms.Fdb = _IQdiv(velocity,Max_demand_velocity);//_IQsat(_IQdiv(velocity,Max_demand_velocity),_IQ(1.0),_IQ(-1.0));//
	am_control_pid_mms.calc(&am_control_pid_mms);		/* calculate PID output */

	return am_control_pid_mms.Out;
}

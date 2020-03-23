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

/*****************************************************************/
/*!	 Main function of Profile Assisted Mode (current control implementation)
	\param state Cinematic state of the system
	\param init will be 1 if the mode has to be initialized (0 if not)
*/
/*****************************************************************/
void assisted_mode_operation(motion_state_struct *state, char init )
{
	long int velocity_demand = 0, detent_velocity_demand = 0, acc_factor = 0, tmp = 0;
	long norm_factor = 0;
	_iq fb_slow_vel = 0; // vel = 0, dem = 0;
	_iq position_effort = 0, pos_velocity_max = 0;//, actual_effort = 0;
	_iq feedfw_aux = 0;
	static _iq last_dem = 0;
	static long int detent_position = 0;
	//static unsigned short i = 0, limit_error = 0;
	long int actual_vel_err = 0;
	short reset_detent_timer = 0;
	//_iq vel_ref = 0, Ki_actual = 0;

	if(init) {
		am_initialize(state);		/* initialize the mode if necessary */
		reset_detent_timer = 1;
	}

	if( Device_control_word & HALT_MASKBIT ) {
			velocity_demand = 0;
			am_control_pid.OutMax = _IQ( 1.0);
			am_control_pid.OutMin = _IQ(-1.0);
	}
	else
	{
		if( Assisted_mode_current_demand > 0 ) {
			velocity_demand = + (int)Assisted_mode_conf_Max_velocity;
			am_control_pid.OutMax = _IQsat(_IQ( Assisted_mode_current_demand / 1000.0),_IQ(1.0),_IQ(-1.0));
			am_control_pid.OutMin = _IQsat(_IQ( -Assisted_mode_current_demand / 1000.0),_IQ(1.0),_IQ(-1.0));
		}
		else if(Assisted_mode_current_demand < 0) {
			velocity_demand = - (int)Assisted_mode_conf_Max_velocity;
			am_control_pid.OutMax = _IQsat(_IQ( -Assisted_mode_current_demand / 1000.0),_IQ(1.0),_IQ(-1.0));
			am_control_pid.OutMin = _IQsat(_IQ( Assisted_mode_current_demand / 1000.0),_IQ(1.0),_IQ(-1.0));
		}
	}
	/*
	if( last_velocity_demand != velocity_demand )
	{
		pv_newVelocityTarget( (velocity_demand/10), &am_trajectory, state );
	}
	last_velocity_demand = velocity_demand;
	am_velocity_demand = pv_trajectory_point( &am_trajectory, state->time );

	// Set velocity limit if we are going out of the limits
	if (am_position_limits() || limit_error)
	{
		if(velocity_demand > 0)	limit_error = 2;
		if(velocity_demand < 0)	limit_error = 3;
		velocity_demand = am_velocity_demand;
	}
	if(limit_error == 2 && velocity_demand < 0) limit_error = 0;
	if(limit_error == 3 && velocity_demand > 0) limit_error = 0;
	*/

	/* Calculate the velocity normalization factor */
	/* Use Max_profile_velocity to normalize the position PID */
	norm_factor = ext2int_vel(Max_profile_velocity);

	/* Detent position calculator */
	switch(am_state)
	{
		case DETENTS_INACTIVE:
			/* Reset detent leave */
			Detents_config_Leave = 0;
			/* Debug position control loop */
			if(Assisted_mode_dentent_active)
			{
				am_state = DETENTS_ACTIVE;
				detent_position = ext2int_pos(Assisted_mode_debug_position,Home_offset); // Unit conversion of target position
				break;
			}

			if (detent_nearby(int2ext_pos(state->position, Home_offset), &Detents_config_Active_detent) &&
			   (int2ext_vel(labs(state->velocity)) <= Detents_config_Max_velocity))
			{
				am_state = DETENTS_ACTIVE;
				detent_position = ext2int_pos(Detents_config_Active_detent, Home_offset);	/* converto to internal units */

				reset_detent_timer = 1;

				/* update Position_demand_value and Position_demand_value_in_increments */
				Position_demand_value_in_increments = detent_position;	/* [inc] */
				Position_demand_value = Detents_config_Active_detent;		/* [position units] */
				break;
			}
			break;
		case DETENTS_ACTIVE:
			/* am_position_controller */
			/* Not apply external current saturation */
			am_control_pid.OutMax = _IQ( 1.0);
			am_control_pid.OutMin = _IQ(-1.0);
			slow_vel_am_control_pid.OutMax = _IQsat(_IQ( Slow_vel_saturation / 1000.0),_IQ(1.0),_IQ(-1.0));
			slow_vel_am_control_pid.OutMin = _IQsat(-_IQ( Slow_vel_saturation / 1000.0),_IQ(1.0),_IQ(-1.0));
			/* Maximum velocity to achieve position target */
			pos_velocity_max = (((long long)ext2int_vel(labs(Assisted_mode_debug_velocity))) << GLOBAL_Q) / norm_factor;
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
				am_state = DETENTS_LEAVING;
			/* Became inactive if position is too far from detent point */
			if (int2ext_pos(labs(state->position - detent_position), 0) > Detents_config_Skip_distance)
			{
				am_state = DETENTS_INACTIVE;
			}
			break;
		case DETENTS_LEAVING:
			if (!detent_nearby(int2ext_pos(state->position, Home_offset), &detent_position))
				am_state = DETENTS_INACTIVE;
			break;
		default:		/* should never happen */
			am_state = DETENTS_INACTIVE;
			break;
	}
	/* Check and set the status flags */
	am_detent_status_flags(am_state, detent_position, state, reset_detent_timer);

	/* Set velocity limit if we are going out of the limits */
	am_set_max_vel(&velocity_demand);

	/* Velocity Demand Filtering */
	switch(Assisted_mode_conf_filter)
	{
		case 0: // No filter
			velocity_demand_filtered = velocity_demand;
			break;
		case 1: // 1er order 10ms
			velocity_demand_filtered = _IQ30mpy(velocity_demand,_IQ30(0.333)) + _IQ30mpy(velocity_demand_filtered,_IQ30(0.667));
			break;
		case 2: // 1er order 20ms
			velocity_demand_filtered = _IQ30mpy(velocity_demand,_IQ30(0.2)) + _IQ30mpy(velocity_demand_filtered,_IQ30(0.8));
			break;
		case 3: // 1er order 40ms
			velocity_demand_filtered = _IQ30mpy(velocity_demand,_IQ30(0.111)) + _IQ30mpy(velocity_demand_filtered,_IQ30(0.889));
			break;
		case 4: // 2o order 10ms
			velocity_demand_filtered2 = _IQ30mpy(velocity_demand,_IQ30(0.333)) + _IQ30mpy(velocity_demand_filtered2,_IQ30(0.667));
			velocity_demand_filtered = _IQ30mpy(velocity_demand_filtered2,_IQ30(0.333)) + _IQ30mpy(velocity_demand_filtered,_IQ30(0.667));
			break;
		case 5: // 2o order 20ms
			velocity_demand_filtered2 = _IQ30mpy(velocity_demand,_IQ30(0.2)) + _IQ30mpy(velocity_demand_filtered2,_IQ30(0.8));
			velocity_demand_filtered = _IQ30mpy(velocity_demand_filtered2,_IQ30(0.2)) + _IQ30mpy(velocity_demand_filtered,_IQ30(0.8));
			break;
		case 6: // 2o order 40ms
			velocity_demand_filtered2 = _IQ30mpy(velocity_demand,_IQ30(0.111)) + _IQ30mpy(velocity_demand_filtered2,_IQ30(0.889));
			velocity_demand_filtered = _IQ30mpy(velocity_demand_filtered2,_IQ30(0.111)) + _IQ30mpy(velocity_demand_filtered,_IQ30(0.889));
			break;
		case 7: // Ramp
			if ((velocity_demand_filtered - velocity_demand) < 0)
			{
				if ((velocity_demand - velocity_demand_filtered) > Assisted_mode_conf_ramp)
				{
					velocity_demand_filtered += Assisted_mode_conf_ramp;
				}
				else
				{
					velocity_demand_filtered = velocity_demand;
				}
			}
			if ((velocity_demand_filtered - velocity_demand) > 0)
			{
				if ((velocity_demand_filtered - velocity_demand) > Assisted_mode_conf_ramp)
				{
					velocity_demand_filtered -= Assisted_mode_conf_ramp;
				}
				else
				{
					velocity_demand_filtered = velocity_demand;
				}
			}
			break;
		default:
			velocity_demand_filtered = velocity_demand;
			break;
	}

	/* Save velocity to assert position antiwindup */
	last_dem = (((long long)ext2int_vel(velocity_demand_filtered)) << GLOBAL_Q) / norm_factor;

	/* Check if accelerating to set current limit for BLDC motor*/
	acc_factor = _IQmpyI32int(_IQ(0.9), ext2int_vel(velocity_demand));
	am_current_limit = (labs(acc_factor) > labs(state->velocity))? current_limit_acc : current_limit;

	/* AM_Velocity_controller */
	/* Convert velocity demand to internal units */


	tmp = ext2int_vel(velocity_demand_filtered);

	/* Calculate the error in internal units */
	actual_vel_err = state->velocity - tmp;
	/* Filtering error if active */
#if 0
	if(Assisted_mode_error_filter)
		vel_err = _IQ30mpy(actual_vel_err,error_filter_factor) + _IQ30mpy(vel_err,(1-error_filter_factor)); //fc=10Hz
	else
		vel_err = actual_vel_err;
#else
	vel_err = actual_vel_err;
#endif

#if 0
	/* Limit the maximum error to 'Velocity_control_margin' */
	if(labs(vel_err) > labs(vel_error_normFactor))
		vel_err = sign(vel_err) * vel_error_normFactor;
	if(labs(actual_vel_err) > labs(slow_vel_error_normFactor))
		slow_vel_err = sign(actual_vel_err) * slow_vel_error_normFactor;
	else
		slow_vel_err = actual_vel_err;
#endif
	/* Convert to IQ and Divide by normFactor*/
	fb_vel =(((long long)vel_err) << GLOBAL_Q) / vel_error_normFactor;


	am_control_pid.Ref = 0;
	am_control_pid.Fdb = fb_vel;
	/* calculate PID output */
	am_control_pid.calc(&am_control_pid);
	/* estimate feed forward if configured */
	if (velocity_demand > 25)
	{
		feedfw_aux = feedforward;
	}
	if (velocity_demand < -25)
	{
		feedfw_aux = -feedforward;
	}

	control_effort  =  _IQsat(am_control_pid.Out,_IQ(1.0),_IQ(-1.0)); //complete functionality



	/* Debug part */

	//Debug_long1 = velocity_demand_filtered;
	//Debug_long2 = int2ext_vel(state->velocity);
	//Debug_long3 = int2ext_pos(detent_position, Home_offset);
	//Debug_long4 = int2ext_pos(state->position, Home_offset);

	//Debug_word_1 = am_control_pid.Err >> (GLOBAL_Q-10);
	//Debug_word_2 = am_control_pid.Out >> (GLOBAL_Q-10);
	//Debug_word_3 = slow_vel_am_control_pid.Out >> (GLOBAL_Q-10);
	//Debug_word_4 = Iq >> Q_CURRENT;
	//Debug_word_5 = Id >> Q_CURRENT;
	//Debug_word_6 = int2ext_vel(state->velocity);

	#ifdef VECT_CONTROL_DEBUG
	//if(index_vect_control_debug < VECT_CONTROL_BUFFER_LEN)
	//{
		//if( i > 2 )
		//{
			//i = 0;
			DINT;
			/* Variables for debug close loop */
			vect_control_buffer1[index_vect_control_debug] = Debug_long1;//velocity_demand_filtered;//int2ext_pos(Debug_long1, Home_offset);
			vect_control_buffer2[index_vect_control_debug] = Debug_long2;//int2ext_vel(state->velocity);//int2ext_pos(Debug_long2, Home_offset);
			vect_control_buffer3[index_vect_control_debug] = Debug_word_1; //fb_vel >> (GLOBAL_Q-10);
			vect_control_buffer4[index_vect_control_debug] = Debug_word_2; //control_effort >> (GLOBAL_Q-10);
			vect_control_buffer5[index_vect_control_debug] = Debug_word_3; //Assisted_mode_current_demand;
			vect_control_buffer6[index_vect_control_debug] = Debug_word_4; //
			index_vect_control_debug++;
			index_vect_control_debug = (index_vect_control_debug==VECT_CONTROL_BUFFER_LEN?0:index_vect_control_debug);
			EINT;
		//}
		//else
		//{
		//	i++;
		//}
	//}
	#endif
//			TickDebugSci8bytesPositionMode();

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
/*!	 Main function of Profile Assisted Mode
	\param state Cinematic state of the system
	\param init will be 1 if the mode has to be initialized (0 if not)
*/
/*****************************************************************/
void assisted_mode_operation_old(motion_state_struct *state, char init )
{
	long int velocity_demand;		/* Will be maped in the OD in a Manufacturer specific Entry */
	long int v_target;
	int diff;
	static int am_state = AM_REF_NULL, am_next_state = AM_REF_NULL;
	static long long start_time = 0;
	long long tmp;
	long v_max_limit, v_min_limit;

	if(init) {
		am_initialize(state);		/* initialize the mode if necessary */
		am_following_target = 0;
		am_state = AM_REF_NULL;
		am_next_state = AM_REF_NULL;
		start_time = 0;
	}

	if( Device_control_word & HALT_MASKBIT ) {
		if( am_trajectory.target_v != 0 ) {
			pv_newVelocityTarget( 0, &am_trajectory, state );	/* velocity target = 0 */
		}
	} else {
		/* get difference from center */
		switch ( Assisted_mode_conf_Analog_input ) {
			case 1: diff = (int)AnalogIn1_Value - Assisted_mode_conf_Center; break;
			case 2: diff = (int)AnalogIn2_Value - Assisted_mode_conf_Center; break;
			case 3: diff = (int)AnalogIn3_Value - Assisted_mode_conf_Center; break;
			case 4: diff = (int)AnalogIn4_Value - Assisted_mode_conf_Center; break;
		}
		if( !am_config_err && !am_check_params() ) {
			switch (am_state) {
				case AM_REF_NULL:
					if( abs(diff) <= Assisted_mode_conf_Threshold ) {
						start_time = 0;
						am_next_state = AM_REF_NULL;
					} else {
						if( diff > 0 ) { /* change to AM_REF_POS if diff>0 more than window time */
							if( am_next_state != AM_REF_POS ) {
								am_next_state = AM_REF_POS;
								start_time = state->time;
							} else {
								tmp = state->time - start_time;
								tmp = (tmp * 1000) / lcounts_p_s;
								if ( tmp > Assisted_mode_conf_Window_time ) {
									am_state = AM_REF_POS;
									start_time = 0;
								}
							}
						} else {		/* change to AM_REF_NEG if diff<0 more than window time */
							if( am_next_state != AM_REF_NEG ) {
								am_next_state = AM_REF_NEG;
								start_time = state->time;
							} else {
								tmp = state->time - start_time;
								tmp = (tmp * 1000) / lcounts_p_s;
								if ( tmp >Assisted_mode_conf_Window_time ) {
									am_state = AM_REF_NEG;
									start_time = 0;
								}
							}
						}
					}
					break;
				case AM_REF_POS:
					if( diff < Assisted_mode_conf_Threshold ) { /* change to AM_REF_NULL if diff<threshold more than window time */
						if( am_next_state != AM_REF_NULL ) {
							am_next_state = AM_REF_NULL;
							start_time = state->time;
						} else {
							tmp = state->time - start_time;
							tmp = (tmp * 1000) / lcounts_p_s;
							if ( tmp > Assisted_mode_conf_Window_time ) {
								am_state = AM_REF_NULL;
								start_time = 0;
							}
						}
					} else {
						start_time = 0;
						am_next_state = AM_REF_POS;
					}
					break;
				case AM_REF_NEG:
					if( diff > -(int)Assisted_mode_conf_Threshold ) { /* change to AM_REF_NULL if diff>-threshold more than window time */
						if( am_next_state != AM_REF_NULL ) {
							am_next_state = AM_REF_NULL;
							start_time = state->time;
						} else {
							tmp = state->time - start_time;
							tmp = (tmp * 1000) / lcounts_p_s;
							if ( tmp > Assisted_mode_conf_Window_time ) {
								am_state = AM_REF_NULL;
								start_time = 0;
							}
						}
					} else {
						start_time = 0;
						am_next_state = AM_REF_NEG;
					}
					break;
				default:
					am_state = AM_REF_NULL;
					am_next_state = AM_REF_NULL;
					break;
			}

			if( am_position_limits_old( Velocity_actual_value, Velocity_demand_value, Position_actual_value, &v_max_limit, &v_min_limit ) ) {
				am_state = AM_REF_NULL;
				am_next_state = AM_REF_NULL;
			}

			if( am_state == AM_REF_NULL ) {
				v_target = 0;
			} else {
				v_target = (long)diff * Assisted_mode_conf_Gain_numerator;
				v_target /= Assisted_mode_conf_Gain_divisor;
				if( v_target > v_max_limit ) v_target = v_max_limit;
				if( v_target < v_min_limit ) v_target = v_min_limit;
			}
			pv_newVelocityTarget( v_target, &am_trajectory, state );
			am_following_target = 1;
			Target_velocity = v_target;	/* update target velocity */
		}
	}

	velocity_demand = pv_trajectory_point( &am_trajectory, state->time );	/* Apply the trajectory calculated */

	Velocity_demand_value_in_increments = velocity_demand;		/* [inc/s] */
	Velocity_demand_value = int2ext_vel(velocity_demand);			/* [velocity units] */

	/* update velocity error */
	Velocity_following_error = Velocity_actual_value - Velocity_demand_value;

	control_effort = pv_velocity_controller( velocity_demand, state->velocity );

	/* set StatusWord mode specific flags */
	am_status_flags( Target_velocity, Assisted_mode_conf_Max_velocity );
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

	/* reset error check */
	am_config_err = 0;

	/* Reset integral current error BLAC current control*/
	Ui_d=0;
	SatErr_d=0;
	Ui_q=0;
	SatErr_q=0;

	/* Init detent state */
	am_state = DETENTS_INACTIVE;
#if 0
	/* Init BLDC current control*/
	current_control_pid.Err = 0;
	current_control_pid.Ui = 0;
	current_control_pid.Ud = 0;
	current_control_pid.Kp = _IQ15toIQ(Kp);
	current_control_pid.Ki = _IQ15toIQ(Ki);
	current_control_pid.Kd = _IQ(0.0);
	current_control_pid.Kc = _IQ15toIQ(Kc);
	current_control_pid.Period = _IQ(1.0); //_IQ(0.00005882353);
	current_control_pid.Frequency = 17000;
	current_control_pid.OutMax = _IQ( 1.0);
	current_control_pid.OutMin = _IQ(-1.0);
#endif
	init_current_controller_dc_bldc_from_assited_mode();
	/* Initialize velocity and error filters */
	velocity_demand_filtered = 0;
	velocity_demand_filtered2 = 0;
	vel_err = 0;
	if(Assisted_mode_error_filter)
	{
		tmp = _IQmpyI32(_IQ(0.00628318), Assisted_mode_error_filter); // (2pi*fsample)*fc
		tmp = _IQdiv(tmp, (_IQ(1.0) + tmp));
		error_filter_factor = _IQtoIQ30(tmp);
	}
	else
	{
		error_filter_factor = _IQ30(1.0);
	}
	/* Set normalization factor to velocity PID */
	vel_error_normFactor = ext2int_vel(Assisted_mode_control_margin);
	slow_vel_error_normFactor = ext2int_vel(Slow_vel_control_margin);

	/* initialize PID */
	feedforward = _IQsat(_IQ( Slow_vel_feedforward / 1000.0),_IQ(1.0),_IQ(-1.0));

	am_control_pid.Err = 0;
	am_control_pid.Up = 0;
	am_control_pid.Ui = 0;
	am_control_pid.Ud = 0;
	am_control_pid.OutPreSat = 0;
	am_control_pid.Out = 0;
	am_control_pid.SatErr = 0;
	am_control_pid.Err1 = 0;
	am_control_pid.Kd = 0;
	am_control_pid.OutMax = _IQ( 1.0);
	am_control_pid.OutMin = _IQ(-1.0);
	am_control_pid.Period = _IQ(0.001);
	am_control_pid.Frequency = 1000;

	ul_tmp = Assisted_mode_conf_Kp;
	ul_tmp = ul_tmp << 16;							/* multiply by 2^16 */
	ul_tmp /= Assisted_mode_conf_control_parameter_Divisor;			/* divide by Divisor */
	am_control_pid.Kp = ul_tmp << (GLOBAL_Q - 16);		/* convert to IQ and divide by 2^16 */

	ul_tmp = Assisted_mode_conf_Ki;
	ul_tmp = ul_tmp << 16;						/* multiply by 2^16 */
	ul_tmp /= Assisted_mode_conf_control_parameter_Divisor;			/* divide by Divisor */
	am_control_pid.Ki = _IQmpyI32(_IQ(0.0000152587890625),ul_tmp);	/* divide by 2^16 */

	/* Dynamic integral part not activated
	Ki_max = _IQmpyI32(_IQ(0.0000152587890625),ul_tmp);
	ul_tmp = High_vel_Ki_min;
	ul_tmp = ul_tmp << 16;
	ul_tmp /= Assisted_mode_conf_control_parameter_Divisor;
	Ki_min = _IQmpyI32(_IQ(0.0000152587890625),ul_tmp);
	Ki_factor = Ki_min - Ki_max;
	*/

	ul_tmp = Assisted_mode_conf_Kc;
	ul_tmp = ul_tmp << 16;						/* multiply by 2^16 */
	ul_tmp /= Assisted_mode_conf_control_parameter_Divisor;			/* divide by Divisor */
	am_control_pid.Kc = _IQmpyI32(_IQ(0.0000152587890625),ul_tmp);	/* divide by 2^16 */

	slow_vel_am_control_pid.Err = 0;
	slow_vel_am_control_pid.Up = 0;
	slow_vel_am_control_pid.Ui = 0;
	slow_vel_am_control_pid.Ud = 0;
	slow_vel_am_control_pid.OutPreSat = 0;
	slow_vel_am_control_pid.Out = 0;
	slow_vel_am_control_pid.SatErr = 0;
	slow_vel_am_control_pid.Err1 = 0;
	slow_vel_am_control_pid.Kd = 0;
	//slow_vel_am_control_pid.OutMax = _IQ( 1.0);
	//slow_vel_am_control_pid.OutMin = _IQ(-1.0);
	slow_vel_am_control_pid.Period = _IQ(0.001);
	slow_vel_am_control_pid.Frequency = 1000;

	ul_tmp = Slow_vel_Kp;
	ul_tmp = ul_tmp << 16;							/* multiply by 2^16 */
	ul_tmp /= Slow_vel_control_parameter_Divisor;			/* divide by Divisor */
	slow_vel_am_control_pid.Kp = ul_tmp << (GLOBAL_Q - 16);		/* convert to IQ and divide by 2^16 */

	ul_tmp = Slow_vel_Ki;
	ul_tmp = ul_tmp << 16;						/* multiply by 2^16 */
	ul_tmp /= Slow_vel_control_parameter_Divisor;			/* divide by Divisor */
	slow_vel_am_control_pid.Ki = _IQmpyI32(_IQ(0.0000152587890625),ul_tmp);	/* divide by 2^16 */

	ul_tmp = Slow_vel_Kc;
	ul_tmp = ul_tmp << 16;						/* multiply by 2^16 */
	ul_tmp /= Slow_vel_control_parameter_Divisor;			/* divide by Divisor */
	slow_vel_am_control_pid.Kc = _IQmpyI32(_IQ(0.0000152587890625),ul_tmp);	/* divide by 2^16 */

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
	stop_length = (long long)Velocity_actual_value * Velocity_actual_value;
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
	} else
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
/*!	 Checks if the current speed (or the demand speed) would make the drive go further
  position limits and sets clamping values for velocity target
	\param vel current velocity [external units]
	\param vdemand velocity demand [external units]
	\param pos current position [external units]
	\param v_limit_max pointer to the max clamping value for target velocity [external units]
	\param v_limit_min pointer to rhe min clamping value for target velocity [external units]
	\return 0 if drive can stop between limits, 1 if not.
*/
/*****************************************************************/
int am_position_limits_old( long vel, long vdemand, long pos, long *v_limit_max, long *v_limit_min )
{
	long long stop_length;
	long dec;
	long vmax, vmin, vabsmax;

	/* get maximum and minimum between current and demand speed */
	vmax = maximum( vel, vdemand );
	vmin = minimum( vel, vdemand );
	vabsmax = maximum ( abs( vel ), abs( vdemand ) );

	/* calculate deceleration */
	dec = minimum(Profile_deceleration, Max_deceleration);

	/* calculate stop length (absolute value) */
	stop_length = (long long)vabsmax * vabsmax;
	stop_length /= dec;
	stop_length = stop_length / 2;

	/* multiply stop_length by 1.125 for some margin */
	stop_length += (stop_length >> 3);

	/* preload limits with defaul values */
	*v_limit_max = Assisted_mode_conf_Max_velocity;
	*v_limit_min = -Assisted_mode_conf_Max_velocity;

	/* check limits */
	if((Polarity & 0x40) == ((Polarity & 0x80) >> 1))	/* if position polarity == velocity polarity (most usual) */
	{
		if ( vmax > 0 ) {
			if ((pos + (long)stop_length) >= Software_position_limit_Max_position_limit) {
				*v_limit_max = 0;
				return 1;
			}
			else return 0;
		}
		if ( vmin < 0 ) {
			if ((pos - (long)stop_length) <= Software_position_limit_Min_position_limit) {
				*v_limit_min = 0;
				return 1;
			}
			else return 0;
		}
		return 0;
	} else
	{
		if ( vmax > 0 ) {
			if ((pos - (long)stop_length) <= Software_position_limit_Min_position_limit) {
				*v_limit_max = 0;
				return 1;
			}
			else return 0;
		}
		if ( vmin < 0 ) {
			if ((pos + (long)stop_length) >= Software_position_limit_Max_position_limit) {
				*v_limit_min = 0;
				return 1;
			}
			else return 0;
		}
		return 0;
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

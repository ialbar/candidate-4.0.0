/*!	\file manual_mode.c
	\brief Functions for Manual Mode (with software detents)
*/

#include "amc.h"
#include "debug_sci.h"
#include "manual_mode.h"



manual_state_t manual_state = DETENTS_INACTIVE;	/*!< state of the manual mode */
static long current_detent_pos;			/*!< position of the detent active [inc] */
static _iq duty_limit = 0;		/*!< Duty cycle limit used in detents */
static long max_travel;


#define FREQ_MANUAL_POSITION_LOOP 10// 10*1ms

PIDREG manual_vel_control_pid = PIDREG_DEFAULTS;	/*!< PID struct for manual velocity control */
PIDREG manual_pos_control_pid = PIDREG_DEFAULTS;
_iq control_effort_manual_profile		=_IQ(0.0);
_iq control_effort_manual_velocity	=_IQ(0.0);
_iq control_effort_manual_position = _IQ(0.0);

long int Position_demand_value_trajectory=0;

static _iq manual_controller(long int demand, long int position, long int velocity);
static _iq manual_position_controller(long int demand, long int position);
static _iq manual_velocity_controller(_iq demand, long int velocity);
static void init_pm_position_pid(void);
static void init_pm_velocity_pid(void);
static void init_pm_current_pid(void);

unsigned int state_machine_manual_mode_dbg=0;
unsigned int manual_mode_trans_dbg=0;
unsigned int target_reach_dbg=0;

long int position_last_detent=0;
unsigned int avoid_last_detent=0;
unsigned int move_away_output_distance=0;




static unsigned int test_position_last_detent (void)
{
	if (move_away_output_distance)
	{
		if (labs(Position_actual_value - position_last_detent) < Detents_config_Max_output_distance)
		{
			avoid_last_detent 	= 1;
		}
		else
		{
			move_away_output_distance   = 0;
			avoid_last_detent 			= 0;
		}
	}
	return avoid_last_detent;
}

/*****************************************************************/
/*!	 Main function of Manual Mode
	\param state Cinematic state of the system
	\param init will be 1 if the mode has to be initialized (0 if not)
*/
/*****************************************************************/
void manual_mode_operation(motion_state_struct *state, char init)
{
	int reset_timers = 0;
	long int Position_demand_from_trajectory;
	if(init)
	{
		// if last state was ANTIVE or LEAVING it's necessary to move awway at least Detents_config_Max_output_distance
		if (manual_state == DETENTS_ACTIVE || manual_state == DETENTS_LEAVING)
			move_away_output_distance = 1;
		manual_initialize(state);		/* initialize the mode if necessary */
		reset_timers = 1;
		/* if a cluth is used for manual mode use external absolute sensors */
		if (Axle_clutch_Position)
		{
			ATM_seti(&manual_with_clutch, 1);
			ATM_seti(&reset_filters, 1);
		}
	}
  switch(manual_state)
	{

	case DETENTS_INACTIVE:
		release_clutch();
//		SET_POWER_ON();
    	ATM_seti(&ready_to_power_on, 0);
		manual_mode_trans_dbg = 1000;
		if (test_position_last_detent()==0)
		{
		if (detent_nearby(int2ext_pos(state->position, Home_offset), &Detents_config_Active_detent) &&
			(int2ext_vel(labs(state->velocity)) <= Detents_config_Max_velocity))
		{
			manual_initialize(state);		/* initialize the mode */
			manual_state = DETENTS_ACTIVE;
			current_detent_pos = ext2int_pos(Detents_config_Active_detent, Home_offset);	/* converto to internal units */
			reset_timers = 1;
				manual_mode_trans_dbg = 2000;


			/* update Position_demand_value and Position_demand_value_in_increments */
			Position_demand_value_in_increments = current_detent_pos;	/* [inc] */
			Position_demand_value = Detents_config_Active_detent;		/* [position units] */
				/* Trajectory generation external units */
	#if 0
				mp_trajectory_generator(Position_demand_value, &mp_trajectory, state);
				mp_trajectory.next_target_active = 0;		/* clear any pending non-immediate target */
	#endif
				init_pm_velocity_pid();
		}
		}

		break;

	case DETENTS_ACTIVE:
		activate_clutch();
		manual_mode_trans_dbg = 2000;
#if 0
		/* get position demand according to current trajectory */
		Position_demand_from_trajectory = mp_trajectory_point(&mp_trajectory, state->time);
		Position_demand_value_trajectory = int2ext_pos(Position_demand_from_trajectory, Home_offset);		/* [position units] */
#endif

#ifndef NESTED_LOOPS_MANUAL_MODE
		control_effort = _IQsat(position_controller_manual(current_detent_pos, state->position), duty_limit, -duty_limit);
#else
		control_effort_manual_profile = manual_controller(Position_demand_value, Position_actual_value,Velocity_actual_value);
		control_effort = control_effort_manual_profile;
#endif
		ATM_seti(&ready_to_power_on, 1);

		/* leave detent if Detents_config_Leave is '1' */
		if (Detents_config_Leave)
		{
			manual_state = DETENTS_LEAVING;
			manual_mode_trans_dbg = 3000;
		}

		/* became inactive if position is too far from detent point */
		if (int2ext_pos(labs(state->position - current_detent_pos), 0) > (Detents_config_Max_output_distance))
		{
			manual_state = DETENTS_INACTIVE;
			manual_mode_trans_dbg = 4000;
		}

		position_last_detent = Position_demand_value;

		break;

	case DETENTS_LEAVING:
//		SET_POWER_ON();
		release_clutch();
		ATM_seti(&ready_to_power_on, 0);
		if (!detent_nearby(int2ext_pos(state->position, Home_offset), &current_detent_pos))
		{
			manual_state = DETENTS_INACTIVE;
			manual_mode_trans_dbg = 3000;
		}
//		move_away_output_distance =1;
		break;

	default:		/* should never happen */
		manual_state = DETENTS_INACTIVE;
		manual_mode_trans_dbg = 4000;
		break;
	}

	manual_status_flags(manual_state, current_detent_pos, state, reset_timers);

	/* Debug */
#ifdef DEBUG_MANUAL_LOOP
	TickDebugSci14bytesManualMode();
#endif
}



static _iq manual_controller(long int Position_demand_value, long int Position_actual_value, long int Velocity_actual_value)
{
	static unsigned int ntimes_pos_loop;
	if (ntimes_pos_loop==FREQ_MANUAL_POSITION_LOOP-1)
	{
		control_effort_manual_position =  manual_position_controller(Position_demand_value, Position_actual_value);
		ntimes_pos_loop = 0;
}
	else
		ntimes_pos_loop++;

	control_effort_manual_velocity = manual_velocity_controller(control_effort_manual_position, Velocity_actual_value);

	return control_effort_manual_velocity;
}

static _iq manual_position_controller(long int demand, long int position)
{
	manual_pos_control_pid.Ref = _IQsat(_IQdiv(demand	,max_travel),_IQ(1.0),_IQ(-1.0));
	manual_pos_control_pid.Fdb = _IQdiv(position	,max_travel);
	manual_pos_control_pid.calc(&manual_pos_control_pid);		/* calculate PID output */
	return manual_pos_control_pid.Out;
}

static _iq manual_velocity_controller(_iq demand, long int velocity)
{
	manual_vel_control_pid.Ref = demand;
	manual_vel_control_pid.Fdb = _IQdiv(velocity,Max_demand_velocity);
	manual_vel_control_pid.calc(&manual_vel_control_pid);		/* calculate PID output */
	return manual_vel_control_pid.Out;
}

/*****************************************************************/
/*!	 Initializations of Manual Mode
	\param state Cinematic state of the system
*/
/*****************************************************************/
void manual_initialize(motion_state_struct *state)
{
	unsigned long ul_tmp;

	/* mode specific initializations */
	manual_state = DETENTS_INACTIVE;
	Detents_config_Leave = 0;
	ATM_seti(&ready_to_power_on, 0);

	/* reset mode specific bits of Statusword */
	Device_status_word &= ~(TARGET_REACHED_MASKBIT | DETENT_ACTIVE_MASKBIT | FOLLOWING_ERROR_MASKBIT);

#ifndef NESTED_LOOPS_MANUAL_MODE
	/* initialize PID */
	position_control_pid.Err = 0;
	position_control_pid.Up = 0;
	position_control_pid.Ui = 0;
	position_control_pid.Ud = 0;
	position_control_pid.OutPreSat = 0;
	position_control_pid.Out = 0;
	position_control_pid.SatErr = 0;
	position_control_pid.Err1 = 0;

	/* initialize duty_limit */
	duty_limit = _IQmpyI32(_IQ(0.001), (long)Detents_config_Max_duty);

	DINT;
	p_current_limit = &current_limit;	/* set current limit to use */
	EINT;
#endif

	// OJO, NO S� SI INICIALIZAR LOS LAZOS CON LOS VALORES POR DEFECTO DEL LAZO DE VELOCIDAD, CON SUS LIMITACIONES DE CORRIENTE, ETC.
	init_pm_current_pid();
	init_pm_position_pid();
	init_pm_velocity_pid();
	max_travel = pp_get_max_travel();


}

/*****************************************************************/
/*!	 Sets Manual Mode status flags (Target Reached and Detent Active)
	\param manual_state State of the Manual Mode state machine
	\param detent Position of the active detent (if any) [inc]
	\param state Cynematic state (position, velocity and time)
	\param reset If reset = 1, timers will be restarted
*/
/*****************************************************************/
void manual_status_flags(manual_state_t manual_state, long int detent, motion_state_struct *state, int reset)
{
	static long long t_reached_started = 0;
	unsigned long long tmp;
	static unsigned int ntimes=0;

	if( reset ) t_reached_started = 0;

	/* manage Detent Active status flag */
	if (manual_state == DETENTS_ACTIVE)
	{
		Device_status_word |= DETENT_ACTIVE_MASKBIT;		/* set Detent Active flag */
		/* manage Target Reached status flag */
		if (labs(int2ext_pos(state->position - detent, 0)) <= Position_Detent_window)//Position_window)
		{
			manual_mode_trans_dbg = 6000;
			if(!t_reached_started) t_reached_started = state->time;		/* Start time counting if not started */
			tmp = (long long)(state->time - t_reached_started) * 1000;
			tmp /= lcounts_p_s;
			if(tmp > Position_Detent_window_time)//Position_window_time)
			{
				manual_mode_trans_dbg = 7000;
				Device_status_word |= TARGET_REACHED_MASKBIT;			/* set Target Reached flag */
				SET_POWER_OFF();
			}
		}
		else
		{
			t_reached_started = 0;			/* stop time counting */
		}
#if 0
		if (labs(int2ext_pos(state->position - detent, 0)) <= Position_Detent_window*Detents_Position_Window_factor_change_Ki)//Position_window*Detents_Position_Window_factor_change_Ki)
		{
			if (ntimes == Detents_Position_Window_factor_change_Ki_window_time)
			  	manual_vel_control_pid.Ki = manual_vel_control_pid.Kc; // Increase the Ki value
			else
			{
				set_Ki_vel_manual();
				ntimes++;
			}
		}
		else
		{
			set_Ki_vel_manual();
			ntimes =0;
		}
#endif
	}
	else
	{
		Device_status_word &= ~DETENT_ACTIVE_MASKBIT;		/* clear Detent Active flag */
		Device_status_word &= ~TARGET_REACHED_MASKBIT;		/* clear Target Reached flag */
		ntimes =0;
		set_Ki_vel_manual();
	}
}

/*****************************************************************/
/*!	 Calculates the PID output in Profile Position Mode
	\param demand Position demand for this control cycle
	\param position Current position
	\return PID output [-1,1]
*/
/*****************************************************************/
_iq position_controller_manual(long int demand, long int position)
{
	long long tmp;

	if(!pos_error_normFactor) return 0;

	tmp = (long long)(position - demand) * Position_factor_Feed_constant;
	tmp = tmp << 5;	/* multiply by 32 to avoid resolut	return position_control_pid.Out;ion losses */
	tmp /= Position_factor_Numerator;					/* tmp = 32 * error [position units] */
	if(llabs(tmp) > (long long)(pos_error_normFactor<<5))
		tmp = sign(tmp) * (long long)(pos_error_normFactor<<5); /* limit the error to 'Position_control_margin' */

	/* reference position is always '0' and current position is set as the relative error */
	position_control_pid.Ref = _IQ(0);
	/* Divide by normFactor and later convert to IQ and divide by 2^5 */
	position_control_pid.Fdb = (tmp << (GLOBAL_Q - 5)) / pos_error_normFactor ;	/* position error related to 'Position_control_margin' */

	position_control_pid.calc(&position_control_pid);		/* calculate PID output */

	return position_control_pid.Out;
}


static void init_pm_position_pid(void)
{
	unsigned long ul_tmp;
	/* initialize PID */
	manual_pos_control_pid.Err = 0;
	manual_pos_control_pid.Up = 0;
	manual_pos_control_pid.Ui = 0;
	manual_pos_control_pid.Ud = 0;
	manual_pos_control_pid.OutPreSat = 0;
	manual_pos_control_pid.Out = 0;
	manual_pos_control_pid.SatErr = 0;
	manual_pos_control_pid.Err1 = 0;
	manual_pos_control_pid.OutMax = MAX_DUTY_CYCLE;
	manual_pos_control_pid.OutMin = MIN_DUTY_CYCLE;

	manual_pos_control_pid.Period = _IQ(0.01);
	manual_pos_control_pid.Frequency = 100;


	ul_tmp = Position_Manual_control_parameters[0];		/* Kp (has to be divided by Divisor) */
	ul_tmp = ul_tmp << 16;							/* multiply by 2^16 */
	ul_tmp /= Position_Manual_control_parameters[4];			/* divide by Divisor */
	manual_pos_control_pid.Kp = ul_tmp << (GLOBAL_Q - 16);		/* convert to IQ and divide by 2^16 */

	ul_tmp = Position_Manual_control_parameters[1];		/* Ki (has to be divided by Divisor) */
	ul_tmp = ul_tmp << 16;							/* multiply by 2^16 */
	ul_tmp /= Position_Manual_control_parameters[4];			/* divide by Divisor */
	manual_pos_control_pid.Ki = ul_tmp << (GLOBAL_Q - 16);		/* convert to IQ and divide by 2^16 */

	ul_tmp = Position_Manual_control_parameters[2];		/* Kd (has to be divided by Divisor) */
	ul_tmp = ul_tmp << 16;							/* multiply by 2^16 */
	ul_tmp /= Position_Manual_control_parameters[4];			/* divide by Divisor */
	manual_pos_control_pid.Kd = ul_tmp << (GLOBAL_Q - 16);		/* convert to IQ and divide by 2^16 */

	ul_tmp = Position_Manual_control_parameters[3];		/* Kc (has to be divided by Divisor) */
	ul_tmp = ul_tmp << 16;							/* multiply by 2^16 */
	ul_tmp /= Position_Manual_control_parameters[4];			/* divide by Divisor */
	manual_pos_control_pid.Kc = ul_tmp << (GLOBAL_Q - 16);		/* convert to IQ and divide by 2^16 */
}




static void init_pm_velocity_pid(void)
{
	unsigned long ul_tmp;
	manual_vel_control_pid.Err = 0;
	manual_vel_control_pid.Up = 0;
	manual_vel_control_pid.Ui = 0;
	manual_vel_control_pid.Ud = 0;
	manual_vel_control_pid.OutPreSat = 0;
	manual_vel_control_pid.Out = 0;
	manual_vel_control_pid.SatErr = 0;
	manual_vel_control_pid.Err1 = 0;
	manual_vel_control_pid.OutMax = MAX_DUTY_CYCLE;
	manual_vel_control_pid.OutMin = MIN_DUTY_CYCLE;

	manual_vel_control_pid.Period = _IQ(0.001);
	manual_vel_control_pid.Frequency = 1000;


	ul_tmp = Velocity_Manual_control_parameters[0];		/* Kp (has to be divided by Divisor) */
	ul_tmp = ul_tmp << 16;							/* multiply by 2^16 */
	ul_tmp /= Velocity_Manual_control_parameters[4];			/* divide by Divisor */
	manual_vel_control_pid.Kp = ul_tmp << (GLOBAL_Q - 16);		/* convert to IQ and divide by 2^16 */

	ul_tmp = Velocity_Manual_control_parameters[1];		/* Ki (has to be divided by Divisor) */
	ul_tmp = ul_tmp << 16;							/* multiply by 2^16 */
	ul_tmp /= Velocity_Manual_control_parameters[4];			/* divide by Divisor */
	manual_vel_control_pid.Ki = ul_tmp << (GLOBAL_Q - 16);		/* convert to IQ and divide by 2^16 */

	ul_tmp = Velocity_Manual_control_parameters[2];		/* Kd (has to be divided by Divisor) */
	ul_tmp = ul_tmp << 16;							/* multiply by 2^16 */
	ul_tmp /= Velocity_Manual_control_parameters[4];			/* divide by Divisor */
	manual_vel_control_pid.Kd = ul_tmp << (GLOBAL_Q - 16);		/* convert to IQ and divide by 2^16 */

	ul_tmp = Velocity_Manual_control_parameters[3];		/* Kc (has to be divided by Divisor) */
	ul_tmp = ul_tmp << 16;							/* multiply by 2^16 */
	ul_tmp /= Velocity_Manual_control_parameters[4];			/* divide by Divisor */
	manual_vel_control_pid.Kc = ul_tmp << (GLOBAL_Q - 16);		/* convert to IQ and divide by 2^16 */
}


void set_Ki_vel_manual(void)
{
	unsigned long ul_tmp;
	ul_tmp = Velocity_Manual_control_parameters[1];		/* Ki (has to be divided by Divisor) */
	ul_tmp = ul_tmp << 16;							/* multiply by 2^16 */
	ul_tmp /= Velocity_Manual_control_parameters[4];			/* divide by Divisor */
	manual_vel_control_pid.Ki = ul_tmp << (GLOBAL_Q - 16);		/* convert to IQ and divide by 2^16 */
}

static void init_pm_current_pid(void)
{

	init_current_controller_dc_bldc();

	set_Kp_current_control_dc_bldc(Current_Manual_control_parameters[0],
			Current_Manual_control_parameters[4]);

	set_Ki_current_control_dc_bldc(Current_Manual_control_parameters[1],
			Current_Manual_control_parameters[4]);

	set_Kd_current_control_dc_bldc(Current_Manual_control_parameters[2],
			Current_Manual_control_parameters[4]);

	set_Kc_current_control_dc_bldc(Current_Manual_control_parameters[3],
			Current_Manual_control_parameters[4]);
}

/*
 * quick_stop.c
 *
 *  Created on: Sep 24, 2019
 *      Author: user
 */

#include "quick_stop.h"
#include "amc.h"
#include "amc_od.h"



int reload_change_time=0;
int test_feedforward_current=0;
_iq factor_current_down =_IQ(0.0);
_iq test_current_ref = _IQ(0.0);


/*****************************************************************/
/*! 	Makes necessary initializations for quick stop (set target velocity = 0)
	\param state Cinematic state of the system
	\param mode quick stop mode
*/
/*****************************************************************/
void quick_stop_initialization( motion_state_struct *state, int *mode )
{

	DINT;
	p_current_limit = &current_limit;	/* set current limit to use */
	EINT;

	Device_status_word &= ~MODE_SPECIFIC_BITS;
	if( !Quick_ramp_time ||
		((Quick_stop_option_code != STOP_BRAKE) && (Quick_stop_option_code != STOP_QUICK)) ||
		(Modes_of_operation_display==OPERATION_MODE_MANUAL) ||
		(Modes_of_operation_display==OPERATION_MODE_MANUAL_2))
	    {
			*mode = STOP_BRAKE;
	    }
    else
    {
    	*mode = Quick_stop_option_code;
    }
	quick_stop_init( state, mode );
}



void quick_stop_init( motion_state_struct *state, int *mode )
{
	reload_change_time = 0;
	switch(*mode)
	{
	case STOP_BRAKE:
	default:
		load_ramp_time(0);
		*mode = STOP_BRAKE;
		break;

	case STOP_QUICK:
		if (SAFETY == SAFETY_ALLOW)
		{
			load_ramp_time(Quick_ramp_time);
			_LOGmessage(0x9903,"Ramp management", 0, 0);
			switch(Modes_of_operation_display)
			{
			case OPERATION_MODE_POSITION:
				pp_trajectory_set_stop_point(Quick_stop_deceleration, &pp_trajectory, state);
				break;
			default:
				load_ramp_time(0);
				*mode = STOP_BRAKE;
				break;
			}
		}
		else
		{
			load_ramp_time(0);
			*mode = STOP_BRAKE;
		}
		break;
	}
}


/*****************************************************************/
/*! 	Control velocity for quick stop
	\param state Cinematic state of the system
	\param mode quick stop mode
*/
/*****************************************************************/
void quick_stop_control( motion_state_struct *state, int mode )
{

	switch( mode )
	{
		case STOP_BRAKE:
		default:
			load_ramp_time(0);
			break;
		case STOP_QUICK:
			if( (Safety == SAFETY_ALLOW) &&
				!(Device_status_word &TARGET_REACHED_MASKBIT) &&
				!(Device_status_word &HALT_REACHED_MASKBIT) )
			{
				switch(Modes_of_operation_display)
				{
					case OPERATION_MODE_POSITION:
						profile_position_mode_operation(state, 0);
						break;
					default:
						break;
				}
			}
			else
			{
				load_ramp_time(0);
			}
				break;
			}

	TickDebugSci14bytesPositionMode();



    if (ramp_time == 0)
	{
		SET_POWER_OFF();

		if( !reload_change_time )
		{
#if 0
			test_current_ref = current_control_pid.Ref;

			current_control_pid.Ki = _IQmpy(current_control_pid.Ki,_IQ(4.0));
			//control_effort = _IQ (0.0);
  			//current_control_pid.Ref =_IQ(0.0);
			/* Set motor current sign in final_current */
#endif
			reload_change_time = 1;
			mode_change_time = maximum(Quick_mode_change_time, MINIMUM_MODE_CHANGE_TIME);
#if 0
			factor_current_down = _IQ16toIQ(_IQ16div(_IQtoIQ16(current_control_pid.Ref),_IQ16(mode_change_time)));
#endif

		}
//		test_current_ref = test_current_ref - factor_current_down;
		//current_control_pid.Ref = current_control_pid.Ref - factor_current_down;
#if 0
		control_effort = control_effort - factor_current_down;
#endif
//		if (current_control_pid.Ref < _IQ(0.0))
//			current_control_pid.Ref =_IQ(0.0);
	}

	if( reload_change_time && mode_change_time == 0 )
	{
		test_current_ref= _IQ(0.0);
		test_feedforward_current =current_control_pid.Out;
		ATM_seti(&ready_to_power_on, 0);
		setDriveState((unsigned short *)&Device_status_word, SWITCH_ON_DISABLED); /* automatically change from Quick_Stop_Active to Switch_On_Disabled */
		pp_trajectory_reset( &pp_trajectory, state );
		pv_trajectory_reset( &pv_trajectory, state );
		force_init = INIT_POS_MODE|INIT_VEL_MODE|INIT_ASSISTED_MODE;
	}
}

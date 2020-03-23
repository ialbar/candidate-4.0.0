/*!	\file errors.c
	\brief Definitions for erros and warnings management

* File: error.h
* Device: TMS320F28235
* Author: Luis Jimenez.
* Description: Definitions for errors and warnings management
*/

#include "amc.h"
#include "errors.h"



static unsigned long active_faults = 0;		/*!< variable that tracks the faults that are still active */
static unsigned long correcting_faults = 0;	/*!< variable that tracks the faults that are being corrected */

static long long fault_pos_following_timer = 0;
static long long fault_vel_following_timer = 0;
static long long fault_iron_cable_timer = 0;
static long long fault_vel_limit_timer = 0;
static long long fault_hall_timer = 0;
static long long fault_overcurrent_timer = 0;


static unsigned long warning_faults = 0;
static unsigned long current_faults = 0;

short current_filt_to_check_overcurrent = 0;			/*!< Servo dc current measured in mA */
unsigned short int dbg_overcurrent = 0;
unsigned int Overcurrent_error_window = 0;

extern int homing_zeroed;
/*****************************************************************/
/*!	Function that sends a LOG message using SCI port
	\param code The code of the LOG message
*/
/*****************************************************************/
void log_sci(unsigned int code)
{
	static int last;
	char text[7] = "L";

	if( last != code )
	{
		last = code;
		itohex(&(text[1]), code);
		text[5] = '\n';
		text[6] = 0;
#if DEBUG_SCI_TO_DEBUG_LOOPS
#else
		SCI_send(text);
#endif
	}
}


/*****************************************************************/
/*!	Function that sends a WAR message using SCI port
	\param code The code of the LOG message
*/
/*****************************************************************/
void war_sci(unsigned int code)
{
	static int last;
	char text[7] = "W";

	if( last != code )
	{
		last = code;
		itohex(&(text[1]), code);
		text[5] = '\n';
		text[6] = 0;
#if DEBUG_SCI_TO_DEBUG_LOOPS
#else
		SCI_send(text);
#endif
	}
}

/*****************************************************************/
/*!	Function that sends a ERR message using SCI port
	\param code The code of the LOG message
*/
/*****************************************************************/
void err_sci(unsigned int code)
{
	static int last;
	char text[7] = "E";

	if( last != code )
	{
		last = code;
		itohex(&(text[1]), code);
		text[5] = '\n';
		text[6] = 0;
#if DEBUG_SCI_TO_DEBUG_LOOPS
#else
		SCI_send(text);
#endif
	}
}

/*****************************************************************/
/*!	Function that sends a ERR message using SCI port
	\param code The code of the LOG message
*/
/*****************************************************************/
void deb_sci(unsigned int code)
{
	static int last;
	char text[7] = "D";

	if( last != code )
	{
		last = code;
		itohex(&(text[1]), code);
		text[5] = '\n';
		text[6] = 0;
#if DEBUG_SCI_TO_DEBUG_LOOPS
#else
		SCI_send(text);
#endif
	}
}


/*****************************************************************/
/*!	Function that converts an unsinged int to a text string
	\param text The number is written to this string
	\param number The number to convert
*/
/*****************************************************************/
void itohex(char *text, unsigned int number)
{
	unsigned int aux;

	text[0] = (((aux = ((number >> 12) & 0x000F)) < 10)? '0' : '7') + aux;
	text[1] = (((aux = ((number >> 8) & 0x000F)) < 10)? '0' : '7') + aux;
	text[2] = (((aux = ((number >> 4) & 0x000F)) < 10)? '0' : '7') + aux;
	text[3] = (((aux = ((number >> 0) & 0x000F)) < 10)? '0' : '7') + aux;
	text[4] = 0;
}


/*****************************************************************/
/*!	Function that ejecutes necessary actions when entered "Fault Reaction Active"
	\param state Cinematic state of the system
*/
/*****************************************************************/
void fault_management(motion_state_struct *state)
{
	if(!correcting_faults)
	{
	  setDriveState((unsigned short *)&Device_status_word, FAULT); /* automatically change from Fault_Reaction_Active to Fault */
	  pp_trajectory_reset( &pp_trajectory, state );
	  pv_trajectory_reset( &pv_trajectory, state );
	}
	else
	{
		if( correcting_faults  )
		{
			activate_brake();
			release_clutch();
			ATM_seti(&ready_to_power_on, 0);
		}
		if(correcting_faults & FAULT_IRON_CABLE_BREAK)
		{
			DISABLE_MOTOR;	/* all 6 PWM's off (free motor movement) */
			DINT;
			correcting_faults &= ~FAULT_IRON_CABLE_BREAK;
			EINT;
		}
		if(correcting_faults & FAULT_OVERCURRENT)
		{
			DISABLE_MOTOR;	/* all 6 PWM's off (free motor movement) */
			DINT;
			correcting_faults &= ~FAULT_OVERCURRENT;
			EINT;
		}
		if(correcting_faults & FAULT_HEARTBEAT)
		{	/* end fault action if motor stopped */
				DINT;
				correcting_faults &= ~FAULT_HEARTBEAT;
				EINT;
		}
		if(correcting_faults & FAULT_NULL_ACC)
		{	/* end fault action immediately because quick stop function can not be executed */
			DINT;
			correcting_faults &= ~FAULT_NULL_ACC;		/* no fault action */
			EINT;
		}
		if(correcting_faults & FAULT_CONTROL_CYCLE)
		{	/* end fault action if motor stopped */
				DINT;
				correcting_faults &= ~FAULT_CONTROL_CYCLE;
				EINT;
		}
		if(correcting_faults & FAULT_PASSIVE_CAN)
		{	/* end fault action if motor stopped */
				DINT;
				correcting_faults &= ~FAULT_PASSIVE_CAN;
				EINT;
		}
		if(correcting_faults & FAULT_BUSOFF_CAN)
		{	/* end fault action if motor stopped */
				DINT;
				correcting_faults &= ~FAULT_BUSOFF_CAN;
				EINT;
		}
		if(correcting_faults & FAULT_OVERTEMP)
		{	/* end fault action if motor stopped */
				DINT;
				correcting_faults &= ~FAULT_OVERTEMP;
				EINT;
		}
		if(correcting_faults & FAULT_VPOWER)
		{	/* end fault action if motor stopped */
				DINT;
				correcting_faults &= ~FAULT_VPOWER;
				EINT;
		}
		if(correcting_faults & FAULT_VEL_FOLLOWING)
		{	/* end fault action if motor stopped */
				DINT;
				correcting_faults &= ~FAULT_VEL_FOLLOWING;
				EINT;
		}
		if(correcting_faults & FAULT_POS_FOLLOWING)
		{	/* end fault action if motor stopped */
				DINT;
				correcting_faults &= ~FAULT_POS_FOLLOWING;
				EINT;
		}
		if(correcting_faults & FAULT_VEL_LIMIT)
		{	/* a velocity limit fault has been registered */
			DINT;
			correcting_faults &= ~FAULT_VEL_LIMIT;		 /* no fault action, just disable voltage */
			EINT;
		}
		if(correcting_faults & FAULT_POS_LIMIT)
		{	/* a software position limit fault has been registered */
			DINT;
			correcting_faults &= ~FAULT_POS_LIMIT;		 /* no fault action, just disable voltage */
			EINT;
		}
		if(correcting_faults & FAULT_MOTOR_PHASE)
		{
			DISABLE_MOTOR;	/* all 6 PWM's off (free motor movement) */
			DINT;
			correcting_faults &= ~FAULT_MOTOR_PHASE;
			EINT;
		}
		if(correcting_faults & FAULT_CURRENTS_FOLLOW)
		{
			DISABLE_MOTOR;	/* all 6 PWM's off (free motor movement) */
			DINT;
			correcting_faults &= ~FAULT_CURRENTS_FOLLOW;
			EINT;
		}
		if(correcting_faults & FAULT_MOTOR_OVERTEMP)
		{	/* end fault action if motor stopped */
				DINT;
				correcting_faults &= ~FAULT_MOTOR_OVERTEMP;
				EINT;
		}
		if(correcting_faults & FAULT_IGBT_ISR)
		{
			DISABLE_MOTOR;	/* all 6 PWM's off (free motor movement) */
			DINT;
			correcting_faults &= ~FAULT_IGBT_ISR;
			EINT;
		}
		if(correcting_faults & FAULT_HALL)
		{
			DISABLE_MOTOR;	/* all 6 PWM's off (free motor movement) */
			DINT;
			correcting_faults &= ~FAULT_HALL;
			EINT;
		}
		if(correcting_faults & FAULT_NCE_MEASURE)
		{
			DISABLE_MOTOR;	/* all 6 PWM's off (free motor movement) */
			DINT;
			correcting_faults &= ~FAULT_NCE_MEASURE;
			EINT;
		}
		if(correcting_faults & FAULT_POT_MEASURE)
		{
			DISABLE_MOTOR;	/* all 6 PWM's off (free motor movement) */
			DINT;
			correcting_faults &= ~FAULT_POT_MEASURE;
			EINT;
		}
		if(correcting_faults & FAULT_ENCODER)
		{
			DISABLE_MOTOR;	/* all 6 PWM's off (free motor movement) */
			DINT;
			correcting_faults &= ~FAULT_ENCODER;
			EINT;
		}
		if(correcting_faults & FAULT_UNUSED)
		{	/* a non-valid fault has been registered */
			DINT;
			correcting_faults &= ~FAULT_UNUSED;		/* no fault action because fault is unknown */
			EINT;
		}
	}
}


/*****************************************************************/
/*!	Function that registers a new fault
	\param fault Fault to be registered. Possible values defined in errors.h
*/
/*****************************************************************/
void setFault(unsigned long fault)
{
	if (homing_zeroed)
  {
    if(!(fault & FAULT_UNUSED))		/* if fault is configured */
    {
		  DINT;
		  active_faults |= fault;			/* register the fault */
		  correcting_faults |= fault;		/* begin the fault action */
		  EINT;
		  if(ATM_seti(&manual_with_clutch, 0)) ATM_seti(&reset_filters, 1);
		  setDriveState((unsigned short *)&Device_status_word, FAULT_REACTION_ACTIVE); /* change state to Fault_Reaction_Active */
    }
	  else
    {
	   _LOGmessage(0x0024,"Fault not configured", 0, 0)
    }
  }
}




/*****************************************************************/
/*!	Function that checks if any of the active faults is unrecoverable
	This function is called from drive_state_machine.c when FAULT_RESET command is received and state is FAULT
	\return 0 if no one active fault is unrecoverable.
*/
/*****************************************************************/
unsigned long unrecoverableFaults(void)
{


	if(active_faults & FAULT_IRON_CABLE_BREAK)
	{
		if( !(FAULT_IRON_CABLE_BREAK & GetQueuedFault()))
		{
			DINT;
			active_faults &= ~FAULT_IRON_CABLE_BREAK;
			EINT;
			EMCY_errorRecovered(&amc_od_Data, 0x1616);	/* clear the error */
		}
	}

	if(active_faults & FAULT_OVERCURRENT)
	{	/* end fault action when current < Overcurrent_error_window */
		if(current < Overcurrent_error_window)
		{
		    DINT;
			active_faults &= ~FAULT_OVERCURRENT;
			EINT;
			EMCY_errorRecovered(&amc_od_Data, 0x2310);	/* clear the error */
		}
	}

	if(active_faults & FAULT_HEARTBEAT)
	{	/* it is recoverable, communication is operative if "Fault reset" command arrived */
		DINT;
		active_faults &= ~FAULT_HEARTBEAT;
		EINT;
		EMCY_errorRecovered(&amc_od_Data, 0x8130);	/* clear the error */
	}

	if(active_faults & FAULT_NULL_ACC)
	{	/* all acceleration values have to be defined */
		if(Profile_acceleration && Profile_deceleration && Quick_stop_deceleration
			&& Max_acceleration && Max_acceleration)
		{
			DINT;
			active_faults &= ~FAULT_NULL_ACC;
			EINT;
			EMCY_errorRecovered(&amc_od_Data, 0xff03);	/* clear the error */
		}
	}
	if(active_faults & FAULT_CONTROL_CYCLE)
	{	/* it is recoverable */
		DINT;
		active_faults &= ~FAULT_CONTROL_CYCLE;
		EINT;
		EMCY_errorRecovered(&amc_od_Data, 0xff02);	/* clear the error */
	}
	if(active_faults & FAULT_PASSIVE_CAN)
	{	/* it is recoverable */
		if(ECanaRegs.CANES.bit.EP == 0)
		{
			DINT;
			active_faults &= ~FAULT_PASSIVE_CAN;
			EINT;
			EMCY_errorRecovered(&amc_od_Data, 0x8120);	/* clear the error */
			CLEAR_RED_CAN_LED;
		}
	}
	if(active_faults & FAULT_BUSOFF_CAN)
	{	/* it is recoverable */
		if(ECanaRegs.CANES.bit.BO == 0)
		{
			DINT;
			active_faults &= ~FAULT_BUSOFF_CAN;
			EINT;
			EMCY_errorRecovered(&amc_od_Data, 0x8140);	/* clear the error */
			CLEAR_RED_CAN_LED;
		}
	}
	if(active_faults & FAULT_OVERTEMP)
	{	/* it is recoverable when temperature < Temperature_limit */
		if(Temperature < Temperature_limit)
		{
			DINT;
			active_faults &= ~FAULT_OVERTEMP;
			EINT;
			EMCY_errorRecovered(&amc_od_Data, 0x4310);	/* clear the error */
		}
	}

	if(active_faults & FAULT_VPOWER)
	{	/* it is recoverable when Vpower is between voltege limits */
		if((Power_voltage >= Power_voltage_limit[0]) && (Power_voltage <= Power_voltage_limit[1]))
		{
			DINT;
			active_faults &= ~FAULT_VPOWER;
			EINT;
			EMCY_errorRecovered(&amc_od_Data, 0x3100);	/* clear the error */
		}
	}

	if(active_faults & FAULT_VEL_FOLLOWING)
	{
		DINT;
		active_faults &= ~FAULT_VEL_FOLLOWING;
		EINT;
		EMCY_errorRecovered(&amc_od_Data, 0x8400);	/* clear the error */
	}

	if(active_faults & FAULT_POS_FOLLOWING)
	{	/* it is recoverable */
		DINT;
		active_faults &= ~FAULT_POS_FOLLOWING;
		EINT;
		EMCY_errorRecovered(&amc_od_Data, 0x8611);	/* clear the error */
	}

	if(active_faults & FAULT_VEL_LIMIT)
	{
		if( !(FAULT_VEL_LIMIT & GetQueuedFault()))
		{
		DINT;
		active_faults &= ~FAULT_VEL_LIMIT;
		EINT;
		EMCY_errorRecovered(&amc_od_Data, 0xFF09);	/* clear the error */
	}
	}

	if(active_faults & FAULT_POS_LIMIT)
	{
		//no recovered in position limit
		if(Enable_Recovery_Faults == 1)
		{
			/* it is recoverable */
			DINT;
			active_faults &= ~FAULT_POS_LIMIT;
			EINT;
			EMCY_errorRecovered(&amc_od_Data, 0xFF0A);	/* clear the error */
		}
	}

	if(active_faults & FAULT_HALL)
	{
		if( !(FAULT_HALL & GetQueuedFault()))
		{
			DINT;
			active_faults &= ~FAULT_HALL;
			EINT;
			EMCY_errorRecovered(&amc_od_Data, 0xFF0F);	/* clear the error */
		}
	}

	if(active_faults & FAULT_NCE_MEASURE)
	{
		if( !(FAULT_NCE_MEASURE & GetQueuedFault()))
		{
			DINT;
			active_faults &= ~FAULT_NCE_MEASURE;
			EINT;
			EMCY_errorRecovered(&amc_od_Data, 0xFF08);	/* clear the error */
		}
	}

	if(active_faults & FAULT_POT_MEASURE)
	{
		if( !(FAULT_POT_MEASURE & GetQueuedFault()))
		{
			DINT;
			active_faults &= ~FAULT_POT_MEASURE;
			EINT;
			EMCY_errorRecovered(&amc_od_Data, 0xFF06);	/* clear the error */
		}
	}

	if(active_faults & FAULT_ENCODER)
	{
		if( !(FAULT_ENCODER & GetQueuedFault()))
		{
			DINT;
			active_faults &= ~FAULT_ENCODER;
			EINT;
			EMCY_errorRecovered(&amc_od_Data, 0xFF05);	/* clear the error */
		}
	}

	if(active_faults & FAULT_MOTOR_PHASE)
	{
		DINT;
		active_faults &= ~FAULT_MOTOR_PHASE;
		EINT;
		EMCY_errorRecovered(&amc_od_Data, 0xFF0C);	/* clear the error */
	}

	//if(active_faults & FAULT_CURRENTS_FOLLOW)
	//{
	//	DINT;
	//	active_faults &= ~FAULT_CURRENTS_FOLLOW;
	//	EINT;
	//	EMCY_errorRecovered(&amc_od_Data, 0xXXXX);	/* clear the error */
	//}

	if(active_faults & FAULT_MOTOR_OVERTEMP)
	{	/* it is recoverable when temperature < Temperature_limit */
		if(Motor_Temperature < Motor_Temperature_limit)
		{
			DINT;
			active_faults &= ~FAULT_MOTOR_OVERTEMP;
			EINT;
			EMCY_errorRecovered(&amc_od_Data, 0x4311);	/* clear the error */
		}
	}

	if(active_faults & FAULT_IGBT_ISR)
	{
		if( OVERCURRENT == OVERCURRENT_OK ) /*A3625 pin is cleared*/
		{
			EALLOW;
			EPwm1Regs.TZCLR.bit.OST = 1;
			EPwm1Regs.TZCLR.bit.INT = 1;
			EPwm2Regs.TZCLR.bit.OST = 1;
			EPwm2Regs.TZCLR.bit.INT = 1;
			EPwm3Regs.TZCLR.bit.OST = 1;
			EPwm3Regs.TZCLR.bit.INT = 1;
			EDIS;

			DINT;
			active_faults &= ~FAULT_IGBT_ISR;
			EINT;
			EMCY_errorRecovered(&amc_od_Data, 0x2311);	/* clear the error */
		}
	}

	return active_faults;
}

/*****************************************************************/
/*!	Function that checks if an error is active
	\param fault Fault to be registered. Possible values defined in errors.h
*/
/*****************************************************************/
int isFaultActive(unsigned long fault)
{
	return (active_faults & fault)? 1 : 0;
}




static unsigned int filter_current_to_check_overcurrent (unsigned int input)
{
    unsigned int i=0;
    _iq16 a1_theta= _IQ16(-0.939101367424293); // fs =1e3 (periodic control task) fc = 10Hz
    _iq16 b0_theta= _IQ16( 0.060898632575707);

    static _iq16 ydifftheta[2]={_IQ16(0.0),_IQ16(0.0)};
    _iq16 input_iq;

    input_iq = _IQ16(input);

    for (i=1;i>0;i--)
        ydifftheta[i]=ydifftheta[i-1];

    ydifftheta[0]=    _IQ16mpy(b0_theta,input_iq)
                    -_IQ16mpy(a1_theta,ydifftheta[1]);

    return (ydifftheta[0]>>16);
}

void check_overcurrent (void)
{
	unsigned long long tmp;
	static long long c_error_started=0;

	if( !Sensors_configuration_active )
	{
		c_error_started = 0;		/* stop time counting */
		DeQueueFault(FAULT_OVERCURRENT);
		return;
	}

	Overcurrent_error_window = current_limit * (OverCurrent_percent_error_window/100.0);
	current_filt_to_check_overcurrent = filter_current_to_check_overcurrent(current);

	if ( current_filt_to_check_overcurrent > Overcurrent_error_window )
	{
		if(!c_error_started)
		{
			/* Start time counting if not started */
			c_error_started = getltime();		/* set current time */
			dbg_overcurrent = 10000;
		}
		tmp = (long long)(getltime() - c_error_started) * 1000;
		tmp /= lcounts_p_s;
		if(tmp > OverCurrent_error_time_out)
		{
			/* set the current following error */
			if(!(isFaultActive(FAULT_OVERCURRENT)))
			{
				_ERRORmessage(0x2310, 0x02, 0x0000, "Motor overcurrent", 0, 0);
				setFault(FAULT_OVERCURRENT);
			}
			QueueFault(FAULT_OVERCURRENT);

		}
		else
			DeQueueFault(FAULT_OVERCURRENT);
	}
	else
	{
		dbg_overcurrent = 0;
		c_error_started = 0;			/* stop time counting */
		DeQueueFault(FAULT_OVERCURRENT);
	}
}




void QueueFault( unsigned long fault )
{
	if( (fault & FAULT_POS_FOLLOWING) )
	{
		if( !(current_faults & FAULT_POS_FOLLOWING) )
		{
			fault_pos_following_timer = getltime();
		}
	}
	if( (fault & FAULT_VEL_FOLLOWING) )
	{
		if( !(current_faults & FAULT_VEL_FOLLOWING) )
		{
			fault_vel_following_timer = getltime();
		}
	}
	if( (fault & FAULT_IRON_CABLE_BREAK) )
	{
		if( !(current_faults & FAULT_IRON_CABLE_BREAK) )
		{
			fault_iron_cable_timer = getltime();
		}
	}
	if( (fault & FAULT_VEL_LIMIT) )
	{
		if( !(current_faults & FAULT_VEL_LIMIT) )
		{
			fault_vel_limit_timer = getltime();
		}
	}
	if( (fault & FAULT_HALL) )
	{
		if( !(current_faults & FAULT_HALL) )
		{
			fault_hall_timer = getltime();
		}
	}

	if( (fault & FAULT_OVERCURRENT) )
	{
		if( !(current_faults & FAULT_OVERCURRENT) )
		{
			fault_overcurrent_timer = getltime();
		}
	}

	current_faults |= fault;
}


void DeQueueFault( unsigned long fault )
{
	if( (current_faults & FAULT_POS_FOLLOWING) )
	{
		if( (fault & FAULT_POS_FOLLOWING) )
		{
			fault_pos_following_timer = 0;
		}
	}
	if( (current_faults & FAULT_VEL_FOLLOWING) )
	{
		if( (fault & FAULT_VEL_FOLLOWING) )
		{
			fault_vel_following_timer = 0;
		}
	}
	if( (current_faults & FAULT_IRON_CABLE_BREAK) )
	{
		if( (fault & FAULT_IRON_CABLE_BREAK) )
		{
			fault_iron_cable_timer = 0;
		}
	}
	if( (current_faults & FAULT_VEL_LIMIT) )
	{
		if( (fault & FAULT_VEL_LIMIT) )
		{
			fault_vel_limit_timer = 0;
		}
	}
	if( (current_faults & FAULT_HALL) )
	{
		if( (fault & FAULT_HALL) )
		{
			fault_hall_timer = 0;
		}
	}

	if( (current_faults & FAULT_OVERCURRENT) )
	{
		if( (fault & FAULT_OVERCURRENT) )
		{
			fault_overcurrent_timer = 0;
		}
	}

	current_faults &= ~fault;
}

unsigned long GetQueuedFault( void )
{
	return current_faults;
}

void set_fault_flags(void)
{
	unsigned long long tmp =0;
	long long current_time = getltime();

	if( fault_pos_following_timer )
	{
		tmp = (long long)(current_time - fault_pos_following_timer) * 1000;
		tmp /= lcounts_p_s;
		if(tmp > FAULT_POS_FOLLOWING_ERROR_TIMEOUT )
			DeQueueFault(FAULT_POS_FOLLOWING);
	}
	if( fault_vel_following_timer )
	{
		tmp = (long long)(current_time - fault_vel_following_timer) * 1000;
		tmp /= lcounts_p_s;
		if(tmp > FAULT_VEL_FOLLOWING_ERROR_TIMEOUT )
			DeQueueFault(FAULT_VEL_FOLLOWING);
	}
	if( fault_iron_cable_timer )
	{
		tmp = (long long)(current_time - fault_iron_cable_timer) * 1000;
		tmp /= lcounts_p_s;
		if(tmp > FAULT_IRON_CABLE_ERROR_TIMEOUT )
			DeQueueFault(FAULT_IRON_CABLE_BREAK);
	}
	if( fault_vel_limit_timer )
	{
		tmp = (long long)(current_time - fault_vel_limit_timer) * 1000;
		tmp /= lcounts_p_s;
		if(tmp > FAULT_VEL_LIMIT_ERROR_TIMEOUT )
			DeQueueFault(FAULT_VEL_LIMIT);
	}
	if( fault_hall_timer )
	{
		tmp = (long long)(current_time - fault_hall_timer) * 1000;
		tmp /= lcounts_p_s;
		if(tmp > FAULT_HALL_ERROR_TIMEOUT )
			DeQueueFault(FAULT_HALL);
	}
	if( fault_overcurrent_timer )
	{
		tmp = (long long)(current_time - fault_overcurrent_timer) * 1000;
		tmp /= lcounts_p_s;
		if(tmp > FAULT_OVERCURRENT_ERROR_TIMEOUT )
			DeQueueFault(FAULT_OVERCURRENT);
	}

	if( current_faults )
		Device_status_word |= ERROR_MASKBIT;  //set error flag
	else
		Device_status_word &= ~ERROR_MASKBIT; //clear error flag
	Error_statusword = current_faults;
}

void QueueWarning( unsigned long fault )
{
	warning_faults |= fault;
}

void DeQueueWarning( unsigned long fault )
{
	warning_faults &= ~fault;
}

void set_warning_flags(void)
{
	if( warning_faults )
		Device_status_word |= WARNING_MASKBIT;  //set warning flag
	else
		Device_status_word &= ~WARNING_MASKBIT; //clear warning flag
	Warning_statusword = warning_faults;
}

unsigned long GetQueuedWarning( void )
{
	return warning_faults;
}

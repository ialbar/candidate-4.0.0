/*!	\file homing_mode.c
	\brief Functions for Homing Mode

\verbatim
*********************************************************************
* File: homing_mode.c
* Devices: TMS320F28XXX
* Author: Alvaro Garcia.
* History:
*   26/07/07 - original
*********************************************************************
\endverbatim
*/

#include "DSP2833x_Device.h"

#include "amc.h"

long absolute_sensor;


/*****************************************************************/
/*!	 Main function of Profile Homing Mode
	\param state Cinematic state of the system
	\param init will be 1 if the mode has to be initialized (0 if not)
*/
/*****************************************************************/
int homing_mode_operation(motion_state_struct *state, char init)
{
	static int zeroed = 0;
	static UNS16 active;

	if(!abs_pos_counter)
    {
		_WARNINGmessage(0xffed, 0x80, 0x2022, "Wrong absolute position sensor", 0, 0);
		Device_status_word |= HOMING_ERROR;
		zeroed = active = 0;
		return zeroed;
    }

	if( (Device_control_word & HOMING_OPERATION_START) )
	{
		zeroed = 0;
		active = 1;
		Device_control_word &= ~HOMING_OPERATION_START;
		Device_status_word &= ~(TARGET_REACHED_MASKBIT | HOMING_ATTAINED | HOMING_ERROR); /* Initialize the ControlWord */
		set_fault_flags();
		set_brakes_flags();
		send_controlword();
	}
	else if( active )
	{
		if( ++active == 1000 )
		{
			absolute_sensor = abs_pos_counter(SAVED_PARAMS);
			//Debug_long2 = pot;
			//Debug_long3 = int2ext_pos(state->position, 0);
			Home_offset = absolute_sensor - int2ext_pos(state->position, 0); /* convert to external units not considering offset */
			///Debug_long5 = position_counter(SAVED_PARAMS);
			Device_status_word |= HOMING_ATTAINED;
			Device_status_word |= TARGET_REACHED_MASKBIT;
			zeroed = 1;
			active = 0;
			_LOGmessage(0x002A,"Homing attained",0 ,0);
		}
	}
	return zeroed;
}

/*!	\file drive_state_machine.c
	\brief Functions to handle the state machine of the servo
*/

#include "drive_state_machine.h"
#include "amc.h"


//
/*!	Function that processes a received control command and changes to a new state if neccesary
	\param c Received control command
	\param old_state Current state
*/
void processCommand(control_command c, drive_state old_state)
{
	printControlCommand(c);
	switch(c)
	{
	case SHUTDOWN:
		switch(old_state)
		{
		case SWITCH_ON_DISABLED:		/* transition 2 */
			setDriveState((unsigned short *)&Device_status_word, READY_TO_SWITCH_ON);
			break;
		case SWITCHED_ON:				/* transition 6 */
			setDriveState((unsigned short *)&Device_status_word, READY_TO_SWITCH_ON);
			break;
		case OPERATION_ENABLE:			/* transition 8 */
			ATM_seti(&ready_to_power_on, 0);
			if(ATM_seti(&manual_with_clutch, 0)) ATM_seti(&reset_filters, 1);
			setDriveState((unsigned short *)&Device_status_word, READY_TO_SWITCH_ON);
			break;
		case READY_TO_SWITCH_ON:		/* already in READY_TO_SWITCH_ON */
			break;
		case FAULT_REACTION_ACTIVE:
			ATM_seti(&ready_to_power_on, 0);
			break;
		default:
			_WARNINGmessage(0x1606, 0x20, Device_control_word, "Command '%d' unsupported from state '%d'", c, old_state);
			break;
		}
		break;

	case SWITCH_ON_DISABLE_OPERATION:
		switch(old_state)
		{
		case READY_TO_SWITCH_ON:		/* transition 3 */
			setDriveState((unsigned short *)&Device_status_word, SWITCHED_ON);
			break;
		case OPERATION_ENABLE:			/* transition 5 */
			if(ATM_seti(&manual_with_clutch, 0)) ATM_seti(&reset_filters, 1);
			ATM_seti(&ready_to_power_on, 0);
			setDriveState((unsigned short *)&Device_status_word, SWITCHED_ON);
			break;
		case SWITCHED_ON:				/* already in SWITCHED_ON */
			break;
		case FAULT_REACTION_ACTIVE:
			ATM_seti(&ready_to_power_on, 0);
			if(ATM_seti(&manual_with_clutch, 0)) ATM_seti(&reset_filters, 1);
			break;
		default:
			_WARNINGmessage(0x1607, 0x20, Device_control_word, "Command '%d' unsupported from state '%d'", c, old_state);
			break;
		}
		break;

//		case SWITCH_ON_ENABLE_OPERATION:
	case ENABLE_OPERATION:
		switch(old_state)
		{
		case SWITCHED_ON:				/* transition 4 */
		  //if( (FPGA_configuration_rd & 3) != WORKING_MODE_OPERATIONAL )
			//		return 0;
		  if (Sensors_configuration_active == 1)
			{
				if (SAFETY == SAFETY_ALLOW )
				{
					setDriveState((unsigned short *)&Device_status_word, OPERATION_ENABLE);
				}
				else
				{
					_WARNINGmessage(0xffd9, 0x80, 0x0000, "Safety signal is not active", 0, 0);
				}
			}
			else
			{
				_WARNINGmessage(0xfff4, 0x20, 0x0000, "Encoder configuration is not active", 0, 0);
			}
			break;

		case OPERATION_ENABLE:				/* already in OPERATION_ENABLE */
			break;

		default:
			_WARNINGmessage(0x1601, 0x20, Device_control_word, "Command '%d' unsupported from state '%d'", c, old_state);
			break;
		}
		break;

	case DISABLE_VOLTAGE:
		switch(old_state)
		{
		case READY_TO_SWITCH_ON:		/* transition 7 */
			setDriveState((unsigned short *)&Device_status_word, SWITCH_ON_DISABLED);
			break;
		case OPERATION_ENABLE:			/* transition 9 */
			ATM_seti(&ready_to_power_on, 0);
			if(ATM_seti(&manual_with_clutch, 0)) ATM_seti(&reset_filters, 1);
			setDriveState((unsigned short *)&Device_status_word, SWITCH_ON_DISABLED);
			break;
		case SWITCHED_ON:				/* transition 10 */
			setDriveState((unsigned short *)&Device_status_word, SWITCH_ON_DISABLED);
			break;
		case QUICK_STOP_ACTIVE:			/* transition 12 */
			ATM_seti(&ready_to_power_on, 0);
			if(ATM_seti(&manual_with_clutch, 0)) ATM_seti(&reset_filters, 1);
			setDriveState((unsigned short *)&Device_status_word, SWITCH_ON_DISABLED);
			break;
		case SWITCH_ON_DISABLED:				/* already in SWITCH_ON_DISABLED */
			break;
		case FAULT_REACTION_ACTIVE:
			ATM_seti(&ready_to_power_on, 0);
			if(ATM_seti(&manual_with_clutch, 0)) ATM_seti(&reset_filters, 1);
			break;
		default:
			_WARNINGmessage(0x1602, 0x20, Device_control_word, "Command '%d' unsupported from state '%d'", c, old_state);
			break;
		}
		break;

	case QUICK_STOP:
		switch(old_state)
		{
		case READY_TO_SWITCH_ON:		/* transition 7 */
			setDriveState((unsigned short *)&Device_status_word, SWITCH_ON_DISABLED);
			break;
		case SWITCHED_ON:				/* transition 10 */
			setDriveState((unsigned short *)&Device_status_word, SWITCH_ON_DISABLED);
			break;
		case OPERATION_ENABLE:			/* transition 11 */
			setDriveState((unsigned short *)&Device_status_word, QUICK_STOP_ACTIVE);
			break;
		case QUICK_STOP_ACTIVE:				/* do nothing */
		case SWITCH_ON_DISABLED:
			break;
		default:
			_WARNINGmessage(0x1603, 0x20, Device_control_word, "Command '%d' unsupported from state '%d'", c, old_state);
			break;
		}
		break;

	case FAULT_RESET:
		switch(old_state)
		{
		case FAULT:						/* transition 15 */
			if(!unrecoverableFaults()) setDriveState((unsigned short *)&Device_status_word, SWITCH_ON_DISABLED);
			break;
		default:
			_WARNINGmessage(0x1604, 0x20, Device_control_word, "Command '%d' unsupported from state '%d'", c, old_state);
			break;
		}
		break;

	case HALT:
		switch(old_state)
		{
		case OPERATION_ENABLE:			/* Stop the motor with profile deceleration */
			ATM_seti(&init_mode, 1);	/* mode of operation has to be initialized */
			break;
		default:
			_WARNINGmessage(0x1605, 0x20, Device_control_word, "Command '%d' unsupported from state '%d'", c, old_state);
			break;
		}
		break;

	default:
		_WARNINGmessage(0xfff5, 0x20, Device_control_word, "Command '%d' unimplemented", c, 0);
		break;
	}
}


/*!	Function that converts received ControlWord in a control command
	\param controlword The entry of the Object Dictionary that contains received ControlWord
	\return A member of control_command enum
*/
control_command getCommand(unsigned short int controlword)
{
	if(controlword & FAULT_RESET_MASKBIT) return FAULT_RESET;
	else if(~controlword & ENABLE_VOLTAGE_MASKBIT) return DISABLE_VOLTAGE;
	else if(~controlword & QUICK_STOP_MASKBIT) return QUICK_STOP;
	else if(~controlword & SWITCH_ON_MASKBIT) return SHUTDOWN;
//	else if(controlword & ENABLE_OPERATION_MASKBIT) return SWITCH_ON_ENABLE_OPERATION;
	else if(controlword & ENABLE_OPERATION_MASKBIT)
		{
			if(controlword & HALT_MASKBIT) return HALT;
			/* else if(controlword & MODE_SPECIFIC_BIT4_MASKBIT) return MODE_SPECIFIC_BIT4; */
			else return ENABLE_OPERATION;
		}
	else return SWITCH_ON_DISABLE_OPERATION;
}

/*!	Function that converts StatusWord in a member of drive_state enum
	\param statusword The entry of the Object Dictionary that contains the StatusWord
	\return A member of drive_state enum
*/
drive_state getDriveState(unsigned short int statusword)
{
	switch(statusword & STATEWORD_MASK1)
	{
		case NOT_READY_TO_SWITCH_ON_BITS: return NOT_READY_TO_SWITCH_ON;
		case SWITCH_ON_DISABLED_BITS: return SWITCH_ON_DISABLED;
		case FAULT_REACTION_ACTIVE_BITS: return FAULT_REACTION_ACTIVE;
		case FAULT_BITS: return FAULT;
	}
	switch(statusword & STATEWORD_MASK2)
	{
		case READY_TO_SWITCH_ON_BITS: return READY_TO_SWITCH_ON;
		case SWITCHED_ON_BITS: return SWITCHED_ON;
		case OPERATION_ENABLE_BITS: return OPERATION_ENABLE;
		case QUICK_STOP_ACTIVE_BITS: return QUICK_STOP_ACTIVE;
	}
	return STATE_ERROR;		// if no one state matches
}

/*!	Function that sets a new state in StatusWord
	\param p_statusword Pointer to the StatusWord to modify
	\param new_state The new state
	\return 1 if successful or 0 if unknown state
*/
int setDriveState(unsigned short int *p_statusword, drive_state new_state)
{
	if(new_state == STATE_ERROR) return 0;		// state not changed

	switch(new_state)
	{
		case START:
			_LOGmessage(0x000F,"START is only initial state", 0, 0);
			return 0;			// state not changed
			//break;
		case NOT_READY_TO_SWITCH_ON:
			DINT;			/* Disable Global interrupt INTM */
			*p_statusword = ((*p_statusword & ~STATEWORD_MASK1) | NOT_READY_TO_SWITCH_ON_BITS);
			EINT;			/* Enable interrupts again */
			break;
		case SWITCH_ON_DISABLED:
			DINT;			/* Disable Global interrupt INTM */
			*p_statusword = ((*p_statusword & ~STATEWORD_MASK1) | SWITCH_ON_DISABLED_BITS);
			EINT;			/* Enable interrupts again */
			break;
		case READY_TO_SWITCH_ON:
			DINT;			/* Disable Global interrupt INTM */
			*p_statusword = ((*p_statusword & ~STATEWORD_MASK2) | READY_TO_SWITCH_ON_BITS);
			EINT;			/* Enable interrupts again */
			break;
		case SWITCHED_ON:
			DINT;			/* Disable Global interrupt INTM */
			*p_statusword = ((*p_statusword & ~STATEWORD_MASK2) | SWITCHED_ON_BITS);
			EINT;			/* Enable interrupts again */
			break;
		case OPERATION_ENABLE:
			DINT;			/* Disable Global interrupt INTM */
			*p_statusword = ((*p_statusword & ~STATEWORD_MASK2) | OPERATION_ENABLE_BITS);
			EINT;			/* Enable interrupts again */
			break;
		case QUICK_STOP_ACTIVE:
			DINT;			/* Disable Global interrupt INTM */
			*p_statusword = ((*p_statusword & ~STATEWORD_MASK2) | QUICK_STOP_ACTIVE_BITS);
			EINT;			/* Enable interrupts again */
			break;
		case FAULT_REACTION_ACTIVE:
			DINT;			/* Disable Global interrupt INTM */
			*p_statusword = ((*p_statusword & ~STATEWORD_MASK1) | FAULT_REACTION_ACTIVE_BITS);
			EINT;			/* Enable interrupts again */
			break;
		case FAULT:
			DINT;			/* Disable Global interrupt INTM */
			*p_statusword = ((*p_statusword & ~STATEWORD_MASK1) | FAULT_BITS);
			EINT;			/* Enable interrupts again */
			break;
		default:
			return 0;		// unknown state
			//break;
	}
	printDriveState(*p_statusword);

	/* send txPDO 1 (StatusWord) */
	set_fault_flags();
	set_brakes_flags();
	send_controlword();

	return 1;
}

/*!	Prints current status in log 'trace'
	\param statusword The OD entry StatusWord
*/
void printDriveState(unsigned short int statusword)
{
	switch(getDriveState(statusword))
	{
		case STATE_ERROR: _LOGmessage(0x0010,"Statusword: 0x%4.4x\t State: %s",statusword,"STATE_ERROR"); break;
		case START: _LOGmessage(0x0011,"Statusword: 0x%4.4x\t State: %s",statusword,"START"); break;
		case NOT_READY_TO_SWITCH_ON: _LOGmessage(0x0012,"Statusword: 0x%4.4x\t State: %s",statusword,"NOT_READY_TO_SWITCH_ON"); break;
		case SWITCH_ON_DISABLED: _LOGmessage(0x0013,"Statusword: 0x%4.4x\t State: %s",statusword,"SWITCH_ON_DISABLED"); break;
		case READY_TO_SWITCH_ON: _LOGmessage(0x0014,"Statusword: 0x%4.4x\t State: %s",statusword,"READY_TO_SWITCH_ON"); break;
		case SWITCHED_ON: _LOGmessage(0x0015,"Statusword: 0x%4.4x\t State: %s",statusword,"SWITCHED_ON"); break;
		case OPERATION_ENABLE: _LOGmessage(0x0016,"Statusword: 0x%4.4x\t State: %s",statusword,"OPERATION_ENABLE"); break;
		case QUICK_STOP_ACTIVE: _LOGmessage(0x0017,"Statusword: 0x%4.4x\t State: %s",statusword,"QUICK_STOP_ACTIVE"); break;
		case FAULT_REACTION_ACTIVE: _LOGmessage(0x0018,"Statusword: 0x%4.4x\t State: %s",statusword,"FAULT_REACTION_ACTIVE"); break;
		case FAULT: _LOGmessage(0x0019,"Statusword: 0x%4.4x\t State: %s",statusword,"FAULT"); break;
	default: _LOGmessage(0x001A,"Unknown state", 0, 0); break;
	}
}

/*!	returns de controlword that has to be sent to the drive for a given control command
	\param c Control command that wants to be sent
	\return ControlWord to be written to the OD
*/
unsigned short int setCommand(control_command c)
{
	switch (c)
	{
		case SHUTDOWN: return 0x0006;
		case SWITCH_ON_DISABLE_OPERATION: return 0x0007;
		case ENABLE_OPERATION: return 0x000F;
		case DISABLE_VOLTAGE: return 0x0000;
		case QUICK_STOP: return 0x0002;
		case FAULT_RESET: return 0x0080;
	}
	return 0;
}

/*!	Prints a control command in log 'trace'
	\param cmd command to be printed
*/
void printControlCommand(control_command cmd)
{
	switch(cmd)
	{
		case SHUTDOWN: _LOGmessage(0x001B,"Received ControlWord:  SHUTDOWN", 0, 0); break;
		case SWITCH_ON_DISABLE_OPERATION: _LOGmessage(0x001C,"Received ControlWord:  SWITCH_ON_DISABLE_OPERATION", 0, 0); break;
		case ENABLE_OPERATION: _LOGmessage(0x001D,"Received ControlWord:  ENABLE_OPERATION", 0, 0); break;
		case DISABLE_VOLTAGE: _LOGmessage(0x001E,"Received ControlWord:  DISABLE_VOLTAGE", 0, 0); break;
		case QUICK_STOP: _LOGmessage(0x001F,"Received ControlWord:  QUICK_STOP", 0, 0); break;
		case FAULT_RESET: _LOGmessage(0x0020,"Received ControlWord:  FAULT_RESET", 0, 0); break;
		case HALT: _LOGmessage(0x0021,"Received ControlWord:  HALT", 0, 0); break;
		/* case MODE_SPECIFIC_BIT4: _LOGmessage(0x0022,"Received ControlWord:  MODE_SPECIFIC_BIT4", 0, 0); break; */
	default: _LOGmessage(0x0023,"Received unknown command", 0, 0); break;
	}
}


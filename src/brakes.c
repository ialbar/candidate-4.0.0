/*!	\file brakes.c
	\brief Functions to handle the brakes
	
	There are three brakes in AMC, each of them is configured using an entry in the OD (0x2101 for Brake1, 0x2102 for Brake2 and 0x2013 for Brake3). These entries have the next structure:\n
	\li \e Mode: Mode of operation of the brake (explained later)
	\li \e Start: Enables or disables the brake (0,1)
	\li \e Current: Brake current expressed in mA.
	\li <em>Fixed duty 1</em>: Fixed duty cycle during first \c Time_duty1 ms in \c mode 2 (expressed in values per thousand).
	\li <em>Fixed duty 2</em>: Fixed dity cycle after \c Time_duty1 ms in \c mode 2 (expressed in values per thousand).
	\li <em>Time duty1</em>: Time in ms (0-65535) for first period of \c mode 2.
	\li <em>Duty cycle</em>: Duty cycle applied to the brake in \c mode 3.
	
	There are three modes of operation selected in \c mode:\n
	\li <em> Mode 1: ON/OFF</em> Brake is enabled or disabled with Start bit. Vpower is applied to the brake.
	\li <em> Mode 2: Timed duty1/duty2</em>: when started (also using \c Start bit) Duty1 is applied to the brake for \c Time_duty1 ms, and later Duty2 is applied to the brake.
	\li <em> Mode 3: Direct PWM</em>: When started \c Duty_cycle is applied to the brake.	
*/

#include "DSP2833x_Device.h"
#include "amc.h"

static void brake1_operation(void);
static void brake2_operation(void);

/*****************************************************************/
/*!	Function that controls brakes	*/
/*****************************************************************/
void set_brakes_flags(void)
{
	if( ( Axle_brake_Position > 0 ) && ( Axle_brake_Position <= 2 ) && Axle_brake_Start )
		Device_status_word |= BRAKE_MASKBIT;	/* set brake flag */
	else
		Device_status_word &= ~BRAKE_MASKBIT;	/* clear brake flag */
}


void brakes_operation(void)
{
	brake1_operation();
	brake2_operation();
}


/*****************************************************************/
/*!	Function that controls Brake1 as described in brakes.c
	\param current_time Current time	*/
/*****************************************************************/
static void brake1_operation(void)
{
	/* check if brake1 is used as axle brake or as cluth brake */
	if( Axle_brake_Position == 1 ) Brake1_Start = Axle_brake_Start ^ Axle_brake_Polarity;
	else if( Axle_clutch_Position == 1 ) Brake1_Start = Axle_clutch_Start ^ Axle_clutch_Polarity;

	if (Brake1_Start)
	{
		switch(Brake1_Mode)
		{
			case 0:
				/* EPwm6A forced low. Brake 1 turned off */
				EPwm6Regs.AQCTLA.bit.ZRO = AQ_CLEAR;
				EPwm6Regs.AQCTLA.bit.CAU = AQ_CLEAR;
				EPwm6Regs.AQCTLA.bit.PRD = AQ_CLEAR;
				break;
			case 1:
				/* EPwm6A forced high. Brake 1 turned on*/
				EPwm6Regs.AQCTLA.bit.ZRO = AQ_SET;
				EPwm6Regs.AQCTLA.bit.CAU = AQ_SET;
				EPwm6Regs.AQCTLA.bit.PRD = AQ_SET;
				break;
			default:
				_WARNINGmessage(0xfffd, 0x80, 1, "Unavailabe Brake1 Mode", 0, 0);
				break;
		}
	}
	else
	{
		/* EPwm6A forced low. Brake 1 turned off */
		EPwm6Regs.AQCTLA.bit.ZRO = AQ_CLEAR;
		EPwm6Regs.AQCTLA.bit.CAU = AQ_CLEAR;
		EPwm6Regs.AQCTLA.bit.PRD = AQ_CLEAR;
	}
}


/*****************************************************************/
/*!	Function that controls Brake2 as described in brakes.c
	\param current_time Current time	*/
/*****************************************************************/
static void brake2_operation(void)
{
	/* check if brake2 is used as axle brake or as axle clutch */
	if( Axle_brake_Position == 2 ) Brake2_Start = Axle_brake_Start ^ Axle_brake_Polarity;
	else if( Axle_clutch_Position == 2 ) Brake2_Start = Axle_clutch_Start ^ Axle_clutch_Polarity;
	
	if (Brake2_Start)
	{
		switch(Brake2_Mode)
		{
			case 0:
				/* EPwm6B forced low. Brake 2 turned off */
				EPwm6Regs.AQCTLB.bit.ZRO = AQ_CLEAR;
				EPwm6Regs.AQCTLB.bit.CBU = AQ_CLEAR;
				EPwm6Regs.AQCTLB.bit.PRD = AQ_CLEAR;
				break;
			case 1:
				/* EPwm6B forced high. Brake 2 turned on*/
				EPwm6Regs.AQCTLB.bit.ZRO = AQ_SET;
				EPwm6Regs.AQCTLB.bit.CBU = AQ_SET;
				EPwm6Regs.AQCTLB.bit.PRD = AQ_SET;
				break;
			default:
				_WARNINGmessage(0xfffd, 0x80, 2, "Unavailabe Brake2 Mode", 0, 0);
				break;
		}
	} else
	{
		/* EPwm6B forced low. Brake 2 turned off */
		EPwm6Regs.AQCTLB.bit.ZRO = AQ_CLEAR;
		EPwm6Regs.AQCTLB.bit.CBU = AQ_CLEAR;
		EPwm6Regs.AQCTLB.bit.PRD = AQ_CLEAR;
	}
}


/*****************************************************************/
/*!	Function that activates axle brake (if any)
	\param current_time Current time	*/
/*****************************************************************/
void activate_brake(void)
{
	Axle_brake_Start = 1;
}


/*****************************************************************/
/*!	Function that releases axle brake (if any)
	\param current_time Current time	*/
/*****************************************************************/
void release_brake(void)
{
	Axle_brake_Start = 0;	
}

/*****************************************************************/
void activate_clutch(void)
{
	Axle_clutch_Start = 1;
}


/*****************************************************************/
/*!	Function that releases axle brake (if any)
	\param current_time Current time	*/
/*****************************************************************/
void release_clutch(void)
{
	Axle_clutch_Start = 0;		
}

/*!	\file pid_reg.c
	\brief The PID controller with anti-windup

=====================================================================================

 File name:        PID_REG.C  (IQ version)

 Originator:	Digital Control Systems Group
			Texas Instruments

 Description:  The PID controller with anti-windup

=====================================================================================

 History:
 16/03/2007
 Modified by Luis Jimenez for making gain parameters really match PID parameter (not affected by control period)
as a drawback, now there is to specify control period and frequency.

-------------------------------------------------------------------------------------

based on:
 04-15-2005	Version 3.20 (of old pid_reg3.h)

------------------------------------------------------------------------------*/

#include "IQmathLib.h"		/* Include header for IQmath library */
							/* Don't forget to set a proper GLOBAL_Q in "IQmathLib.h" file */
#include "dmctype.h"
#include "pid_reg.h"


/*!	Calculates the output of the PID handled by \a v
	\param v The handler of the PID
*/
void pid_reg_calc(PIDREG *v)
{
	/* Compute the error */
	v->Err = v->Ref - v->Fdb;

	/* Compute the proportional output */
	v->Up = _IQmpy(v->Kp,v->Err);

	/* Compute the integral output */
	v->Ui = v->Ui + _IQmpy((_IQmpy(v->Ki,v->Err) + _IQmpy(v->Kc,v->SatErr)),v->Period);

	/* Compute the derivative output */
	v->Ud = _IQmpyI32(_IQmpy(v->Kd,(v->Err - v->Err1)),v->Frequency);

	/* Compute the pre-saturated output */
	v->OutPreSat = v->Up + v->Ui + v->Ud;

	/* Saturate the output */
	v->Out = _IQsat(v->OutPreSat, v->OutMax, v->OutMin);

	/* Compute the saturate difference */
	v->SatErr = v->Out - v->OutPreSat;

	/* Update the previous error */
	v->Err1 = v->Err;
}

/*!	Calculates the output of the PID handled by \a v
	\param v The handler of the PID
*/
void pid_reg_calc_iq16(PIDREG16 *v)
{
	/* Compute the error */
	v->Err = v->Ref - v->Fdb;

	/* Compute the proportional output */
	v->Up = _IQ16mpy(v->Kp,v->Err);

	/* Compute the integral output */
	v->Ui = v->Ui + _IQ16mpy((_IQ16mpy(v->Ki,v->Err) + _IQ16mpy(v->Kc,v->SatErr)),v->Period);

	/* Compute the derivative output */
	v->Ud = _IQ16mpyI32(_IQ16mpy(v->Kd,(v->Err - v->Err1)),v->Frequency);

	/* Compute the pre-saturated output */
	v->OutPreSat = v->Up + v->Ui + v->Ud;

	/* Saturate the output */
	v->Out = _IQsat(v->OutPreSat, v->OutMax, v->OutMin);

	/* Compute the saturate difference */
	v->SatErr = v->Out - v->OutPreSat;

	/* Update the previous error */
	v->Err1 = v->Err;
}



/*!	Calculates the output of the PID handled by \a v
	\param v The handler of the PID
*/
void pid_controller_calc(PIDCONTROLLER *v)
{
	/* Compute the error */
	v->Err = v->Ref - v->Fdb;

	/* Compute the proportional output */
	v->Up = _IQmpy(v->Kp,v->Err);

	/* Compute the integral output */
	v->Ui = v->Ui + _IQmpy((_IQmpy(v->Ki,v->Err) + _IQmpy(v->Kc,v->SatError)),v->Period);

	/* Compute the derivative output */
	v->Ud = _IQmpyI32(_IQmpy(v->Kd,(v->Err - v->Err1)),v->Frequency);

	/* Compute the feed-forward output */
	v->Uff = _IQmpy(v->Kff,v->FeedForward);

	/* Compute the output */
	v->Out = v->Up + v->Ui + v->Ud + v->Uff;

	/* Compute the saturate difference */
	v->SatError = v->AntiWindup - v->Out;

	/* Update the previous error */
	v->Err1 = v->Err;
}

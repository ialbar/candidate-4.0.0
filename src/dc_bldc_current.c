#include "amc.h"
#include "fpga.h"
#include "debug_sci.h"
#include "dc_bldc_current.h"
//#define OPEN_LOOP

PIDREG current_control_pid = PIDREG_DEFAULTS;	/*!< PID struct for position control */

_iq current_controller_dc_bldc(_iq Ref, _iq Fdb)
{
	static unsigned int send_17Kkhz_div_2=0;
	current_control_pid.Ref = _IQsat(Ref, _IQdiv(_IQ(Max_sat_current),_IQ(100.0)), _IQdiv(_IQ(Min_sat_current),_IQ(-100.0)));
	current_control_pid.Fdb = Fdb;
	current_control_pid.calc(&current_control_pid);		/* calculate PID output */


#ifdef DEBUG_CURRENT_LOOP_BLDC
	// Enviamos una de cada dos, porque debug_sci no da para enviar 8 bytes a 17Khz
	if (send_17Kkhz_div_2 == 0)
	{
		TickDebugSci14bytesCurrentBLDC();
		send_17Kkhz_div_2++;
	}
	else
		send_17Kkhz_div_2=0;
#endif

#ifdef 	OPEN_LOOP
	return current_control_pid.Ref;
#else if
	return current_control_pid.Out;
#endif
}


void init_current_controller_dc_bldc(void)
{
	current_control_pid.Ref = 0;
	current_control_pid.Up = 0;
	current_control_pid.Ui = 0;
	current_control_pid.Ud = 0;
	current_control_pid.OutPreSat = 0;
	current_control_pid.Out = 0;
	current_control_pid.SatErr = 0;
	current_control_pid.Err1 = 0;
	current_control_pid.OutMax = _IQdiv(_IQ(Max_sat_pwm_duty),_IQ(100.0));
	current_control_pid.OutMin = _IQdiv(_IQ(Min_sat_pwm_duty),_IQ(-100.0));

	current_control_pid.Period = _IQ(1.0);//_IQ(0.00005882353);
	current_control_pid.Frequency = 1;//17000;

}

void set_Kp_current_control_dc_bldc(uint16_t Kp,uint16_t divisor)
{
	unsigned long ul_tmp;
	ul_tmp = Kp;
	ul_tmp = ul_tmp << 16;							/* multiply by 2^16 */
	ul_tmp /= divisor;			/* divide by Divisor */
	current_control_pid.Kp = ul_tmp << (GLOBAL_Q - 16);		/* convert to IQ and divide by 2^16 */

}
void set_Ki_current_control_dc_bldc(uint16_t Ki,uint16_t divisor)
{
	unsigned long ul_tmp;
	ul_tmp = Ki;
	ul_tmp = ul_tmp << 16;							/* multiply by 2^16 */
	ul_tmp /= divisor;			/* divide by Divisor */
	current_control_pid.Ki = ul_tmp << (GLOBAL_Q - 16);		/* convert to IQ and divide by 2^16 */
}

void set_Kd_current_control_dc_bldc(uint16_t Kd,uint16_t divisor)
{
	unsigned long ul_tmp;
	ul_tmp = Kd;
	ul_tmp = ul_tmp << 16;							/* multiply by 2^16 */
	ul_tmp /= divisor;			/* divide by Divisor */
	current_control_pid.Kd = ul_tmp << (GLOBAL_Q - 16);		/* convert to IQ and divide by 2^16 */
}

void set_Kc_current_control_dc_bldc(uint16_t Kc,uint16_t divisor)
{
	unsigned long ul_tmp;
	ul_tmp = Kc;
	ul_tmp = ul_tmp << 16;							/* multiply by 2^16 */
	ul_tmp /= divisor;			/* divide by Divisor */
	current_control_pid.Kc = ul_tmp << (GLOBAL_Q - 16);		/* convert to IQ and divide by 2^16 */
}

void init_current_controller_dc_bldc_from_assited_mode(void)
{
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
}

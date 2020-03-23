/*!	\file vectorial_current_mode.c
	\brief Functions for Vectorial Current analisys and control
*/

#include "amc.h"
#include "fpga.h"
#include "debug_sci.h"

_iq15 cheli;
/*! Vectorial components of instantaneous currents (_IQ15, mA)*/
_iq Id = 0, Iq = 0, modI = 0;
_iq Ud = 0, Uq = 0;
_iq Id_ref_off = 0, Iq_ref_off = 0;
_iq Id_ref = 0, Iq_ref = 0;
_iq Ud_ref_off = 0, Uq_ref_off = 0;
_iq Ud_ref = 0, Uq_ref = 0;
_iq sin, cos;
_iq duty_a, duty_b, duty_c;

/* Vectorial Current control Parameters */
/* Kp, Ki & Kff are divided by 10, because of use Udc in hundredths of volts instead
 of mV (like current, in mA) */
//TODO AMQ: study to move to PIDREG struct
_iq Kp  = _IQCurrent(0.0);
_iq Ki  = _IQCurrent(0.0);
_iq Kc  = _IQCurrent(0.0);
_iq Kff = _IQCurrent(0.0);
_iq Period = _IQCurrent(0.0000625); /* Fs=16KHz, Ts=62.5us */

_iq Up_d=0, Ui_d=0, SatErr_d=0;
_iq Up_q=0, Ui_q=0, SatErr_q=0;

_iq Ref_iq_jaf;
_iq Iq_ref_jaf;

#ifdef VECT_CONTROL_DEBUG
#pragma DATA_SECTION( index_vect_control_debug, "storedata" );
#pragma DATA_SECTION( vect_control_buffer1, "storedata" );
#pragma DATA_SECTION( vect_control_buffer2, "storedata" );
#pragma DATA_SECTION( vect_control_buffer3, "storedata" );
#pragma DATA_SECTION( vect_control_buffer4, "storedata" );
#pragma DATA_SECTION( vect_control_buffer5, "storedata" );
#pragma DATA_SECTION( vect_control_buffer6, "storedata" );
int vect_control_buffer1[VECT_CONTROL_BUFFER_LEN];
int vect_control_buffer2[VECT_CONTROL_BUFFER_LEN];
int vect_control_buffer3[VECT_CONTROL_BUFFER_LEN];
int vect_control_buffer4[VECT_CONTROL_BUFFER_LEN];
int vect_control_buffer5[VECT_CONTROL_BUFFER_LEN];
int vect_control_buffer6[VECT_CONTROL_BUFFER_LEN];
unsigned int index_vect_control_debug=0;
#endif

/*****************************************************************/
/*!	 Main function of Vectorial Current Analisys
	\param param1 tbd
*/
/*****************************************************************/
void vc_analisys(_iq resolver_angle)
{

		/* Vectorial current analysis variables */
		_iq Ia=0, Ib=0, Ic=0, Ialpha=0, Ibeta=0;  //_IQCurrent, mA
		_iq theta;

		/* Vectorial current analysis */
		DINT;
		Ia = ((long)current1) << Q_CURRENT;
		Ib = ((long)current2) << Q_CURRENT;
		Ic = ((long)current3) << Q_CURRENT;
		EINT;
		/* Power Invariant Transformation */
		Ialpha = _IQ30mpy(_IQ30(sqrt_TwoThree),Ia) - _IQ30mpy(_IQ30(sqrt_OneSix),Ib+Ic);
		Ibeta  = _IQ30mpy(_IQ30(sqrt_OneTwo)  ,Ib-Ic);
		theta = (resolver_angle + _IQ(0.25)) & (_IQ(1.0)-1); /* Oriented in q axis */
		sin = _IQ30toIQ(*(long *)(0x3FE000 + (((theta + (1 << (GLOBAL_Q - 9 - 1)) >> GLOBAL_Q - 9) & 0x1ff) << 1)));
		theta = (resolver_angle ) & (_IQ(1.0)-1);
		cos = - _IQ30toIQ(*(long *)(0x3FE000 + (((theta + (1 << (GLOBAL_Q - 9 - 1)) >> GLOBAL_Q - 9) & 0x1ff) << 1)));;
		// theta, sin & cos are GLOBAL_Q
		DINT; //Access to Id & Iq, global variables (periodic function access to publicate)
		Id =  _IQmpy( cos, Ialpha) + _IQmpy(sin, Ibeta); // Id in Q_CURRENT, mA
		Iq =  _IQmpy(-sin, Ialpha) + _IQmpy(cos, Ibeta); // Iq in Q_CURRENT, mA
		EINT;

#ifdef _VECT_CONTROL_DEBUG
{
	static unsigned long Udc_fil=0;
	vect_control_buffer1[index_vect_control_debug] = Ia >> Q_CURRENT;
	vect_control_buffer2[index_vect_control_debug] = Ic >> Q_CURRENT;
	vect_control_buffer3[index_vect_control_debug] = Id >> Q_CURRENT;
	vect_control_buffer4[index_vect_control_debug] = Iq >> Q_CURRENT;
	vect_control_buffer5[index_vect_control_debug] = Ud >> Q_CURRENT;
	vect_control_buffer6[index_vect_control_debug] = Uq >> Q_CURRENT;
	index_vect_control_debug++;
	index_vect_control_debug = (index_vect_control_debug==VECT_CONTROL_BUFFER_LEN?VECT_CONTROL_BUFFER_LEN-1:index_vect_control_debug);
		}
#endif

}

#define _IQ2UINT16(x) ((x>0?x<<1:x^0xFFFFFFFF+1) >> 16)
/*inline unsigned int iq2dacval(_IQ x)
{
	int aux;
	aux = x >> 20;
	return ((int)(aux + 0x07FF));
}*/

#define iq2dacval(x) (unsigned int)((int)((x>>20)+0x07FF))
#define iqcurrent2dacval(x) (unsigned int)((int)((x>>18)+0x07FF))
/*****************************************************************/
/*!	 Function that execute the Vectorial Current Control
	\param cc_ref Current control reference input
	\param ss_offset Spread Spectrum offset
*/
/*****************************************************************/
void vc_control(_iq cc_ref, int ss_offset)
{
					_iq Err_d=0, Up_d=0;
					//static _iq Ui_d=0, SatErr_d=0;
					_iq Err_q=0, Up_q=0;
					//static _iq Ui_q=0, SatErr_q=0;
					_iq Ref_Iq, Ref_Id;
					_iq Ualpha, Ubeta, aux;
					_iq Ua, Ub, Uc, Udq_sat;
					static _iq Udc=0;
					//_iq duty_a, duty_b, duty_c; TODO AMQ: return to the function
					//static int cicle_count = 0;

					DINT;

			        Udc = ((unsigned long)voltage) << Q_CURRENT; //update only in periodic control function (Filtered variable)
					Iq_ref = _IQ9mpy(_IQsat(cc_ref,_IQ(1.0),_IQ(-1.0)), current_limit); // (IQ24+IQ0)-IQ9=IQ15 (IQCurrent)
					Iq_ref_jaf = Iq_ref;
//					Iq_ref_jaf = _IQ15toIQ(_IQ15mpy(_IQtoIQ15(_IQsat(cc_ref,_IQ(1.0),_IQ(-1.0))), _IQ15(current_limit)));
//					cheli = _IQ15mpy(_IQtoIQ15(_IQsat(_IQ(0.25),_IQ(1.0),_IQ(-1.0))), _IQ15(current_limit)); // (IQ24+IQ0)-IQ9=IQ15 (IQCurrent)



					Ref_Iq = Iq_ref_off + Iq_ref;  //Add offset current in Iq
					Ref_Iq = _IQsat(Ref_Iq, _IQCurrent(current_limit), _IQCurrent(-current_limit));
					Ref_iq_jaf = Ref_Iq;
					Ref_Id = Id_ref_off + Id_ref;
					Ref_Id = _IQsat(Ref_Id, _IQCurrent(current_limit), _IQCurrent(-current_limit));

					Err_d = Ref_Id - Id;
					Err_q = Ref_Iq - Iq;

					EINT;

					/* Udq (loop output) saturation */
					// Udq_sat will be saturate to 32768 to avoid overflows in the Integral part (Ui)
					Udq_sat = _IQmpy_Current(Udc,_IQCurrent(0.57735026919)); /* |Udq| < Udc / sqrt(3) */

					/* d-axis control loop */
					Up_d = _IQsmpy_Current(Kp,Err_d); //multiply with saturation in IQmax/2. See macro
					Ui_d = _IQsat(Ui_d + _IQsmpy_Current(Ki,Err_d),Udq_sat,-Udq_sat);
					SatErr_d = _IQsat((Up_d + Ui_d),Udq_sat,-Udq_sat);
					//SatErr_d = 0;  // Open d-axis control loop
					/* q-axis control loop */
					Up_q = _IQsmpy_Current(Kp,Err_q);
					Ui_q = _IQsat(Ui_q + _IQsmpy_Current(Ki,Err_q),Udq_sat,-Udq_sat);
					SatErr_q = _IQsat((Up_q + Ui_q),Udq_sat,-Udq_sat);
					//SatErr_q = _IQsmpy_Current(Ref_Iq,_IQCurrent(10.0));// Open d-axis control loop

					DINT;
					Ud = SatErr_d;
					Uq = SatErr_q;
					EINT;

					Ualpha = _IQmpy(cos, Ud) + _IQmpy(-sin, Uq); // Ualpha in Q_CURRENT, mV
					Ubeta  = _IQmpy(sin, Ud) + _IQmpy( cos, Uq); // Ubeta in Q_CURRENT, mV

					Ua =   _IQ30mpy(_IQ30(sqrt_TwoThree), Ualpha);
					Ub = - _IQ30mpy(_IQ30(sqrt_OneSix),   Ualpha) + _IQ30mpy(_IQ30(sqrt_OneTwo), Ubeta);
					Uc = - _IQ30mpy(_IQ30(sqrt_OneSix),   Ualpha) - _IQ30mpy(_IQ30(sqrt_OneTwo), Ubeta);

					aux = _IQ1div(_IQDuty(1.0),Udc>> Q_CURRENT); //IQ1div include 1/2 for Udc
					duty_a = _IQmpy_Current( Ua, aux); // duty_a = [-1,1], _IQ30. IQCurrent(15)+IQ30-IQCurrent(15)=IQ30
					duty_b = _IQmpy_Current( Ub, aux); // duty_b = [-1,1], _IQ30
					duty_c = _IQmpy_Current( Uc, aux); // duty_c = [-1,1], _IQ30

					duty_a = _IQsat(duty_a,_IQDuty(1.0),_IQDuty(-1.0));
					duty_b = _IQsat(duty_b,_IQDuty(1.0),_IQDuty(-1.0));
					duty_c = _IQsat(duty_c,_IQDuty(1.0),_IQDuty(-1.0));

					ENABLE_PWM1(_IQmpyInt_Duty(_IQmpy_Duty(_IQDuty(1.0) - duty_a, _IQDuty(0.5)),Half_Period + ss_offset));
					ENABLE_PWM2(_IQmpyInt_Duty(_IQmpy_Duty(_IQDuty(1.0) - duty_b, _IQDuty(0.5)),Half_Period + ss_offset));
					ENABLE_PWM3(_IQmpyInt_Duty(_IQmpy_Duty(_IQDuty(1.0) - duty_c, _IQDuty(0.5)),Half_Period + ss_offset));
					//TickDebugSci8bytesPositionMode();

}


/*!	\file interpolated_mode.h
	\brief Header file containing declarations for Interpolated Position Mode
*/

#ifndef _VECTORIAL_CURRENT_MODE_H_
#define _VECTORIAL_CURRENT_MODE_H_

/*constants for vectorial transformations */
#define sqrt_TwoThree	0.816496580928 // sqrt(2/3)
#define	sqrt_OneTwo	0.707106781187 // 1/sqrt(2)
#define	sqrt_OneSix	0.408248290464 // 1/sqrt(6)

/* IQ format for vectorial operations */
#define Q_CURRENT 15
#define _IQCurrent(A) 			_IQ15(A)
#define _IQmpy_Current(A,B) 	_IQ15mpy(A,B)
#define _IQsmpy_Current(A,B) 	(_IQ16rsmpy(A,B)>>1)
#define _IQsqrt_Current(A) 		_IQ15sqrt(A)
#define _IQdiv_Current(A,B)		_IQ15div(A,B)
#define _IQCurrent2Int(A)		_IQ15int(A)

/* IQ format for BLAC duty cycle calculations */
#define Q_DUTY 30

/* Operation to calculate BLAC_DUTY: IQCurrent(15)+IQGetDuty(20)-IQdiv_GetDuty(5) = IQDuty(30) */
#define _IQGetDuty(A)				_IQ20(A)
#define _IQdiv_GetDuty(A,B)			_IQ5div(A,B)
#define _IQDuty(A)					_IQ30(A)
#define _IQmpy_Duty(A,B)			_IQ30mpy(A,B)
#define _IQmpyInt_Duty(A,B)			_IQ30mpyI32int(A,B)

/* extern variables */
extern _iq modI; //TODO: review
extern _iq duty_a;		/*!< Vectorial Current Control Duty cycle, applied in motor phase A to be monitorized */
extern _iq duty_b;		/*!< Vectorial Current Control Duty cycle applied in motor phase B to be monitorized */
extern _iq duty_c;		/*!< Vectorial Current Control Duty cycle applied in motor phase C to be monitorized */

extern _iq Ref_iq_jaf;
extern _iq Iq_ref_jaf;

#define VECT_CONTROL_DEBUG
//uncomment to store the data read of the resolver.
#ifdef VECT_CONTROL_DEBUG
//#define VECT_CONTROL_BUFFER_LEN 4096
#define VECT_CONTROL_BUFFER_LEN 8192
extern int vect_control_buffer1[VECT_CONTROL_BUFFER_LEN];
extern int vect_control_buffer2[VECT_CONTROL_BUFFER_LEN];
extern int vect_control_buffer3[VECT_CONTROL_BUFFER_LEN];
extern int vect_control_buffer4[VECT_CONTROL_BUFFER_LEN];
extern int vect_control_buffer5[VECT_CONTROL_BUFFER_LEN];
extern int vect_control_buffer6[VECT_CONTROL_BUFFER_LEN];
extern unsigned int index_vect_control_debug;
#endif

/* function declarations */
void vc_analisys(_iq);
void vc_control(_iq, int);

extern _iq15 cheli;

extern _iq Up_d, Ui_d, SatErr_d;
extern _iq Up_q, Ui_q, SatErr_q;

#endif  /* end _VECTORIAL_CURRENT_MODE_H_ definition */

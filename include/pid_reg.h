/*!	\file pid_reg.h
	\brief Header file containing constants, data type definitions, and function prototypes for the PIDREG.

=====================================================================================

File name: PID_REG:H  (IQ version)

Originator:	Digital Control Systems Group   Texas Instruments

Description:
Header file containing constants, data type definitions, and
function prototypes for the PIDREG.

=====================================================================================

 History:
 16/03/2007
 Modified by Luis Jimenez for making gain parameters really match PID parameter (not affected by control period)
as a drawback, now there is to specify control period and frequency.

-------------------------------------------------------------------------------------

based on:
 04-15-2005	Version 3.20 (of old pid_reg3.h from TI)
*/
#ifndef __PIDREG_H__
#define __PIDREG_H__

/*! struct containing parameters and variables of a PID */
typedef struct {  _iq  Ref;   			/*!< Input: Reference input */
				  _iq  Fdb;   			/*!< Input: Feedback input */
				  _iq  Err;				/*!< Variable: Error*/
				  _iq  Kp;				/*!< Parameter: Proportional gain*/
				  _iq  Up;				/*!< Variable: Proportional output */
				  _iq  Ui;				/*!< Variable: Integral output */
				  _iq  Ud;				/*!< Variable: Derivative output */
				  _iq  OutPreSat; 		/*!< Variable: Pre-saturated output */
				  _iq  OutMax;		    /*!< Parameter: Maximum output */
				  _iq  OutMin;	    	/*!< Parameter: Minimum output */
				  _iq  Out;   			/*!< Output: PID output */
				  _iq  SatErr;			/*!< Variable: Saturated difference */
				  _iq  Ki;			    /*!< Parameter: Integral gain */
				  _iq  Kc;		     	/*!< Parameter: Integral correction gain */
				  _iq  Kd; 		        /*!< Parameter: Derivative gain */
				  _iq  Period;			/*!< Control period */
				  int  Frequency;		/*!< Control frequency (period inverse) */
				  _iq  Err1;		   	/*!< History: Previous error */
		 	 	  void  (*calc)();	  	/*!< Pointer to calculation function */
				 } PIDREG;

/*! pointer to PIDREG struct */
typedef PIDREG *PIDREG_handle;

/*! Default initalizer for the PIDREG object. */
#define PIDREG_DEFAULTS { 0, \
                           0, \
                           0, \
                           _IQ(1.3), \
                           0, \
                           0, \
                           0, \
                           0, \
                           _IQ(1), \
                           _IQ(-1), \
                           0, \
                           0, \
                           _IQ(1.0), \
                           _IQ(0.1), \
                           _IQ(1.05), \
						   _IQ(0.001), \
						   1000, \
                           0, \
              			  (void (*)(Uint32))pid_reg_calc }

/* Prototypes for the functions in pid_reg.c */

void pid_reg_calc(PIDREG_handle);



/*! struct containing parameters and variables of a PID */
typedef struct {  _iq16  Ref;   			/*!< Input: Reference input */
				  _iq16  Fdb;   			/*!< Input: Feedback input */
				  _iq16  Err;				/*!< Variable: Error*/
				  _iq16  Kp;				/*!< Parameter: Proportional gain*/
				  _iq16  Up;				/*!< Variable: Proportional output */
				  _iq16  Ui;				/*!< Variable: Integral output */
				  _iq16  Ud;				/*!< Variable: Derivative output */
				  _iq16  OutPreSat; 		/*!< Variable: Pre-saturated output */
				  _iq16  OutMax;		    /*!< Parameter: Maximum output */
				  _iq16  OutMin;	    	/*!< Parameter: Minimum output */
				  _iq16  Out;   			/*!< Output: PID output */
				  _iq16  SatErr;			/*!< Variable: Saturated difference */
				  _iq16  Ki;			    /*!< Parameter: Integral gain */
				  _iq16  Kc;		     	/*!< Parameter: Integral correction gain */
				  _iq16  Kd; 		        /*!< Parameter: Derivative gain */
				  _iq16  Period;			/*!< Control period */
				  int  Frequency;		/*!< Control frequency (period inverse) */
				  _iq16  Err1;		   	/*!< History: Previous error */
		 	 	  void  (*calc)();	  	/*!< Pointer to calculation function */
				 } PIDREG16;

/*! pointer to PIDREG struct */
typedef PIDREG16 *PIDREG16_handle;

/*! Default initalizer for the PIDREG object. */
#define PIDREG16_DEFAULTS { 0, \
                           0, \
                           0, \
                           0, \
                           0, \
                           0, \
                           0, \
                           0, \
                           _IQ(32767.999984741), \
                           _IQ(-32768), \
                           0, \
                           0, \
                           0, \
                           0, \
                           0, \
						   0, \
						   0, \
                           0, \
              			  (void (*)(Uint32))pid_reg_calc_iq16 }

/* Prototypes for the functions in pid_reg.c */

void pid_reg_calc_iq16(PIDREG16_handle);






/*! struct containing parameters and variables of a PID */
typedef struct {  _iq  Ref;   			/*!< Input: Reference input */
				  _iq  Fdb;   			/*!< Input: Feedback input */
				  _iq  AntiWindup;		/*!< Input: Real saturated input */
				  _iq  FeedForward;		/*!< Input: Feed-forward input */
				  _iq  Err;				/*!< Variable: Error*/
				  _iq  Kp;				/*!< Parameter: Proportional gain*/
				  _iq  Ki;			    /*!< Parameter: Integral gain */
				  _iq  Kc;		     	/*!< Parameter: Anti-windup gain */
				  _iq  Kd; 		        /*!< Parameter: Derivative gain */
				  _iq  Kff; 		    /*!< Parameter: Feed-forward gain */
				  _iq  Up;				/*!< Variable: Proportional output */
				  _iq  Ui;				/*!< Variable: Integral output */
				  _iq  SatError;		/*!< Variable: SatError output */
				  _iq  Ud;				/*!< Variable: Derivative output */
				  _iq  Uff;				/*!< Variable: Feed-forward output */
				  _iq  Out;   			/*!< Output: PID output */
				  _iq  Period;			/*!< Control period */
				  int  Frequency;		/*!< Control frequency (period inverse) */
				  _iq  Err1;		   	/*!< History: Previous error */
		 	 	  void  (*calc)();	  	/*!< Pointer to calculation function */
				 } PIDCONTROLLER;

/*! pointer to PIDCONTROLLER struct */
typedef PIDCONTROLLER *PIDCONTROLLER_handle;

/*! Default initalizer for the PIDREG object. */
#define PIDCONTROLLER_DEFAULTS {	0, \
									0, \
									0, \
									0, \
									0, \
									_IQ(0.1), \
									_IQ(0.01), \
									_IQ(0.01), \
									_IQ(0.001), \
									_IQ(0.001), \
									0, \
									0, \
									0, \
									0, \
									0, \
									0, \
									_IQ(0.001), \
									1000, \
									0, \
									(void (*)(Uint32))pid_controller_calc }

	/* Prototypes for the functions in pid_controller.c */
	void pid_controller_calc(PIDCONTROLLER_handle);

#endif /* __PIDREG_H__ */

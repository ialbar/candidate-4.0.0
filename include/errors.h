/*!	\file errors.h
	\brief Declarations for erros and warnings management

* File: error.h
* Device: TMS320F28235
* Author: Luis Jimenez.
* Description: Declarations for errors and warnings management
*/

#ifndef _ERRORS_H_
#define _ERRORS_H_

#include "DSP2833x_Device.h"
//#include "amc.h"
#include "motion.h"

/* neccesary for LOG messages */
#include <std.h>
#include <log.h>

void log_sci(unsigned int);
void war_sci(unsigned int);
void err_sci(unsigned int);
void deb_sci(unsigned int);
void itohex(char *text, unsigned int number);

/* Erros and Warnings definitions */
#define _ERRORS_LOG		/*!< Sends the errors to the serial port and to LOG_printf DSP/BIOS function */
#define _ERRORS_EMCY		/*!< Sends an EMCY message when the is an Error, **SHOULD NOT BE UNDEFINED** */
#define _WARNINGS_LOG	/*!< Sends the warnings to the serial port and to LOG_printf DSP/BIOS function */
#define _WARNINGS_EMCY	/*!< Sends an EMCY message when the is a Warning */
#define _LOGS_LOG			/*!< Sends log messages to the serial port and to LOG_printf DSP/BIOS function */
#define _DEBUG_LOG
#define _DEBUG_EMCY


#ifdef _DEBUG
#define DSPBIOS_LOG(a,b,c,d) LOG_printf(a,b,c,d)
#else
#define DSPBIOS_LOG(a,b,c,d)
#endif


#ifdef _ERRORS_LOG
#ifdef _ERRORS_EMCY
/*! Notifies the Errors as configured in the defines */
#if 0 //Iv��n Albarr��n
#define _ERRORmessage(errCode, errRegBits, addInfo, message, value1, value2)\
	{DSPBIOS_LOG(&trace, "ERR: " message, value1, value2);\
	err_sci(errCode);\
	EMCY_setError(&amc_od_Data, errCode, errRegBits, addInfo);}
#endif

#define _ERRORmessage(errCode, errRegBits, addInfo, message, value1, value2)\
	EMCY_setError(&amc_od_Data, errCode, errRegBits, addInfo);


#else
	#define _ERRORmessage(errCode, errRegBits, addInfo, message, value1, value2)\
	{DSPBIOS_LOG(&trace, "ERR: " message, value1, value2);\
	err_sci(errCode);}
#endif
#else
#ifdef _ERRORS_EMCY
#define _ERRORmessage(errCode, errRegBits, addInfo, message, value1, value2)\
	EMCY_setError(&amc_od_Data, errCode, errRegBits, addInfo);
#else
#define _ERRORmessage(errCode, errRegBits, addInfo, message, value1, value2)
#endif
#endif


#ifdef _WARNINGS_LOG
#ifdef _WARNINGS_EMCY
/*! Notifies the Warnings as configured in the defines */
#if 0 //Iv��n Albarr��n
#define _WARNINGmessage(errCode, errRegBits, addInfo, message, value1, value2)\
	{DSPBIOS_LOG(&trace, "WAR: " message, value1, value2);\
	war_sci(errCode);\
	EMCY_setError(&amc_od_Data, errCode, errRegBits, addInfo);\
	EMCY_errorRecovered(&amc_od_Data, errCode);}
#endif

#define _WARNINGmessage(errCode, errRegBits, addInfo, message, value1, value2)\
	EMCY_setError(&amc_od_Data, errCode, errRegBits, addInfo);\
	EMCY_errorRecovered(&amc_od_Data, errCode);


#else
	#define _WARNINGmessage(errCode, errRegBits, addInfo, message, value1, value2)\
	{DSPBIOS_LOG(&trace, "WAR: " message, value1, value2);\
	war_sci(errCode);}
#endif
#else
#ifdef _WARNINGS_EMCY
#define _WARNINGmessage(errCode, errRegBits, addInfo, message, value1, value2)\
	{EMCY_setError(&amc_od_Data, errCode, errRegBits, addInfo);\
	EMCY_errorRecovered(&amc_od_Data, errCode);}
#else
#define _WARNINGmessage(errCode, errRegBits, addInfo, message, value1, value2)
#endif
#endif

#ifdef _LOGS_LOG
/*! Notifies the Log messages as configured in the defines */

#if 0 //Iv��n Albarr��n
#define _LOGmessage(logCode,message, value1, value2)\
	{DSPBIOS_LOG(&trace, "LOG: " message, value1, value2);\
	log_sci(logCode);}
#endif


#define _LOGmessage(logCode,message, value1, value2)





#else
#define _LOGmessage(logCode,message, value1, value2)
#endif

#ifdef _DEBUG_LOG
#ifdef _DEBUG_EMCY
/*! Notifies the Errors as configured in the defines */

#if 0 //Iv��n Albarr��n
#define _DEBUGmessage(errCode, errRegBits, addInfo, message, value1, value2)\
	{DSPBIOS_LOG(&trace, "DEB: " message, value1, value2);\
	deb_sci(errCode);\
	EMCY_setError(&amc_od_Data, errCode, errRegBits, addInfo);}
#endif
#define _DEBUGmessage(errCode, errRegBits, addInfo, message, value1, value2)\
	EMCY_setError(&amc_od_Data, errCode, errRegBits, addInfo);




#else
	#define _DEBUGmessage(errCode, errRegBits, addInfo, message, value1, value2)\
	{DSPBIOS_LOG(&trace, "DEB: " message, value1, value2);\
	deb_sci(errCode);}
#endif
#else
#ifdef _DEBUG_EMCY
#define _DEBUGmessage(errCode, errRegBits, addInfo, message, value1, value2)\
	EMCY_setError(&amc_od_Data, errCode, errRegBits, addInfo);
#else
#define _DEBUGmessage(errCode, errRegBits, addInfo, message, value1, value2)
#endif
#endif




/*! possible device faults */
#define FAULT_IRON_CABLE_BREAK    (0x00000001L)		/*!< iron cable break error */
#define FAULT_OVERCURRENT         (0x00000002L)		/*!< motor overcurrent error */
#define FAULT_HEARTBEAT           (0x00000004L)		/*!< heartbeat timeout error */
#define FAULT_NULL_ACC            (0x00000008L)		/*!< one of the acceleration values is null */
#define FAULT_CONTROL_CYCLE       (0x00000010L)		/*!< the period of the contol cycle has  grown in last cycles */
#define FAULT_PASSIVE_CAN         (0x00000020L)		/*!< CAN bus is in passive mode */
#define FAULT_BUSOFF_CAN          (0x00000040L)		/*!< CAN bus is in bus-off state */
#define FAULT_OVERTEMP            (0x00000800L)		/*!< over temperature fault */
#define FAULT_VPOWER              (0x00001000L)		/*!< power voltage error */
#define FAULT_VEL_FOLLOWING       (0x00004000L)		/*!< we are too far from the velocity reference */
#define FAULT_POS_FOLLOWING       (0x00008000L)		/*!< we are too far from the position reference */
#define FAULT_REDUNDANCY          (0x00010000L)		/*!< any redundant position value is different from the others */
#define FAULT_HYBRID_REDUNDANCY   (0x00020000L)		/*!< any redundant position value is different from the others */
#define FAULT_VEL_LIMIT           (0x00040000L)		/*!<  The velocity is higher than the max profile velocity */
#define FAULT_POS_LIMIT           (0x00080000L)		/*!< The position is out of the software position limits */
#define FAULT_MOTOR_PHASE         (0x00100000L)		/*!< at least one phase of the motor is not working */
#define FAULT_CURRENTS_FOLLOW     (0x00200000L)		/*!< we are too far from the current reference */
#define FAULT_HALL                (0x00400000L)		/*!< hall effect sensor error */
#define FAULT_MOTOR_OVERTEMP      (0x00800000L)
#define FAULT_NCE_MEASURE         (0x01000000L)     /*!< NCE sensor measure error */
#define FAULT_IGBT_ISR            (0x02000000L)
#define FAULT_POT_MEASURE         (0x04000000L)     /*!< Potentiometer measure error */
#define FAULT_ENCODER             (0x08000000L)     /*!< Encoder error */
#define FAULT_RATCHET             (0x10000000L)     /*!< Ratchet malfunction error */
#define FAULT_UNUSED              (0xE0002780L)		/*!< unused bits in faults (used in faults checking) */












extern short current_filt_to_check_overcurrent;
extern unsigned short int dbg_overcurrent;
extern unsigned int Overcurrent_error_window;

/* function declarations */
void fault_management(motion_state_struct *state);
void setFault(unsigned long fault);
unsigned long unrecoverableFaults(void);
int isFaultActive(unsigned long fault);
void fault_reaction_initialization( motion_state_struct *state, int *mode );
void check_overcurrent (void);


void QueueFault( unsigned long);
void DeQueueFault( unsigned long);
unsigned long GetQueuedFault(void);
void set_fault_flags(void);
void QueueWarning( unsigned long);
void DeQueueWarning( unsigned long );
void set_warning_flags(void);

#endif  /* end of _ERRORS_H_ */

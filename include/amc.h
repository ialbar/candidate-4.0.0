/*!	\file amc.h
	\brief General include file for amc project

* File: amc.h
* Device: TMS320F28235
* Author: Luis Jimenez.
* Description: Include file for amc  project.  Include this file in all C-source files.
*/

#ifndef AMC_H
#define AMC_H


//---------------------------------------------------------------------------
// Include Standard C Language Header Files
//
#include <string.h>
#include <stdlib.h>


//---------------------------------------------------------------------------
// Include DSP/BIOS Header Files
//
#include <std.h>

#include <clk.h>
#include <log.h>
#include <mbx.h>
#include <sem.h>
#include <tsk.h>
#include <swi.h>
#include <hwi.h>
#include <atm.h>

// FLASH or RAM are defined in the CCS project build options
#ifdef FLASH_PRJ
    #include "amc_flashcfg.h"
#endif

#ifdef RAM_PRJ
    #include "amc_ramcfg.h"
#endif


//---------------------------------------------------------------------------
// Include any other Header Files
//
#include "IQmathLib.h"
#include "pid_reg.h"
#include "motion.h"
#include "position_mode.h"
#include "velocity_mode.h"
#include "homing_mode.h"
#include "manual_mode.h"
#include "circle_mode.h"
#include "interpolated_mode.h"
#include "assisted_mode.h"
#include "vectorial_current_mode.h"
#include "store.h"
#include "Ecap.h"
#include "Gpio.h"
#include "24xx32a.h"
#include "motor_phase_check.h"

//#include <applicfg.h>		//CANOpen includes
//#include <can_driver.h>
//#include <timers_driver.h>
#include <canfestival.h>

#include "fir.h"		/* fir filter declarations */

/********Defines********/
/* Revision of amc board */
#define BOARD_REVISION 2

#define FIR_ORDER 20		/* filter order */
#define FIR16_COEFF {\
			6526,-59569,-4583015,-13628820,-24968175,-26542080,65131,68288131,174915376,295305146,\
			391118847}

/*! Average filter, the returned value is not the filtered value but 16*filtered_value (this avoids quantification errors) */
#define FILTER_SUM(new, accumulated) (((((unsigned long)(accumulated) * 15) + 8 ) >> 4) + (new))
#define FILTER_OUT(accumulated) (((accumulated) + 8) >> 4)
/*! Hard Average filter, for slow signals */
//#define FILTER2_SUM(new, accumulated) (((((unsigned long)(accumulated) * 31) + 16 ) >> 5) + (new))
//#define FILTER2_OUT(accumulated) (((accumulated) + 16) >> 5)
//#define FILTER_SUM_2(new, accumulated) (((((unsigned long)(accumulated) * 63) + 32 ) >> 6) + (new))
//#define FILTER_OUT_2(accumulated) (((accumulated) + 32) >> 6)
#define FILTER_SUM_2(new, accumulated) (((((unsigned long)(accumulated) * 127) + 64 ) >> 7) + (new))
#define FILTER_OUT_2(accumulated) (((accumulated) + 64) >> 7)
/*! Very Hard Average filter, for very slow signals */
//#define FILTER_SUM_3(new, accumulated) (((((unsigned long)(accumulated) * 255) +  128) >> 8) + (new))
//#define FILTER_OUT_3(accumulated) (((accumulated) + 128) >> 8)
#define FILTER_SUM_3(new, accumulated) (((((unsigned long)(accumulated) * 511) +  256) >> 9) + (new))
#define FILTER_OUT_3(accumulated) (((accumulated) + 256) >> 9)

/*! Enable and set duty cycle of ePWM1 Module. */
#define ENABLE_PWM1(x) EPwm1Regs.DBCTL.all = 0x000B; \
                       EPwm1Regs.AQCTLA.all = 0x0060; \
                       EPwm1Regs.CMPA.half.CMPA = x;

/*! Enable and set duty cycle of ePWM2 Module. */
#define ENABLE_PWM2(x) EPwm2Regs.DBCTL.all = 0x000B; \
                       EPwm2Regs.AQCTLA.all = 0x0060; \
                       EPwm2Regs.CMPA.half.CMPA = x;

/*! Enable and set duty cycle of ePWM3 Module. */
#define ENABLE_PWM3(x) EPwm3Regs.DBCTL.all = 0x000B; \
                       EPwm3Regs.AQCTLA.all = 0x0060; \
                       EPwm3Regs.CMPA.half.CMPA = x;

#define DISABLE_PWM1   EPwm1Regs.DBCTL.all = 0x00; EPwm1Regs.AQCTLA.all = 0x0055;
#define DISABLE_PWM2   EPwm2Regs.DBCTL.all = 0x00; EPwm2Regs.AQCTLA.all = 0x0055;
#define DISABLE_PWM3   EPwm3Regs.DBCTL.all = 0x00; EPwm3Regs.AQCTLA.all = 0x0055;

/*! Disable the motor PWMs */
#define DISABLE_MOTOR DISABLE_PWM1 DISABLE_PWM2 DISABLE_PWM3


/*! device control commands triggered by bit patterns in the 'controlword' */
typedef enum {
	NO_MOTOR,			/*!< motor type has not been configurated */
	DC_MOTOR_a,			/*!< frequency controlled DC motor */
	DC_MOTOR_b,			/*!< frequency controlled DC motor (inverted polarity) */
	BLDC_MOTOR_a,		/*!< trapezoidal Brushless DC motor (sequence as Delta 57BL74 motor) */
	BLDC_MOTOR_b,		/*!< trapezoidal Brushless DC motor (sequence as Pini MB057GA240 and Maxon ECmax-40 motors) */
	BLDC_MOTOR_c,		/*!< trapezoidal Brushless DC motor (sequence as Delta 42BL61 motor) */
	BLAC_MOTOR_a,		/*!< sinusoidal Brushless motor (Lenze motor, 4 pole pairs) PWM positive up */
	BLAC_MOTOR_b		/*!< sinusoidal Brushless motor (Lenze motor, 4 pole pairs) PWM positive fall-down */
	} motor_type;

/*! CAN bus status */
typedef enum {
	ERROR_ACTIVE,		/*!< Normal operation */
	WARNING_LEVEL,		/*!< Normal operation, but one error counter is above 96 */
	ERROR_PASSIVE,		/*!< The CAN module is in error-passive mode */
	BUS_OFF				/*!< The CAN module is in bus-off mode. No messages can be received or transmitted */
} can_bus_state_t;

//---------------------------------------------------------------------------
// Function Prototypes
//

extern void DelayUs(Uint16);
extern void SetDBGIER(Uint16);
extern void InitSysCtrl(void);
extern void InitXintf(void);
extern void InitXintrupt(void);
extern void InitPieCtrl(void);
extern void InitFlash(void);
extern void InitAdc(void);
extern void InitSci(void);
void UserInit(void);
void adc_read_seq1(void);
void adc_read_seq2(void);
void pwm_out(void) ;
inline _iq currentFactor(int curr);
void periodic_control_function(void);
unsigned int get_max_error(void);
unsigned int max(unsigned int num1, unsigned int num2, unsigned int num3, unsigned int num4, unsigned int num5);
void new_redundancy(motion_state_struct *state );
int  velocity_over_limit(long velocity, int assisted);
void check_velocity_limit(long velocity, motion_state_struct *state);
int  position_over_limit(long int position);
void check_position_limit(long position, motion_state_struct *state);
void constant_pwm_mode_operation(char init);
int constant_pwm_position_limits(void);
void quick_stop_initialization( motion_state_struct *state, int *mode );
void quick_stop_control( motion_state_struct *state, int mode );
void quick_stop_init(motion_state_struct *state, int *mode );
long root(unsigned long long radicand);
long long getltime(void);
void periodic_txpdo_swi(void);
int readNodeID(void);
int readIDswitch(void);
void analog_measurements(void);
int interpolate(int x, int y, int z, int a, int b);
long linterpolate(long long x, long long y, long long z, long a, long b);
void manageLeds(void);
void velocity_filter(long *value, int reset);
void position_filter(long *value, int reset);
long half_acc_t2(long acc, unsigned long time);
void resetNode(void);
long int2ext_pos(long internal_pos, long offset);
long ext2int_pos(long external_pos, long offset);
long int2ext_vel(long internal_vel);
long ext2int_vel(long external_vel);
void launch_amc_loader(void);
void launch_fpga_loader(void);
void set_stopped_flag( motion_state_struct *state );
void send_controlword( void );
void safety(void);
int detent_nearby(long current_pos, long *detent_pos);
unsigned long GetQueuedFault( void );
unsigned long GetQueuedWarning( void );
void set_fault_flags(void);
void set_warning_flags(void);


void ramp_management(void);
void load_ramp_time(int set);
void operation_mode_change(char mode);
void manual_with_clutch_state(motion_state_struct *state);

/* functions defined in brakes.c */
void set_brakes_flags(void);
void brakes_operation(void);
void activate_brake(void);
void release_brake(void);
void activate_clutch(void);
void release_clutch(void);

/* functions defined in Sci.c */
int SCI_send(char *message);
int SCI_send_debug(unsigned char *message);
/* functions defined in can.c */
void InitCan(int baudrate);
void can_rxtx(void);
void manage_can_bus(void);
/* functions defined in EQep.c */
void InitEQep(void);
/* functions defined in Epwm.c */
void InitEPwmModules(void);
void InitSWEPwmModules(void);


int isIronCableBroken( void );
void QueueFault( unsigned long fault );
void DeQueueFault( unsigned long fault );
int manual_mode_back(motion_state_struct *state);

void eval_hall_state(unsigned char hall_state, motor_type motor);
void BLDC_apply_pwm(unsigned int positive, unsigned int negative, unsigned char hall_state);
void check_hall_effect(manual_state_t manual_state);
//inline _iq current_controller(_iq Ref, _iq Fdb);

void check_potentiometer(motion_state_struct *state);

void pp_set_max_travel(long travel);

//---------------------------------------------------------------------------
// Global symbols defined in the linker command file
//
extern Uint16 hwi_vec_loadstart;	/*!< Start of the load memory of section HWI_vec (is loaded in FLASH but runs form RAM) */
extern Uint16 hwi_vec_loadend;		/*!< End of the load memory of section HWI_vec (is loaded in FLASH but runs form RAM) */
extern Uint16 hwi_vec_runstart;		/*!< Start of the run memory of section HWI_vec (is loaded in FLASH but runs form RAM) */
extern Uint16 secureRamFuncs_loadstart;		/*!< Start of the load memory of section secureRamFuncs (is loaded in FLASH but runs form RAM) */
extern Uint16 secureRamFuncs_loadend;		/*!< End of the load memory of section secureRamFuncs (is loaded in FLASH but runs form RAM) */
extern Uint16 secureRamFuncs_runstart;		/*!< Start of the run memory of section secureRamFuncs (is loaded in FLASH but runs form RAM) */
extern Uint16 trcdata_loadstart;	/*!< Start of the load memory of section trcdata (is loaded in FLASH but runs form RAM) */
extern Uint16 trcdata_loadend;		/*!< End of the load memory of section trcdata (is loaded in FLASH but runs form RAM) */
extern Uint16 trcdata_runstart;		/*!< Start of the run memory of section trcdata (is loaded in FLASH but runs form RAM) */
extern Uint16 criticalFuncs_loadstart;		/*!< Start of the load memory of section criticalFuncs (is loaded in FLASH but runs form RAM) */
extern Uint16 criticalFuncs_loadend;		/*!< End of the load memory of section criticalFuncs (is loaded in FLASH but runs form RAM) */
extern Uint16 criticalFuncs_runstart;		/*!< Start of the run memory of section criticalFuncs (is loaded in FLASH but runs form RAM) */

//---------------------------------------------------------------------------
// Global definitions
//

/*! number of can mailboxes used for transmission (max 16) */
#define CAN_TX_MAILBOXES 8

/*! CAN bus speed */
//#define CAN_SPEED "1M"
#define CAN_SPEED "500K"

/*! CAN transmit messages time-out (in bits) */
//#define CAN_MSG_TIME_OUT 2000		/* 2ms (when baudrate = 1MBit) */
#define CAN_MSG_TIME_OUT 5000		/* 10ms (when baudrate = 500kBit) */

/*! Master Node-id */
#define MASTER_NODEID 0x7F

/*! Max servo current (in adc units) for software limit */
#define MAX_SERVO_CURRENT 12000		/* 12 A*/
#define MAX_CURRENT_N_ERRORS	20		/*<! Number of overcurrent values before error */

#define minimum(a,b) (((a) < (b)) ? (a) : (b))		/*!< Calculates the minimum of \a a and \a b */
#define maximum(a,b) (((a) > (b)) ? (a) : (b))		/*!< Calculates the maximum of \a a and \a b */
#define CLAMP(x, low, high)  (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))


#define sign(a) (((a) >= 0 ) ? 1 : -1 )		/*!< Returns 1 if positive or zero and -1 if negative */

#define LONGLONG_MAX_VAL 9223372036854775807LL;		/* 2^63 -1 is the max value of a long long */


/*! Timer 1 counter for half of the period of the PWM's */
//#define HALF_PERIOD 1875		/* for 20 KHz in up/down mode (18.7-21.5 KHz with frequency dispersion) */
#define HALF_PERIOD_17KHz 2200		/* for 17 KHz in up/down mode (16.1-18.1 KHz with frequency dispersion) */
#define HALF_PERIOD_16KHz 2344		/* for 16 KHz in up/down mode (XX.X-XX.X KHz with frequency dispersion) */
#define HALF_PERIOD_15KHz 2500		/* for 15 KHz in up/down mode (XX.X-XX.X KHz with frequency dispersion) */
#define HALF_PERIOD_14KHz 2680		/* for 14 KHz in up/down mode (XX.X-XX.X KHz with frequency dispersion) */
#define HALF_PERIOD_12KHz 3125		/* for 12 KHz in up/down mode (XX.X-XX.X KHz with frequency dispersion) */
#define HALF_PERIOD_10KHz 3750		/* for 10 KHz in up/down mode (XX.X-XX.X KHz with frequency dispersion) */
#define HALF_PERIOD_8KHz  4688		/* for  8 KHz in up/down mode (XX.X-XX.X KHz with frequency dispersion) */


// TODO: reduce DEAD_BAND if everything is ok */
/*! Motor PWM signals dead band (cycles) */
//#define DEAD_BAND 32		/* ~425 ns */
#define DEAD_BAND 75		/* ~1 us */

/*! maximum duty cycle to avoid discharge of bootstrap capacitors */
#define MAX_DUTY_CYCLE _IQ(0.95)

/*! minimum duty cycle to avoid discharge of bootstrap capacitors */
#define MIN_DUTY_CYCLE _IQ(-0.95)

/*! ePWM6 Time Base Module period of 3750 to obtain a PWM period of 20 KHz */
#define BRAKES_PERIOD 3750		/* for 20 KHz upcount mode */

/*! ePWM4 Time Base Module period of 3750 to obtain a PWM period of 10 KHz */
#define RESOLVER_PWM_PERIOD  3750		/* for 10 KHz upcount mode (action is toggle) */
//#define RESOLVER_READ_POINT  1875
#define RESOLVER_READ_POINT  3400   /* delayed after increasing filter capacitors */

/*! Time to delay bake release if Control_effort_offset is used */
#define BRAKE_RELEASE_DELAY_DUTY_OFFSET 200

/*! Serial log buffer size (must be a power of 2) */
#define SERIAL_LOG_BUFFER_SIZE 64

/*! Value of the redundancy error counter before it launches an error */
#define REDUNDANCY_ERROR_FILTER 5

/*! Number of control cycles between redundancy checks */
#define REDUNDANCY_N_CYCLES 2

/*! Minimum number of cycles with voltage outside the limit for an error to occur */
#define VOLTAGE_ERROR_FILTER 20

/*! Minimum number of cycles with temperature above the limit for an error to occur */
#define TEMPERATURE_ERROR_FILTER 10

/*! Number of cycles waiting for the periodic_control_function to report an error */
#define n_CONTROL_CYCLE_ERRRORS 3

/*! Number of cycles (ms) between velocity references in Assisted Mode */
#define AM_REF_NCYCLES 32

#define STOP_BRAKE     -1  /*!< Quick stop mode: brake 1 activated */
#define STOP_DISABLE    0  /*!< Quick stop mode: disable drive function */
#define STOP_SLOW       1  /*!< Quick stop mode: slow down on slow down ramp */
#define STOP_QUICK      2  /*!< Quick stop mode: slow down on quick stop ramp */
#define STOP_SLOW_STAY  5  /*!< Quick stop mode: slow down on slow down ramp and stay in QUICK_STOP */
#define STOP_QUICK_STAY 6  /*!< Quick stop mode: slow down on quick stop ramp and stay in QUICK_STOP */


/* position_flags maskbits */
#define MIN_SW_POS_LIMIT     0x0001   /*!< minimum software position limit reached */
#define MAX_SW_POS_LIMIT     0x0002   /*!< maximum software position limit reached */

#define MIN_POS_MASKBIT      MIN_SW_POS_LIMIT   /*!< maskbit including any of the minimum position flags */
#define MAX_POS_MASKBIT      MAX_SW_POS_LIMIT   /*!< maskbit including any of the maximum position flags */
#define SW_POS_MASKBIT       0x0003   /*!< maskbit including any of the software limits flags */


#define SET_POWER_OFF()  \
		do { \
			activate_brake(); \
			release_clutch(); \
        }while(0)
			//set_motor_phase_check_start(); \
		}while(0)


#define SET_POWER_ON()  \
		do { \
			release_brake(); \
			activate_clutch(); \
		}while(0)

extern char force_init;
extern int ramp_time;
extern int mode_change_time;
extern int safety_disable;


//---------------------------------------------------------------------------
// Global symbols defined in source files
//
extern long (*position_counter)(char init);
extern long (*velocity_counter)(char init);
extern long (*abs_pos_counter)(char init);

extern INTEGER32 *redundant_enc;
extern INTEGER32 *redundant_pot;

/*! Variable that reflects whether power is applied to the motor or not */
extern int ready_to_power_on;

/*! Determines if the mode of operations has to be initialized in the next cycle (if it has just enabled or mode changed) */
extern int init_mode;

/*! Determines if the mode of operations has to be changed (in the next cycle or when it's possible) */
extern int change_mode;

/*! CANopen nodeID read from DIP switch (switch1=LSB, switch7=MSB) */
extern int nodeID;

/*! global variable set in main() that converts ltime in seconds */
extern unsigned int lcounts_p_s;

/*! global variable used to force StatusWord sending in every cycle (protected by CanFestival mutex) */
extern int send_SWord;

/* Analog variables */
extern unsigned resolver_in1;//resolver_in1;
extern unsigned long analog_hybrid;
extern unsigned an1, an2, an3, an4;
extern unsigned thermistor, analog_motorcurrent, analog_motorcurrent2, analog_ntc;
extern unsigned analog_vpower, analog_ref1v5, analog_vpower2;

/* Resolver variables */
extern _iq resolver_angle;
extern long accumulated_sin;
extern long accumulated_cos;

extern volatile Uint16 *xint_register;

extern int current_limit;			/*!< Current limit in mA used for most modes */
extern int current_limit_acc;		/*!< Current limit in mA used during acceleration */
extern int current_limit_ip;		/*!< Current limit in mA used in IP mode */
extern int *p_current_limit;		/*!< pointer to the current limit used */

/*! Motor current in mA */
extern short current;
extern short current_total;
extern short current1;
extern short current2;
extern short current3;


/*! DC bus voltage in dV */
extern unsigned voltage;

/*! indicates the type of the motor (DC, BLDC...) */
extern motor_type motor;

extern long long edge_time;

extern unsigned long pos_error_normFactor;		/*!< Factor to normalize position error */
extern unsigned long vel_normFactor;		/*!< Factor to normalize velocity */

extern unsigned char log_buff[SERIAL_LOG_BUFFER_SIZE];	/*!< serial log buffer */
extern unsigned buff_tosend;		/*!< index of the position of the next character in the serial log buffer to be send */
extern unsigned buff_towrite;		/*!< index of the position in the serial log buffer for the next char to be written */

extern unsigned long pos2int_factor_num;	/*!< numerator of the factor to convert position increments to internal units */
extern unsigned long pos2int_factor_den;	/*!< denominator of the factor to convert position increments to internal units */

extern unsigned long vel2int_factor_num;	/*!< numerator of the factor to convert velocity increments to internal units */
extern unsigned long vel2int_factor_den;	/*!< denominator of the factor to convert velocity increments to internal units */

extern unsigned long adc_gain;		/*!< ADC calibration gain (intead of divide by 1000, we will multiply by 524 and shift right 19) */
extern int adc_offset;			/*!< ADC calibration offset */

extern unsigned ZeroCurrent_motor;	/*!< ACD measure of the motor current sensor when current = 0A (should be ~0.5v) */
extern unsigned ZeroCurrent_motor2;	/*!< ACD measure of the motor current 2 sensor when current = 0A (should be ~0.5v) */
extern int ZeroCurrent_set;         /*!< when '1' indicates that ZeroCurrent has been read from initial adc current */
extern short CurrentMeasurement_Type;	/*!< 0=Idc, 1=Iu+Iw*/
extern short CurrentSensor_Type; /*!< 0= 0=ACS713/ACS712, 1=ACS710 */

/* trajectory variables */
extern velocity_trajectory_struct pv_trajectory;		/*!< Struct that keeps velocity trajectory in pv mode */
extern velocity_trajectory_struct hm_trajectory;		/*!< Struct that keeps velocity trajectory in hm mode */
extern velocity_trajectory_struct am_trajectory;		/*!< Struct that keeps velocity trajectory in am mode */
extern position_trajectory_struct pp_trajectory;		/*!< Struct that keeps position trajectory in pp mode */
extern interpolated_trajectory_struct ip_trajectory;	/*!< Struct that keeps interpolated position trajectory in ip mode */
extern circle_trajectory_struct circle_trajectory;		/*!< Struct that keeps trajectory in circle mode */
extern PIDREG pv_control_pid;		/*!< PID struct for velocity control */
extern PIDREG pv_control_pid_mms;		/*!< PID struct for velocity control */
extern PIDREG am_control_pid_mms;		/*!< PID struct for velocity control */
extern PIDCONTROLLER pv_position_control_pid;
extern PIDCONTROLLER pv_position_control_pid_test;
extern PIDREG position_control_pid;	/*!< PID struct for position control */
extern PIDREG position_control_pid_mms;/*!< PID struct for position control */
extern PIDREG16 position_control_pid_mms_iq16;
extern PIDREG pos_vel_control_pid_mms;/*!< PID struct for position vel control */
#if 0
extern PIDREG current_control_pid;	/*!< PID struct for current control */
#endif
extern _iq duty;

/* Vectorial Current Control */
extern _iq Id, Iq, Id_ref_off, Iq_ref_off, Id_ref, Iq_ref;
extern _iq Ud, Uq, Ud_ref_off, Uq_ref_off; //, Ud_ref, Uq_ref;
extern _iq Kp, Ki, Kc, Kff, Period;

extern _iq control_effort;		/*! Control effort [-1,1] calculated in PID regulators (will be assigned to 'duty') */
extern _iq control_effort_plus;		/*! Control effort [-1,1] calculated in PID regulators (will be assigned to 'duty') */
extern _iq control_effort_pos_mm; /*! Control effort [-1,1] calculated in PID regulators (will be assigned to 'duty') */
extern _iq control_effort_vel_mms; /*! Control effort [-1,1] calculated in PID regulators (will be assigned to 'duty') */


extern _iq i_measured;
extern _iq i_current;
extern short final_current;
extern int am_current_limit;


extern can_bus_state_t can_bus_state;	/*!< State of the CAN bus */

extern unsigned int position_flags;    /*!< some flags to signal if position limits or switches have been reached */

extern int manual_with_clutch;  /*!< used to signal that a manual mode with a clutch is in use */
extern int reset_filters;       /*!< Indicates that position/velocity filters have to be reset in the next control cycle */

extern unsigned int dimitri;

extern long accum_resolver_vel;

extern unsigned short Half_Period;
extern unsigned char flagDebugScia; // Flag to control de Debug System from SCI
extern long long tmp_red;


#include "canopen_amc.h"			//These "includes" are here because they need some of the previous "defines"
#include "drive_state_machine.h"
#include "peripherals.h"
#include "errors.h"

#include "doxygen_asm.h"

enum
{
	WORKING_MODE_SAFE = 0,
	WORKING_MODE_SERVICE = 1,
	WORKING_MODE_OPERATIONAL = 2
};


enum
{
	OPERATION_MODE_ASSISTED     =  -4,
	OPERATION_MODE_MANUAL       =  -3,
	OPERATION_MODE_CIRCLE       =  -2,
	OPERATION_MODE_PWM          =  -1,
	OPERATION_MODE_POSITION     =   1,
	OPERATION_MODE_VELOCITY     =   3,
	OPERATION_MODE_MOTOR_CHECK  =   5,
	OPERATION_MODE_HOMING       =   6,
	OPERATION_MODE_INTERPOLATED =   7,
	OPERATION_MODE_ASSISTED_2   = 252,
	OPERATION_MODE_MANUAL_2     = 253,
	OPERATION_MODE_CIRCLE_2     = 254,
	OPERATION_MODE_PWM_2        = 255
};
#endif  /* end of AMC_H definition */

/*** end of file *****************************************************/

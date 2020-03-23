/*!	\file drive_state_machine.h
	\brief Definitions of the state machine of the drive.

For detailed information look at:
"CiA 402 DSP V2.0: CANopen device profile for drives and motion control"
*/

#ifndef _DRIVE_STATE_MACHINE_H_
#define _DRIVE_STATE_MACHINE_H_

/* bits of 'controlword' */
#define FAULT_RESET_MASKBIT           0X0080
#define ENABLE_OPERATION_MASKBIT      0X0008
#define QUICK_STOP_MASKBIT            0X0004
#define ENABLE_VOLTAGE_MASKBIT        0X0002
#define SWITCH_ON_MASKBIT             0X0001
#define HALT_MASKBIT                  0x0100
#define MODE_SPECIFIC_BIT4_MASKBIT    0x0010
#define HOMING_OPERATION_START        0x0010
#define NEW_SET_POINT                 0x0010
#define CHANGE_IMMEDIATELY            0x0020
#define ABS_REL                       0x0040
#define CHECK_MOTOR_PHASE_START       0x0010

/* bits of 'statusword' */
#define STATEWORD_MASK1               0x004F
#define STATEWORD_MASK2               0x006F
#define NOT_READY_TO_SWITCH_ON_BITS   0x0000
#define SWITCH_ON_DISABLED_BITS       0x0040
#define READY_TO_SWITCH_ON_BITS       0x0021
#define SWITCHED_ON_BITS              0x0023
#define OPERATION_ENABLE_BITS         0x0027
#define QUICK_STOP_ACTIVE_BITS        0x0007
#define FAULT_REACTION_ACTIVE_BITS    0x000F
#define FAULT_BITS                    0x0008
#define MODE_SPECIFIC_BITS            0x3400

#define TARGET_REACHED_MASKBIT        0x0400
#define ZERO_SPEED_MASKBIT            0x1000
#define MAX_SLIPPAGE_MASKBIT          0x2000
#define HOMING_ATTAINED               0x1000
#define HOMING_ERROR                  0x2000
#define FOLLOWING_ERROR_MASKBIT       0x2000
#define SET_POINT_ACKNOWLEDGE_MASKBIT 0x1000
#define DETENT_ACTIVE_MASKBIT         0x1000
#define IP_ACTIVE_MASKBIT             0x1000
#define BIT13_MASKBIT                 0x2000
#define AM_OPERATION_MASKBIT          0x1000
#define AM_MAX_VEL_MASKBIT            0x2000
#define CHECK_MOTOR_PHASE_ATTAINED    0x1000
#define CHECK_MOTOR_PHASE_ERROR       0x2000
/* manufacturer specific flags in StatusWord */
#define STOPPED_MASKBIT               0x0100
#define BRAKE_MASKBIT                 0x4000
#define ERROR_MASKBIT                 0x0010
#define HALT_REACHED_MASKBIT          0x8000
#define WARNING_MASKBIT		          0x0080

#define MAX_ENABLE_DISABLE_DELAY 10000   /*!< maximum delay in ms of a ENABLE or DISABLE command because of brake/cluth */

/*! possible device states */
typedef enum {
	STATE_ERROR,				/*!< not a state, just an error to return in newState and getState functions */
	START,						/*!< initial state */
	NOT_READY_TO_SWITCH_ON,		/*!< the drive is being initialized or is running self test */
	SWITCH_ON_DISABLED,			/*!< initialization complete, drive function disabled */
	READY_TO_SWITCH_ON,			/*!< high voltage may be applied to the drive, drive function disabled */
	SWITCHED_ON,				/*!< high voltage applied to the drive, power amplifier ready */
	OPERATION_ENABLE,			/*!< power is applied to the motor */
	QUICK_STOP_ACTIVE,			/*!< the quick stop function is being executed (power applied to the motor) */
	FAULT_REACTION_ACTIVE,		/*!< a fault has occurred, quick stop function executed (power applied to the motor) */
	FAULT						/*!< a fault has occurred, the drive function is disabled */
} drive_state;

/*! device control commands triggered by bit patterns in the 'controlword' */
typedef enum {
	SHUTDOWN,			/*!< "Shutdown" command, sends the servo to READY_TO_SWITCH_ON state */
	SWITCH_ON_DISABLE_OPERATION,	/*!< "Switch on " and "Disable operation" commands send the servo to SWITCHED_ON state */
	/* SWITCH_ON_ENABLE_OPERATION, */
	ENABLE_OPERATION,	/*!< "Enable operation" command changes the state to OPERATION_ENABLE state */
	DISABLE_VOLTAGE,	/*!< "Disable voltage" command changes the state to SWITCH_ON_DISABLED state */
	QUICK_STOP,			/*!< "Quick stop" command executes a quick stop. This behaviour depends on Quick_stop_option_code */
	FAULT_RESET,		/*!< From FAULT state, when a "Fault reset" command is recived and no fault exists the state changes to SWITCH_ON_DISABLED */
	HALT,			/*!< "Halt" command stops the motor as specified in every operational mode */
	/* MODE_SPECIFIC_BIT4, */
	NOT_KNOWN_COMMAND	/*!< Command received is unknown */
} control_command;


/* function prototypes */
/* drive_state newState(drive_state old_state, control_command c); */
void processCommand(control_command c, drive_state old_state);
control_command getCommand(unsigned short int controlword);
drive_state getDriveState(unsigned short int statusword);
int setDriveState(unsigned short int *p_statusword, drive_state new_state);
void printDriveState(unsigned short int statusword);
unsigned short int setCommand(control_command c);
void printControlCommand(control_command cmd);
unsigned int check_EnableOperation_delay(void);
unsigned int check_DisableOperation_delay(void);

#endif			/* end of _DRIVE_STATE_MACHINE_H_ */

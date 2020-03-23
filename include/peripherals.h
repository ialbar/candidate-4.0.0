/*!	\file peripherals.h
	\brief include file for peripherals configuration functions

\verbatim
* File: peripherals.h
* Device: TMS320F28235
* Author: Alvaro Garcia.
* Description: Include file for peripherals functions.Included in amc.h
\endverbatim
*/

#ifndef _PERIPHERALS_H_
#define _PERIPHERALS_H_

/*! max length of adc-temp table */
#define MAX_ADCTEMP_LENGTH 30

/*! type used by temperature look-up tables */
typedef struct {
	int n;
	int adcTable[MAX_ADCTEMP_LENGTH];
	int tempTable[MAX_ADCTEMP_LENGTH];
	} adc_temp_table_t;

/* configure positioning sensors, unit conversion factors, etc */
int  checkPositioningSensorsConfig(void);
int  checkFactors(void);
void configurePositioningSensors(void);
void configurePowerBoardValues( void );
void updateFactors(void);
int  dimensionalOrder(unsigned long *num, unsigned long *den, int index_num, int index_div);
void fractionReduction(unsigned long *num, unsigned long *den);

/* Function that will read the motion state of the drive */
void getMotionState(motion_state_struct *state);

/* read peripherals and show its value in the OD */
void managePeripherals(void);
char used_by_control(char peripheral);

unsigned int get_position_resolution(int sensor);
unsigned int get_velocity_resolution(int sensor);
void get_pos_num_den(unsigned int pos_place, unsigned int vel_place, unsigned int pos_res, unsigned int vel_res, unsigned long *num, unsigned long *den);
void get_pos_vel_factors(int pos_place, int vel_place, unsigned int pos_res, unsigned int vel_res);
long int2ext_place(long internal_pos, long offset, int pos_per, int pos_place, int vel_per, int vel_place);

void manage_AN1(void);
void manage_AN2(void);
void manage_Enc1(void);
void manage_Enc2(void);
void manage_NCE(void);
void manage_Resolver(void);

/* Sensors used for position/velocity/additional_pos */
#define P_UNUSED    0       /*!< Unused peripheral */
#define P_ENC1      1       /*!< Encoder 1 */
#define P_ENC2      2       /*!< Encoder 2 */
#define P_AN1       3       /*!< Analog Input 1 */
#define P_AN2       4       /*!< Analog Input 2 */
#define P_AN3       5       /*!< Analog Input 3 */
#define P_AN4       6       /*!< Analog Input 4 */
#define P_MAG_ENC   7       /*!< Magnetic Encoder*/
#define P_PWM_ENC   8       /*!< PWM output encoder */
#define P_RESOLVER  9       /*!< Resolver */
#define P_HYBRID	10      /*!< Hybrid sensor */

/* special sensors connected in GPIO port */
#define GPIO_UNUSED    0    /*!< No peripheral connected in GPIO */
#define GPIO_2I2O      1    /*!< 2 inputs and 2 outputs */
#define GPIO_MAG       2    /*!< Magnetic encoder (Renishaw) in GPIO */
#define GPIO_8b_SHIFT  3    /*!< 8-bit shift register in GPIO */
#define GPIO_16b_SHIFT 4    /*!< 16-bit shift register in GPIO */
#define GPIO_32b_SHIFT 5    /*!< 32-bit shift register in GPIO */
#define GPIO_NC_ABS_ENCODER 6 /*!< Non-contacting Absolute encoder */
#define GPIO_12b_NCE_SENSOR 7 /*!< 12-bit 1-wheel absolute encoder */
#define GPIO_TRINQUETE 9
#define GPIO_12b_NCE_SENSOR_TRINQUETE 10

/* Position of the sensors */
#define PLACE_UNDEF 0       /*!< Position of the sensor not defined */
#define PLACE_BACK  1       /*!< At the back of the motor */
#define PLACE_FRONT 2       /*!< After the gearbox (output pulley) */
#define PLACE_EXT   3       /*!< External units */

/* What to do with stored params in some function calls */
#define SAVED_PARAMS 0      /*!< Don't initialize, use saved params */
#define INITIALIZE   1      /*!< Initialize params with values from OD */
#define SAVED_RESULT 2      /*!< do nothing, just return the last value */
#define INIT_PARAMS  3      /*!< initialize only parameters, not counters or similar */

/* Definition of Power Boards to be manage by AMC2 */
#define NO_BOARD       	  0x00
#define A3625_01_BOARD    0x01    /*!< + 24Vdc power board with Analog DC current (Idc) */
#define A3625_02_BOARD    0x02    /*!< + 24Vdc power board with Analog two fase currents (Iu, Iw) */
#define A3625_03_BOARD    0x05
#define A3666_01_BOARD    0x03    /*!< +325Vdc power board with Analog two fase currents (Iu, Iw) */
#define A3666_01_C_BOARD  0x04    /*!< +325Vdc power board with Analog two fase currents (Iu, Iw) and Udc */
#define A3666_02_BOARD    0x06    /*!< +325Vdc power board with Analog two fase currents (Iu, Iw) and Udc */
#define EMUL_BOARD        0xFF	   /*!< Emulation board */

/* Masks */
#define GPI2_POLARITY_MASK 0x0002 /*!< mask for gpi 2 value. */

/*! pointer to function that calculates the value of the PWM sensor */
extern int (*calculate_pwm_sensor)(unsigned long high, unsigned long period, unsigned int value);
/*! value of the PWM sensor */
extern unsigned int pwm_sensor_value;

/* pointers to the functions that return values from power board */
extern unsigned (*get_vpower) ( void );
extern short (*get_temp)   ( void );
extern short (*get_current)( void );
extern short (*get_current2)( void );

/* pointer to the function that return value from motor type */
extern short (*get_motor_temp)  ( void);


extern long velocity_counter_filter;

extern unsigned int old_velocity_counter;
extern 	short int vel_calc;

#endif  /* end _PERIPHERALS_H_ definition */


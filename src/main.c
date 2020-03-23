/*!	\file main.c
	\brief Main file of Aries Motion Controller

Includes global variables, function main with initializations and most important software interrupts and threads

\verbatim
*********************************************************************
* File: main.c
* Devices: TMS320F28XXX
* Author: Luis Jimenez
* History:
*   06/11/2006 - original (based on DSP281x header files v1.00, D. Alter)
*********************************************************************
\endverbatim
*/

#include "DSP2833x_Device.h"

#include "amc.h"
#include "fpga.h"
#include "i2c.h"
#include "debug_sci.h"
#include "canopen_amc.h"
#include "errors.h"
#include "dc_bldc_current.h"
#include "quick_stop.h"
//------------------------------------------
// Global variables
//
#define INIT_POS_MODE 		0x01
#define INIT_VEL_MODE 		0x02
#define INIT_ASSISTED_MODE	0x04

#define iq2dacval(x) (unsigned int)((int)((x)+0x07FF))

unsigned short Half_Period = HALF_PERIOD_17KHz;
/*! PWM duty cycle applied to the motor */
_iq duty = _IQ(0);	/*!< [-1,1]  -1 = max_dcha,   1 = max_izq */
_iq demand_duty = _IQ(0);	/*!< [-1,1]  -1 = max_dcha,   1 = max_izq Needed for current control sign compensation*/
/*! duty cycle offset derived from Control effort offset */
_iq duty_offset = _IQ(0);
/*! Limited Duty cicle [-1,1] */
_iq limited_duty = 0;
/*! Control effort [-1,1] calculated in PID regulators (will be assigned to 'duty') */
_iq control_effort = _IQ(0);
_iq control_effort_plus = _IQ(0);
_iq control_effort_pos_mm = _IQ(0);
_iq control_effort_vel_mms= _IQ(0);
/*! Control limits for Zero PWM Offset */
_iq up_lim_zeropwm_offset = _IQ(0);
_iq down_lim_zeropwm_offset = _IQ(0);
_iq i_measured;
_iq i_current;

short PWM_reference_inst = 0;
short current = 0;			/*!< Servo dc current measured in mA */
short current_total = 0;    /*!< Servo dc current measured in mA */
short current1 = 0;			/*!< Servo current phase Iu measured in mA */
short current2 = 0;			/*!< Servo current phase Iv measured in mA */
short current3 = 0;			/*!< Servo current phase Iw measured in mA */

int current_limit = 0;		/*!< Current limit in mA, comes from Max_current and Motor_rated_current */
int current_limit_acc = 0;		/*!< Current limit in mA used during acceleration */
int current_limit_ip = 0;		/*!< Current limit in mA used in IP mode */
int *p_current_limit = &current_limit;	/*!< pointer to the current limit used */
unsigned voltage = 0;			/*!< Servo dc voltage in dV */
unsigned resolver_in1        = 0; /*!< analog input value 1 of the resolver */
unsigned resolver_in2        = 0; /*!< analog input value 2 of the resolver */
unsigned an1                 = 0; /*!< General purpose analog input 1 */
unsigned an2                 = 0; /*!< General purpose analog input 2 */
unsigned an3                 = 0; /*!< General purpose analog input 3 */
unsigned an4                 = 0; /*!< General purpose analog input 4 */
unsigned thermistor          = 0; /*!< Thermistor value at the motor */
unsigned analog_motorcurrent = 0; /*!< Current */
unsigned analog_motorcurrent2 = 0; /*!< Current 2 */
unsigned analog_ntc          = 0; /*!< voltage of NTC used as thermistor */
unsigned analog_vpower       = 0; /*!< voltage from analog optocoupler */
unsigned analog_vpower2       = 0; /*!< voltage from analog external optocoupler */
unsigned analog_ref1v5       = 0; /*!< voltage reference of 1.5V for resolver */
unsigned long analog_hybrid       = 0; /*!< hybrid position 32 bits measurement */

unsigned sum_an1                 = 0; /*!< filter sum of an1 */
unsigned sum_an2                 = 0; /*!< filter sum of an2 */
unsigned sum_an3                 = 0; /*!< filter sum of an3 */
unsigned sum_an4                 = 0; /*!< filter sum of an4 */
unsigned long sum_thermistor          = 0; /*!< filter sum of thermistor */
unsigned long sum_analog_ntc          = 0; /*!< filter sum of analog_ntc */
//unsigned sum_analog_vpower       = 0; /*!< filter sum of analog_vpower */
unsigned long sum_analog_ref1v5       = 0; /*!< filter sum of analog ref1v5 */
unsigned long sum_current        = 0; /*!< filter sum of current */
unsigned long sum_voltage        = 0; /*!< filter sum of voltage */
unsigned int dimitri			 = 0;

_iq resolver_angle = 0;               /*!< angle measured by the resolver */
_iq fixed_freq_angle = 0;
long accumulated_sin = 0;        /*!< resolver sine, used for filtered measure of the resolver */
long accumulated_cos = 0;        /*!< resolver cosine, used for filtered measure of the resolver */

unsigned long adc_gain = 1000L*524;		/*!< ADC calibration gain (instead of divide by 1000, we will multiply by 524 and shift right 19) */
int adc_offset = 0;			/*!< ADC calibration offset */

unsigned ZeroCurrent_motor = 0;	/*!< ADC measure of the motor current sensor when current = 0A (should be ~0.5v) */
unsigned ZeroCurrent_motor2 = 0;  /*!< ADC measure of the motor current sensor when current = 0A (should be ~0.5v) */
int ZeroCurrent_set = 0;            /*!< when '1' indicates that ZeroCurrent has been read from initial adc current */

velocity_trajectory_struct pv_trajectory;		/*!< Struct that keeps velocity trajectory in pv mode */
velocity_trajectory_struct hm_trajectory;		/*!< Struct that keeps velocity trajectory in hm mode */
velocity_trajectory_struct am_trajectory;		/*!< Struct that keeps velocity trajectory in am mode */
position_trajectory_struct pp_trajectory;		/*!< Struct that keeps position trajectory in pp mode */
interpolated_trajectory_struct ip_trajectory;	/*!< Struct that keeps interpolated position trajectory in ip mode */
circle_trajectory_struct circle_trajectory;		/*!< Struct that keeps trajectory in circle mode */
PIDREG pv_control_pid = PIDREG_DEFAULTS;		/*!< PID struct for velocity control */
PIDREG pv_control_pid_mms = PIDREG_DEFAULTS;		/*!< PID struct for velocity control */
PIDREG am_control_pid_mms = PIDREG_DEFAULTS;		/*!< PID struct for velocity control */
PIDCONTROLLER pv_position_control_pid = PIDCONTROLLER_DEFAULTS; /*!< PID struct for detents velocity control */
#if 1
PIDCONTROLLER pv_position_control_pid_test = PIDCONTROLLER_DEFAULTS; /*!< PID struct for detents velocity control */
#endif
PIDREG position_control_pid = PIDREG_DEFAULTS;	/*!< PID struct for position control */
PIDREG position_control_pid_mms = PIDREG_DEFAULTS;	/*!< PID struct for position control */
PIDREG16 position_control_pid_mms_iq16 = PIDREG16_DEFAULTS;
PIDREG pos_vel_control_pid_mms = PIDREG_DEFAULTS;	/*!< PID struct for position control */
#if 0
PIDREG current_control_pid = PIDREG_DEFAULTS;	/*!< PID struct for position control */
#endif

unsigned int lcounts_p_s = 0;		/*!< global variable set in main() that converts ltime in seconds */

int send_SWord = 0;	/*!< global variable used to force StatusWord sending in every cycle (protected by CanFestival mutex) */

int power_on = 0;		     /*!< Variable that determines when applying power to the drive (during "Operation Enable" state and maybe "Quick Stop Active" or "Fault Reaction Active") */
int ready_to_power_on = 0;  /*!< Variable to drive power_on within the pwm_out function to avoid jerky starts */
int init_mode = 0;		/*!< Determines if the mode of operations has to be initialized in the next cycle (if it has just enabled or mode changed) */
int change_mode = 0;		/*!< Determines if the mode of operations has to be changed (in the next cycle or when it's possible) */

int nodeID = 0;	/*!< CANopen nodeID read from DIP switch (switch1=LSB, switch7=MSB) */

motor_type motor = NO_MOTOR;		/*!< indicates the type of the motor (DC, BLDC...) */

long long edge_time = 0;		/*!< To store the timer value when a edge has been detected by the homing sensor */

unsigned long start_time, end_time, last_start_time = 0, debug_time; /* Study occupacy of DSP */
int pwm_out_time = 0, pwm_period = 0;		/*!< Used to store the busy time of pwm_out() */
int pwm_out_time_max = 0, pwm_out_time_min = 0;
int pwm_out_time_mean = 0, pwm_out_time_max_mean = 0, pwm_out_time_min_mean = 0;
int pwm_period_max = 0, pwm_period_min = 0;
int pwm_period_mean = 0, pwm_period_max_mean = 0, pwm_period_min_mean = 0;

/* Define Constant Coefficient Array and place the .econst/.const section in non-volatile memory */
const long coeff[(FIR_ORDER+2)/2] = FIR16_COEFF;		/*!< current FIR filter coefficients */

/* Define a buffer for serial log */
unsigned char log_buff[SERIAL_LOG_BUFFER_SIZE];		/*!< serial log buffer */
unsigned buff_tosend = 0;		/*!< index of the position of the next character in the serial log buffer to be send */
unsigned buff_towrite = 0;		/*!< index of the position in the serial log buffer for the next char to be written */

unsigned long pos_error_normFactor = 0;		/*!< Factor to normalize position error */
unsigned long vel_normFactor = 0;		/*!< Factor to normalize velocity */

unsigned long pos2int_factor_num = 1;	/*!< numerator of the factor to convert position increments to internal units */
unsigned long pos2int_factor_den = 1;	/*!< denominator of the factor to convert position increments to internal units */

unsigned long vel2int_factor_num = 1;	/*!< numerator of the factor to convert velocity increments to internal units */
unsigned long vel2int_factor_den = 1;	/*!< denominator of the factor to convert velocity increments to internal units */

/*! variable to signal that safety signal is disabled */
int safety_disable = 0;

unsigned int position_flags = 0;    /*!< some flags to signal if position limits or switches have been reached */

int manual_with_clutch = 0;
int reset_filters = 0;

int ramp_time = 0;
int mode_change_time = 0;
static int power_state_delay=0;





int homing_zeroed = 0; //static int homing_zeroed = 0;
static int apply_filters = 0;
char force_init =0;

unsigned char flagDebugScia =0;

long accum_resolver_vel = 0;

short final_current = 0;			/*!< Servo dc current measured in mA */
extern int am_current_limit;

extern short debug_velocity_target = 0;

long long tmp_red = 0;


int in_quick_stop =0;

//#define RESOLVER_DEBUG
//uncomment to store the data read of the resolver.
#ifdef RESOLVER_DEBUG
#define RESOLVER_BUFFER_LEN 1024
#pragma DATA_SECTION( resolver_buffer, "storedata" );
_iq resolver_buffer[4096];
#endif

//#define IGNORE_PROGRAM_STATUS_IN_LOADER
//commented, hex file must be burned with amc_loader_master
//uncommented, hex file must be burned with FLASH_BURN (CC). Not burn A sector (Bootloader)
#ifdef IGNORE_PROGRAM_STATUS_IN_LOADER
#ifdef FLASH_PRJ
/* Used to pre-write 0x55AA value in PRG_STATUS to indicate to the loader that Firmware is ok */
#pragma DATA_SECTION(ProgramStatus, "program_status");
const unsigned int ProgramStatus = 0x55AA;		/*!< Status of the Flash memory checked by loader */
#endif
#endif


static void load_eeprom( void )
{
	static int cfg_init = 0;

	while( cfg_init != 2 )
	{
		DelayUs(1000L);
		if( cfg_init == 0 )
		{
			Apply_sensors_configuration = 0;
			CheckAndApplyMotorConfiguration();
			load_configuration();
			cfg_init = 1;
			_LOGmessage(0x0121,"load_eeprom cfg_init_0", 0, 0);
			continue;
		}

		if( cfg_init == 1 )
		{
			store_service_routine();
			if( store_get_status() != STORE_INACTIVE )
				continue;

			switch( store_get_fsm() )
			{
				case STORE_READ_COMPLETED:
					Apply_sensors_configuration=1;
					CheckAndApplyMotorConfiguration();
					cfg_init = 2;
					_LOGmessage(0x0122,"load_eeprom Configuration loaded OK", 0, 0);
					break;

				case STORE_EEPROM_ERROR:
					cfg_init = 2;
					_LOGmessage(0x0123,"load_eeprom Configuration NOT loaded from EEPROM", 0, 0);
					break;

				case STORE_EEPROM_NODE_ERROR:
					cfg_init = 2;
					_LOGmessage(0x0124,"load_eeprom Configuration loaded: Node mismatch", 0, 0);
					break;

				default:
					break;
			}
			continue;
		}
	}
}


/*****************************************************************/
/*!	Main function for Aries Motion controller

Includes initializations and the copy of critical code sections from flash to RAM for a faster execution
*/
/*****************************************************************/
void main(void)
{
/*** CPU Initialization ***/
	InitSysCtrl();		/* Initialize the CPU (FILE: SysCtrl.c) */
	InitXintf();		/* Initialize the external memory interface (FILE: Xintf.c) */
	InitGpio();			/* Initialize the shared GPIO pins (FILE: Gpio.c) */
	InitPieCtrl();		/* Initialize and enable the PIE (FILE: PieCtrl.c) */
	InitSci();			/* Initialize the SCI connection (FILE: Sci.c) */

	#ifdef _DEBUG
	_LOGmessage(0x0025,"DBG-Program start", 0, 0);
	#else
	_LOGmessage(0x0025,"Program start", 0, 0);
	#endif

/*** Copy all FLASH sections that need to run from RAM (use memcpy() from RTS library) ***/
// Section secureRamFuncs contains user defined code that runs from CSM secured RAM
	memcpy(	&secureRamFuncs_runstart,
			&secureRamFuncs_loadstart,
			&secureRamFuncs_loadend - &secureRamFuncs_loadstart);

#ifdef FLASH_PRJ
// Section criticalFuncs contains user defined code that runs from RAM
	memcpy(	&criticalFuncs_runstart,
			&criticalFuncs_loadstart,
			&criticalFuncs_loadend - &criticalFuncs_loadstart);
#endif

/*** Initialize the FLASH ***/
	InitFlash();              /* Initialize the FLASH (FILE: SysCtrl.c) */

/*** set lcounts_p_s ***/
	lcounts_p_s = (CLK_countspms() * 1000) / CLK_getprd(); /* it is set before any possible read access to the variable */

/*** read nodeID from shift register ***/
	nodeID = readNodeID();
	_LOGmessage(nodeID,"Node id", 0, 0);

/*** Peripheral Initialization ***/
	InitAdc();                /* Initialize the ADC (FILE: Adc.c) */
	InitEPwmModules();        /* Initialize the ePwm Modules for transistors and brakes. */
	InitEQep();               /* Initialize the Quadrature Enconder Module. */
	InitECapture();           /* Initialize the ECapture modules */
	InitXintrupt();           /* Initialize the External interrupts for Homing Mode */
	I2CA_Init(I2C_ADDR_7BITS);              /* Initialize I2C Module. */
	InitFPGA();
	InitStore();

	load_eeprom();

/*** CANOpen Initialization ***/
	InitCanopen();

/*** Callbacks that initialize parameters when some OD entries change ***/
	On_CurrentLimitsUpdate((CO_Data*)NULL, (const indextable *)NULL, 0);

/*** Global structs initializtion ***/
	memset(&pv_trajectory, 0, sizeof(pv_trajectory));
	memset(&hm_trajectory, 0, sizeof(hm_trajectory));
	memset(&pp_trajectory, 0, sizeof(pp_trajectory));
	memset(&circle_trajectory, 0, sizeof(circle_trajectory));

	SET_POWER_OFF();
	ATM_seti(&ready_to_power_on, 0);

	/*** Config Debug SCI System      ***/
	InitDataStructDebugSci();



/*** Enable interrupts ***/
	SetDBGIER(IER | 0x6000);							// enable everything in IER, plus TINT2 and DLOGINT
	*(volatile unsigned int *)0x00000C14 |= 0x0C00;		// set TIMER2 free=soft=1

    // DSP/BIOS will enable global interrupts (INTM and DBGM)

} //end of main()

/*****************************************************************/
/*!	This is the user initialization file to be specified in the DSP/BIOS configuration file, System - Global Settings.
*/
/*****************************************************************/
void UserInit(void)
{
// Section .trcdata is generated by DSP/BIOS.
// It must be copied from its load to its run address BEFORE main().
#ifdef FLASH_PRJ
	memcpy(&trcdata_runstart, &trcdata_loadstart, &trcdata_loadend - &trcdata_loadstart);
#endif
} //end of UserInit()




#pragma CODE_SECTION(filter_mabs_pos_ref, "criticalFuncs")
static unsigned int filter_mabs_pos_ref (unsigned int input)
{
    unsigned int i=0;
    _iq16 a1_theta= _IQ16(-0.995522066470442);
    _iq16 b0_theta= _IQ16(0.004477933529558);

    static _iq16 ydifftheta[2]={_IQ16(0.0),_IQ16(0.0)};
    _iq16 input_iq;

    input_iq = _IQ16(input);

    for (i=1;i>0;i--)
        ydifftheta[i]=ydifftheta[i-1];

    ydifftheta[0]=    _IQ16mpy(b0_theta,input_iq)
                    -_IQ16mpy(a1_theta,ydifftheta[1]);

    return (ydifftheta[0]>>16);
}

#pragma CODE_SECTION(adc_read_seq1, "criticalFuncs")
/*****************************************************************/
/*!	 ADC sequence 1 ISR
*/
/*****************************************************************/
void adc_read_seq1(void)
{


	start_time = CLK_gethtime(); // time in cpu ticks

	ATM_setu(&analog_motorcurrent, AdcMirror.ADCRESULT14);
	ATM_setu(&analog_vpower,  AdcMirror.ADCRESULT1);
	/* AMQ: measurements for BLAC motors */
	ATM_setu(&analog_motorcurrent2, AdcMirror.ADCRESULT15);
	ATM_setu(&analog_vpower2, AdcMirror.ADCRESULT1);

	sum_analog_ref1v5       = FILTER_SUM_3( AdcMirror.ADCRESULT2 , sum_analog_ref1v5 );
	sum_an1                 = FILTER_SUM( AdcMirror.ADCRESULT4 , sum_an1 );
	sum_an2                 = FILTER_SUM( AdcMirror.ADCRESULT5 , sum_an2 );
	sum_an3                 = 0;
	sum_an4                 = 0;
	dimitri					= filter_mabs_pos_ref(AdcMirror.ADCRESULT4);

	AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;		/* Reset ADC Sequencer1 */
	AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;		/* Clear Interrupt Flag ADC Sequencer 1 */


	DINT;
	current1 = get_current();
	current3 = get_current2();
	current_total = current1 + current3;
	current2 = -current1-current3;
	current  = ( abs(current1)+abs(current2)+abs(current3) ) >> 1; //"Ipico" del vector de corriente. No es rms

	voltage = get_vpower();
	EINT;


	/* filter current, only to show a filtered value in the OD, it is used unfiltered for limits */
	//sum_current = (((sum_current * 31) + 16) >> 5) + current;
	sum_current = FILTER_SUM_2(current,sum_current);
	sum_voltage = FILTER_SUM_2(voltage,sum_voltage);
	//AMQ TODO: fix sum_analog_vpower, voltage, sum_voltage and vpower in the new version of control board

#if 0
	//Alive led for switching function
	{
		static int counter = 0;
		#ifdef _DEBUG
			if(counter++ >= 7500)
		#else
			if(counter++ >= 750)
		#endif
		{
			TOGGLE_GP_LED_1;
			counter = 0;
		}
	}
#endif
	Configure_FPGA_system_watchdog( FPGA_SYSTEM_WATCHDOG_TASK1_MASK, FPGA_SYSTEM_WATCHDOG_TASK1_MASK );
	Configure_FPGA_move_watchdog( FPGA_MOVE_WATCHDOG_TASK1_MASK, FPGA_MOVE_WATCHDOG_TASK1_MASK );
	EALLOW;
	SysCtrlRegs.WDKEY = 0x55;		// Serve watchdog #1
	EDIS;
	pwm_out();
}


#pragma CODE_SECTION(adc_read_seq2, "criticalFuncs")
/*****************************************************************/
/*!	 ADC sequence 2 ISR
*/
/*****************************************************************/
void adc_read_seq2(void)
{
	static unsigned int r1_pos = 0, r1_neg = 0, r2_pos = 0, r2_neg = 0;
	int sin_diff, cos_diff;
	static int sin_diff_prev = 0, cos_diff_prev = 0;
	static _iq resolver_angle_prev = 0;
	_iq resolver_diff;
	//static long accum_resolver_vel = 0;

#ifdef RESOLVER_DEBUG
	_iq a;
	static unsigned i = 0;
#endif


	ATM_setu(&resolver_in1, AdcMirror.ADCRESULT8 + AdcMirror.ADCRESULT10 + AdcMirror.ADCRESULT12);
	ATM_setu(&resolver_in2, AdcMirror.ADCRESULT9 + AdcMirror.ADCRESULT11 + AdcMirror.ADCRESULT13);
	sum_thermistor          = FILTER_SUM_3( AdcMirror.ADCRESULT7 , sum_thermistor );
	sum_analog_ntc          = FILTER_SUM_3( AdcMirror.ADCRESULT0 , sum_analog_ntc );

	AdcRegs.ADCTRL2.bit.RST_SEQ2 = 1;			// Reset SEQ2 to CONV00 state
	AdcRegs.ADCST.bit.INT_SEQ2_CLR = 1;			// Clear ADC SEQ2 interrupt flag


	/* calculate angle */
	if (RESOLVER_PWM_PIN == 1)   /* resolver ref has 180��� phase shift from square signal */
	{
		r1_neg = resolver_in1;
		r2_neg = resolver_in2;
	} else
	{
		r1_pos = resolver_in1;
		r2_pos = resolver_in2;
	}

	cos_diff = r1_pos - r1_neg;
	sin_diff = r2_pos - r2_neg;

	/* filter sin_diff and cos_diff, not used for phase commutation */
	//accumulated_cos = ((accumulated_cos * 3) + 2 >> 2) + cos_diff;
	//accumulated_sin = ((accumulated_sin * 3) + 2 >> 2) + sin_diff;
	accumulated_cos = (((accumulated_cos * 7) + 4) >> 3) + cos_diff;
	accumulated_sin = (((accumulated_sin * 7) + 4) >> 3) + sin_diff;

	/* _IQatan2 returns values in [-pi, pi] range */
	/* resolver_angle = _IQatan2((accumulated_sin + 4) >> 3, (accumulated_cos + 4) >> 3); */
	/* calculate resolver_angle almost without filter for minimum delay */
	resolver_angle = _IQatan2PU(sin_diff + sin_diff_prev, cos_diff + cos_diff_prev);
	/* _IQatan2PU returns values in [0, 1] range */

	cos_diff_prev = cos_diff;
	sin_diff_prev = sin_diff;

#ifdef RESOLVER_DEBUG
	a = (unsigned int)(_IQsqrt((long)sin_diff*sin_diff + (long)cos_diff*cos_diff) >> (GLOBAL_Q/2));  /* amplitude */
	resolver_buffer[i++] = resolver_angle;
	if (i == RESOLVER_BUFFER_LEN )
	{
		i = 0;
	}
#endif

	/* get resolver velocity to forward angle according to it */
	resolver_diff = (((resolver_angle - resolver_angle_prev) + _IQ(0.5)) & (_IQ(1.0) - 1)) - _IQ(0.5);
	resolver_angle_prev = resolver_angle;
	/* filter */
	accum_resolver_vel = (((accum_resolver_vel * 7) + 4) >> 3) + resolver_diff;

	/* forward angle for ~400us=8*50us */
	/* resolver_vel = accum_resolver_vel >> 3; */
	resolver_angle += 8 * (accum_resolver_vel >> 3);

	if(WD_Debug_variable == 0xD2) while(1) {};

}

#pragma CODE_SECTION(pwm_out, "criticalFuncs")
/*****************************************************************/
/*!	function that actually generates PWM's
*/
/*****************************************************************/

void pwm_out(void)
{
	unsigned int positive, negative;
	unsigned char hall_state;
	_iq i_factor;
	/* _iq limited_duty; Move to global variable to publicate in OD, periodic_function*/
	_iq resolver_angle_1turn, anglePU_u, anglePU_v, anglePU_w, sin_u, sin_v, sin_w;
	static int prg_val = 0;
	int offset;

	//static int dac_aux = 0;


	/* read hall sensors inputs from GPIO50-52 */
	hall_state = ((GpioDataRegs.GPBDAT.all & 0x001D0000) >> 18);	// bits 0,1,2 = H1,H2,H3

	/* manage BLAC motor */
	if((motor == BLAC_MOTOR_a) || (motor == BLAC_MOTOR_b)) {
		/* rotor angle, used in vectorial and scalar control */
		fixed_freq_angle = fixed_freq_angle + Fixed_Freq;
		resolver_angle_1turn=(Enable_Fixed_Freq?fixed_freq_angle:(resolver_angle<<BLAC_Motor_Poles)) & (_IQ(1.0)-1);

	}
	debug_time = CLK_gethtime(); // time in cpu ticks

	/* power_on is atomically read because it is a 16-bit variable */
	ATM_seti(&power_on, ready_to_power_on);		/* drive operation disabled (atomical write) */
	if((power_on == 0))
	{
		static float sumZeroCurrent = 0, sumZeroCurrent2 = 0;

		brakes_operation();
		control_effort = 0;

		/* Set zerocurrent filtered if no pwm active */
		sumZeroCurrent = (sumZeroCurrent*0.999) + ((float)analog_motorcurrent*0.001);
		sumZeroCurrent2 = (sumZeroCurrent2*0.999) + ((float)analog_motorcurrent2*0.001);

		ATM_setu(&ZeroCurrent_motor, (unsigned)sumZeroCurrent);
		ATM_setu(&ZeroCurrent_motor2,(unsigned)sumZeroCurrent2);
		ZeroCurrent_set = 1;

//		TODO Activate ZeroCurrent Window depending on power board
//		if((sumZeroCurrent > 1935) && (sumZeroCurrent < 2095))
//		{
//			//ATM_setu(&ZeroCurrent_motor, sumZeroCurrent);
//			ZeroCurrent_motor = sumZeroCurrent;
//			if((sumZeroCurrent2 > 1935) && (sumZeroCurrent2 < 2095))
//			{
//				//ATM_setu(&ZeroCurrent_motor2, sumZeroCurrent2);
//				ZeroCurrent_motor2 = sumZeroCurrent2;
//				ZeroCurrent_set = 1;
//			}
//			else
//			{
//				ZeroCurrent_set = 0;
//			}
//		}
//		else
//		{
//			ZeroCurrent_set = 0;
//		}TickDebugSci8bytesPositionMode();
		DISABLE_MOTOR;
	}	/* all 6 PWM's off (free motor movement) */
	else
	{
		/* offset used for frequency spectrum dispersion using a Linear Congruential Generator */
		/* 17kHz => 16.1-18.1 KHz */
		prg_val = (prg_val * 13 + 103) & 0x00ff;
		offset = prg_val - 128;
		offset = 0; //TODO: (AMQ) volver a activar la dispersion en frecuencia (Spread Spectrum)

		/* Apply frecuency dispersion to the period of PWMs */
		EPwm1Regs.TBPRD = Half_Period + offset;
		EPwm2Regs.TBPRD = Half_Period + offset;
		EPwm3Regs.TBPRD = Half_Period + offset;

		if((char)Modes_of_operation_display == OPERATION_MODE_MOTOR_CHECK)
		{
			apply_motor_phase_check();
		}
		else
		{
			switch((char)Modes_of_operation_display)
			{
				case OPERATION_MODE_MANUAL:
				case OPERATION_MODE_MANUAL_2:
					switch(motor)
					{
						case BLDC_MOTOR_c:
						case BLDC_MOTOR_a:
						case BLDC_MOTOR_b:
							/* Set motor current sign in final_current */
							eval_hall_state(hall_state, motor);
							i_measured=_IQdiv(final_current,current_limit);
							i_current = current_controller_dc_bldc(control_effort, i_measured);
							positive = _IQmpyI32int(_IQmpy(_IQ(1.0) - i_current, _IQ(0.5)), Half_Period + offset);
							negative = Half_Period + offset - positive;
							break;
						default:
							break;
					}
				break;
				case OPERATION_MODE_POSITION:
				case OPERATION_MODE_VELOCITY:
					/* If Assisted Mode current control active*/
					switch(motor)
					{
						case BLAC_MOTOR_b:
						case BLAC_MOTOR_a:
							break;
						case DC_MOTOR_a:
							final_current = current_total;
							i_measured=_IQdiv(final_current,current_limit);
							i_current = current_controller_dc_bldc(control_effort, i_measured);
							positive = _IQmpyI32int(_IQmpy(_IQ(1.0) - i_current, _IQ(0.5)), Half_Period + offset);
							negative = Half_Period + offset - positive;
                            break;
						case DC_MOTOR_b:
							final_current = -current_total;
							i_measured=_IQdiv(final_current,current_limit);
							i_current = current_controller_dc_bldc(control_effort, i_measured);
							positive = _IQmpyI32int(_IQmpy(_IQ(1.0) - i_current, _IQ(0.5)), Half_Period + offset);
							negative = Half_Period + offset - positive;
							break;
						case BLDC_MOTOR_c:
						case BLDC_MOTOR_a:
						case BLDC_MOTOR_b:
							/* Set motor current sign in final_current */
							eval_hall_state(hall_state, motor);
							i_measured=_IQdiv(final_current,current_limit);
							i_current = current_controller_dc_bldc(control_effort, i_measured);
							positive = _IQmpyI32int(_IQmpy(_IQ(1.0) - i_current, _IQ(0.5)), Half_Period + offset);
							negative = Half_Period + offset - positive;
							break;
						case NO_MOTOR:
							break;
						default:
							break;
					}
				break;
				case OPERATION_MODE_ASSISTED:
				case OPERATION_MODE_ASSISTED_2:
					/* If Assisted Mode current control active*/
					switch(motor)
					{
						case BLAC_MOTOR_b:
						case BLAC_MOTOR_a:
							break;
						case DC_MOTOR_a:
						case DC_MOTOR_b:
							positive = 0;
							negative = 0;
							break;
						case BLDC_MOTOR_c:
						case BLDC_MOTOR_a:
						case BLDC_MOTOR_b:
							/* Set motor current sign in final_current */
							eval_hall_state(hall_state, motor);

		//					i_measured = _IQmpy((((long long)final_current << GLOBAL_Q)/ current_limit),_IQ(1.0));
							//i_measured = _IQ((long)(final_current/current_limit));
							i_measured=_IQsat(_IQdiv(final_current,current_limit),_IQ(1.0),_IQ(-1.0));
							i_current = current_controller_dc_bldc(control_effort, i_measured);

							positive = _IQmpyI32int(_IQmpy(_IQ(1.0) - i_current, _IQ(0.5)), Half_Period + offset);
							negative = Half_Period + offset - positive;

							break;
						case NO_MOTOR:
							break;
						default:
							break;
					}
					break;

				default:
					/* Normal operation current control not active*/
				if (!in_quick_stop)
				{

					/*limit duty cycle according to current */
					i_factor = currentFactor(current);
					limited_duty = _IQsat(_IQmpy(duty,i_factor),MAX_DUTY_CYCLE,MIN_DUTY_CYCLE); /* limit pwm duty cycle if high current */

					/* duty can be simply read, because it is modified in a tsk, and a swi cannot be interrupted by a tsk */
					positive = _IQmpyI32int(_IQmpy(_IQ(1.0) - limited_duty, _IQ(0.5)), Half_Period + offset);
					negative = Half_Period + offset - positive;
				}
				else
				{
					switch(motor)
					{
					case BLDC_MOTOR_c:
					case BLDC_MOTOR_a:
					case BLDC_MOTOR_b:
						/* Set motor current sign in final_current */
						eval_hall_state(hall_state, motor);
						i_measured=_IQdiv(final_current,current_limit);
						i_current = current_controller_dc_bldc(control_effort, i_measured);
						positive = _IQmpyI32int(_IQmpy(_IQ(1.0) - i_current, _IQ(0.5)), Half_Period + offset);
						negative = Half_Period + offset - positive;
						break;
					case NO_MOTOR:
						break;
					default:
						/*limit duty cycle according to current */
						i_factor = currentFactor(current);
						limited_duty = _IQsat(_IQmpy(duty,i_factor),MAX_DUTY_CYCLE,MIN_DUTY_CYCLE); /* limit pwm duty cycle if high current */

						/* duty can be simply read, because it is modified in a tsk, and a swi cannot be interrupted by a tsk */
						positive = _IQmpyI32int(_IQmpy(_IQ(1.0) - limited_duty, _IQ(0.5)), Half_Period + offset);
						negative = Half_Period + offset - positive;
						break;
					}

				}
				break;
			}

			switch(motor)
			{
			case DC_MOTOR_a:
				ENABLE_PWM1(positive);
				ENABLE_PWM2(negative);
				DISABLE_PWM3;
				break;
			case DC_MOTOR_b:
				ENABLE_PWM1(negative);
				ENABLE_PWM2(positive);
				DISABLE_PWM3;
				break;
			case BLDC_MOTOR_c:		/* Delta motors DPM42 */
			{
				unsigned int tmp = negative;
				negative = positive;	/* the sequence is the same as sequence A, but with motor rotating in the opposite direction */
				positive = tmp;
			}
			case BLDC_MOTOR_a:			/* Delta motor */
				hall_state ^= 7;	/* because of inverting Schmitt Trigger in inputs */
			case BLDC_MOTOR_b:			/* Maxon and Pini motors use inverted hall signals compared to delta motor ones */
				BLDC_apply_pwm(positive, negative, hall_state);
				break;
			case BLAC_MOTOR_b:
				limited_duty = _IQsat(_IQmpy(limited_duty,_IQ(-1.0)),_IQ(1.0),_IQ(-1.0)); /* Change PWM output according to room geometric model */
				demand_duty = _IQsat(_IQmpy(control_effort,_IQ(-1.0)),_IQ(1.0),_IQ(-1.0)); /* Change PWM output according to room geometric model */
			case BLAC_MOTOR_a:

				vc_analisys(resolver_angle_1turn);

				/* set duty*sines in the comparators */

				switch((char)Modes_of_operation_display)
				{
				case OPERATION_MODE_POSITION:
				case OPERATION_MODE_VELOCITY:
				case OPERATION_MODE_ASSISTED:
				case OPERATION_MODE_ASSISTED_2:
					/* If Assisted Mode Vectorial current control active*/
					vc_control(demand_duty, offset);
					break;
				default:
					/* 3-phase Scalar Orientation */
					/* get sinosoids origins for Lenze motor */
					anglePU_u = (resolver_angle_1turn - _IQ(0.25)) & (_IQ(1.0)-1);
					anglePU_v = (anglePU_u - _IQ(0.3333)) & (_IQ(1.0)-1);
					anglePU_w = (anglePU_u + _IQ(0.3333)) & (_IQ(1.0)-1);

					/* get sines from 512 points ROM table */
					sin_u = _IQ30toIQ(*(long *)(0x3FE000 + (((anglePU_u + (1 << (GLOBAL_Q - 9 - 1)) >> GLOBAL_Q - 9) & 0x1ff) << 1)));
					sin_v = _IQ30toIQ(*(long *)(0x3FE000 + (((anglePU_v + (1 << (GLOBAL_Q - 9 - 1)) >> GLOBAL_Q - 9) & 0x1ff) << 1)));
					sin_w = _IQ30toIQ(*(long *)(0x3FE000 + (((anglePU_w + (1 << (GLOBAL_Q - 9 - 1)) >> GLOBAL_Q - 9) & 0x1ff) << 1)));

					/* enable and apply duty cicle to EPwmXRegs.CMPA */
					ENABLE_PWM1(_IQmpyI32int(_IQmpy(_IQ(1.0) - _IQmpy(limited_duty, sin_u), _IQ(0.5)),Half_Period + offset));
					ENABLE_PWM2(_IQmpyI32int(_IQmpy(_IQ(1.0) - _IQmpy(limited_duty, sin_v), _IQ(0.5)),Half_Period + offset));
					ENABLE_PWM3(_IQmpyI32int(_IQmpy(_IQ(1.0) - _IQmpy(limited_duty, sin_w), _IQ(0.5)),Half_Period + offset));
					break;
				}
				break;
				case NO_MOTOR:
					DISABLE_MOTOR;
					break;
				default:
					DISABLE_MOTOR;
					break;
			}
			//brakes_operation();

		}
		brakes_operation();
	}
	end_time = CLK_gethtime();

	/* Profile of pwm_out interruption */
	{
		unsigned long aux1, aux2;

		//if(start_time > end_time)	aux1 = 4294967296 - start_time;
		//else aux1 = - start_time;
		aux1 = (start_time>end_time?4294967296:0) - start_time;
		pwm_out_time = end_time + aux1;  // time in cpu ticks
		if(pwm_out_time > PWM_Out_Time_Limit) {
			_WARNINGmessage(0xff0d, 0x01, pwm_out_time, "Sw Interruption exceed time", 0, 0); }

		if(last_start_time > start_time) {
			aux2 = 4294967296 - last_start_time;
			//Reset maximun and minimum
			pwm_out_time_max=0;
			pwm_out_time_min=0x7FFF;
		}
		else aux2 = - last_start_time;
		//aux2 = (last_start_time>start_time?4294967296:0) - last_start_time;
		pwm_period = start_time + aux2;
		last_start_time = start_time;
		if( (pwm_period > PWM_Period_Limit_Max) || (pwm_period < PWM_Period_Limit_Min) ) {
			_WARNINGmessage(0xff0e, 0x01, pwm_period, "Sw Interruption Period wrong", 0, 0); }

		pwm_out_time_max = pwm_out_time>pwm_out_time_max?pwm_out_time:pwm_out_time_max;
		pwm_out_time_min = pwm_out_time<pwm_out_time_min?pwm_out_time:pwm_out_time_min;
		pwm_out_time_mean = ((long)pwm_out_time_mean * 15 + pwm_out_time) >> 4;
		pwm_out_time_max_mean = (pwm_out_time_max_mean + pwm_out_time_max) >> 1;
		pwm_out_time_min_mean = (pwm_out_time_min_mean + pwm_out_time_min) >> 1;

		pwm_period_max = pwm_period>pwm_period_max?pwm_period:pwm_period_max;
		pwm_period_min = pwm_period<pwm_period_min?pwm_period:pwm_period_min;
		pwm_period_mean = ((long)pwm_period_mean * 15 + pwm_period) >> 4;
		//pwm_period_max_mean = (pwm_period_max_mean + pwm_period_max) >> 1;
		//pwm_period_min_mean = (pwm_period_mean + pwm_period_min) >> 1;
		aux1 = (start_time>debug_time?4294967296:0) - start_time;
		pwm_period_max_mean = debug_time + aux1;  // time in cpu ticks
		pwm_period_min_mean =((long)pwm_period_min_mean * 15 + pwm_period_max_mean) >> 4;

		#ifdef __VECT_CONTROL_DEBUG
		DINT;
		vect_control_buffer1[index_vect_control_debug] = pwm_out_time;
		vect_control_buffer2[index_vect_control_debug] = pwm_out_time_mean;
		vect_control_buffer3[index_vect_control_debug] = pwm_period_max_mean;
		vect_control_buffer4[index_vect_control_debug] = pwm_period_min_mean;
		vect_control_buffer5[index_vect_control_debug] = pwm_period;
		vect_control_buffer6[index_vect_control_debug] = pwm_period_mean;
		index_vect_control_debug++;
		index_vect_control_debug = (index_vect_control_debug==VECT_CONTROL_BUFFER_LEN?0:index_vect_control_debug);
		EINT;
		#endif

	}

	if(WD_Debug_variable == 0xD3) while(1) {};

}

#pragma CODE_SECTION(currentFactor, "criticalFuncs")
/*****************************************************************/
/*!	 Function that limits duty cycle if current is too high
        \param curr current in mA
        \return 0 if current > current_limit; 1 if current < 15/16 * current_limit and a proportional value between them
*/
/*****************************************************************/
_iq currentFactor(int curr)
{
	int c_limit;

	DINT;
		c_limit = *p_current_limit;
	EINT;
         if(c_limit)
        {
                _iq aux = (8L << GLOBAL_Q) / c_limit;
                if(curr > c_limit) return _IQ(0);
                else  return(_IQsat(_IQmpyI32(aux,(c_limit - curr)),_IQ(1),_IQ(0)));
        } else return _IQ(0);
}



/*****************************************************************/
/*!	 Periodic function (1 ms) that sends motion state to peridic_control_tsk
*/
/*****************************************************************/


void periodic_control_function(void)
{
	static motion_state_struct motion_state = {0,0,0};

	/* Apply to the bridge last cycle regulator output */
	DINT;
	duty = control_effort + duty_offset;		/* protect duty from being read while it is being written */
	EINT;

	/* Filter of variables from adc*/
	an1                 = dimitri;//FILTER_OUT(sum_an1);
	an2                 = FILTER_OUT(sum_an2);
	an3                 = FILTER_OUT(sum_an3);
	an4                 = FILTER_OUT(sum_an4);
	thermistor          = FILTER_OUT_3(sum_thermistor);
	analog_ntc          = FILTER_OUT_3(sum_analog_ntc);
	analog_ref1v5       = FILTER_OUT_3(sum_analog_ref1v5);

	/* get position, velocity and time */
	getMotionState(&motion_state);

	if(!MBX_post(&motion_state_mbox, &motion_state, 0))
	{
		_WARNINGmessage(0xfff8, 0x01, 0x0000, "motion_state_mbox full", 0, 0);
	}

	if(WD_Debug_variable == 0xD4) while(1) {};

}


/*****************************************************************/
/*!	 Function that implements periodic control function
*/
/*****************************************************************/
void periodic_control_tsk(void)
{
	long long tmp;
	motion_state_struct state = {0,0,0};
	//unsigned adcMotorCurrent;
	int drive_state;
	static int last_drive_state = 0;
	static int control_period_error = 0;
	char init;
	short int old_statusword;
	UNS16 old_FPGA_mode=0;
	int quick_stop_mode = 0;
	static int reset = 0;
	//static unsigned short reset_motor_phase_error_counter = 0;

	while(1)	/* infinite loop */
	{

		if(!MBX_pend(&motion_state_mbox, &state, 4))		/* wait for mailbox at most 2 ms */
		{
			_LOGmessage(0x0027,"control period delayed", 0, 0);
			control_period_error++;
		}
		else
		{
			control_period_error--;
		}
		control_period_error = CLAMP( control_period_error, 0, n_CONTROL_CYCLE_ERRRORS );
		if( control_period_error == n_CONTROL_CYCLE_ERRRORS )
		{
			if(!isFaultActive(FAULT_CONTROL_CYCLE))
			{
				_ERRORmessage(0xff02, 0x01, 0x0000, "control cycle period error", 0, 0);
				setFault(FAULT_CONTROL_CYCLE);
			}
			QueueFault(FAULT_CONTROL_CYCLE);
		}
		else if( control_period_error == 0 )
		{
			DeQueueFault(FAULT_CONTROL_CYCLE);
		}


		EnterMutex();		/* CanFestival mutex (for accessing OD) */

		send_SWord = 0;	/* initialize send_SWord in this cycle */

		/* show control effort in the OD */
		Control_effort = _IQmpyI32int(control_effort, 10000);	/* values in range [-10000, 10000] */

		/* Calculate limits for zero PWM offset */
		up_lim_zeropwm_offset   =  _IQ( abs(Zero_Pwm_Window) / 10000.0);
		down_lim_zeropwm_offset =  _IQ(-abs(Zero_Pwm_Window) / 10000.0);
		Limited_Duty = _IQmpyI32int(limited_duty, 10000); /* values in range [-10000, 10000] */

		/* show current in mA */
		//Motor_current = (sum_current + 16) >> 5;      /* current in mA */
		Motor_current = FILTER_OUT_2(sum_current);      /* current in mA */
		Motor_current_Iu = current1;
		Motor_current_Iv = current2;
		Motor_current_Iw = current3;

		/* show vectorial currents in mA */
		DINT;
		Motor_current_Id = Id >> Q_CURRENT;  /* Motor_current_Ix in mA */
		Motor_current_Iq = Iq >> Q_CURRENT;
		//Motor_current_absI = modI >> Q_CURRENT;
		Motor_current_absI = current;
		Motor_current_absI = current;

		EINT;

		/* show current control variables */
		DINT;
		Current_control_variables_Ud_ref = Ud >> Q_CURRENT;
		Current_control_variables_Uq_ref = Uq >> Q_CURRENT;
		Current_control_variables_duty_a = duty_a >> (Q_DUTY-10); /* Show in IQ10 [-1024, 1024]*/
		Current_control_variables_duty_b = duty_b >> (Q_DUTY-10);
		Current_control_variables_duty_c = duty_c >> (Q_DUTY-10);
		Current_control_variables_EPwm1_CMPA = EPwm1Regs.CMPA.half.CMPA;
		Current_control_variables_EPwm2_CMPA = EPwm2Regs.CMPA.half.CMPA;
		Current_control_variables_EPwm3_CMPA = EPwm3Regs.CMPA.half.CMPA;
		EINT;

		#ifdef VECT_CONTROL_DEBUG
		DINT;
		Current_control_variables_st_debug_index = index_vect_control_debug;
		Current_control_variables_debug_var1 = vect_control_buffer1[Current_control_variables_debug_index];
		Current_control_variables_debug_var2 = vect_control_buffer2[Current_control_variables_debug_index];
		Current_control_variables_debug_var3 = vect_control_buffer3[Current_control_variables_debug_index];
		Current_control_variables_debug_var4 = vect_control_buffer4[Current_control_variables_debug_index];
		Current_control_variables_debug_var5 = vect_control_buffer5[Current_control_variables_debug_index];
		Current_control_variables_debug_var6 = vect_control_buffer6[Current_control_variables_debug_index];
		EINT;
		#endif

		DINT;
		PWM_Out_Time_Mean     = pwm_out_time_mean;
		PWM_Out_Time_Max      = pwm_out_time_max;
		PWM_Out_Time_Min      = pwm_out_time_min;
		PWM_Out_Time_Max_Mean = pwm_out_time_max_mean;
		PWM_Out_Time_Min_Mean = pwm_out_time_min_mean;

		PWM_Period_Mean     = pwm_period_mean;
		PWM_Period_Max      = pwm_period_max;
		PWM_Period_Min      = pwm_period_min;
		PWM_Period_Max_Mean = pwm_period_max_mean;
		PWM_Period_Min_Mean = pwm_period_min_mean;
		EINT;


		//adcMotorCurrent = analog_motorcurrent;
		/* if not measured voltage when current = 0A (~0.5v) */
		//if(!ZeroCurrent_set && (adcMotorCurrent > 614) && (adcMotorCurrent < 819) && !power_on && (state.time > 50)) {

		/* Moved to pwm_out interruption */
//		if(!ZeroCurrent_set && !ready_to_power_on && (state.time > 50))
//		{
//			static unsigned long sumZeroCurrent = 0, sumZeroCurrent2 = 0;
//			static int i = 0;
//
//			if (++i <= 64)
//			{
//				sumZeroCurrent += analog_motorcurrent; //adcMotorCurrent;
//				sumZeroCurrent2 += analog_motorcurrent2;
//			}
//			if(i == 64)
//			{
//				ATM_setu(&ZeroCurrent_motor, sumZeroCurrent >> 6); /* set ZeroCurrent if ADC value is 0.5v -10%/+20% */
//				ATM_setu(&ZeroCurrent_motor2, sumZeroCurrent2 >> 6); /* set ZeroCurrent if ADC value is 0.5v -10%/+20% */
//				ZeroCurrent_set = 1;
//				_LOGmessage(0x0055,"Calibrate ZeroCurrents", 0, 0);
//			}
//		}

		/* show current in value per thousand of Motor_rated_current */
		Current_actual_value = ((long)Motor_current * 1000) / Motor_rated_current;
		//FPGA_final_current = FPGA_current_value(FPGA_current);
		//FPGA_current_value(FPGA_current);

		/* apply a pending Enable/disable command */
		drive_state = getDriveState(Device_status_word);
		old_statusword = Device_status_word;
	    //12/02/2016 JAT: Added OPERATION_MODE_VELOCITY filter because it sends many many unnecessary messages (DSP hangs)
		if( !(Modes_of_operation_display == OPERATION_MODE_MANUAL) &&
			!(Modes_of_operation_display == OPERATION_MODE_MANUAL_2) &&
			!(Modes_of_operation_display == OPERATION_MODE_VELOCITY) &&
			!(Modes_of_operation_display == OPERATION_MODE_HOMING) )
		{
			if (Device_status_word & SET_POINT_ACKNOWLEDGE_MASKBIT)
				Device_status_word &= ~SET_POINT_ACKNOWLEDGE_MASKBIT;
	    }

		/* disable if SAFETY is in DENY state */
		if( old_FPGA_mode != FPGA_mode )
		{
			volatile unsigned *pointer = (unsigned *)FPGA_ADD_ACOUSTIC_MODE;
			(*pointer) = (UNS16) 0;
			(*pointer) = (UNS16) 8;
			_LOGmessage(0x0111, "Working Mode Changed", 0, 0);
		}
		if( power_state_delay > -1 )
		{
			--power_state_delay;
		}

		if( (drive_state == OPERATION_ENABLE) && ((safety_disable) || (old_FPGA_mode != FPGA_mode)) )
		{
				SET_POWER_OFF();
				ATM_seti(&ready_to_power_on, 0);
				safety_disable = 0;
				setDriveState((unsigned short *)&Device_status_word, SWITCHED_ON);
				drive_state = SWITCHED_ON;
				_LOGmessage(0x0112, "Working Mode Changed", 0, 0);
		}

		/* publish SAFETY signal status in the OD */
		if(Safety != SAFETY)
		{
			Safety = SAFETY;
			if (Safety == SAFETY_ALLOW)
			{
				_LOGmessage(0x003C,"Safety is in ALLOW mode", 0, 0);
			}
			else
			{
				_LOGmessage(0x003D,"Safety is in DENY mode", 0, 0);
			}
		}

		operation_mode_change(Modes_of_operation);

    		/* convert position to internal */
		tmp = (long long)state.pos_prev * pos2int_factor_num;
		if(tmp >= 0) tmp += (pos2int_factor_den >> 1); else tmp -= (long long)(pos2int_factor_den >> 1);
		tmp /= pos2int_factor_den;
		state.position = tmp;

		/* convert velocity to internal */
		tmp = (long long)state.vel_prev * vel2int_factor_num;
		if(tmp >= 0) tmp += (vel2int_factor_den >> 1); else tmp -= (long long)(vel2int_factor_den >> 1);
		tmp /= vel2int_factor_den;
		state.velocity = tmp;

		/* if we are in manual mode with a clutch positon and vel are different */
		if (manual_with_clutch)
		{
			manual_with_clutch_state(&state);
			EQep1Regs.QEPCTL.bit.SWI = 1; /* Software init position counter */
			EQep2Regs.QEPCTL.bit.SWI = 1; /* Software init position counter */
		}
		else
		{
			ATM_seti(&reset_filters, 0);
			reset = manual_mode_back(&state);
			/* filter velocity */
			velocity_filter(&(state.velocity), reset);
			/* filter position */
			position_filter(&(state.position), reset);

		}

		/* show velocity */
		Velocity_actual_value = int2ext_vel(state.velocity);

		/* show position */
		Position_actual_value_in_increments = state.position;
		Position_actual_value = int2ext_pos(state.position, Home_offset);

		/* Reset motor phase error after 5 sec if present */
//		if((GetQueuedFault() & FAULT_MOTOR_PHASE))
//		{
//			reset_motor_phase_error_counter++;
//			if(reset_motor_phase_error_counter == 5000)
//			{
//				DeQueueFault(FAULT_MOTOR_PHASE);
//				reset_motor_phase_error_counter = 0;
//			}
//		}
//		else
//		{
//			reset_motor_phase_error_counter = 0;
//		}

		switch(drive_state)
		{
			case OPERATION_ENABLE:
			    if(ATM_seti(&init_mode, 0))
				{
					init = 1;
					send_SWord = 1;
				}
				else
				{
					init = 0;
				}

				switch((char)Modes_of_operation_display)
				{
					case OPERATION_MODE_ASSISTED: /* Assisted Mode */
					case OPERATION_MODE_ASSISTED_2:
						assisted_mode_operation(&state, (char)(init|(force_init&INIT_ASSISTED_MODE)));
						SET_POWER_ON();
						force_init &= ~INIT_ASSISTED_MODE;
						break;

					case OPERATION_MODE_MANUAL: /* Manual Mode (with detents) */
					case OPERATION_MODE_MANUAL_2:
						if( init )
						{
							//skip_motor_phase_checking();
							release_brake();
							release_clutch();
						}
						manual_mode_operation(&state, init);
						break;

					case OPERATION_MODE_CIRCLE:	    /* circle mode */
					case OPERATION_MODE_CIRCLE_2:	/* chars are 16 bits, 0xfe is treated as 254 */
						circle_mode_operation(&state, init);
						SET_POWER_ON();
						break;

					case OPERATION_MODE_PWM:	/* Constant PWM Mode */
					case OPERATION_MODE_PWM_2:	/* chars are 16 bits, 0xff is treated as 255 */
						constant_pwm_mode_operation(init);
						SET_POWER_ON();
						break;

					case OPERATION_MODE_POSITION:		/* Profile Position Mode */
						if( !profile_position_mode_operation(&state, (char)(init|(force_init&INIT_POS_MODE)) ) )
							SET_POWER_ON();
						force_init &= ~INIT_POS_MODE;
						break;

					case OPERATION_MODE_VELOCITY:		/* Velocity Mode */
						if( !profile_velocity_mode_operation(&state, (char)(init|(force_init&INIT_VEL_MODE)) ) )
							SET_POWER_ON();
						force_init &= ~INIT_VEL_MODE;
						break;

					case OPERATION_MODE_INTERPOLATED:		/* Interpolated Position Mode */
						ip_mode_operation(&state, init);
						break;

					case OPERATION_MODE_HOMING:
						SET_POWER_OFF();
						ATM_seti(&ready_to_power_on, 0);
						setDriveState((unsigned short *)&Device_status_word, SWITCHED_ON);
						break;
					case OPERATION_MODE_MOTOR_CHECK:		/* Motor phase check Mode */
						SET_POWER_OFF();
						motor_phase_check(init);
						break;
					default:
						SET_POWER_OFF();
						ATM_seti(&ready_to_power_on, 0);
						setDriveState((unsigned short *)&Device_status_word, SWITCHED_ON);
						_WARNINGmessage(0xFFF7, 0x20, 0x0000, "Unsupported mode of operation", 0, 0);
						break;
				}
				break;


			case QUICK_STOP_ACTIVE:
				in_quick_stop = 1;
				if(last_drive_state != QUICK_STOP_ACTIVE)
				{
					quick_stop_initialization( &state, &quick_stop_mode );
				}
				quick_stop_control( &state, quick_stop_mode );
				break;

			case FAULT_REACTION_ACTIVE:
				fault_management(&state);
				break;

			case FAULT:
				break;

			case SWITCHED_ON:
				switch((char)Modes_of_operation_display)
				{
					case OPERATION_MODE_HOMING: /* Homing Mode */
						if(ATM_seti(&init_mode, 0))
						{
							init = 1;
							send_SWord = 1;
						}
						else
						{
							init = 0;
						}

						homing_zeroed = homing_mode_operation(&state, init);
						SET_POWER_OFF();
						ATM_seti(&ready_to_power_on, 0);
						break;

					default:
						break;
				}
				break;
			default:
				break;
		}

		/* add Control_effort_offset to control_effort and saturate */
		duty_offset = _IQsat(_IQmpyI32(_IQ(0.001), (long)Control_effort_offset), _IQ(1.0), _IQ(-1.0));
		/* 06/10/16 C.R.G Saturations modified due to control_effort used in current control  */
		//duty_offset = _IQsat(_IQmpyI32(_IQ(0.001), (long)Control_effort_offset), MAX_DUTY_CYCLE, MIN_DUTY_CYCLE);
		//control_effort = _IQsat(control_effort, MAX_DUTY_CYCLE - duty_offset, MIN_DUTY_CYCLE - duty_offset);

		ramp_management();
		set_brakes_flags();				//manages the BRAKE_MASKBIT of Device_status_word
		set_stopped_flag( &state );     //manages the STOPPED_MASKBIT of Device_status_word.
										//Keep after set_brakes_flags because it uses BRAKE_MASKBIT

		last_drive_state = drive_state;	/* save last drive state */

		if( OVERCURRENT != OVERCURRENT_OK )
		{
//			It's done in DefaultIsr
//			if(!(isFaultActive(FAULT_IGBT_ISR)))
//			{
//				_ERRORmessage(0x2311, 0x02, 0x0000, "Motor overcurrent IGBT ISR", 0, 0);
//				setFault(FAULT_IGBT_ISR);
//			}
			QueueFault(FAULT_IGBT_ISR);
		}
		else
		{
		   DeQueueFault(FAULT_IGBT_ISR);
		}

		set_fault_flags();
		set_warning_flags();

		if(old_statusword != Device_status_word )
			send_SWord = 1;
		if(old_FPGA_mode != FPGA_mode )
			send_SWord = 1;
		old_FPGA_mode = FPGA_mode;
		if( send_SWord )
		{
		    send_controlword();
		}

		/* handle peripherals */
		analog_measurements();
		managePeripherals();			/* handle peripherals connected in X1-X8 */
		manageLeds();
		store_service_routine();	/* any write to EEPROM pending? */
		if(Load_eeprom == 1)
		{
			Load_eeprom = 0;
			load_configuration();
		}

		/* check limits */
        check_hall_effect(manual_state);
		check_overcurrent();
		new_redundancy(&state);
		check_velocity_limit((long)Velocity_actual_value, &state);
		check_position_limit((long)Position_actual_value, &state);
		//Update position control loops
		set_position_control_dataset();
		set_position_velocity_control_dataset();
		set_position_current_control_dataset();
		//Update velocity control loops
		set_velocity_control_dataset();
		set_velocity_current_control_dataset();

		LeaveMutex();

		/* manage CAN bus status */
		manage_can_bus();

		Configure_FPGA_system_watchdog( FPGA_SYSTEM_WATCHDOG_TASK2_MASK, FPGA_SYSTEM_WATCHDOG_TASK2_MASK );
		Configure_FPGA_move_watchdog( FPGA_MOVE_WATCHDOG_TASK2_MASK, FPGA_MOVE_WATCHDOG_TASK2_MASK );
		//_LOGmessage(0x0119,"SysCtrlRegs.WDKEY = 0xAA", 0, 0);
		EALLOW;
		SysCtrlRegs.WDKEY = 0xAA;		// and serve watchdog #2
		EDIS;

		if(WD_Debug_variable == 0xD5) while(1) {};

	}
}


unsigned int max(unsigned int num1, unsigned int num2, unsigned int num3, unsigned int num4, unsigned int num5) {
    unsigned int result = 0;
    if (num1 >= num2 && num1 >= num3 && num1 >= num4 && num1 >= num5){
        result = num1;
    } else if (num2 >= num1 && num2 >= num3 && num2 >= num4 && num2 >= num5){
        result = num2;
    } else if (num3 >= num1 && num3 >= num2 && num3 >= num4 && num3 >= num5){
        result = num3;
    } else if (num4 >= num1 && num4 >= num2 && num4 >= num3 && num4 >= num5){
        result = num4;
    } else if (num5 >= num1 && num5 >= num2 && num5 >= num3 && num5 >= num4){
        result = num5;
    }
    return result;
}

unsigned int get_max_error(void)
{
	unsigned int err_value_an1_an2 = 0;
	unsigned int err_value_an1_enc1 = 0;
	unsigned int err_value_an1_enc2 = 0;
	unsigned int err_value_an1_nce = 0;
	unsigned int err_value_an1_resolver = 0;
	unsigned int err_value_an2_enc1 = 0;
	unsigned int err_value_an2_enc2 = 0;
	unsigned int err_value_an2_nce = 0;
	unsigned int err_value_an2_resolver = 0;
	unsigned int err_value_enc1_enc2 = 0;
	unsigned int err_value_enc1_nce = 0;
	unsigned int err_value_enc1_resolver = 0;
	unsigned int err_value_enc2_nce = 0;
	unsigned int err_value_enc2_resolver = 0;
	unsigned int err_value_nce_resolver = 0;
	unsigned int group1_err = 0;
	unsigned int group2_err = 0;
	unsigned int group3_err = 0;
	unsigned int group4_err = 0;
	unsigned int group5_err = 0;
	unsigned int max_err_value = 0;

	if (AN1_used_for_position){
		if (AN2_used_for_position){
			err_value_an1_an2 = labs(labs(AN1_position)-labs(AN2_position));
		}
		if (Enc1_used_for_position){
			err_value_an1_enc1 = labs(labs(AN1_position)-labs(Enc1_position));
		}
		if (Enc2_used_for_position){
			err_value_an1_enc2 = labs(labs(AN1_position)-labs(Enc2_position));
		}
		if (NCE_used_for_position){
			err_value_an1_nce = labs(labs(AN1_position)-labs(NCE_position));
		}
		if (Resolver_used_for_position){
			err_value_an1_resolver = labs(labs(AN1_position)-labs(Resolver_position));
		}
	}
	else{
		err_value_an1_an2 = 0;
		err_value_an1_enc1 = 0;
		err_value_an1_enc2 = 0;
		err_value_an1_nce = 0;
		err_value_an1_resolver = 0;
	}
	if (AN2_used_for_position){
		if (Enc1_used_for_position){
			err_value_an2_enc1 = labs(labs(AN2_position)-labs(Enc1_position));
		}
		if (Enc2_used_for_position){
			err_value_an2_enc2 = labs(labs(AN2_position)-labs(Enc2_position));
		}
		if (NCE_used_for_position){
			err_value_an2_nce = labs(labs(AN2_position)-labs(NCE_position));
		}
		if (Resolver_used_for_position){
			err_value_an2_resolver = labs(labs(AN2_position)-labs(Resolver_position));
		}
	}
	else{
		err_value_an2_enc1 = 0;
		err_value_an2_enc2 = 0;
		err_value_an2_nce = 0;
		err_value_an2_resolver = 0;
	}
	if (Enc1_used_for_position){
		if (Enc2_used_for_position){
			err_value_enc1_enc2 = labs(labs(Enc1_position)-labs(Enc2_position));
		}
		if (NCE_used_for_position){
			err_value_enc1_nce = labs(labs(Enc1_position)-labs(NCE_position));
		}
		if (Resolver_used_for_position){
			err_value_enc1_resolver = labs(labs(Enc1_position)-labs(Resolver_position));
		}
	}
	else{
		err_value_enc1_enc2 = 0;
		err_value_enc1_nce = 0;
		err_value_enc1_resolver = 0;
	}
	if (Enc2_used_for_position){
		if (NCE_used_for_position){
			err_value_enc2_nce = labs(labs(Enc2_position)-labs(NCE_position));
		}
		if (Resolver_used_for_position){
			err_value_enc2_resolver = labs(labs(Enc2_position)-labs(Resolver_position));
		}
	}
	else{
		err_value_enc2_nce = 0;
		err_value_enc2_resolver = 0;
	}
	if (NCE_used_for_position){
		if (Resolver_used_for_position){
			err_value_nce_resolver = labs(labs(NCE_position)-labs(Resolver_position));
		}
	}
	else{
		err_value_nce_resolver = 0;
	}

	if	((Axle_clutch_Position != 0) &&
		((Modes_of_operation_display==OPERATION_MODE_MANUAL)  ||
		(Modes_of_operation_display==OPERATION_MODE_MANUAL_2)))
	{
		err_value_an1_enc1 = 0;
		err_value_an1_enc2 = 0;
		err_value_an2_enc1 = 0;
		err_value_an2_enc2 = 0;
		err_value_enc1_enc2 = 0;
		err_value_enc1_nce = 0;
		err_value_enc1_resolver = 0;
		err_value_enc2_nce = 0;
		err_value_enc2_resolver = 0;
	}

	group1_err = max(err_value_an1_an2, err_value_an1_enc1, err_value_an1_enc2, err_value_an1_nce, err_value_an1_resolver);
	group2_err = max(err_value_an2_enc1, err_value_an2_enc2, err_value_an2_nce, err_value_an2_resolver, 0);
	group3_err = max(err_value_enc1_enc2, err_value_enc1_nce, err_value_enc1_resolver, 0, 0);
    group4_err = max(err_value_enc2_nce, err_value_enc2_resolver, 0, 0, 0);
    group5_err = err_value_nce_resolver;

    max_err_value = max(group1_err, group2_err, group3_err, group4_err, group5_err);

	return max_err_value;
}

void new_redundancy(motion_state_struct *state )
{
	static short q_warning_redundancy_timer = 0;
	static short d_warning_redundancy_timer = 0;

	if(state) /* For normal redundancy test */
	{
		if( !Sensors_configuration_active )
			return;

		if( !homing_zeroed )
			return;

		New_redundancy_max_err = get_max_error();

		if(New_redundancy_error_enable)
		{
			if(labs(New_redundancy_max_err) > New_redundancy_error_window)
			{
				d_warning_redundancy_timer = 0;
				if(!q_warning_redundancy_timer)
				{
					q_warning_redundancy_timer = state->time;		/* Start time counting if not started */
				}
				tmp_red = (long long)(state->time - q_warning_redundancy_timer) * 1000;
				tmp_red /= lcounts_p_s;
				if(tmp_red > New_redundancy_error_timeout)
				{
					QueueWarning(FAULT_REDUNDANCY);

				}
		}
		else
		{
				q_warning_redundancy_timer = 0;			/* stop time counting */
				if(!d_warning_redundancy_timer)
	{
					d_warning_redundancy_timer = state->time;		/* Start time counting if not started */
		}

				tmp_red = (long long)(state->time - d_warning_redundancy_timer) * 1000;
				tmp_red /= lcounts_p_s;
				if(tmp_red > New_redundancy_error_timeout)
		{
					DeQueueWarning(FAULT_REDUNDANCY);
		}
		}
	}
	else
	{
		DeQueueWarning(FAULT_REDUNDANCY);
	}
}
}


/*****************************************************************/
/*!	 Check if current velocity is over the limit
	\param velocity Current velocity [velocity units]
*/
/*****************************************************************/
int velocity_over_limit(long velocity, int assisted)
{
	if(assisted)
{
		if(labs(velocity) > Max_profile_velocity + Overlimit_offset_error + Max_slippage)
		{
			return 1;
		}
		else
		{
			return 0;
		}

	}
	else
	{
		if(labs(velocity) > Max_profile_velocity + Overlimit_offset_error)
		{
			return 1;
		}
		else
		{
	return 0;
		}
	}
}


/*****************************************************************/
/*!	 Check if current position is over the limit
	\param position Current velocity [velocity units]
*/
/*****************************************************************/
int position_over_limit(long position)
{
	long lim_max = Outofbound_position_limit_Max + Following_error_window;
	long lim_min = Outofbound_position_limit_Min - Following_error_window;
	if(position < lim_min || position > lim_max) return 1;

	return 0;
}

/*****************************************************************/
/*!	 Check if current velocity is over the limit during a time interval previously definied
	\param velocity Current velocity [velocity units]
	\param state Cynematic state (position, velocity and time)
*/
/*****************************************************************/
void check_velocity_limit(long velocity, motion_state_struct *state)
{
	static long long v_error_started = 0;
	long long tmp;
	int slip = 0;

	if( !Sensors_configuration_active )
	{
		v_error_started = 0;		/* stop time counting */
		//DeQueueFault(FAULT_VEL_LIMIT);
		return;
	}

	if( (Modes_of_operation_display == OPERATION_MODE_HOMING) || !homing_zeroed )
	{
		v_error_started = 0;		/* stop time counting */
		//DeQueueFault(FAULT_VEL_LIMIT);
		return;
	}

	if( (Modes_of_operation_display==OPERATION_MODE_MANUAL)  ||
		(Modes_of_operation_display==OPERATION_MODE_MANUAL_2) )
	{
		v_error_started = 0;		/* stop time counting */
		//DeQueueFault(FAULT_VEL_LIMIT);
		return;
	}

//	if( (Modes_of_operation_display==OPERATION_MODE_ASSISTED)  ||
//		(Modes_of_operation_display==OPERATION_MODE_ASSISTED_2) )
//	{
//		v_error_started = 0;		/* stop time counting */
//		//DeQueueFault(FAULT_VEL_LIMIT);
//		return;
//	}
	if( (Modes_of_operation_display==OPERATION_MODE_ASSISTED)  ||
		(Modes_of_operation_display==OPERATION_MODE_ASSISTED_2) ||
		(Modes_of_operation_display==OPERATION_MODE_VELOCITY))
	{
		slip = 1;
	}

	/* Check if current velocity is higher than the maximum velocity */
	if(velocity_over_limit(velocity, slip))
	{
		if(!v_error_started)
			v_error_started = state->time;		/* Start time counting if not started */
		tmp = state->time - v_error_started;
		tmp = (tmp * 1000) / lcounts_p_s;
		if(tmp >= Velocity_threshold_time)
		{
			if(!(isFaultActive(FAULT_VEL_LIMIT)))
			{
				_ERRORmessage(0xFF09, 0x80, 0x0000, "Maximum velocity reached", 0, 0);
				setFault(FAULT_VEL_LIMIT);
			}
			QueueFault(FAULT_VEL_LIMIT);
		}
	}
	else
	{
		v_error_started = 0;		/* stop time counting */
		//DeQueueFault(FAULT_VEL_LIMIT);
	}
}



/*****************************************************************/
/*!	 Check if current position is over the limit during a time interval previously definied
	\param position Current position [position units]
	\param state Cynematic state (position, velocity and time)
*/
/*****************************************************************/
void check_position_limit(long position, motion_state_struct *state)
{
	static long long p_error_started = 0;
	long long tmp;

	/* check minimum software position limit and set/clear flag */
	if(position >= (long)(Software_position_limit_Min_position_limit - Following_error_window))
		ATM_andu(&position_flags, ~MIN_SW_POS_LIMIT);    /* clear MIN_SW_POS_LIMIT */
	else
		ATM_oru(&position_flags, MIN_SW_POS_LIMIT);    /* set MIN_SW_POS_LIMIT */

	/* check maximum software position limit and set/clear flag */
	if(position <= (long)(Software_position_limit_Max_position_limit + Following_error_window))
		ATM_andu(&position_flags, ~MAX_SW_POS_LIMIT);    /* clear MAX_SW_POS_LIMIT */
	else
		ATM_oru(&position_flags, MAX_SW_POS_LIMIT);    /* set MAX_SW_POS_LIMIT */

	if( !Sensors_configuration_active )
	{
		p_error_started = 0;		/* stop time counting */
		DeQueueFault(FAULT_POS_LIMIT);
		return;
	}

	if( (Modes_of_operation_display == OPERATION_MODE_HOMING) || !homing_zeroed )
	{
		p_error_started = 0;		/* stop time counting */
		DeQueueFault(FAULT_POS_LIMIT);
		return;
	}

	if( (Modes_of_operation_display==OPERATION_MODE_MANUAL)  ||
		(Modes_of_operation_display==OPERATION_MODE_MANUAL_2) )
	{
		p_error_started = 0;		/* stop time counting */
		DeQueueFault(FAULT_POS_LIMIT);
		return;
	}

	/* Check if current position is out of the limits */
	if(position_over_limit(position))
	{
		if(!p_error_started)
			p_error_started = state->time;		/* Start time counting if not started */
		tmp = (long long)(state->time - p_error_started) * 1000;
		tmp /= lcounts_p_s;
		if(tmp > Following_error_time_out)
		{
			if(!(isFaultActive(FAULT_POS_LIMIT)) ) /* not Fault signal until we get into limits */
			{
				_ERRORmessage(0xFF0A, 0x80, 0x0000, "Position value out of the limits", 0, 0);
				setFault(FAULT_POS_LIMIT);
			}
			QueueFault(FAULT_POS_LIMIT);
		}
	}
	else
	{
		p_error_started = 0;		/* stop time counting */
		DeQueueFault(FAULT_POS_LIMIT);
	}
}


/*****************************************************************/
/*!	 Check potentiometer
	\param position Current position [position units]
	\param state Cynematic state (position, velocity and time)
*/
/*****************************************************************/
void check_potentiometer(motion_state_struct *state)
{
	static long long p_potentiometer_error_started = 0;
	long long tmp;

	if( !Sensors_configuration_active )
	{
		p_potentiometer_error_started = 0;		/* stop time counting */
		DeQueueFault(FAULT_POT_MEASURE);
		return;
	}

	if( (Modes_of_operation_display == OPERATION_MODE_HOMING) || !homing_zeroed )
	{
		p_potentiometer_error_started = 0;		/* stop time counting */
		DeQueueFault(FAULT_POT_MEASURE);
		return;
	}

	/* Check if current position is out of the limits */
	if(position_over_limit((long)state->abs_pos_cntr))
	{
		if(!p_potentiometer_error_started)
			p_potentiometer_error_started = state->time;		/* Start time counting if not started */
		tmp = (long long)(state->time - p_potentiometer_error_started) * 1000;
		tmp /= lcounts_p_s;

		if(tmp > Following_error_time_out)
		{
			if(!(isFaultActive(FAULT_POT_MEASURE)) ) /* not Fault signal until we get into limits */
			{
				_ERRORmessage(0xFF06, 0x80, 0x0000, "Potentiometer measure error", 0, 0);
				setFault(FAULT_POT_MEASURE);
			}
			QueueFault(FAULT_POT_MEASURE);
		}
	}
	else
	{
		p_potentiometer_error_started = 0;	/* stop time counting */
		DeQueueFault(FAULT_POT_MEASURE);
	}
}


/*****************************************************************/
/*!	 Main function of Constant PWM Mode
	\param init will be 1 if the mode has to be initialized (0 if not)
*/
/*****************************************************************/
void constant_pwm_mode_operation(char init)
{
	if(init)
	{
		PWM_reference_inst = 0;

		/* reset mode specific bits of Statusword */
		Device_status_word &= ~MODE_SPECIFIC_BITS;
		send_SWord = 1;

		DINT;
		p_current_limit = &current_limit;	/* set current limit to use */
		EINT;

		ATM_seti(&ready_to_power_on, 1);
	}

	/* if we are going out of limits and PWM_reference is not null, stop */
	if (PWM_reference && constant_pwm_position_limits())
	{
		/* reduce PWM_reference */
		PWM_reference += (-PWM_reference >> 2);
	}

	/* clear PWM_reference if HALT bit is set */
	if(Device_control_word & HALT_MASKBIT)
	{
		PWM_reference = 0;
		PWM_reference_inst = 0;
	}

	if(Enable_Ramp_PWM_mode==1)
	{
		if(PWM_reference_inst < PWM_reference)
			PWM_reference_inst++;
		else
			if(PWM_reference_inst > PWM_reference)
				PWM_reference_inst--;
	}
	else PWM_reference_inst = PWM_reference;
	control_effort = _IQsat(_IQmpyI32(_IQ(0.0001), (long)PWM_reference_inst), _IQ(1.0), _IQ(-1.0));
}


/*****************************************************************/
/*!	 Checks if the current speed would make the drive go further position limits
	\return 0 if drive can stop between limits, 1 if not.
*/
/*****************************************************************/
int constant_pwm_position_limits(void)
{
	long long stop_length;
	long dec;
	long position, velocity, max_pos, min_pos;

	/* choose a deceleration */
	dec = Max_deceleration / 4;

	/* calculate stop length (absolute value) */
	stop_length = (long long)Velocity_actual_value * Velocity_actual_value;
	stop_length /= dec;
	stop_length = stop_length / 2;

	/* multiply stop_length by 2 for some margin */
	stop_length *= 2;

	/* Convert values according to polarities */
	velocity = (Polarity & 0x40)? -Velocity_actual_value : Velocity_actual_value;
	if(Polarity & 0x80)
	{
		position = -Position_actual_value;
		max_pos = -Software_position_limit_Min_position_limit;
		min_pos = -Software_position_limit_Max_position_limit;
	}
	else
	{
		position = Position_actual_value;
		max_pos = Software_position_limit_Max_position_limit;
		min_pos = Software_position_limit_Min_position_limit;
	}

	/* BLAC limits */
	//AMQ TODO: check the working of original limitation with BLAC Motor
	if(Power_board == A3666_01_BOARD)
	{
		if((Position_actual_value >= (Software_position_limit_Max_position_limit-100)) && (PWM_reference > 0))
			return 1;
		else
			if ((Position_actual_value <= (Software_position_limit_Min_position_limit+100)) && (PWM_reference < 0))
				return 1;
			else
				return 0;
	}

	/* check limits */
	if ((velocity > 0) && (PWM_reference > 0))
	{
		if ((position + (long)stop_length) >= max_pos) return 1;
		else return 0;
	}
	if ((velocity < 0) && (PWM_reference < 0))
	{
		if ((position - (long)stop_length) <= min_pos) return 1;
		else return 0;
	}
	return 0;		/* Velocity_actual_value = 0 */
}


/*****************************************************************/
/*! 	Calculates the square root of a unsigned long long using the IQ library
	\param state Cinematic state of the system
*/
/*****************************************************************/
long root(unsigned long long radicand)
{
	_iq1 aux;
	if(radicand <= 1073741823)		/* 1073741823 is the max value of _iq1 */
	{
		aux = _IQ1(radicand);
		aux = _IQ1sqrt(aux);
		return _IQ1int(aux);
	} else if(radicand <= 274877906943)
		{
			aux = _IQ1(radicand >> 8);
			aux = _IQ1sqrt(aux);
			return _IQ1int(aux) << 4;
		} else if(radicand <= 70368744177663)
			{
				aux = _IQ1(radicand >> 16);
				aux = _IQ1sqrt(aux);
				return _IQ1int(aux) << 8;
			} else if(radicand <= 18014398509481983)
				{
					aux = _IQ1(radicand >> 24);
					aux = _IQ1sqrt(aux);
					return _IQ1int(aux) << 12;
				} else if(radicand <= 4611686018427387903)
					{
						aux = _IQ1(radicand >> 32);
						aux = _IQ1sqrt(aux);
						return _IQ1int(aux) << 16;
					} else
						{
							aux = _IQ1(radicand >> 40);
							aux = _IQ1sqrt(aux);
							return _IQ1int(aux) << 20;
						}
}


/*****************************************************************/
/*!	Gives time in lcount units as a 64-bit variable, never overflows
	\return time in lcounts as a 64-bit variable */
/*****************************************************************/
long long getltime(void)
{
	static long long time = 0;
	static unsigned long ltime = 0;
	unsigned long old_ltime = ltime;

	DINT;
	ltime = CLK_getltime();
	time += (ltime - old_ltime);
	EINT;
	return time;
}


/*****************************************************************/
/*!	Reads the nodeID from switchs 1-7 in DIP switch (switch1=LSB, switch7=MSB)
	\return CANopen nodeID */
/*****************************************************************/
int readNodeID(void)
{
    int id = 0;
	id = readIDswitch();
	return (id & 0x007F)? (id & 0x007F) : 0x007F;		/* return 127 if null value read */
}


/*****************************************************************/
/*!	Reads the ID switch
	\return ID switch value */
/*****************************************************************/
int readIDswitch(void)
{
	int id = 0;
	volatile unsigned *pointer;

	pointer = (unsigned *)FPGA_ADD_NODE_ID;
	id = (*pointer);
	return id;
}


/*****************************************************************/
/*!	Updates the OD entries that show Temperature, Vcontrol5, Vpower... */
/*****************************************************************/
void analog_measurements(void)
{
	static int n_tempError = 0;
	static int n_tempMotorError = 0;
	static int n_vpowerError = 0;

	/* check Vpower */
	/* Show Vpower in hundredths of volts */
	Power_voltage = FILTER_OUT_2(sum_voltage);
	DC_link_circuit_voltage = (long)Power_voltage * 10;
	if((Power_voltage >= Power_voltage_limit[0]) && (Power_voltage <= Power_voltage_limit[1]))
	{
		n_vpowerError--;
	}
	else
	{
		n_vpowerError++;
	}
	n_vpowerError = CLAMP( n_vpowerError, 0, VOLTAGE_ERROR_FILTER );
	if(n_vpowerError == VOLTAGE_ERROR_FILTER)	/* wait for 20 Vpower values out of the limits */
	{
		if(!isFaultActive(FAULT_VPOWER))
		{
			_ERRORmessage(0x3100, 0x04, Power_voltage, "power voltage error", 0, 0);
			setFault(FAULT_VPOWER);
		}
		QueueFault(FAULT_VPOWER);
	}
	else if( n_vpowerError == 0 )
	{
		DeQueueFault(FAULT_VPOWER);
	}

	/* check maximum temperature */
	Temperature = get_temp();
	if(Temperature < Temperature_limit)
		n_tempError--;
	else
		n_tempError++;
	n_tempError = CLAMP( n_tempError, 0, TEMPERATURE_ERROR_FILTER );
	if(n_tempError == TEMPERATURE_ERROR_FILTER)	/* wait for 10 temperature values above 80degC */
	{
		if(!isFaultActive(FAULT_OVERTEMP))
		{
			_ERRORmessage(0x4310, 0x08, 0x0000, "Overtemperature error", 0, 0);
			setFault(FAULT_OVERTEMP);
		}
		QueueFault(FAULT_OVERTEMP);
	}
	else if(n_tempError == 0)
	{
		DeQueueFault(FAULT_OVERTEMP);
	}

	/* check the motor temperature */
	Motor_Temperature = get_motor_temp();
	if(Motor_Temperature < Motor_Temperature_limit)
		n_tempMotorError--;
	else
		n_tempMotorError++;
	n_tempMotorError = CLAMP( n_tempMotorError, 0, TEMPERATURE_ERROR_FILTER );
	if(n_tempMotorError == TEMPERATURE_ERROR_FILTER)	/* wait for 10 temperature values above 80degC */
	{
		if(!isFaultActive(FAULT_MOTOR_OVERTEMP))
		{
			_ERRORmessage(0x4311, 0x08, 0x0000, "Overtemperature motor error", 0, 0);
			setFault(FAULT_MOTOR_OVERTEMP);
		}
		QueueFault(FAULT_MOTOR_OVERTEMP);
	}
	else if(n_tempMotorError == 0)
	{
		DeQueueFault(FAULT_MOTOR_OVERTEMP);
	}

	/* Show 10 analog inputs in 12-bit ADC units */
	Analog_input_Resolver_in1 = resolver_in1;
	Analog_input_Resolver_in2 = resolver_in2;
	Analog_input_An1 = an1;
	Analog_input_An2 = an2;
	Analog_input_An3 = an3;
	Analog_input_An4 = an4;
	Analog_input_Thermistor = thermistor;
	Analog_input_NTC = analog_ntc;
	Analog_input_Motorcurrent = analog_motorcurrent;
	Analog_input_Ref1v5 = analog_ref1v5;
	Analog_input_Vpower = analog_vpower;
	Analog_input_Hybrid = analog_hybrid;

	/* AMQ DEBUG Statements */
	{
		unsigned int temp;

		ATM_setu(&temp,AdcMirror.ADCRESULT0); //read Iu adc
		//Debug_word_1=((INTEGER32)temp); //write in OD protected by Mutex
		ATM_setu(&temp,AdcMirror.ADCRESULT2); //read Iw adc
		//Debug_word_3=(INTEGER32)temp; //write in OD protected by Mutex
		//Debug_word_2=-Debug_word_1-Debug_word_3; //Calculate Iv, with offset of Iu, Iw
		//Debug_word_4=_IQtoIQ10(resolver_angle);
		//Debug_word_5=_IQtoIQ10(pwm_angle);
		Debug32_word_1=_IQtoIQ10(resolver_angle);
		//Debug_word_6=_IQtoIQ10(sin_u);
	}
	/* end DEBUG statements */


}


/*****************************************************************/
/*!	Interpolates or extrapolates for 'x' when 'y'->'a' and 'z'->'b'
	\param x Point to interplate
	\param y First known point
	\param z Second known point
	\param a Value for first point
	\param b Value for second point	*/
/*****************************************************************/
int interpolate(int x, int y, int z, int a, int b)
{
	long temp;

	temp = (long)(b - a) * (x - y);
	temp /= (z - y);
	return a + temp;
}


/*****************************************************************/
/*!	Interpolates or extrapolates for 'x' when 'y'->'a' and 'z'->'b'
	\param x Point to interplate (long long)
	\param y First known point (long long)
	\param z Second known point (long long)
	\param a Value for first point (long)
	\param b Value for second point (long)	*/
/*****************************************************************/
long linterpolate(long long x, long long y, long long z, long a, long b)
{
	long long temp;

	temp = (b - a) * (x - y);
	temp /= (z - y);
	return a + temp;
}

/*****************************************************************/
/*!	Function that manages the Status LEDs and turns off yellow CAN LED
	*/
/*****************************************************************/
void manageLeds(void)
{
	int leds;	/*!< bit 0: yellow half period 1; bit 1: yellow half period 2; bit 2: red half period 1; bit 3 red half period 2 */
	static unsigned i = 399;
	static unsigned i_yellowCAN = 0;

	if(amc_od_Data.nodeState == Operational)
	{
		switch(getDriveState(Device_status_word))
		{
			case OPERATION_ENABLE:
				leds = 0x03;		/* yellow */
				break;
			case FAULT_REACTION_ACTIVE:
				leds = 0x0C;		/* red */
				break;
			case FAULT:
				leds = 0x04;		/* red  blinking */
				break;
			default:
				leds = 0x01;		/* yellow  blinking */
				break;
		}
	} else leds = 0x09;		/* yellow and red blinking */


	/* blink leds every with half period of 200 ms */
	if(++i >= 400) i = 0;
	if(i == 0)
	{
		if (leds & 0x01) SET_YELLOW_STATUS_LED;
		else CLEAR_YELLOW_STATUS_LED;
		if (leds & 0x04) SET_RED_STATUS_LED;
		else CLEAR_RED_STATUS_LED;
	} else if (i == 200)
	{
		if (leds & 0x02) SET_YELLOW_STATUS_LED;
		else CLEAR_YELLOW_STATUS_LED;
		if (leds & 0x08) SET_RED_STATUS_LED;
		else CLEAR_RED_STATUS_LED;
		TOGGLE_GP_LED_2;
	}


	/* turn off yellow CAN LED 20 ms after it was turned on */
	if(YELLOW_CAN_LED && (++i_yellowCAN > 20))
	{
		CLEAR_YELLOW_CAN_LED;
		i_yellowCAN = 0;
	}

}


/*****************************************************************/
/*!	Function that filters velocity
	v = (v(0) + v(-1)*a + v(-2)*a^2 + ...) / (1 + a + a^2 + ...)
		a = 15/16
	\param value Pointer to velocity current value
	*/
/*****************************************************************/
void velocity_filter(long *value, int reset)
{
	static long long accumulated = 0;

	if (reset) accumulated = (long long)*value << 4;

	accumulated = ((accumulated * 15) + 8 >> 4) + *value;
	*value = (accumulated + 8) >> 4;
}


/*****************************************************************/
/*!	Function that filters position
	s = (s(0) + s(-1)*a + s(-2)*a^2 + ...) / (1 + a + a^2 + ...)
		a = 7/8
	\param value Pointer to position current value
	*/
/*****************************************************************/
void position_filter(long *value, int reset)
{
	static long long accumulated = 0;

	if (reset) accumulated = (long long)*value << 3;

	accumulated = ((accumulated * 7) + 4 >> 3) + *value;
	*value = (accumulated + 4) >> 3;
}


/*****************************************************************/
/*!	 Calculates 1/2 * acc * t^2
	\param acc Acceleration
	\param time time increment
	\return 1/2 * acc * t^2 (position in increments)
*/
/*****************************************************************/
long half_acc_t2(long acc, unsigned long time)
{
	long long tmp;
	unsigned long aux;

	tmp = (long long)time * time;
	aux = (unsigned long)lcounts_p_s * lcounts_p_s * 2;
	tmp = (tmp * acc) / aux;
	return (long)tmp;
}


/*****************************************************************/
/*!	 function that resets the DSP by waiting for a Watchdog Reset
*/
/*****************************************************************/
void _resetNode(void)
{
	DINT;		/* disable interrupts */
	EALLOW;
	SysCtrlRegs.SCSR = 1;			/* WD generates a reset, and WDDIS can be modified */
	SysCtrlRegs.WDCR = 0x00A8;		/* enable the WatchDog (should be already enabled) */
	SysCtrlRegs.WDCR = 0x0000;		/* force a reset by writing WDCHK != 101 */
	EDIS;
	EINT;
}

/*****************************************************************/
/*!	 function that resets the DSP through the FPGA
*/
/*****************************************************************/
void resetNode(void)
{
	DINT;		/* disable interrupts */
	FPGA_WriteConfigurationRegister( FPGA_REG_CONF_RESET_WR_MASK, FPGA_REG_CONF_RESET_WR_MASK );
	EINT;
}




/*****************************************************************/
/*!	Function that converts position from internal units (increments) to external units (m*10^-x)
	\param internal_pos Position in internal units
	 \param offset offset applied (must be 0 for diferential conversions)
	\return Position in external units
*/
/*****************************************************************/
long int2ext_pos(long internal_pos, long offset)
{
	long long tmp;

	tmp = (long long)internal_pos * Position_factor_Feed_constant;
	if(tmp >= 0) tmp += (Position_factor_Numerator >> 1); else tmp -= (long long)(Position_factor_Numerator >> 1);
	tmp /= Position_factor_Numerator;
	tmp = (Polarity & 0x80) ? -tmp : tmp;
	return tmp + offset;		/* [position units] */
}


/*****************************************************************/
/*!	Function that converts position from  external units (m*10^-x) to internal units (increments)
	\param external_pos Position in internal units
	\param offset offset applied (must be 0 for diferential conversions)
	\return Position in internal units
*/
/*****************************************************************/
long ext2int_pos(long external_pos, long offset)
{
	long long tmp;
	long tmp2;

	tmp2 = external_pos - offset;
	tmp2 = (Polarity & 0x80) ? -tmp2 : tmp2;		/* check position polarity */
	tmp = (long long)tmp2 * Position_factor_Numerator;
	return tmp / Position_factor_Feed_constant;
}



/*****************************************************************/
/*!	Function that converts velocity from internal units (inc/s) to external units (m*10^-x / s)
	\param internal_vel Velocity in internal units
	\return Velocity in external units
*/
/*****************************************************************/
long int2ext_vel(long internal_vel)
{
	long long tmp;

	tmp = (long long)internal_vel * Velocity_encoder_factor_Divisor;
	if(tmp >= 0) tmp += (Velocity_encoder_factor_Numerator >> 1); else tmp -= (long long)(Velocity_encoder_factor_Numerator >> 1);
	tmp /= Velocity_encoder_factor_Numerator;
	return (Polarity & 0x40) ? -tmp : tmp;		/* [velocity units] */
}


/*****************************************************************/
/*!	Function that converts position from  external units (m*10^-x / s) to internal units (increments/s)
	\param external_vel Velocity in internal units
	\return Velocity in internal units
*/
/*****************************************************************/
long ext2int_vel(long external_vel)
{
	long long tmp;

	tmp = (long long)external_vel * Velocity_encoder_factor_Numerator;
	tmp /= Velocity_encoder_factor_Divisor;		/* convert to inc/s */
	return (Polarity & 0x40) ? -tmp : tmp;			/* check velocity polarity */
}


/*****************************************************************/
/*!	Function that launches AMC-loader
*/
/*****************************************************************/
void launch_amc_loader(void)
{
	DINT;		/* disable interrupts */
	asm(" LB 0x338000");		/* Launch AMC */
}

/*****************************************************************/
/*!	Function that launches AMC-loader to program FPGA's flash
*/
/*****************************************************************/
void launch_fpga_loader(void)
{
	DINT;		/* disable interrupts */
	asm(" LB 0x338000");		/* Launch AMC */
}

/*!	 Function that sets global flag STOPPED in StatusWord
	\param state Cinematic state of the system
*/
void set_stopped_flag( motion_state_struct *state )
{
	static long long stopped_started = 0;
	static long long motion_started = 0;
	unsigned long long tmp;

	if (Device_status_word & BRAKE_MASKBIT)
	{
		Device_status_word |= STOPPED_MASKBIT;
		return;
	}

	//In manual always in motion
	if( (Modes_of_operation_display == OPERATION_MODE_MANUAL) ||
		(Modes_of_operation_display == OPERATION_MODE_MANUAL_2) )
	{
		Device_status_word &= ~STOPPED_MASKBIT;
		return;
	}

	/* Check if velocity == 0 */
	if(labs(int2ext_vel(state->velocity)) <= Velocity_threshold)
	{
		motion_started = 0;			/* stop time counting */
		if(!stopped_started) stopped_started = state->time;		/* Start time counting if not started */
		tmp = (state->time - stopped_started) * 1000;
		tmp /= lcounts_p_s;
		if(tmp >= Velocity_threshold_time)
			Device_status_word |= STOPPED_MASKBIT;	/* set STOPPED manufacturer specific flag */
	}
	else
	{
		stopped_started = 0;			/* stop time counting */
		if(!motion_started) motion_started = state->time;		/* Start time counting if not started */
		tmp = (state->time - motion_started) * 1000;
		tmp /= lcounts_p_s;
		if(tmp >= Velocity_threshold_time)
			Device_status_word &= ~STOPPED_MASKBIT;	/* clear STOPPED manufacturer specific flag */
	}
}


/*!	 Function that sends ControlWord (1st txPDO)
*/
void send_controlword( void )
{
	/* send txPDO1 (PDO index 0) */
//	sendOnePDOevent( &amc_od_Data, 0 );
	if( ! buildPDO( &amc_od_Data, 0, &amc_od_Data.PDO_status[0].last_message ) )
		canSend( amc_od_Data.canHandle, &amc_od_Data.PDO_status[0].last_message );
}


/*! Disables motor if state is ENABLED
*/
void safety(void)
{
	if (SAFETY == SAFETY_DENY)
	{
		/* StatusWord can't be acessed directly because an ISR cannot wait in a Mutex */
		safety_disable = 1;
	}
}



/*! Apply pending mode of operation change
	\param mode New mode of operation
*/
void operation_mode_change(char mode)
{
	static char old_mode;

	if( (! ATM_seti(&change_mode, 0)))
		return;

	if( (old_mode == OPERATION_MODE_MANUAL) || (old_mode == OPERATION_MODE_MANUAL_2));
	{
		if (Axle_clutch_Position != 0)
		{
			ATM_seti(&manual_with_clutch, 0);
			ATM_seti(&reset_filters, 1);
			apply_filters = 1;
		}
	}
	old_mode = mode;

	switch(mode)
	{
		case OPERATION_MODE_POSITION:			/* profile position mode */
			_LOGmessage(0x0100,"Profile position mode used", 0, 0);
			ATM_seti(&init_mode, 1);	/* mode of operation has to be initialized */
			Modes_of_operation_display = (int)mode;
			break;
		case OPERATION_MODE_VELOCITY:			/* profile velocity mode */
			_LOGmessage(0x0101,"Profile velocity mode used", 0, 0);
			ATM_seti(&init_mode, 1);	/* mode of operation has to be initialized */
			Modes_of_operation_display = (int)mode;
			break;
		case OPERATION_MODE_HOMING:			/* homing mode */
			_LOGmessage(0x0103,"Homing mode used", 0, 0);
			ATM_seti(&init_mode, 1);	/* mode of operation has to be initialized */
			Modes_of_operation_display = (int)mode;
			break;
		case OPERATION_MODE_INTERPOLATED:			/* interpolated position mode */
			_LOGmessage(0x0102,"Interpolated position mode used", 0, 0);
			ATM_seti(&init_mode, 1);	/* mode of operation has to be initialized */
			Modes_of_operation_display = (int)mode;
			break;
		case OPERATION_MODE_PWM:			/* constant pwm mode */
		case OPERATION_MODE_PWM_2:		/* -1 is seen as 255 */
			_LOGmessage(0x0105,"Constant PWM mode used", 0, 0);
			ATM_seti(&init_mode, 1);	/* mode of operation has to be initialized */
			Modes_of_operation_display = (int)mode;
			break;
		case OPERATION_MODE_CIRCLE:			/* circle mode */
		case OPERATION_MODE_CIRCLE_2:		/* -2 is seen as 254 */
			_LOGmessage(0x0106,"Circle mode used", 0, 0);
			ATM_seti(&init_mode, 1);	/* mode of operation has to be initialized */
			Modes_of_operation_display = (int)mode;
			break;
		case OPERATION_MODE_MANUAL:			/* manual mode */
		case OPERATION_MODE_MANUAL_2:		/* -3 is seen as 253 */
			_LOGmessage(0x0104,"Manual mode used", 0, 0);
			ATM_seti(&init_mode, 1);	/* mode of operation has to be initialized */
			Modes_of_operation_display = (int)mode;
			break;
		case OPERATION_MODE_ASSISTED:			/* manual mode */
		case OPERATION_MODE_ASSISTED_2:		/* -4 is seen as 252 */
			_LOGmessage(0x0107,"Assisted mode used", 0, 0);
			ATM_seti(&init_mode, 1);	/* mode of operation has to be initialized */
			Modes_of_operation_display = (int)mode;
			break;
		case OPERATION_MODE_MOTOR_CHECK:		/* motor phase check mode */
			ATM_seti(&init_mode, 1);	/* mode of operation has to be initialized */
			Modes_of_operation_display = (int)mode;
			break;
		default:
			SET_POWER_OFF();
			ATM_seti(&ready_to_power_on, 0);
			if(getDriveState(Device_status_word) == OPERATION_ENABLE)
				setDriveState((unsigned short *)&Device_status_word, SWITCHED_ON);
			_WARNINGmessage(0xFFF7, 0x20, 0x0000, "Unsupported mode of operation", 0, 0);
			break;
	}
}


void manual_with_clutch_state(motion_state_struct *state)
{
	static long pos_prev;
	static long vel_prev;
	long long tmp;
	static int i=0;
	unsigned int reset = 0;

	if(ATM_seti(&reset_filters, 0))
	{
		reset = 1;
		//redundancy_test(1, state);
		//hybrid_redundancy_test(1, state);
	}
	state->position = ext2int_pos(state->abs_pos_cntr, Home_offset);
	position_filter(&(state->position), reset);

	if(init_mode || reset)
	{
		pos_prev = state->position;
		state->velocity = 0;
		velocity_filter(&(state->velocity), reset);
		i = 0;
	}
	else
	{
		if (++i%8 == 0)
		{
			tmp = (long long)(lcounts_p_s) * (int)(state->position - pos_prev);
			tmp /= 2*8;  /* place here the lcounts period of the PRD function */
			state->velocity = tmp;
			velocity_filter(&(state->velocity), reset);
			pos_prev = state->position;
		}
		else
		{
			state->velocity = vel_prev;
		}
	}

	vel_prev = state->velocity;
}


void ramp_management(void)
{
	if (ramp_time > 0)
	  ramp_time--;

	if( mode_change_time > 0 )
		mode_change_time--;
}


void load_ramp_time(int set)
{
  ramp_time = set;
}

int manual_mode_back (motion_state_struct *state)
{
	/* filter velocity */
	if (!apply_filters)
	  return 0;

	apply_filters = 0;
	absolute_sensor = abs_pos_counter(SAVED_PARAMS);
	Home_offset = absolute_sensor - int2ext_pos(state->position, 0); /* convert to external units not considering offset */
	EQep1Regs.QEPCTL.bit.SWI = 0; /* Software init position counter */
	EQep2Regs.QEPCTL.bit.SWI = 0; /* Software init position counter */
	return 1;
}
void eval_hall_state(unsigned char hall_state, motor_type motor){

	static short current_prev = 0;

	if (motor == BLDC_MOTOR_a )
	{
		hall_state ^= 7;
	}

	switch (hall_state)
	{
		case 1:  //0xb001:        // U->W             Q1->Q6
			if ( current_prev >= 0 )
			{
				final_current = current;
				if ( (current1 < 0) && (current3 > 0) )
					final_current = -current;
				break;
			}
			if ( current_prev < 0 )
			{
				final_current = -current;
				if ( (current1 > 0) && (current3 < 0) )
					final_current = current;
				break;
			}
		case 3:  //0xb011:        // V->W             Q3->Q6
			if ( current_prev >= 0 )
			{
				final_current = current;
				if ( (current2 < 0) && (current3 > 0) )
					final_current = -current;
				break;
			}
			if ( current_prev < 0 )
			{
				final_current = -current;
				if ( (current2 > 0) && (current3 < 0) )
					final_current = current;
				break;
			}
		case 2:  //0xb010:        // V->U             Q3->Q2
			if ( current_prev >= 0 )
			{
				final_current = current;
				if ( (current2 < 0) && (current1 > 0) )
					final_current = -current;
				break;
			}
			if ( current_prev < 0 )
			{
				final_current = -current;
				if ( (current2 > 0) && (current1 < 0) )
					final_current = current;
				break;
			}
		case 6:  //0xb110:        // W->U             Q5->Q2
			if ( current_prev >= 0 )
			{
				final_current = current;
				if ( (current3 < 0) && (current1 > 0) )
					final_current = -current;
				break;
			}
			if ( current_prev < 0 )
			{
				final_current = -current;
				if ( (current3 > 0) && (current1 < 0) )
					final_current = current;
				break;
			}
		case 4:  //0xb100:        // W->V             Q5->Q4
			if ( current_prev >= 0 )
			{
				final_current = current;
				if ( (current3 < 0) && (current2 > 0) )
					final_current = -current;
				break;
			}
			if ( current_prev < 0 )
			{
				final_current = -current;
				if ( (current3 > 0) && (current2 < 0) )
					final_current = current;
				break;
			}
		case 5:  //0xb101:        // U->V             Q1->Q4
			if ( current_prev >= 0 )
			{
				final_current = current;
				if ( (current1 < 0) && (current2 > 0) )
					final_current = -current;
				break;
			}
			if ( current_prev < 0 )
			{
				final_current = -current;
				if ( (current1 > 0) && (current2 < 0) )
					final_current = current;
				break;
			}
		default:
			break;
	}
	current_prev = final_current;
}

void BLDC_apply_pwm(unsigned int positive, unsigned int negative, unsigned char hall_state){

  switch (hall_state)
  {
    case 1:  //0xb001:		// U->W		Q1->Q6
	  ENABLE_PWM1(positive);
	  DISABLE_PWM2;
	  ENABLE_PWM3(negative);
	  break;
    case 3:  //0xb011:		// V->W		Q3->Q6
	  DISABLE_PWM1;
	  ENABLE_PWM2(positive);
	  ENABLE_PWM3(negative);
	  break;
	case 2:  //0xb010:		// V->U		Q3->Q2
	  ENABLE_PWM1(negative);
	  ENABLE_PWM2(positive);
	  DISABLE_PWM3;
	  break;
	case 6:  //0xb110: 		// W->U		Q5->Q2
	  ENABLE_PWM1(negative);
	  DISABLE_PWM2;
	  ENABLE_PWM3(positive);
	  break;
	case 4:  //0xb100: 		// W->V		Q5->Q4
	  DISABLE_PWM1;
	  ENABLE_PWM2(negative);
	  ENABLE_PWM3(positive);
	  break;
	case 5:  //0xb101: 		// U->V		Q1->Q4
	  ENABLE_PWM1(positive);
	  ENABLE_PWM2(negative);
	  DISABLE_PWM3;
	  break;
    default:
	  DISABLE_MOTOR;			/* disable motor drive */
	  break;
  }
}

/*****************************************************************/
/*!	 Function that detects wheter there is a detent nearby or not (closer than  Detents_config_Max_distance)
	\param current_pos Current position expressed in external units
	\param detent_pos Pointer to the variable in which the detent position will be saved (in external units)
	\return The number of the detent (1 to MAX_DETENTS) if any detente nearby and '0' if not.
*/
/*****************************************************************/
int detent_nearby(long current_pos, long *detent_pos)
{
	int i;
	long d_pos;
	UNS32 errorCode;
	ODCallback_t *Callback;
	const indextable* detents = (amc_od_Data.scanIndexOD)(0x2110, &errorCode, &Callback);

	for (i=0; i<MAX_DETENTS; ++i)
	{
		d_pos = *(long *)detents->pSubindex[i+1].pObject;
		if ((0x01 << i) & Detents_config_Enable_detents) 	/* chech if the detent is configured */
		{
			if (labs(current_pos - d_pos) <= Detents_config_Max_input_distance)
			{
				*detent_pos = d_pos;
				return i+1;
			}
		}
	}
	return 0;		/* no nearby detent fount */
}

void check_hall_effect(manual_state_t manual_state)
{
	unsigned char hall_state_for_error = 0;

	if ((motor != BLDC_MOTOR_a) && (motor != BLDC_MOTOR_b) && (motor != BLDC_MOTOR_c))
	{
		return;
	}

	hall_state_for_error=((GpioDataRegs.GPBDAT.all & 0x001D0000) >> 18);	// bits 0,1,2 = H1,H2,H3

	if ((motor == BLDC_MOTOR_a) || (motor == BLDC_MOTOR_c))
	{
		hall_state_for_error ^= 7;
	}

	if ((hall_state_for_error >= 1) && (hall_state_for_error <=6))
	{
		DeQueueFault(FAULT_HALL);
	}
	else
	{
		if(((Modes_of_operation_display == OPERATION_MODE_MANUAL)||(Modes_of_operation_display == OPERATION_MODE_MANUAL_2))
				&& manual_state != DETENTS_ACTIVE)
			DeQueueFault(FAULT_HALL);
		else{

			if(!(isFaultActive(FAULT_HALL)))
			{
				_ERRORmessage(0xFF0F, 0x80, 0x0000, "Hall effect sensor invalid", 0, 0);
				setFault(FAULT_HALL);
			}
			QueueFault(FAULT_HALL);
		}
	}
}

/*** end of file *****************************************************/

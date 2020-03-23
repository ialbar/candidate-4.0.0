/*!
 * \file  Ecap.c
 * \brief Source file of the functions for the capture modules.
 * ****************************************************************************/

#include "DSP2833x_Device.h"
#include "Ecap.h"


#define PWM_RESOLUTION 4096

/*!
 * \typedef t_pwm_sensor
 * \brief   Type to save information to calculate the sensor value.
 * ****************************************************************************/
typedef struct{
	unsigned int resolution; /*!< resolution of the sensor */
	unsigned long duty_on;    /*!< Capture ticks of active pulse */
	unsigned long period;     /*!< Capture ticks of total period */
}t_capture;

t_capture pwm_sensor; /*!< instance to save last values from capture */
t_capture pwm_in1;    /*!< instance to save last values from capture */
t_capture pwm_in2;    /*!< instance to save last values from capture */
t_capture pwm_in3;    /*!< instance to save last values from capture */

/* Local functions' prototypes */
void InitECapture2( void );
void InitECapture5( void );
void InitECapture6( void );


/*!
 * \brief funcion to intialize the ECap1 capture module.
 * - Disable capture interrupts and clear all cap interrupt flags.
 * - Disable load of registers and stop counter.
 * - CAP1 falling edge, CAP2 rising, CAP3 fallin, CAP4 rising.
 * - No reset counter at CAP1, CAP2, CAP3 event. Reset on CAP4 event.
 * - Prescaler bypassed.
 * - Stop on emulation suspend.
 * - Continuous mode operation.
 * - Wrap after capture event 4.
 * - No rearming control.
 * - Disable sync in option.
 * - Disable sync out signal.
 * - No software sync.
 * - Capture mode.
 * - APWM polarity has no effect on CAP mode.
 * - Starts counter and enable load of capture registers.
 * - Capture interrupts at second and fourth edge.
 * ****************************************************************************/
void InitECapture2( void )
{
	/* Disables and turns off capture module to configure it */
	ECap2Regs.ECEINT.all           = 0x0000;
	ECap2Regs.ECCLR.all            = 0xFFFF;
	ECap2Regs.ECCTL1.bit.CAPLDEN   = 0;
	ECap2Regs.ECCTL2.bit.TSCTRSTOP = 0;

	/* Control Register 1 */
	ECap2Regs.ECCTL1.bit.CAP1POL   = 1;
	ECap2Regs.ECCTL1.bit.CAP2POL   = 0;
	ECap2Regs.ECCTL1.bit.CAP3POL   = 1;
	ECap2Regs.ECCTL1.bit.CAP4POL   = 0;
	ECap2Regs.ECCTL1.bit.CTRRST1   = 0;
	ECap2Regs.ECCTL1.bit.CTRRST2   = 0;
	ECap2Regs.ECCTL1.bit.CTRRST3   = 0;
	ECap2Regs.ECCTL1.bit.CTRRST4   = 1; 
	ECap2Regs.ECCTL1.bit.PRESCALE  = 0; 
	ECap2Regs.ECCTL1.bit.FREE_SOFT = 0;
	
	/* Control Register 2 */
	ECap2Regs.ECCTL2.bit.CONT_ONESHT = 0;
	ECap2Regs.ECCTL2.bit.STOP_WRAP   = 3;
	ECap2Regs.ECCTL2.bit.REARM       = 0;
	ECap2Regs.ECCTL2.bit.SYNCI_EN    = 0;
	ECap2Regs.ECCTL2.bit.SYNCO_SEL   = 2;
	ECap2Regs.ECCTL2.bit.SWSYNC      = 0;
	ECap2Regs.ECCTL2.bit.CAP_APWM    = 0;
	ECap2Regs.ECCTL2.bit.APWMPOL     = 0;
	
	/* Enabling and turning on the capture module after configuration */
	ECap2Regs.ECCTL2.bit.TSCTRSTOP = 1;
	ECap2Regs.ECCTL1.bit.CAPLDEN   = 1;
	ECap2Regs.ECEINT.bit.CEVT2     = 1;
	ECap2Regs.ECEINT.bit.CEVT4     = 1;

	/* Saves the resolution of the sensor */
	pwm_in1.resolution = PWM_RESOLUTION;
}

/*!
 * \brief Function to initialize capture module 4, for a PWM sensor.
 * \param [in] resolution Decimal value of PWM resolution.
 * \param [in] polarity   \li 1: Active at high level
 *                        \li 0: Active at low level.
 * ****************************************************************************/
void InitECap4PWMSensor( unsigned short resolution, unsigned char polarity )
{	
	/* Disables and turns off capture module to configure it */
	ECap4Regs.ECEINT.all           = 0x0000;
	ECap4Regs.ECCLR.all            = 0xFFFF;
	ECap4Regs.ECCTL1.bit.CAPLDEN   = 0;
	ECap4Regs.ECCTL2.bit.TSCTRSTOP = 0;

	/* Control Register 1 */
	ECap4Regs.ECCTL1.bit.CAP1POL   = polarity;
	ECap4Regs.ECCTL1.bit.CAP2POL   = !polarity;
	ECap4Regs.ECCTL1.bit.CAP3POL   = polarity;
	ECap4Regs.ECCTL1.bit.CAP4POL   = !polarity;
	ECap4Regs.ECCTL1.bit.CTRRST1   = 0;
	ECap4Regs.ECCTL1.bit.CTRRST2   = 0;
	ECap4Regs.ECCTL1.bit.CTRRST3   = 0;
	ECap4Regs.ECCTL1.bit.CTRRST4   = 1; 
	ECap4Regs.ECCTL1.bit.PRESCALE  = 0; 
	ECap4Regs.ECCTL1.bit.FREE_SOFT = 0;
	
	/* Control Register 2 */
	ECap4Regs.ECCTL2.bit.CONT_ONESHT = 0;
	ECap4Regs.ECCTL2.bit.STOP_WRAP   = 3;
	ECap4Regs.ECCTL2.bit.REARM       = 0;
	ECap4Regs.ECCTL2.bit.SYNCI_EN    = 0;
	ECap4Regs.ECCTL2.bit.SYNCO_SEL   = 2;
	ECap4Regs.ECCTL2.bit.SWSYNC      = 0;
	ECap4Regs.ECCTL2.bit.CAP_APWM    = 0;
	ECap4Regs.ECCTL2.bit.APWMPOL     = 0;
	
	/* Enabling and turnin on the capture module after configuration */
	ECap4Regs.ECCTL2.bit.TSCTRSTOP = 1;
	ECap4Regs.ECCTL1.bit.CAPLDEN   = 1;
	ECap4Regs.ECEINT.bit.CEVT2     = 1;
	ECap4Regs.ECEINT.bit.CEVT4     = 1;
   
	/* Saves the resolution of the sensor */
	pwm_sensor.resolution = resolution;
}

/*!
 * \brief Function to initialize capture module 5.
 * Same as previous.
 * ****************************************************************************/
void InitECapture5( void )
{
	/* Disables and turns off capture module to configure it */
	ECap5Regs.ECEINT.all           = 0x0000;
	ECap5Regs.ECCLR.all            = 0xFFFF;
	ECap5Regs.ECCTL1.bit.CAPLDEN   = 0;
	ECap5Regs.ECCTL2.bit.TSCTRSTOP = 0;

	/* Control Register 1 */
	ECap5Regs.ECCTL1.bit.CAP1POL   = 1;
	ECap5Regs.ECCTL1.bit.CAP2POL   = 0;
	ECap5Regs.ECCTL1.bit.CAP3POL   = 1;
	ECap5Regs.ECCTL1.bit.CAP4POL   = 0;
	ECap5Regs.ECCTL1.bit.CTRRST1   = 0;
	ECap5Regs.ECCTL1.bit.CTRRST2   = 0;
	ECap5Regs.ECCTL1.bit.CTRRST3   = 0;
	ECap5Regs.ECCTL1.bit.CTRRST4   = 1; 
	ECap5Regs.ECCTL1.bit.PRESCALE  = 0; 
	ECap5Regs.ECCTL1.bit.FREE_SOFT = 0;
	
	/* Control Register 2 */
	ECap5Regs.ECCTL2.bit.CONT_ONESHT = 0;
	ECap5Regs.ECCTL2.bit.STOP_WRAP   = 3;
	ECap5Regs.ECCTL2.bit.REARM       = 0;
	ECap5Regs.ECCTL2.bit.SYNCI_EN    = 0;
	ECap5Regs.ECCTL2.bit.SYNCO_SEL   = 2;
	ECap5Regs.ECCTL2.bit.SWSYNC      = 0;
	ECap5Regs.ECCTL2.bit.CAP_APWM    = 0;
	ECap5Regs.ECCTL2.bit.APWMPOL     = 0;
	
	/* Enabling and turning on the capture module after configuration */
	ECap5Regs.ECCTL2.bit.TSCTRSTOP = 1;
	ECap5Regs.ECCTL1.bit.CAPLDEN   = 1;
	ECap5Regs.ECEINT.bit.CEVT2     = 1;
	ECap5Regs.ECEINT.bit.CEVT4     = 1;

	/* Saves the resolution of the sensor */
	pwm_in2.resolution = PWM_RESOLUTION;
}

/*!
 * \brief Function to initialize capture module 6.
 * Same as previous.
 * ****************************************************************************/
void InitECapture6( void )
{
	/* Disables and turns off capture module to configure it */
	ECap6Regs.ECEINT.all           = 0x0000;
	ECap6Regs.ECCLR.all            = 0xFFFF;
	ECap6Regs.ECCTL1.bit.CAPLDEN   = 0;
	ECap6Regs.ECCTL2.bit.TSCTRSTOP = 0;

	/* Control Register 1 */
	ECap6Regs.ECCTL1.bit.CAP1POL   = 1;
	ECap6Regs.ECCTL1.bit.CAP2POL   = 0;
	ECap6Regs.ECCTL1.bit.CAP3POL   = 1;
	ECap6Regs.ECCTL1.bit.CAP4POL   = 0;
	ECap6Regs.ECCTL1.bit.CTRRST1   = 0;
	ECap6Regs.ECCTL1.bit.CTRRST2   = 0;
	ECap6Regs.ECCTL1.bit.CTRRST3   = 0;
	ECap6Regs.ECCTL1.bit.CTRRST4   = 1; 
	ECap6Regs.ECCTL1.bit.PRESCALE  = 0; 
	ECap6Regs.ECCTL1.bit.FREE_SOFT = 0;
	
	/* Control Register 2 */
	ECap6Regs.ECCTL2.bit.CONT_ONESHT = 0;
	ECap6Regs.ECCTL2.bit.STOP_WRAP   = 3;
	ECap6Regs.ECCTL2.bit.REARM       = 0;
	ECap6Regs.ECCTL2.bit.SYNCI_EN    = 0;
	ECap6Regs.ECCTL2.bit.SYNCO_SEL   = 2;
	ECap6Regs.ECCTL2.bit.SWSYNC      = 0;
	ECap6Regs.ECCTL2.bit.CAP_APWM    = 0;
	ECap6Regs.ECCTL2.bit.APWMPOL     = 0;
	
	/* Enabling and turning on the capture module after configuration */
	ECap6Regs.ECCTL2.bit.TSCTRSTOP = 1;
	ECap6Regs.ECCTL1.bit.CAPLDEN   = 1;
	ECap6Regs.ECEINT.bit.CEVT2     = 1;
	ECap6Regs.ECEINT.bit.CEVT4     = 1;

	/* Saves the resolution of the sensor */
	pwm_in3.resolution = PWM_RESOLUTION;
}

/*!
 * \brief Function to initialize Ecap modules for the motor.
 * ****************************************************************************/
void InitECapture( void )
{
	InitECapture2();
	InitECapture5();
	InitECapture6();
	//TODO: Modules 2, 5 and 6 polarity. Call of the sensor init */
}

/*!
 * \brief Module 2 Capture function.
 * ****************************************************************************/
void pwm_in1_capture( void )
{
	static unsigned long cap1 = 0;
	static unsigned long cap2 = 0;
	static unsigned long cap3 = 0;
	static unsigned long cap4 = 0;
	
	if ( ECap2Regs.ECFLG.bit.CEVT2 )
	{
		cap1 = ECap2Regs.CAP1;
		cap2 = ECap2Regs.CAP2;
		pwm_sensor.duty_on = cap1 - cap4;
		pwm_sensor.period  = cap2 - cap4;
		ECap2Regs.ECCLR.bit.CEVT2 = 1;
	}
	if ( ECap1Regs.ECFLG.bit.CEVT4 )
	{
		cap3 = ECap2Regs.CAP3;
		cap4 = ECap2Regs.CAP4;
		pwm_sensor.duty_on = cap3 - cap2;
		pwm_sensor.duty_on = cap4 - cap2;
		ECap2Regs.ECCLR.bit.CEVT4 = 1;
	}
	
	/* Clear interrupt flag, arm one-shot seq */
   ECap2Regs.ECCLR.bit.INT = 1;
}

/*!
 * \brief Module 5 Capture function.
 * ****************************************************************************/
void pwm_in2_capture( void )
{
	static unsigned long cap1 = 0;
	static unsigned long cap2 = 0;
	static unsigned long cap3 = 0;
	static unsigned long cap4 = 0;
	
	if ( ECap5Regs.ECFLG.bit.CEVT2 )
	{
		cap1 = ECap5Regs.CAP1;
		cap2 = ECap5Regs.CAP2;
		pwm_sensor.duty_on = cap1 - cap4;
		pwm_sensor.period  = cap2 - cap4;
		ECap5Regs.ECCLR.bit.CEVT2 = 1;
	}
	if ( ECap5Regs.ECFLG.bit.CEVT4 )
	{
		cap3 = ECap5Regs.CAP3;
		cap4 = ECap5Regs.CAP4;
		pwm_sensor.duty_on = cap3 - cap2;
		pwm_sensor.duty_on = cap4 - cap2;
		ECap5Regs.ECCLR.bit.CEVT4 = 1;
	}
	
	/* Clear interrupt flag, arm one-shot seq */
   ECap5Regs.ECCLR.bit.INT = 1;
}

/*!
 * \brief Module 6 Capture function.
 * ****************************************************************************/
void pwm_in3_capture( void )
{
	static unsigned long cap1 = 0;
	static unsigned long cap2 = 0;
	static unsigned long cap3 = 0;
	static unsigned long cap4 = 0;
	
	if ( ECap6Regs.ECFLG.bit.CEVT2 )
	{
		cap1 = ECap6Regs.CAP1;
		cap2 = ECap6Regs.CAP2;
		pwm_sensor.duty_on = cap1 - cap4;
		pwm_sensor.period  = cap2 - cap4;
		ECap6Regs.ECCLR.bit.CEVT2 = 1;
	}
	if ( ECap6Regs.ECFLG.bit.CEVT4 )
	{
		cap3 = ECap6Regs.CAP3;
		cap4 = ECap6Regs.CAP4;
		pwm_sensor.duty_on = cap3 - cap2;
		pwm_sensor.duty_on = cap4 - cap2;
		ECap6Regs.ECCLR.bit.CEVT4 = 1;
	}
	
	/* Clear interrupt flag, arm one-shot seq */
   ECap6Regs.ECCLR.bit.INT = 1;
}

/******************************************************************************/
/*!
 * \brief Module 4 Capture function.
 */
/******************************************************************************/
void pwm_sensor_capture( void )
{
	static unsigned long cap1 = 0;
	static unsigned long cap2 = 0;
	static unsigned long cap3 = 0;
	static unsigned long cap4 = 0;
	
	if ( ECap4Regs.ECFLG.bit.CEVT2 )
	{
		cap1 = ECap4Regs.CAP1;
		cap2 = ECap4Regs.CAP2;
		pwm_sensor.duty_on = cap1 - cap4;
		pwm_sensor.period  = cap2 - cap4;
		ECap4Regs.ECCLR.bit.CEVT2 = 1;
	}
	if ( ECap4Regs.ECFLG.bit.CEVT4 )
	{
		cap3 = ECap4Regs.CAP3;
		cap4 = ECap4Regs.CAP4;
		pwm_sensor.duty_on = cap3 - cap2;
		pwm_sensor.duty_on = cap4 - cap2;
		ECap4Regs.ECCLR.bit.CEVT4 = 1;
	}
	
	/* Clear interrupt flag, arm one-shot seq */
   ECap4Regs.ECCLR.bit.INT = 1;
}

/*!
 * \brief Calculates the value from the last capture data from ECap2.
 * \return Value from 0 to sensor resolution - 1.
 * ****************************************************************************/
unsigned short calculate_pwm_in1_value( void )
{
	unsigned long duty_on;
	unsigned long period;
	unsigned int resolution;
	
	ECap2Regs.ECEINT.bit.CEVT2 = 0;
	ECap2Regs.ECEINT.bit.CEVT4 = 0;
		
	duty_on     = pwm_in1.duty_on;
	period      = pwm_in1.period;
	resolution  = pwm_in1.resolution;
	
	ECap2Regs.ECEINT.bit.CEVT2 = 1;
	ECap2Regs.ECEINT.bit.CEVT4 = 1;
	
	return ( unsigned short ) ( ((unsigned long long)(duty_on)*( resolution + 1 ) +
								period/2 ) / period ) - 1;
}

/*!
 * \brief Calculates the value from the last capture data from ECap5.
 * \return Value from 0 to sensor resolution - 1.
 * ****************************************************************************/
unsigned short calculate_pwm_in2_value( void )
{
	unsigned long duty_on;
	unsigned long period;
	unsigned int resolution;
	
	ECap5Regs.ECEINT.bit.CEVT2 = 0;
	ECap5Regs.ECEINT.bit.CEVT4 = 0;

	duty_on     = pwm_in2.duty_on;
	period      = pwm_in2.period;
	resolution  = pwm_in2.resolution;

	ECap5Regs.ECEINT.bit.CEVT2 = 1;
	ECap5Regs.ECEINT.bit.CEVT4 = 1;
	
	return ( unsigned short ) ( ((unsigned long long)(duty_on)*( resolution + 1 ) +
								period/2 ) / period ) - 1;
}

/*!
 * \brief Calculates the value from the last capture data from ECap6.
 * \return Value from 0 to sensor resolution - 1.
 * ****************************************************************************/
unsigned short calculate_pwm_in3_value( void )
{
	unsigned long duty_on;
	unsigned long period;
	unsigned int resolution;
	
	ECap6Regs.ECEINT.bit.CEVT2 = 0;
	ECap6Regs.ECEINT.bit.CEVT4 = 0;
	
	duty_on     = pwm_in3.duty_on;
	period      = pwm_in3.period;
	resolution  = pwm_in3.resolution;
	
	ECap6Regs.ECEINT.bit.CEVT2 = 1;
	ECap6Regs.ECEINT.bit.CEVT4 = 1;

	return ( unsigned short ) ( ((unsigned long long)(duty_on)*( resolution + 1 ) +
								period/2 ) / period ) - 1;
}

/*!
 * \brief Calculates the value from the last capture data from ECap4.
 * \return Value from 0 to sensor resolution - 1.
 * ****************************************************************************/
void calculate_pwm_sensor_value( unsigned short *value, unsigned long *per )
{
	unsigned long duty_on;
	unsigned long period;
	unsigned int resolution;
	
	ECap4Regs.ECEINT.bit.CEVT2 = 0;
	ECap4Regs.ECEINT.bit.CEVT4 = 0;
	
	duty_on     = pwm_sensor.duty_on;
	period      = pwm_sensor.period;
	resolution  = pwm_sensor.resolution;

	ECap4Regs.ECEINT.bit.CEVT2 = 1;
	ECap4Regs.ECEINT.bit.CEVT4 = 1;

	if(value) *value = ( unsigned short ) ( ((unsigned long long)(duty_on)*( resolution + 1 ) +
								period/2 ) / period ) - 1;
	if(per) *per = period;
}

/*!	\file motor_phase_check.c
	\brief Functions to handle the motor phase check of the servo
*/

#include "motor_phase_check.h"
#include "amc_od.h"
#include "amc.h"

#define MINIMUM_CURRENT			750		// Minimum positive current needed for check each phase
#define MINIMUM_ZEROCURRENT		-200	// Minimum current admitted for zero current
#define MAXIMUM_ZEROCURRENT		200		// Maximum current admitted for zero current
#define WAIT_CYCLES				10		// Number of waiting cycles to measure first currents
#define MAX_RETRIES				10		// Maximum number of tries if test fails

static motor_phase_check_state check_state = IDLE;		// Current state
unsigned char s_start = 0;								// Start and finish fsm variables
unsigned int phase_positive = 0, phase_negative = 0;	// CMP values for ePWM
unsigned char check_running = 0;						// Test is stopped/running

/*****************************************************************/
/*!	Function that checks that all the motor phases are
	connected correctly
*/
/*****************************************************************/
void motor_phase_check(char init)
{
	static unsigned short n_cycles = 0, n_retries = 0;
	unsigned int pwm_effort = 0;
	_iq aux = 0;

	/* Initialization */
	if(init)
	{
		check_state = IDLE;
		n_retries = 0;
		ATM_seti(&ready_to_power_on, 1);
		/* Set duty cycle for the test */
		pwm_effort = PWM_Check_effort;
		/* Calculate CMP values */
		aux = _IQsat(_IQmpyI32(_IQ(0.0001), (long)pwm_effort), _IQ(1.0), _IQ(-1.0));
		phase_positive = _IQmpyI32int(_IQmpy(_IQ(1.0) - aux, _IQ(0.5)), Half_Period); //spread spectrum offset not taken into account
		phase_negative = Half_Period - phase_positive;
		return;
	}

	/* Check control word to start	 */
	if( (Device_control_word & CHECK_MOTOR_PHASE_START) )
	{
		s_start = 1;
		Device_control_word &= ~CHECK_MOTOR_PHASE_START;
		Device_status_word &= ~(CHECK_MOTOR_PHASE_ATTAINED | CHECK_MOTOR_PHASE_ERROR); /* Initialize the Control and status Word */
		set_fault_flags();
		set_brakes_flags();
		send_controlword();
	}

	/* Checking motor state machine */
	switch(check_state)
	{
		/* Initial state */
		case IDLE:
			if(s_start == 1)
			{
				s_start = 0;					// Reset start variable
				check_running = 1; 				// Set running variable until the fsm ends
				check_state = CHECKING_PHASE_1;
			}
			break;
		/* Checking U and V phases */
		case CHECKING_PHASE_1:
			n_cycles++;
			/* Wait until current raise to the minimum current acceptable or set error */
			if (n_cycles < WAIT_CYCLES)
			{
				if(current1 > MINIMUM_CURRENT && (current3 > MINIMUM_ZEROCURRENT && current3 < MAXIMUM_ZEROCURRENT))
				{
					if(motor == DC_MOTOR_a || motor == DC_MOTOR_b)
					{
						n_cycles = 0;
						check_state = CHECKED_OK;
					}
					else
					{
						n_cycles = 0;
						check_state = WAITING_ZEROCURRENT;
					}
				}
			}
			else
			{
				n_cycles = 0;
				check_state = CHECKED_ERROR;
			}
			break;
		/* Wait for zero current in the motor */
		case WAITING_ZEROCURRENT:
			pwm_effort = 0;
			n_cycles++;
			/* Wait until currents decrease to the zero current or set error */
			if (n_cycles < WAIT_CYCLES)
			{
				if(current1 < MAXIMUM_ZEROCURRENT && current3 < MAXIMUM_ZEROCURRENT)
				{
					n_cycles = 0;
					check_state = CHECKING_PHASE_2;
				}
			}
			else
			{
				n_cycles = 0;
				check_state = CHECKED_ERROR;
			}
			break;
		/* Checking U and W phases */
		case CHECKING_PHASE_2:
			n_cycles++;
			/* Wait until current raise to the minimum current acceptable or set error */
			if (n_cycles < WAIT_CYCLES)
			{
				short result = 0;

				result = current1 + current3;
				if(current1 > MINIMUM_CURRENT && (result > MINIMUM_ZEROCURRENT && result < MAXIMUM_ZEROCURRENT))
				{
					n_cycles = 0;
					check_state = CHECKED_OK;
				}
			}
			else
			{
				n_cycles = 0;
				check_state = CHECKED_ERROR;
			}
			break;
		/* Motor phase check error state */
		case CHECKED_ERROR:
			if(++n_retries > MAX_RETRIES)
			{
				Device_status_word |= CHECK_MOTOR_PHASE_ERROR;
				if(!(isFaultActive(FAULT_MOTOR_PHASE)))
				{
					/* Set the Fault flag and the error information. Additional info */
					_ERRORmessage(0xFF0C, 0x80, 0x0000, "Motor phase error", 0, 0);
					setFault(FAULT_MOTOR_PHASE);
				}
				QueueFault(FAULT_MOTOR_PHASE);
				check_running = 0;	// Reset running variable
				check_state = IDLE;
			}
			else
			{
				check_state = IDLE;
				s_start = 1;
			}
			break;
		/* Motor phases checked correctly */
		case CHECKED_OK:
			Device_status_word |= CHECK_MOTOR_PHASE_ATTAINED;
			DeQueueFault(FAULT_MOTOR_PHASE);
			n_retries = 0;
			check_running = 0;	// Reset running variable
			check_state = IDLE;
			break;
		case SKIP_CHECK:
			check_running = 0;	// Reset running variable
			check_state = IDLE;
		default:
			check_state = IDLE;
			break;
	}

	/* End of function */
}

/*****************************************************************/
/*!	 Function that return the current state
        \return check_state
*/
/*****************************************************************/
motor_phase_check_state get_motor_phase_check_state(void)
{
	return check_state;
}

/*****************************************************************/
/*!	 Function that starts the check fsm
*/
/*****************************************************************/
void set_motor_phase_check_start(void)
{
	s_start = 1;
}

/*****************************************************************/
/*!	 Function that applies the CMP values to ePWMs
*/
/*****************************************************************/
void apply_motor_phase_check(void)
{
	switch(check_state)
	{
		case CHECKING_PHASE_1:
			ENABLE_PWM1(phase_positive);
			ENABLE_PWM2(phase_negative);
			DISABLE_PWM3;
			break;
		case CHECKING_PHASE_2:
			ENABLE_PWM1(phase_positive);
			DISABLE_PWM2;
			ENABLE_PWM3(phase_negative);
			break;
		case WAITING_ZEROCURRENT:
			DISABLE_PWM1;
			DISABLE_PWM2;
			DISABLE_PWM3;
			break;
		default:
			DISABLE_PWM1;
			DISABLE_PWM2;
			DISABLE_PWM3;
			break;
	}
}

/*****************************************************************/
/*!	 Function that returns 0 if stopped 1 if running test
*/
/*****************************************************************/
unsigned char motor_phase_check_running(void)
{
	return check_running;
}

void skip_motor_phase_checking(void)
{
	check_state = SKIP_CHECK;
	motor_phase_check(0);
}

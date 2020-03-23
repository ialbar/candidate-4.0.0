/*!
 \file   EQep.c
 \brief  Functions for eQEP initialization.
 \author Manuel Godoy
 \note   Devices: TMS320F2833x
 */
/******************************************************************************/

/* Includes */
#include "DSP2833x_Device.h"
#include "amc.h"

/*!
 \brief Initializes the EQep module 
 - Unit Time period for 166.67 Hz at 150 MHz SYSCLKOUT. (10000rpm = 166.67 Hz)
 - QEP quadrature count mode.
 - Unaffected by emulation suspend.
 - QPOSCNT resent on the maximum position.
 - Unit Timeout Enable.
 - Latch on unit time out.
 - Position max: 0xFFFFFFFF
 - Enables qep.
 - UPEVNT = QCLK/32
 - CAPCLK = SYSCLKOUT/128
 
 Math explanation:
 Frec(min speed) of quep = (10rev/min*4000pulses/rev)/(60s/min*32)
 T( min speed) = 1/frec = 0.048 s
 CAPCLK = 65535/0.048 = 13365313 Hz
 Min Prescaler = 150*10^6/1365313 = 110. ---> 128
 */
 

void InitEQep(void)
{
	/* Encoder 1 */
	EQep1Regs.QUPRD                = 900000;      /* Unit Timer for 166.67Hz at 150 MHz SYSCLKOUT */

	EQep1Regs.QDECCTL.bit.QSRC     = 00;          /* QEP quadrature count mode */
	EQep1Regs.QDECCTL.bit.QAP      = 1;           /* invert A input */
	EQep1Regs.QDECCTL.bit.QBP      = 1;           /* invert B input */
	EQep1Regs.QDECCTL.bit.QIP      = 1;           /* invert Index input */

	EQep1Regs.QEPCTL.bit.FREE_SOFT = 2;           /* Position counter is unaffected by emulation suspend */
	EQep1Regs.QEPCTL.bit.PCRM      = 01;          /* PCRM=01 mode - QPOSCNT reset on the maximum position */
	EQep1Regs.QEPCTL.bit.UTE       = 1;           /* Unit Timeout Enable */
	EQep1Regs.QEPCTL.bit.QCLM      = 1;           /* Latch on unit time out */
	EQep1Regs.QPOSMAX              = 0xFFFFFFFF;
	EQep1Regs.QEPCTL.bit.QPEN      = 1;           /* QEP enable */

	EQep1Regs.QCAPCTL.bit.UPPS     = 5;           /* 1/32 for unit position */
	EQep1Regs.QCAPCTL.bit.CCPS     = 7;           /* 1/128 for CAP clock */
	EQep1Regs.QCAPCTL.bit.CEN      = 1;           /* QEP Capture Enable */

	/* Encoder 2 */
	EQep2Regs.QUPRD                = 900000;      /* Unit Timer for 166.67Hz at 150 MHz SYSCLKOUT */

	EQep2Regs.QDECCTL.bit.QSRC     = 00;          /* QEP quadrature count mode */
	EQep2Regs.QDECCTL.bit.QAP      = 1;           /* invert A input */
	EQep2Regs.QDECCTL.bit.QBP      = 1;           /* invert B input */
	EQep2Regs.QDECCTL.bit.QIP      = 1;           /* invert Index input */

	EQep2Regs.QEPCTL.bit.FREE_SOFT = 2;           /* Position counter is unaffected by emulation suspend */
	EQep2Regs.QEPCTL.bit.PCRM      = 01;          /* PCRM=01 mode - QPOSCNT reset on the maximum position */
	EQep2Regs.QEPCTL.bit.UTE       = 1;           /* Unit Timeout Enable */
	EQep2Regs.QEPCTL.bit.QCLM      = 1;           /* Latch on unit time out */
	EQep2Regs.QPOSMAX              = 0xFFFFFFFF;
	EQep2Regs.QEPCTL.bit.QPEN      = 1;           /* QEP enable */

	EQep2Regs.QCAPCTL.bit.UPPS     = 5;           /* 1/32 for unit position */
	EQep2Regs.QCAPCTL.bit.CCPS     = 7;           /* 1/128 for CAP clock */
	EQep2Regs.QCAPCTL.bit.CEN      = 1;           /* QEP Capture Enable */
}


//===========================================================================
// End of file.
//===========================================================================

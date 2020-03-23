/*!	\file Adc.c
	\brief Functions to initialize the ADC module of the TMS320F2812

\verbatim
*********************************************************************
* File: Adc.c
* Devices: TMS320F28XXX
* Author: David M. Alter, Texas Instruments Inc.
* History:
*   09/08/03 - original (based on DSP281x header files v1.00, D. Alter)
*   03/18/04 - fixed comment field for ADCTRL1.5 bit
*********************************************************************
\endverbatim
*/

#include "DSP2833x_Device.h"
#include "amc.h"

#define ADC_usDELAY  10000L

/******************************************************************************/
/*!
 * \brief Adc initial configuration
 *
 * - Ignores emulation suspend.
 * - Acquisition window size at 16 ADCLK.
 * - Core clock prescaler at half frecuency of HSCLK.
 * - Continuous run disabled.
 * - Sequencer override disabled.
 * - Cascaded sequence mode enabled.
 *
 * - Interrupt enable SEQ1 for every SEQ1. Enable SEQ1 from ePWM SOCA trigger.
 *
 * - Bandgap, reference and analog circuitry power up. No ADC clock prescaler.
 * - 
 * - 7 Conversions.
 * - ADCINA0 and ADCINB0 as 1st SEQ conversion.
 * - ADCINA1 and ADCINB1 as 2nd SEQ conversion.
 * - ADCINA2 and ADCINB2 as 3th SEQ conversion.
 * - ADCINA3 and ADCINB3 as 4th SEQ conversion.
 * - ADCINA4 and ADCINB4 as 5th SEQ conversion.
 * - ADCINA5 and ADCINB5 as 6th SEQ conversion.
 * - ADCINA6 and ADCINB6 as 7th SEQ conversion.
 */
/******************************************************************************/
void InitAdc(void)
{
    extern void DSP28x_usDelay(Uint32 Count);

	// *IMPORTANT*
	// The ADC_cal function, which  copies the ADC calibration values from TI reserved
	// OTP into the ADCREFSEL and ADCOFFTRIM registers, occurs automatically in the
	// Boot ROM. If the boot ROM code is bypassed during the debug process, the
	// following function MUST be called for the ADC to function according
	// to specification. The clocks to the ADC MUST be enabled before calling this
	// function.
	// See the device data manual and/or the ADC Reference
	// Manual for more information.

	 /*   EALLOW;
		SysCtrlRegs.PCLKCR0.bit.ADCENCLK = 1;
		ADC_cal();
		EDIS;*/

	// To powerup the ADC the ADCENCLK bit should be set first to enable
	// clocks, followed by powering up the bandgap, reference circuitry, and ADC core.
	// Before the first conversion is performed a 5ms delay must be observed
	// after power up to give all analog circuits time to power up and settle

	// Please note that for the delay function below to operate correctly the
	// CPU_RATE define statement in the DSP2833x_Examples.h file must
	// contain the correct CPU clock period in nanoseconds.

	/* ADCTRL1 Register */
	AdcRegs.ADCTRL1.bit.SUSMOD   = 0;
	AdcRegs.ADCTRL1.bit.ACQ_PS   = 0xF;
	AdcRegs.ADCTRL1.bit.CPS      = 1;
	AdcRegs.ADCTRL1.bit.CONT_RUN = 0;
	AdcRegs.ADCTRL1.bit.SEQ_OVRD = 0;
	AdcRegs.ADCTRL1.bit.SEQ_CASC = 0;
	
	/* ADCTRL2 Register */
	AdcRegs.ADCTRL2.bit.EPWM_SOCB_SEQ  = 0;
	AdcRegs.ADCTRL2.bit.RST_SEQ1       = 0;
	AdcRegs.ADCTRL2.bit.SOC_SEQ1       = 0;
	AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1   = 1;
	AdcRegs.ADCTRL2.bit.INT_MOD_SEQ1   = 0;
	AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1 = 1;
	AdcRegs.ADCTRL2.bit.EXT_SOC_SEQ1   = 0;
	AdcRegs.ADCTRL2.bit.RST_SEQ2       = 0;
	AdcRegs.ADCTRL2.bit.SOC_SEQ2       = 0;
	AdcRegs.ADCTRL2.bit.INT_ENA_SEQ2   = 1;
	AdcRegs.ADCTRL2.bit.INT_MOD_SEQ2   = 0;
	AdcRegs.ADCTRL2.bit.EPWM_SOCB_SEQ2 = 1;
	
	/* ADCTRL3 Register. Must be set together. See lines below */	
	AdcRegs.ADCTRL3.all = 0x00E1;
	/*AdcRegs.ADCTRL3.bit.ADCBGRFDN    = 3;
	AdcRegs.ADCTRL3.bit.ADCPWDN        = 1;
	AdcRegs.ADCTRL3.bit.ADCCLKPS       = 0;
	AdcRegs.ADCTRL3.bit.SMODE_SEL      = 1;*/
	DelayUs(ADC_usDELAY);         // Delay before converting ADC channels
	
	/* Configure ADC */
	AdcRegs.ADCMAXCONV.all             = 0x0033; /* 4 double conversions seq1, 4 double convs seq2 */
	AdcRegs.ADCCHSELSEQ1.bit.CONV00    = 4;		/* ADCINA4/B4 */
	AdcRegs.ADCCHSELSEQ1.bit.CONV01    = 5;		/* ADCINA5/B5 */
	AdcRegs.ADCCHSELSEQ1.bit.CONV02    = 1;		/* ADCINA1/B1 */
	AdcRegs.ADCCHSELSEQ1.bit.CONV03    = 2;		/* ADCINA2/B2 */
	AdcRegs.ADCCHSELSEQ3.bit.CONV08    = 0;		/* ADCINA0/B0 */
	AdcRegs.ADCCHSELSEQ3.bit.CONV09    = 0;		/* ADCINA0/B0 again */
	AdcRegs.ADCCHSELSEQ3.bit.CONV10    = 0;		/* ADCINA0/B0 again */
	AdcRegs.ADCCHSELSEQ3.bit.CONV11    = 3;		/* ADCINA3/B3 */
	AdcRegs.ADCREFSEL.bit.REF_SEL      = 0;	
	
	/* Init ADC */
	AdcRegs.ADCTRL2.bit.RST_SEQ1       = 1;
	AdcRegs.ADCTRL2.bit.RST_SEQ2       = 1;
	
	/* Enable the ADC interrupt */
	PieCtrlRegs.PIEIER1.bit.INTx1 = 1;	/* Enable SEQ1 interrupt in PIE group 1 */
	PieCtrlRegs.PIEIER1.bit.INTx2 = 1;	/* Enable SEQ2 interrupt in PIE group 1 */
	IER |= 0x0001;						      /* Enable INT1 in IER to enable PIE group */
}


/*** end of file *****************************************************/

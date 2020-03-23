/*!
 \file   Epwm.c
 \brief  Functions for PWMs initialization.
 \author Manuel Godoy
 \note   Devices: TMS320F2833x
 */
/******************************************************************************/

#include "DSP2833x_Device.h"
#include "amc.h"

/* Local functions' prototypes */
void InitPWM1(void);
void InitPWM2(void);
void InitPWM3(void);
void InitPWM4(void);
void InitPWM6(void);

/*!
 \brief Configures all the ePWM modules needed.
 */
void InitEPwmModules(void)
{
   InitPWM1( );
   InitPWM2( );
   InitPWM3( );
   InitPWM4( );
   InitPWM6( );
}

/*!
 \brief Configures the ePWM modules related with motor SW.
 */
void InitSWEPwmModules(void)
{
   InitPWM1( );
   InitPWM2( );
   InitPWM3( );
}
/******************************************************************************/
/*!
 * \brief Function to configure ePWM1 Module.
 * - CLK and HSPCLK prescaler fixed to 1 and 1/2 respectively.
 *   TBCLK=SYSCLKOUT/(HSPCLKDIV*CLKDIV) = 75 MHz => T(TBCLK) = 13.333 ns.
 * - Counter mode Up-down count. Symetrical mode.
 * - Ignore the sync input to load the TBPHS value. Master module.
 * - Load TBPRD from shadow register when TBCTR is equal to zero.
 * - Send a sync output signal when TB_CTR is equal to zero.
 * - PWM_PERIOD = 2*TBPRD*T(TBCLK) = 2*1875*13.333ns = 50us.
 * - Set Phase register to zero.
 * - Set Use Shadows registers to CMPA and CMPB counters.
 * - Load values from shadow registers to CMPs if TBCTR is equal to zero.
 * - If TBCTR = CMPA and counter incrementing => EPWM1A high.
 * - If TBCTR = CMPA and counter decrementing => EPWM1A low.
 * - Enable deadband for rising and falling edges.
 * - Polarity select control high complementary. EPWM1B is inverted.
 * - Falling edge DeadBand fixed to and Rising edge fixed to 75 TBCLKs ~ 1us.
 * - Enable One-Shot Trip Zone 1. Force low for this trip event.
 * - Enable One-shot interrupt on trip-zone.
 */
/******************************************************************************/
void InitPWM1( void )
{		
	/* TimeBase Controler Register */
	EPwm1Regs.TBCTL.bit.CLKDIV     = TB_DIV1;
	EPwm1Regs.TBCTL.bit.HSPCLKDIV  = TB_DIV2;
	EPwm1Regs.TBCTL.bit.CTRMODE    = TB_COUNT_UPDOWN;
	EPwm1Regs.TBCTL.bit.PHSEN      = TB_DISABLE;
	EPwm1Regs.TBCTL.bit.PRDLD      = TB_SHADOW;
	EPwm1Regs.TBCTL.bit.SYNCOSEL   = TB_CTR_ZERO;
	
	/* Period and Phase */
	EPwm1Regs.TBPRD = Half_Period;
	EPwm1Regs.TBPHS.half.TBPHS     = 0;
	
	/* Compare Control Register */
	EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
	EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
	
	/* Action Controler Registers */
	EPwm1Regs.AQCTLA.bit.CAU       = AQ_SET;
	EPwm1Regs.AQCTLA.bit.CAD       = AQ_CLEAR;
	
	/* Dead Band Registers */
	EPwm1Regs.DBFED = DEAD_BAND;
	EPwm1Regs.DBRED = DEAD_BAND;
	
	/* Trip Zone configuration */
	EALLOW;
	EPwm1Regs.TZSEL.bit.OSHT1      = TZ_ENABLE;
	EPwm1Regs.TZCTL.bit.TZA        = TZ_FORCE_LO;
	EPwm1Regs.TZCTL.bit.TZB        = TZ_FORCE_LO;
	EPwm1Regs.TZEINT.bit.OST       = TZ_ENABLE;
	EDIS;
	
	/* enable TZINT interrupt */
	PieCtrlRegs.PIEIER2.bit.INTx1 = 1;
	IER |= M_INT2;
	
	/* Enable SOC on A group on period in first event. */
	EPwm1Regs.ETSEL.bit.SOCAEN  = 1;
	EPwm1Regs.ETSEL.bit.SOCASEL = ET_CTR_PRD;
	EPwm1Regs.ETPS.bit.SOCAPRD  = 1;
}

/******************************************************************************/
/*!
 * \brief Function to configure ePWM2 Module.
 * Same configuration as ePWM1 with PHSEN enable as a slave module and a output
 * sync flow-through.
 */
/******************************************************************************/
void InitPWM2( void )
{
	/* TimeBase Controler Register */
	EPwm2Regs.TBCTL.bit.CLKDIV     = TB_DIV1;
	EPwm2Regs.TBCTL.bit.HSPCLKDIV  = TB_DIV2;
	EPwm2Regs.TBCTL.bit.CTRMODE    = TB_COUNT_UPDOWN;
	EPwm2Regs.TBCTL.bit.PHSEN      = TB_ENABLE;
	EPwm2Regs.TBCTL.bit.PRDLD      = TB_SHADOW;
	EPwm2Regs.TBCTL.bit.SYNCOSEL   = TB_SYNC_IN;
	
	/* Period and Phase */
	EPwm2Regs.TBPRD = Half_Period;
	EPwm2Regs.TBPHS.half.TBPHS     = 0;
	
	/* Compare Control Register */
	EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
	EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
	
	/* Action Controler Registers */
	EPwm2Regs.AQCTLA.bit.CAU       = AQ_SET;
	EPwm2Regs.AQCTLA.bit.CAD       = AQ_CLEAR;
	
	/* Dead Band Registers */
	EPwm2Regs.DBFED = DEAD_BAND;
	EPwm2Regs.DBRED = DEAD_BAND;
	
	/* Trip Zone configuration */
	EALLOW;
	EPwm2Regs.TZSEL.bit.OSHT1      = TZ_ENABLE;
	EPwm2Regs.TZCTL.bit.TZA        = TZ_FORCE_LO;
	EPwm2Regs.TZCTL.bit.TZB        = TZ_FORCE_LO;
	//EPwm2Regs.TZEINT.bit.OST       = TZ_ENABLE;
	EDIS;
	
	/* enable TZINT interrupt */
	//PieCtrlRegs.PIEIER2.bit.INTx2 = 1;
	//IER |= M_INT2;
}

/******************************************************************************/
/*!
 * \brief Function to configure ePWM3 Module.
 * Same configuration as ePWM1 with PHSEN enable as a slave module and a output
 * sync flow-through.
 */
/******************************************************************************/
void InitPWM3( void )
{
	/* TimeBase Controler Register */
	EPwm3Regs.TBCTL.bit.CLKDIV     = TB_DIV1;
	EPwm3Regs.TBCTL.bit.HSPCLKDIV  = TB_DIV2;
	EPwm3Regs.TBCTL.bit.CTRMODE    = TB_COUNT_UPDOWN;
	EPwm3Regs.TBCTL.bit.PHSEN      = TB_ENABLE;
	EPwm3Regs.TBCTL.bit.PRDLD      = TB_SHADOW;
	EPwm3Regs.TBCTL.bit.SYNCOSEL   = TB_SYNC_IN;
	
	/* Period and Phase */
	EPwm3Regs.TBPRD = Half_Period;
	EPwm3Regs.TBPHS.half.TBPHS     = 0;
	
	/* Compare Control Register */
	EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
	EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
	
	/* Action Controler Registers */
	EPwm3Regs.AQCTLA.bit.CAU       = AQ_SET;
	EPwm3Regs.AQCTLA.bit.CAD       = AQ_CLEAR;
	
	/* Dead Band Registers */
	EPwm3Regs.DBFED = DEAD_BAND;
	EPwm3Regs.DBRED = DEAD_BAND;
	
	/* Trip Zone configuration */
	EALLOW;
	EPwm3Regs.TZSEL.bit.OSHT1      = TZ_ENABLE;
	EPwm3Regs.TZCTL.bit.TZA        = TZ_FORCE_LO;
	EPwm3Regs.TZCTL.bit.TZB        = TZ_FORCE_LO;
	//EPwm3Regs.TZEINT.bit.OST       = TZ_ENABLE;
	EDIS;
	
	/* enable TZINT interrupt */
	//PieCtrlRegs.PIEIER2.bit.INTx3 = 1;
	//IER |= M_INT2;
}

/******************************************************************************/
/*!
 * \brief Function to configure ePWM4 for a resolver excitement signal.
 * This function configure the module to generate a 50% 10KHz PWM to be filtered
 * by hardware in order to obtain a 10KHz senoidal signal. 
 * - CLK and HSPCLK prescaler fixed to 1 and 1/4 respectively.
 *   TBCLK=SYSCLKOUT/(HSPCLKDIV*CLKDIV) = 70 MHz => T(TBCLK) = 13.333 ns.
 * - Counter mode Upcount.
 * - Ignore the sync input to load the TBPHS value. Master module.
 * - Load TBPRD from shadow register when TBCTR is equal to zero.
 * - No output sync sending.
 * - PWM_PERIOD = (TBPRD+1)*T(TBCLK) = (3750+1)*13.3333 ns = 50us => 20KHz.
 * - Set Phase register to zero.
 * - Set Use Shadows registers to CMPA and CMPB counters.
 * - Load values from shadow registers to CMPs if TBCTR is equal to zero.
 * - Toggle value of PWM output for CMPA = CTR an counter incrementing.
 *   PWM runs at half frecuency of the timer, then.
 * - Enable SOCA.
 * - Eventrigger for SOC at CTR = PRD.
 * - Only in first event.
 * - No trip zone configured.
 */
/******************************************************************************/
void InitPWM4( void )
{
	/* TimeBase Controler Register */
	EPwm4Regs.TBCTL.bit.CLKDIV     = TB_DIV1;
	EPwm4Regs.TBCTL.bit.HSPCLKDIV  = TB_DIV2;
	EPwm4Regs.TBCTL.bit.CTRMODE    = TB_COUNT_UP;
	EPwm4Regs.TBCTL.bit.PHSEN      = TB_DISABLE;
	EPwm4Regs.TBCTL.bit.PRDLD      = TB_SHADOW;
	EPwm4Regs.TBCTL.bit.SYNCOSEL   = TB_SYNC_DISABLE;
	
	/* Period and Phase */
	EPwm4Regs.TBPRD = RESOLVER_PWM_PERIOD;
	EPwm4Regs.TBPHS.half.TBPHS     = 0;
	
	/* Compare Control Register */
	EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm4Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm4Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
	EPwm4Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
	
	/* Action Controler Registers */
	EPwm4Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
	EPwm4Regs.AQCTLA.bit.PRD = AQ_NO_ACTION;
	EPwm4Regs.AQCTLA.bit.CAU = AQ_TOGGLE;        /* toggle output on Compare A up */
	EPwm4Regs.AQCTLA.bit.CAD = AQ_NO_ACTION;
	EPwm4Regs.AQCTLA.bit.CBU = AQ_NO_ACTION;
	EPwm4Regs.AQCTLA.bit.CBD = AQ_NO_ACTION;
	
	/* Set the compare points */
	EPwm4Regs.CMPA.half.CMPA = RESOLVER_READ_POINT;  /* toggle output approx in the half of rising time */
	/* if RESOLVER_READ_POINT is increased ADC is triggered before */
	
	/* Event Trigger Registers */
	EPwm4Regs.ETSEL.bit.SOCBEN  = 1;
	EPwm4Regs.ETSEL.bit.SOCBSEL = ET_CTR_PRD;   /* ADC is triggered on Compare B down */
	EPwm4Regs.ETPS.bit.SOCBPRD  = 1;
}

/******************************************************************************/
/*!
 * \brief Function to configure ePWM6 Module for brakes.
 * - CLK and HSPCLK prescaler fixed to 1 and 1/2 respectively.
 *   TBCLK=SYSCLKOUT/(HSPCLKDIV*CLKDIV) = 75 MHz => T(TBCLK) = 13.333 ns.
 * - Counter mode Upcount.
 * - Ignore the sync input to load the TBPHS value. Master module.
 * - Load TBPRD from shadow register when TBCTR is equal to zero.
 * - No output sync sending.
 * - PWM_PERIOD = (TBPRD+1)*T(TBCLK) = (3750+1)*13.333 ns = 50us => 20KHz.
 * - Action is "toggle" => freq = 10 kHz
 * - Set Phase register to zero.
 * - Set Use Shadows registers to CMPA and CMPB counters.
 * - Load values from shadow registers to CMPs if TBCTR is equal to zero.
 * - Set PWM high when compare gets zero and set PWM low when TBCTR=CMPA and
 *   incrementing.
 * - Enable One-Shot Trip Zone 1. Force low for this trip event.
 *
 * \note GPIO configuration for PWM4 must be set in Gpio.c.
 */
/******************************************************************************/
void InitPWM6( void )
{
	/* TimeBase Controler Register */
	EPwm6Regs.TBCTL.bit.CLKDIV     = TB_DIV1;
	EPwm6Regs.TBCTL.bit.HSPCLKDIV  = TB_DIV2;
	EPwm6Regs.TBCTL.bit.CTRMODE    = TB_COUNT_UP;
	EPwm6Regs.TBCTL.bit.PHSEN      = TB_DISABLE;
	EPwm6Regs.TBCTL.bit.PRDLD      = TB_SHADOW;
	EPwm6Regs.TBCTL.bit.SYNCOSEL   = TB_SYNC_DISABLE;
	
	/* Period and Phase */
	EPwm6Regs.TBPRD = RESOLVER_PWM_PERIOD;
	EPwm6Regs.TBPHS.half.TBPHS     = 0;
	
	/* Compare Control Register */
	EPwm6Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm6Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm6Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
	EPwm6Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
	
	/* Action Controler Registers */
	EPwm6Regs.AQCTLA.bit.ZRO = AQ_SET;
	EPwm6Regs.AQCTLA.bit.CAU = AQ_CLEAR;
	EPwm6Regs.AQCTLB.bit.ZRO = AQ_SET;
	EPwm6Regs.AQCTLB.bit.CBU = AQ_CLEAR;
}


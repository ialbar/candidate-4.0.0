/*!	\file SysCtrl.c
	\brief Initializacions for the CPU and Flash timing

\verbatim
*********************************************************************
* File: SysCtrl.c
* Devices: TMS320F28XXX
* Author: David M. Alter, Texas Instruments Inc.
* History:
*   09/08/03 - original (based on DSP281x header files v1.00, D. Alter)
*   03/18/04 - added PLL lock delay loop (D. Alter)
*   12/07/07 - Changed SCSR init so that WD generates a reset instead of an interrupt.
*              Changed OTPWAIT value to 8 (from incorrect value of 5)
*              (D. Alter)
*********************************************************************
\endverbatim
*/

#include "DSP2833x_Device.h"

#include "amc.h"


/*!	Initializes the CPU.
*/
void InitSysCtrl(void)
{
volatile Uint16 i;					// General purpose Uint16
volatile Uint16 dummy;					// General purpose volatile int16

	asm(" EALLOW");						// Enable EALLOW protected register access

/*** Memory Protection Configuration ***/
	DevEmuRegs.PROTSTART = 0x0100;		// Write default value to protection start register
	DevEmuRegs.PROTRANGE = 0x00FF;		// Write default value to protection range register

/*** Unlock the Code Security Module if CSM not in use ***/
/* Unlocking the CSM will allow code running from non-secure memory
   to access code and data in secure memory.  One would only want to
   unsecure the CSM if code security were not desired, and therefore
   the CSM is not in use (otherwise, unlocking the CSM will compromise
   the security of user code).  If the CSM is not in use, the best
   thing to do is leave the password locations programmed to 0xFFFF,
   which is the flash ERASED state.  When all passwords are 0xFFFF,
   all that is required to unlock the CSM are dummy reads of the
   PWL locations.
*/
	dummy = CsmPwl.PSWD0;				// Dummy read of PWL locations
	dummy = CsmPwl.PSWD1;				// Dummy read of PWL locations
	dummy = CsmPwl.PSWD2;				// Dummy read of PWL locations
	dummy = CsmPwl.PSWD3;				// Dummy read of PWL locations
	dummy = CsmPwl.PSWD4;				// Dummy read of PWL locations
	dummy = CsmPwl.PSWD5;				// Dummy read of PWL locations
	dummy = CsmPwl.PSWD6;				// Dummy read of PWL locations
	dummy = CsmPwl.PSWD7;				// Dummy read of PWL locations

	
	if(SysCtrlRegs.WDCR & 0x0080)
		_WARNINGmessage(0x6010, 0x01, 0x0000, "WatchDog reset", 0, 0);
	
/*** Disable the Watchdog Timer. Now it is implemented in the FPGA ***/
	SysCtrlRegs.WDCR = 0x00E8; //Disable WatchDog
/*
 bit 15-8      0's:    reserved
 bit 7         1:      WDFLAG, write 1 to clear
 bit 6         0:      WDDIS, 0=enable WD
 bit 5-3       101:    WDCHK, WD check bits, always write as 101b
 bit 2-0       011:    WDPS, WD prescale bits, 011: WDCLK=OSCCLK/512/4 (~17.5ms)
*/

/* System and Control Register */
//	SysCtrlRegs.SCSR = 0x0001;  //Watchdog implemented in FPGA
/*
 bit 15-3      0's:    reserved
 bit 2         0:      WDINTS, WD interrupt status bit (read-only)
 bit 1         0:      WDENINT, 0=WD causes reset, 1=WD causes WDINT
 bit 0         1:      WDOVERRIDE, write 1 to disable disabling of the WD (clear-only)
*/

/*** Configure the PLL ***/
/* Note: The DSP/BIOS configuration tool can also be used to intialize the PLL
   instead of doing the initialization here.
*/
	// Make sure the PLL is not running in limp mode
	if (SysCtrlRegs.PLLSTS.bit.MCLKSTS != 1)
	{													// PLL is not running in limp mode
		SysCtrlRegs.PLLSTS.bit.MCLKOFF = 1;				// Turn off missing clock detect before changing PLLCR
		SysCtrlRegs.PLLSTS.bit.DIVSEL = 0;				// DIVSEL must be 0 or 1  (/4 CLKIN mode) before changing PLLCR
		SysCtrlRegs.PLLCR.bit.DIV = 0x000A;				// PLLx10/4 (because DIVSEL is /4)
   
		// Wait for PLL to lock.
		// During this time the CPU will run at OSCCLK/2 until the PLL is stable.
		// Once the PLL is stable the CPU will automatically switch to the new PLL value.
		// Code is not required to sit and wait for the PLL to lock.  However, 
		// if the code does anything that is timing critical (e.g. something that
		// relies on the CPU clock frequency to be at speed), then it is best to wait
		// until PLL lock is complete.  The watchdog should be disabled before this loop
		// (e.g., as was done above), or fed within the loop.
		while(SysCtrlRegs.PLLSTS.bit.PLLLOCKS != 1)		// Wait for PLLLOCKS bit to set
		{
			SysCtrlRegs.WDKEY = 0x0055;					// Service the watchdog while waiting
			SysCtrlRegs.WDKEY = 0x00AA;					//   in case the user enabled it.
		}

		// After the PLL has locked, we are running in PLLx10/4 mode (since DIVSEL is /4).
		// We can now enable the missing clock detect circuitry, and also change DIVSEL
		// to /2.  In this example, I will wait a bit of time to let inrush currents settle,
		// and then change DIVSEL from /4 to /2.  This is only an example.  The amount of
		// time you need to wait depends on the power supply feeding the DSP (i.e., how much
		// voltage droop occurs due to the inrush currents, and how long it takes the
		// voltage regulators to recover).
		SysCtrlRegs.PLLSTS.bit.MCLKOFF = 0;				// Enable missing clock detect circuitry
		DelayUs(20/2);									// Wait 20 us (just an example).  Remember we're running
														// at half-speed here, so divide function argument by 2.
		SysCtrlRegs.PLLSTS.bit.DIVSEL = 0x2;			// Change to /2 mode
	}
	else
	{													// PLL is running in limp mode
	// User should replace the below with a call to an appropriate function,
	// for example shutdown the system (since something is very wrong!).
		asm(" ESTOP0");
	}


/*** Configure the clocks ***/
	SysCtrlRegs.HISPCP.all = 0x0001;		// Hi-speed periph clock prescaler, HSPCLK=SYSCLKOUT/2
	SysCtrlRegs.LOSPCP.all = 0x0002;		// Lo-speed periph clock prescaler, LOSPCLK=SYSCLKOUT/4
	
	SysCtrlRegs.PCLKCR0.bit.ADCENCLK    = 1;   // ADC
	SysCtrlRegs.PCLKCR0.bit.I2CAENCLK   = 1;   // I2C
	SysCtrlRegs.PCLKCR0.bit.SCIAENCLK   = 1;   // SCI-A
	SysCtrlRegs.PCLKCR0.bit.SCIBENCLK   = 0;   // SCI-B
	SysCtrlRegs.PCLKCR0.bit.SCICENCLK   = 0;   // SCI-C
	SysCtrlRegs.PCLKCR0.bit.SPIAENCLK   = 0;   // SPI-A
	SysCtrlRegs.PCLKCR0.bit.MCBSPAENCLK = 0;   // McBSP-A
	SysCtrlRegs.PCLKCR0.bit.MCBSPBENCLK = 0;   // McBSP-B
	SysCtrlRegs.PCLKCR0.bit.ECANAENCLK  = 1;   // eCAN-A
	SysCtrlRegs.PCLKCR0.bit.ECANBENCLK  = 0;   // eCAN-B
	//SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC   = 0;   // Disable TBCLK within the ePWM
// The PCLKCR0.TBCLKSYNC bit is handled separately in InitEPwm() since
// it affects synchronization of the ePWM counters.
	
	SysCtrlRegs.PCLKCR1.bit.EPWM1ENCLK  = 1;   // ePWM1
	SysCtrlRegs.PCLKCR1.bit.EPWM2ENCLK  = 1;   // ePWM2
	SysCtrlRegs.PCLKCR1.bit.EPWM3ENCLK  = 1;   // ePWM3
	SysCtrlRegs.PCLKCR1.bit.EPWM4ENCLK  = 1;   // ePWM4
	SysCtrlRegs.PCLKCR1.bit.EPWM5ENCLK  = 0;   // ePWM5
	SysCtrlRegs.PCLKCR1.bit.EPWM6ENCLK  = 1;   // ePWM6
	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC   = 1;   // Enable TBCLK within the ePWM
	SysCtrlRegs.PCLKCR1.bit.ECAP1ENCLK = 1;  // eCAP1
	SysCtrlRegs.PCLKCR1.bit.ECAP2ENCLK = 1;  // eCAP2
	SysCtrlRegs.PCLKCR1.bit.ECAP3ENCLK = 1;  // eCAP3
	SysCtrlRegs.PCLKCR1.bit.ECAP4ENCLK = 1;  // eCAP4
	SysCtrlRegs.PCLKCR1.bit.ECAP5ENCLK = 1;  // eCAP5
	SysCtrlRegs.PCLKCR1.bit.ECAP6ENCLK = 1;  // eCAP6
	SysCtrlRegs.PCLKCR1.bit.EQEP1ENCLK = 1;  // eQEP1
	SysCtrlRegs.PCLKCR1.bit.EQEP2ENCLK = 1;  // eQEP2

	SysCtrlRegs.PCLKCR3.bit.CPUTIMER0ENCLK = 1; // CPU Timer 0
	SysCtrlRegs.PCLKCR3.bit.CPUTIMER1ENCLK = 1; // CPU Timer 1
	SysCtrlRegs.PCLKCR3.bit.CPUTIMER2ENCLK = 1; // CPU Timer 2
	SysCtrlRegs.PCLKCR3.bit.DMAENCLK       = 1; // DMA Clock
	SysCtrlRegs.PCLKCR3.bit.XINTFENCLK     = 1; // XTIMCLK
	SysCtrlRegs.PCLKCR3.bit.GPIOINENCLK    = 1; // GPIO input clock

/*** Configure the low-power modes ***/
	SysCtrlRegs.LPMCR0.all     = 0x00FC;		// LPMCR0 set to default value
	GpioIntRegs.GPIOLPMSEL.all = 0x0000;		// GPIOLPMSEL set to default value
	//Note: GPIOLPMSEL replaces LPMCR1 on TMS320x2812. No effect of halt and stanby for GPIO0-31.

/*** Finish up ***/
	asm(" EDIS");						// Disable EALLOW protected register access

} //end InitSysCtrl()


/*!	Initializes the F281x flash timing registers.

Notes:
- This function MUST be executed out of RAM.  Executing it out of OTP/FLASH will produce unpredictable results.
- he flash registers are code security module protected.  Therefore, you must either run this function from L0/L1 RAM, or you must 
first unlock the CSM.  Note that unlocking the CSM as part of the program flow can compromise the code security.
*/
#pragma CODE_SECTION(InitFlash, "secureRamFuncs")
void InitFlash(void)
{
	asm(" EALLOW");									// Enable EALLOW protected register access
	FlashRegs.FPWR.bit.PWR = 3;						// Pump and bank set to active mode
	FlashRegs.FSTATUS.bit.V3STAT = 1;				// Clear the 3VSTAT bit
	FlashRegs.FSTDBYWAIT.bit.STDBYWAIT = 0x01FF;	// Sleep to standby transition cycles
	FlashRegs.FACTIVEWAIT.bit.ACTIVEWAIT = 0x01FF;	// Standby to active transition cycles
	FlashRegs.FBANKWAIT.bit.RANDWAIT = 5;			// Random access waitstates
	FlashRegs.FBANKWAIT.bit.PAGEWAIT = 5;			// Paged access waitstates
	FlashRegs.FOTPWAIT.bit.OTPWAIT = 8;				// OTP waitstates
	FlashRegs.FOPT.bit.ENPIPE = 1;					// Enable the flash pipeline
	asm(" EDIS");									// Disable EALLOW protected register access

/*** Force a complete pipeline flush to ensure that the write to the last register
     configured occurs before returning.  Safest thing is to wait 8 full cycles. ***/

    asm(" RPT #6 || NOP");

} //end of InitFlash()

/*** end of file *****************************************************/

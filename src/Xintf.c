/*!	\file Xintf.c
	\brief Initialization of  the external memory interface on the F2812.

\verbatim
*********************************************************************
* File: Xintf.c
* Devices: TMS320F28XXX
* Author: David M. Alter, Texas Instruments Inc.
* Description: This function initializes the External Memory Interface.
*   Do not modify the timings of a zone while accessing that zone.
*   On F2811 and F2810 devices, there is no XINTF peripheral other than
*   the XCLKOUT pin.  This function can still be used to configure
*   XCLKOUT.
* History:
*   09/08/03 - original (based on DSP28 header files v1.00, D. Alter)
*********************************************************************
\endverbatim
*/

#include "DSP2833x_Device.h"

#include "amc.h"


#pragma CODE_SECTION(InitXintf, "initXintf_function")
/*!	Initializes the external memory interface on the F2812.
*/
void InitXintf(void)
{
	asm(" EALLOW");								// Enable EALLOW protected register access

// Make sure write buffer is empty before configuring buffering depth
	while(XintfRegs.XINTCNF2.bit.WLEVEL != 0);	// poll the WLEVEL bit
	XintfRegs.XINTCNF2.bit.WRBUFF = 0;			// No write buffering

//--- XINTCNF2 Register
	XintfRegs.XINTCNF2.bit.XTIMCLK = 1;			// XTIMCLK=SYSCLKOUT/1
	XintfRegs.XINTCNF2.bit.CLKOFF  = 1;			// XCLKOUT is disabled
	XintfRegs.XINTCNF2.bit.CLKMODE = 1;			// XCLKOUT = XTIMCLK/2

//--- XBANK Register
// Example: Assume Zone 7 is slow, so add additional BCYC cycles whenever
// switching from Zone 7 to another Zone.  This will help avoid bus contention.
	XintfRegs.XBANK.bit.BCYC = 7;				// Add 7 cycles
	XintfRegs.XBANK.bit.BANK = 7;				// select zone 7

//--- Zone 0 Configuration
	XintfRegs.XTIMING0.bit.X2TIMING = 1;	// Timing scale factor = 1
	XintfRegs.XTIMING0.bit.XSIZE = 3;		// 3 means 16-bit interface
	XintfRegs.XTIMING0.bit.READYMODE = 1; 	// XREADY is asynchronous
	XintfRegs.XTIMING0.bit.USEREADY = 0;	// Disable XREADY
	XintfRegs.XTIMING0.bit.XRDLEAD = 1;		// Read lead time
	//XintfRegs.XTIMING0.bit.XRDACTIVE = 3;	// Read active time
	//XintfRegs.XTIMING0.bit.XRDTRAIL = 1;	// Read trail time
	XintfRegs.XTIMING0.bit.XRDACTIVE = 7;	// Read active time
	XintfRegs.XTIMING0.bit.XRDTRAIL = 2;	// Read trail time
	XintfRegs.XTIMING0.bit.XWRLEAD = 1;		// Write lead time
	//XintfRegs.XTIMING0.bit.XWRACTIVE = 3;	// Write active time
	//XintfRegs.XTIMING0.bit.XWRTRAIL = 1;	// Write trail time
	XintfRegs.XTIMING0.bit.XWRACTIVE = 7;	// Write active time
	XintfRegs.XTIMING0.bit.XWRTRAIL = 2;	// Write trail time

//--- Zone 6 Configuration
	XintfRegs.XTIMING6.bit.X2TIMING = 0;	// Timing scale factor = 1
	XintfRegs.XTIMING6.bit.XSIZE = 3;		// 3 means 16-bit interface
	XintfRegs.XTIMING6.bit.READYMODE = 1; 	// XREADY is asynchronous
	XintfRegs.XTIMING6.bit.USEREADY = 0;	// Disable XREADY
	XintfRegs.XTIMING6.bit.XRDLEAD = 1;		// Read lead time
	XintfRegs.XTIMING6.bit.XRDACTIVE = 2;	// Read active time
	XintfRegs.XTIMING6.bit.XRDTRAIL = 0;	// Read trail time
	XintfRegs.XTIMING6.bit.XWRLEAD = 1;		// Write lead time
	XintfRegs.XTIMING6.bit.XWRACTIVE = 2;	// Write active time
	XintfRegs.XTIMING6.bit.XWRTRAIL = 0;	// Write trail time

//--- Zone 7 Configuration
	XintfRegs.XTIMING7.bit.X2TIMING = 0;	// Timing scale factor = 1
	XintfRegs.XTIMING7.bit.XSIZE = 3;		// 3 means 16-bit interface
	XintfRegs.XTIMING7.bit.READYMODE = 1; 	// XREADY is asynchronous
	XintfRegs.XTIMING7.bit.USEREADY = 0;	// Disable XREADY
	XintfRegs.XTIMING7.bit.XRDLEAD = 1;		// Read lead time
	XintfRegs.XTIMING7.bit.XRDACTIVE = 2;	// Read active time
	XintfRegs.XTIMING7.bit.XRDTRAIL = 0;	// Read trail time
	XintfRegs.XTIMING7.bit.XWRLEAD = 1;		// Write lead time
	XintfRegs.XTIMING7.bit.XWRACTIVE = 2;	// Write active time
	XintfRegs.XTIMING7.bit.XWRTRAIL = 0;	// Write trail time

//--- Force a complete pipeline flush to ensure that the write to the last register
//    configured occurs before returning.  Safest thing to do is wait 8 full cycles.
	asm(" RPT #7 || NOP");

//--- Finish up
	asm(" EDIS");								// Disable EALLOW protected register access

} // end of InitXintf()


#pragma CODE_SECTION(fast_init_Xintf, "initXintf_function")
/*!	Initializes the external memory interface before _c_int00 so that ebss
	sections can be initialized
*/
void fast_init_Xintf(void)
{
	asm(" EALLOW");
	SysCtrlRegs.PCLKCR3.bit.XINTFENCLK = 1;   /* enable clock to Xintf */
	GpioCtrlRegs.GPAMUX2.all = 0xC00000F0;    /* configure Xintf pins */
	GpioCtrlRegs.GPBMUX1.all = 0xFFFFFF00;
	GpioCtrlRegs.GPCMUX1.all = 0xFFFFFFFF;
	GpioCtrlRegs.GPCMUX2.all = 0x0000FFFF;
	GpioCtrlRegs.GPAPUD.all  = 0xC0000000;    /* disable pull-ups for Xintf pins */
	GpioCtrlRegs.GPBPUD.all  = 0xFFFFFFF0;
	GpioCtrlRegs.GPCPUD.all  = 0x00FFFFFF;
	XintfRegs.XTIMING7.all   = 0x00039428;    /* configure Xintf zone 7 timing */
	asm(" EDIS");
}

/*** end of file *****************************************************/

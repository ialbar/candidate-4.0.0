/*!
 \file   Gpio.c
 \brief  Initialization of General Purpose Input/Output pins
 \author Manuel Godoy
 \note   Devices: TMS320F2833x
 */
/******************************************************************************/

#include "DSP2833x_Device.h"
#include "amc.h"


/*!
 \brief Initializes the shared GPIO pins on the F281x.
 GPIO0-5:   ePwm outputs to control the motor.
 GPIO6:     ePwm output to resolver excitement signal.
 GPIO7:     Configuration dependent. Set GPIO input (PWMin1).
 GPIO8:     Configuration dependent. Set GPIO output (gpio O1).
 GPIO9:     Configuration dependent. Set GPIO input (gpio I1).
 GPIO10:    ePwm output for brake1 signal.
 GPIO11:    ePwm output for brake2 signal.
 GPIO12:    Trip Zone 1.
 GPIO13:    GPIO output (O2).
 GPIO14:    GPIO output (O3).
 GPIO15:    GPIO output (O4).
 GPIO16:    GPIO input (gpio I2).
 GPIO17:    GPIO output (gpio O2).
 GPIO18:    CANRXA.
 GPIO19:    CANTXA.
 GPIO20:    Encoder 1 input A.
 GPIO21:    Encoder 1 input B.
 GPIO22:    Configuration dependent. Set GPIO input (I1).
 GPIO23:    eQEP1I.
 GPIO24-26: eQEP configuration.
 GPIO27:    Configuration dependent. Set GPIO input (I2)
 GPIO28-29: Serial R,T
 GPIO30:    Set GPIO input (I4)
 GPIO31:    XA17.

 GPIO32-33: I2C DATA and CLK.
 GPIO34:    Configuration dependent. Set GPIO output (O1).
 GPIO35:    Configuration dependent. Set GPIO input (I3).
 GPIO36:    XZCS0 -> Chip Select 0 to access FPGA
 GPIO37-47: XA.


 GPIO48:    Configuration dependent. Set GPIO input (PWMIN 2).
 GPIO49:    Configuration dependent. Set GPIO input (PWMIN 3).
 GPIO50-52: GPIO inputs for XINT (Hall sensors).
 GPIO53:    GPIO input for XINT (Safety).
 GPIO54-56: Data(I), LD(O) and clock(O) for switches reading.
 GPIO57-62: LEDs outputs.
 GPIO63:    Not asigned.
 GPIO64-79: XD.

 GPIO80-87: XA.

 Sampling period of 4*Tsysclkout for inputs.
 Qualification using 3 samples => 3samples*4*6.67ns = 80ns
 - Special cases:
 GPIO12 configured as TripZone1 select asynchronous (GPAQSEL1).
*/
void InitGpio(void)
{
   EALLOW;

   // Each GPIO pin can be:
   // a) a GPIO input/output
   // b) peripheral function 1
   // c) peripheral function 2
   // d) peripheral function 3
   // By default, all are GPIO Inputs
   /* Function Mux registers */
   GpioCtrlRegs.GPAMUX1.all = 0x01501555;     // GPIO functionality GPIO0-GPIO15
   GpioCtrlRegs.GPAMUX2.all = 0x852A45F0;     // GPIO functionality GPIO16-GPIO31
   GpioCtrlRegs.GPBMUX1.all = 0xFFFFFF05;     // GPIO functionality GPIO32-GPIO47
   GpioCtrlRegs.GPBMUX2.all = 0x00000000;     // GPIO functionality GPIO48-GPIO63
   GpioCtrlRegs.GPCMUX1.all = 0xFFFFFFFF;     // GPIO functionality GPIO64-GPIO79
   GpioCtrlRegs.GPCMUX2.all = 0xFFFFFFFF;     // GPIO functionality GPIO80-GPIO95

   /* Direction registers: 0 = input. 1 = output */
   GpioCtrlRegs.GPADIR.all = 0xA00AED7F;
   GpioCtrlRegs.GPBDIR.all = 0x7EC0FFF7;
   GpioCtrlRegs.GPCDIR.all = 0xFFFFFFFF;

   /* Control registers (sampling period = 2*T) */
   GpioCtrlRegs.GPACTRL.all = 0x01010101;
   GpioCtrlRegs.GPBCTRL.all = 0x01010101;

   /* Qualification registers (3 samples) */
   GpioCtrlRegs.GPAQSEL1.all = 0x55555555;
   GpioCtrlRegs.GPAQSEL2.all = 0x55555555;
   GpioCtrlRegs.GPBQSEL1.all = 0x55555555;
   GpioCtrlRegs.GPBQSEL2.all = 0x55555555;

   // Pull-ups can be enabled or disabled.
   GpioCtrlRegs.GPAPUD.all = 0xFFFFFFFF;      // Pullup's disabled for most GPIO0-GPIO31
   GpioCtrlRegs.GPBPUD.all = 0x7FFFFFFF;      // Pullup's disabled for most GPIO32-GPIO63 (enabled for GPIO64, unused)
   GpioCtrlRegs.GPCPUD.all = 0x00FFFFFF;      // Pullup's disabled for GPIO64-GPIO87

   //GpioCtrlRegs.GPAPUD.all = 0xFFFF;    // Pullup's disabled GPIO0-GPIO31
   //GpioCtrlRegs.GPBPUD.all = 0xFFFF;    // Pullup's disabled GPIO32-GPIO34
   //GpioCtrlRegs.GPCPUD.all = 0xFFFF     // Pullup's disabled GPIO64-GPIO79

    EDIS;

	 /* initialize output pin values */
	 //SET_ID_SWITCH_SH_LDn;
	 //CLEAR_ID_SWITCH_CLOCK;
	 //CLEAR_BRAKE2_INHIBIT_SAFETY;

} //end InitGpio()


/*** end of file *****************************************************/

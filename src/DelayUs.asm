**********************************************************************
* File: DelayUs.asm                                                  *
* DSP: TMS320F2812, TMS320F2811, TMS320F2810                         *
* Author: David M. Alter, Texas Instruments Inc.                     *
* History:                                                           *
*   11/13/05 - added save/restore of ST1 to preserve EALLOW setting  *
*              (D. Alter)                                            *
*   05/02/05 - added watchdog servicing in outer loop (D. Alter)     *
*   02/11/05 - original (D. Alter)                                   *
*   05/02/05 - added watchdog servicing in outer loop (D. Alter)     *
*   09/01/04 - fixed counter bug by zeroing AH (D. Alter)            *
*   09/08/03 - original (D. Alter)                                   *
**********************************************************************

WDKEY	.set	0x7025

**********************************************************************
* Function: DelayUs()                                                *
* Description: Implements a time delay.                              *
* DSP: TMS320F2812, TMS320F2811, TMS320F2810                         *
* Last Modified: 05/02/05                                            *
* Include files: none                                                *
* Function Prototype: void DelayUs(unsigned int)                     *
* Useage: DelayUs(Usec);                                             *
* Input Parameters: unsigned int Usec = time delay in microseconds   *
* Return Value: none                                                 *
* Notes:                                                             *
*   1) The execution time of this routine is based upon a 150 MHz    *
*      CPUCLK.  It also assumes that the function executes out of    *
*      internal RAM.  If executing out of internal flash, the        *
*      execution speed will be slightly slower.  However, the inner  *
*      loop of this function is essentially invariant to the memory  *
*      it is running in.  Therefore, even when running in flash      *
*      memory, the basic loop time will be only slightly longer than *
*      1 us.                                                         *
*                                                                    *
*   2) The outer loop of this function is interruptible (i.e., every *
*      1 us).  The user should disable interrupts before calling the *
*      function if they need an exact delay time, as opposed to a    *
*      minimum delay time.                                           *
*                                                                    *
**********************************************************************
       .def _DelayUs
       .text

_DelayUs:
        MOVB AH, #0                   ;Zero AH
        PUSH ST1                      ;Save ST1 to preserve EALLOW setting

DelayUs1:                             ;Outer loop

;Service the watchdog in case it is active
        EALLOW
        MOVZ DP, #(WDKEY>>6)
        MOV @WDKEY, #0x0055
        MOV @WDKEY, #0x00AA
        EDIS

;Proceed with the inner loop
        RPT #138                      ;Inner loop
     || NOP

        SUBB ACC,#1                   ;Decrement outer loop counter
        BF DelayUs1, GT               ;Branch for outer loop

;Finish up
        POP ST1                       ;Restore ST1
        LRETR                         ;Return

;end of function DelayUs() ******************************************


;end of file DelayUs.asm
       .end

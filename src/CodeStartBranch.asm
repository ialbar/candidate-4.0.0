***********************************************************************
* File: CodeStartBranch.asm
* Devices: TMS320F28235
* Author: David M. Alter, Texas Instruments Inc.
* History:
*   09/08/03 - original (D. Alter)
***********************************************************************

WD_DISABLE	.set	0		;set to 1 to disable WD, else set to 0

    .ref _c_int00
	 .ref _fast_init_Xintf
	 .ref _ebss_loadstart
	 .ref _ebss_loadend
	 .ref _ipdata_loadstart
	 .ref _ipdata_loadend
    .def code_start

***********************************************************************
* Function: codestart section
*
* Description: Branch to code starting point
***********************************************************************
    .sect "codestart"
code_start:
    LB start_routine       ;Branch to start routine
;end codestart section
***********************************************************************


***********************************************************************
* Function: start_routine
*
* Description: clear .ebss, init Xintf, enable/diable WDT and start
***********************************************************************
    .text
start_routine:
    LCR wd_disable       ;Branch to watchdog disable code
    LCR _fast_init_Xintf
    LCR clear_data_before_init
	 .if WD_DISABLE == 0
    LCR wd_enable        ;re-enable the watchdog timer
    .endif
    LB _c_int00          ;Branch to start of boot.asm in RTS library
;end start_routine
***********************************************************************


***********************************************************************
* Function: wd_disable
*
* Description: Disables the watchdog timer
***********************************************************************
    .text
wd_disable:
    EALLOW					;Enable EALLOW protected register access
    MOVZ DP, #7029h>>6      ;Set data page for WDCR register
    MOV @7029h, #0068h      ;Set WDDIS bit in WDCR to disable WD
    EDIS					;Disable EALLOW protected register accessy
    LRETR
;end wd_disable
***********************************************************************


***********************************************************************
* Function: wd_enable
*
* Description: Enable Xintf and branch to start of boot.asm in RTS library
***********************************************************************
    .text
wd_enable:
    EALLOW					;Enable EALLOW protected register access
    MOVZ DP, #7029h>>6      ;Set data page for WDCR register
    MOV @7029h, #0028h      ;Clear WDDIS bit in WDCR to enable WD
    EDIS					;Disable EALLOW protected register access
    LRETR
;end wd_enable
***********************************************************************


***********************************************************************
* Function: clear_data_before_init
*
* Description: Clear .ebss memory and ipdata before .cinit is applied
***********************************************************************
    .text
clear_data_before_init:
        MOVL      XAR5,#_ebss_loadstart
        MOVL      XAR4,#_ebss_loadend
        MOVL      ACC,XAR4
        CMPL      ACC,XAR5
        B         clrmem2,LOS
clrmem1:    
        MOV       *XAR5++,#0
        MOVL      ACC,XAR4
        CMPL      ACC,XAR5
        B         clrmem1,HI
clrmem2:    
        MOVL      XAR5,#_ipdata_loadstart
        MOVL      XAR4,#_ipdata_loadend
        MOVL      ACC,XAR4
        CMPL      ACC,XAR5
        B         clrmem4,LOS
clrmem3:    
        MOVB      ACC,#0
        MOVL      *XAR5++,ACC
        MOVL      ACC,XAR4
        CMPL      ACC,XAR5
        B         clrmem3,HI
clrmem4:    
        LRETR
;end clear_data_before_init
***********************************************************************

***********************************************************************

	.end                    ; end of file CodeStartBranch.asm
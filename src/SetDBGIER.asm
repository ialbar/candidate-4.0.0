***********************************************************************
* File: SetDBGIER.asm
* Devices: TMS320F2812, TMS320F2811, TMS320F2810
* Author: David M. Alter, Texas Instruments Inc.
* History:
*   09/08/03 - original (D. Alter)
* Notes: none
***********************************************************************


**********************************************************************
* Function: SetDBGIER()                                              *
* Description: Sets the DBGIER register (for realtime emulation).    *
* DSP: TMS320F2812, TMS320F2811, TMS320F2810                         *
* Last Modified: 08/12/03                                            *
* Include files: none                                                *
* Function Prototype: void SetDBGIER(unsigned int)                   *
* Useage: SetDBGIER(value);                                          *
* Input Parameters: Uint16 value = value to put in DBGIER register.  *
* Return Value: none                                                 *
* Notes: none                                                        *
**********************************************************************
		.def _SetDBGIER
		.text
		
_SetDBGIER:
		MOV 	*SP++,AL
		POP 	DBGIER
		LRETR

; end of function SetDBGIER() ****************************************


;end of file SetDBGIER.asm
       .end

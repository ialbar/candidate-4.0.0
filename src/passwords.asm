***********************************************************************
* File: passwords.asm
* Devices: TMS320F2812, TMS320F2811, TMS320F2810
* Author: David M. Alter, Texas Instruments Inc.
* History:
*   09/08/03 - original (D. Alter)
* Notes:
*  1) The section "passwords" contains the actual CSM passwords that get
*     linked to the CSM password locations in flash.  The user must know
*     what these passwords are in order to unlock the CSM.
*  2) Link the section "passwords" to the memory PASSWORDS on page 0.
*  3) It is recommended that all passwords be left as 0xFFFF during code
*     development.  Passwords of 0xFFFF are dummy passwords, and do not
*     lock the code security module (Dummy reads of CSM PWL registers
*     will unlock the CSM).  When code development is complete, the user
*     can modify the passwords to activate the code security module.
*  4) The section "csm_rsvd" is required when using code security.
*     Failure to program addresses 0x3F7F80 through 0x3F7FFF5 in the
*     flash to all 0x0000 can compromise security.  This is documented
*     in the F2810/12 datasheet, SPRS174.
*  5) Link the section "csm_rsvd" to the memory CSM_RSVD on page 0.
***********************************************************************

	.sect "passwords"

	.int	0xFFFF		;PWL0 (LSW of 128-bit password)
	.int	0xFFFF		;PWL1
	.int	0xFFFF		;PWL2
	.int	0xFFFF		;PWL3
	.int	0xFFFF		;PWL4
	.int	0xFFFF		;PWL5
	.int	0xFFFF		;PWL6
	.int	0xFFFF		;PWL7 (MSW of 128-bit password)
	
;----------------------------------------------------------------------

	.sect "csm_rsvd"
	.loop (3F7FF5h - 3F7F80h + 1)
		.int 0x0000
	.endloop

;----------------------------------------------------------------------

	.end

; end of file passwords.asm

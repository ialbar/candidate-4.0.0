/*!	\file tag.c
	\brief Functions that copy version information to the OD
*/

#include "subwcrev.h"
#include "tag.h"
#include "amc_od.h"
#include "fpga.h"
#include "stdlib.h"

void publishSoftwareVersion(void)
{
	volatile unsigned *pointer;

	/* copy firmware version */
	fw_version = PACKAGE_VERSION;
	fw_review  = PACKAGE_REVIEW;
	fw_subrevision = PACKAGE_SUBREVISION;
	fw_tag   = PACKAGE_TAG;

	/* copy svn version */
	svn_revision = atoi(_SVN_REVISION);
	svn_mixed = atoi(_SVN_MIXED);
	svn_mods = atoi(_SVN_MODS);

	/* copy HDL version */
	pointer = (unsigned *)FPGA_ADD_HDL_VER;
	HDL_version = (UNS16) *pointer;

	/* copy DNA identifier */
	pointer = (unsigned *)FPGA_ADD_DNA0;
	DNA_id0 = (UNS16) *pointer;
	pointer = (unsigned *)FPGA_ADD_DNA1;
	DNA_id1 = (UNS16) *pointer;
	pointer = (unsigned *)FPGA_ADD_DNA2;
    DNA_id2 = (UNS16) *pointer;
    pointer = (unsigned *)FPGA_ADD_DNA3;
	DNA_id3 = (UNS16) *pointer;

	pointer = (unsigned *)FPGA_ADD_BOARD_ID0;
	Board_id0 = (UNS16) *pointer;
	pointer = (unsigned *)FPGA_ADD_BOARD_ID1;
	Board_id1 = (UNS16) *pointer;
	pointer = (unsigned *)FPGA_ADD_BOARD_ID2;
	Board_id2 = (UNS16) *pointer;

	/* copy HDL Multiboot version */
	pointer = (unsigned *)FPGA_ADD_HDL_MBOOT_VER;
	HDL_MBOOT_version = (UNS16) *pointer;

	/* copy CanOpen Manufacturer specific protocol version */
	COMSP_version = MANUFACTURER_VERSION;
	COMSP_review = MANUFACTURER_REVIEW;
	COMSP_subrevision = MANUFACTURER_SUBREVISION;

	/* copy DSP Bootloader version */
	pointer = (unsigned *)FPGA_ADD_DSP_BOOT_VERSION;
	Dsp_boot_version = (UNS16) (*pointer);
	pointer = (unsigned *)FPGA_ADD_DSP_BOOT_REVIEW;
	Dsp_boot_review = (UNS16) (*pointer);
	pointer = (unsigned *)FPGA_ADD_DSP_BOOT_SUBREVISION;
	Dsp_boot_subrevision = (UNS16) (*pointer);
	pointer = (unsigned *)FPGA_ADD_DSP_BOOT_TAG;
	Dsp_boot_tag = (UNS16) (*pointer);
}

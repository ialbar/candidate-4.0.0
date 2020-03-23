/*!	\file can.c
	\brief Functions that interact directly with eCAN module of the TMS320F2812

\verbatim
*********************************************************************
* File: can.c
* Devices: TMS320F28XXX
* Author: Luis Jimenez.
* History:
* 15/11/06 - original
*********************************************************************
\endverbatim
*/

#include "DSP2833x_Device.h"
#include "amc.h"

can_bus_state_t can_bus_state;	/*!< State of the CAN bus */

int calculateBRPREG(int);
int j = 0;

/*!	Initializes the ADC on the F281x
	\param baudrate Baudrate to configure
*/
void InitCan(int baudrate)
{
	volatile struct MBOX *p_mbox;
	volatile union CANLAM_REG *p_lam_reg;
	union CANBTC_REG ECanaShadow_CANBTC;
	int i;
	unsigned long aux32;

	asm("  EALLOW");
/* Configure eCAN RX and TX pins for eCAN transmissions using eCAN regs*/
	ECanaRegs.CANTIOC.bit.TXFUNC = 1;
	ECanaRegs.CANRIOC.bit.RXFUNC = 1;

/* Configure eCAN for HECC mode - (reqd to access mailboxes 16 thru 31) */
	ECanaRegs.CANMC.bit.SCB = 1;		/* HECC mode also enables time-stamping feature */

/* Activate Auto Bus On (to recover after a "buss-off" state) */
	ECanaRegs.CANMC.bit.ABO = 1;

	j = readIDswitch() & 0x80;
	_LOGmessage(j,"nodeID", 0, 0);

/* Configure eCAN for Self-Test mode if switch 8 of IDswitch = 1 */
	if (readIDswitch() & 0x80) ECanaRegs.CANMC.bit.STM = 1;		/* self-test mode */

/* Configure bit timing parameters */
	ECanaRegs.CANMC.bit.CCR = 1 ;				/* Set CCR = 1 */
	while(ECanaRegs.CANES.bit.CCE != 1 ) {}	/* Wait for CCE bit to be set.. */

	ECanaShadow_CANBTC.all = 0;
	ECanaShadow_CANBTC.bit.BRPREG = calculateBRPREG(baudrate);
	ECanaShadow_CANBTC.bit.TSEG2REG = 3;
	ECanaShadow_CANBTC.bit.TSEG1REG = 9;	/* sample point = 73% */
	ECanaShadow_CANBTC.bit.SJWREG = 1;		/* sjw=2,  2 TQ allowed for a bit to be shortened or lengthened when resynchronizing */
	ECanaRegs.CANBTC.all = ECanaShadow_CANBTC.all;

	ECanaRegs.CANMC.bit.CCR = 0;					/* Set CCR = 0 */
	while(ECanaRegs.CANES.bit.CCE == !0 ) {}	/* Wait for CCE bit to be cleared.. */

	can_bus_state = ERROR_ACTIVE;

/* Disable all Mailboxes */
 	ECanaRegs.CANME.all = 0;		/* Required before writing the MSGIDs */
	asm("  EDIS");

	/* Specific initilizacion for this program */

	/* Write to the MSGID field */
	for(i=0;i<32;i++)
	{
		p_mbox = &(ECanaMboxes.MBOX0) + i;
		(*p_mbox).MSGID.bit.IDE = 0;			/* standard identifier (for all rx mboxes) */
		(*p_mbox).MSGID.bit.AME = 1;			/* local acceptance mask used (for all rx mboxes) */
		(*p_mbox).MSGID.bit.AAM = 0;			/* no auto answer mode */

		if(i >= 16)			/* tx mailboxes	 */
			(*p_mbox).MSGID.bit.STDMSGID = 0;	/* standard_identifier=0 and msgid=0 (changed before every sent message) */
		else if(i > 7)		/* mailboxes 8-15 are for nodeID-specific messges */
			(*p_mbox).MSGID.bit.STDMSGID = nodeID;	/* messageID=nodeID (all node specific messages:Emergency, PDO, SDO and NMT-monitoring messages) */
		else if(i == 0)    /* mailbox 0 is for heartbeats from master node */
			(*p_mbox).MSGID.bit.STDMSGID = 0x700 + MASTER_NODEID;
		else 			/* messages with ID = 0x0 (NMT), 0x80 (sync) and 0x100 (timestamp) */
			(*p_mbox).MSGID.bit.STDMSGID = 0x0;
	}

	/* Configure Mailboxes 0-15 as rx and 16-31 as tx */
	ECanaRegs.CANMD.all = 0x0000FFFF; 		/* this register is only accessible by 32-bits accesses? */

	/* Enable Mailboxes (16 for reception and CAN_TX_MAILBOXES for transmission) */
	aux32 = 0x0000FFFF;
	for(i=0;i<CAN_TX_MAILBOXES;i++) aux32 = (aux32 << 1) + 1;
	ECanaRegs.CANME.all = aux32; 		/* this register is only accessible by 32-bits accesses? */

	/* Define Local Acceptance Masks for rx mailboxes */
	for(i=0;i<16;i++)
	{
		p_lam_reg = &ECanaLAMRegs.LAM0 + i;		/* pointer to every Local Acceptance Mask */

		(*p_lam_reg).bit.LAMI = 0;				/* only standard identifiers (as MBOX own identifier) */
		(*p_lam_reg).bit.LAM_L = 0x0000;
		if(i == 0)           /* mailbox 0 */
			(*p_lam_reg).bit.LAM_H = 0x0000;	/* only heartbeat messages from master node */
		else if(i < 8)			/* mailboxes 1-7 */
			(*p_lam_reg).bit.LAM_H = 0x0600;	/* bits 7 and 8 from 11-bit identifier are ignored */
												/* valid for ID = 0x000 (NMT), 0x080 (sync) and 0x100 (timestamp) */
		else 				/* mailboxes 8-15 */
			(*p_lam_reg).bit.LAM_H = 0x1E00;	/* only messages for this node */
	}

	/*! rx mailboxes 1-7 and 9-15 are protected against overwrite. When a message arrives it goes to the highest mailbox with right LAM, if it is already full and protected, it goes to the next with right LAM.
	Mailboxes 0 and 8 are unprotected so that, if they have a message, it is overwritten and a RML interrupt is generated */
	ECanaRegs.CANOPC.all = 0x0000FEFE;

	/* mailboxes interrupts generetad on HEC_INT_REQ[1] line */
	ECanaRegs.CANMIL.all = 0xFFFFFFFF;

	EALLOW;
	/* rx mailboxes interrupt enable */
	ECanaRegs.CANMIM.all = 0x0000FFFF;		/* this register is only accessible by 32-bits accesses? */

	/* enable important interruts, and enable both interrupt lines */
	ECanaRegs.CANGIM.all = 0x00022F03;		/* this register is only accessible by 32-bits accesses? */
	EDIS;

	PieCtrlRegs.PIEIER9.bit.INTx5 = 1;		/* Enable ECAN0INT in the PIE: Group 9 interrupt 5 */
	PieCtrlRegs.PIEIER9.bit.INTx6 = 1;		/* Enable ECAN1INT in the PIE: Group 9 interrupt 6 */
	IER |= 0x100;							/* Enable CPU INT9 */
}


/*!	Function that returns BRPREG value to configure eCAN for a given \a baudrate
	\param baudrate Baudrate value to configure

\verbatim
Bit configuration parameters for 150 MHz SYSCLKOUT (75 MHz CAN clock)

The table below shows how BRP field must be changed to achieve different bit rates with a BT of 15, for a 80% SP:
---------------------------------------------------
BT = 15, TSEG1 = 10, TSEG2 = 2, Sampling Point = 80%
---------------------------------------------------
1 Mbps   : BRP+1 = 5    : CAN clock = 15 MHz
500 kbps : BRP+1 = 10   : CAN clock = 7.5 MHz
250 kbps : BRP+1 = 20   : CAN clock = 3.75 MHz
125 kbps : BRP+1 = 40   : CAN clock = 1.875 MHz
100 kbps : BRP+1 = 50   : CAN clock = 1.5 MHz
50 kbps  : BRP+1 = 100  : CAN clock = 0.75 MHz

The table below shows how to achieve different sampling points with a BT of 15:
-------------------------------------------------------------
Achieving desired SP by changing TSEG1 & TSEG2 with BT = 15
-------------------------------------------------------------
TSEG1 = 10, TSEG2 = 2, SP = 80%
TSEG1 = 9,  TSEG2 = 3, SP = 73.3%
TSEG1 = 8,  TSEG2 = 4, SP = 66.6%

The table below shows how to achieve different sampling points with a BT of 25:
-------------------------------------------------------------
Achieving desired SP by changing TSEG1 & TSEG2 with BT = 25
-------------------------------------------------------------
TSEG1 = 18, TSEG2 = 4, SP = 80%
TSEG1 = 17, TSEG2 = 5, SP = 76%
TSEG1 = 16, TSEG2 = 6, SP = 72%
TSEG1 = 15, TSEG2 = 7, SP = 68%
TSEG1 = 14, TSEG2 = 8, SP = 64%

The table below shows how BRP field must be changed to achieve different bit
rates with a BT of 25, for the sampling points shown above:

1 Mbps   : BRP+1 = 3
500 kbps : BRP+1 = 6
250 kbps : BRP+1 = 12
125 kbps : BRP+1 = 24
100 kbps : BRP+1 = 30
50 kbps  : BRP+1 = 60
\endverbatim
*/
int calculateBRPREG(int baudrate)
{
	switch(baudrate)
	{
		case 1000:
			return(4);
		case 500:
			return(9);
		case 250:
			return(19);
		case 125:
			return(39);
		case 100:
			return(49);
		case 50:
			return(99);
		case 20:
			return(249);
		case 10:
			return(499);
		default:
			_LOGmessage(0x0001,"baudrate not supported, 1M used", 0, 0)
			return(4);
	}
}


/*!	called from ECAN1 HWI to process serial received data and to acknowledge finished-transmission interrupts
*/
void can_rxtx(void)
{
	union CANGIF1_REG ECanaShadow_CANGIF1;
	union CANTOS_REG ECanaShadow_CANTOS;
	union CANTOC_REG ECanaShadow_CANTOC;
	volatile struct MBOX *p_mbox;
	unsigned long tmp_u32;
	Message m;

	ECanaShadow_CANGIF1.all = ECanaRegs.CANGIF1.all;
	if(ECanaShadow_CANGIF1.bit.GMIF1 == 1)
	{
		if(ECanaShadow_CANGIF1.bit.MIV1 < 16) 		/* rx mailboxes */
		{
			p_mbox = &(ECanaMboxes.MBOX0) + ECanaShadow_CANGIF1.bit.MIV1;

			DINT;	/* workaround to avoid eCAN register access error (see silicon errata) */
			tmp_u32 = (*p_mbox).MSGID.all;
			if ( tmp_u32 == 0) tmp_u32 = (*p_mbox).MSGID.all;
			EINT;
			m.cob_id = ((tmp_u32 >> 18) & 0x07FF);		/* copy message ID */

			DINT;	/* workaround to avoid eCAN register access error (see silicon errata) */
			tmp_u32 = (*p_mbox).MSGCTRL.all;
			if ( tmp_u32 == 0) tmp_u32 = (*p_mbox).MSGCTRL.all;
			EINT;
			m.len = (tmp_u32 & 0x000F);					/* copy message length */
			m.rtr = ((tmp_u32 >> 4) & 0x0001);			/* copy message RTR bit */

			DINT;	/* workaround to avoid eCAN register access error (see silicon errata) */
			tmp_u32 = (*p_mbox).MDL.all;
			if ( tmp_u32 == 0) tmp_u32 = (*p_mbox).MDL.all;
			EINT;
			m.data[0] = ((tmp_u32 >> 24) & 0x00FF);	/* copy message data bytes */
			m.data[1] = ((tmp_u32 >> 16) & 0x00FF);
			m.data[2] = ((tmp_u32 >>  8) & 0x00FF);
			m.data[3] = ((tmp_u32      ) & 0x00FF);

			DINT;	/* workaround to avoid eCAN register access error (see silicon errata) */
			tmp_u32 = (*p_mbox).MDH.all;
			if ( tmp_u32 == 0) tmp_u32 = (*p_mbox).MDH.all;
			EINT;
			m.data[4] = ((tmp_u32 >> 24) & 0x00FF);
			m.data[5] = ((tmp_u32 >> 16) & 0x00FF);
			m.data[6] = ((tmp_u32 >>  8) & 0x00FF);
			m.data[7] = ((tmp_u32      ) & 0x00FF);

			if(!MBX_post(&can_rx_mbox, &m, 0))
			{
				_WARNINGmessage(0xfffc, 0x01, 0x00, "can_rx_mbox full", 0, 0);
			}

			SET_YELLOW_CAN_LED;			/* turn on yellow CAN led */

			ECanaRegs.CANRMP.all = ((unsigned long)1 << ECanaShadow_CANGIF1.bit.MIV1);		/* clear interrupt flag of the received mailbox */
		}
		else			/* tx mailboxes interrups should be disabled, but... */
		{
			ECanaRegs.CANTA.all = ((unsigned long)1 << ECanaShadow_CANGIF1.bit.MIV1);	/* clear interrupt flag */
		}
	}
	else if(ECanaShadow_CANGIF1.bit.MTOF1 == 1) {	/* time-out of a tx mailbox */
		ECanaShadow_CANTOS.all = ECanaRegs.CANTOS.all;	/* read Time-Out Status Register */

		/* disable Time-Out so that it does not generate another interrupt while the ongoing message ends */
		ECanaShadow_CANTOC.all = ECanaRegs.CANTOC.all;
		ECanaShadow_CANTOC.all &= ~ECanaShadow_CANTOS.all;
		ECanaRegs.CANTOC.all = ECanaShadow_CANTOC.all;

		ECanaRegs.CANTRR.all = ECanaShadow_CANTOS.all;	/* cancel the transmission of timed-out mailbox */
		ECanaRegs.CANTOS.all = ECanaShadow_CANTOS.all;	/* clear the time-out flags */

		_LOGmessage(0x0035,"CAN tx message timed out", 0, 0);
	} else {
		/* do nothing */
		/* _LOGmessage(0x0002,"Only mailboxes interrupts should be in CAN1 interrupt", 0, 0); */
	}
}


/*!	Task that waits for CANOpen frames to send
*/
void canSendLoop()
{
	Message m;
	union CANME_REG ECanaShadow_CANME;
	union CANTOC_REG ECanaShadow_CANTOC;
	union CANMDL_REG ECanaShadow_CANMDL;
	union CANMDH_REG ECanaShadow_CANMDH;
	int i;
	unsigned long aux32;
	volatile struct MBOX *p_mbox;
	volatile Uint32 *p_moto_reg;

	while (1)
	{
		MBX_pend(&can_tx_mbox, &m, SYS_FOREVER); 		/* wait for a message in can tx mailbox */

		for(i=16, aux32=0x00010000; i<CAN_TX_MAILBOXES+16; i++, aux32 = aux32 << 1)
			if(!(ECanaRegs.CANTRS.all & aux32)) break;		/* loop until a tx mailbox that is not transmitting */

		if(i<CAN_TX_MAILBOXES+16)
		{
			p_mbox = &(ECanaMboxes.MBOX0) + i;		/* pointing to the first free mailbox */

			/* clear TA if set */
			if ( ECanaRegs.CANTA.all & aux32 ) {
				ECanaRegs.CANTA.all = aux32;		/* clear TA */
				while ( ECanaRegs.CANTA.all & aux32 ) {;}		/* wait until TA flag is cleared */
			}

			/* disable mailbox for changing message identifier */
			ECanaShadow_CANME.all = ECanaRegs.CANME.all;
			ECanaShadow_CANME.all &= ~aux32;		/* disable the mailbox that is going to be used to transmit */
			ECanaRegs.CANME.all = ECanaShadow_CANME.all;

			/* (*p_mbox).MSGID.bit.STDMSGID = m.cob_id; */
			DINT;	/* workaround to avoid eCAN register access error (see silicon errata) */
			(*p_mbox).MSGID.all = ((unsigned long)(m.cob_id) << 18);
			asm("	RPT #3 || NOP");	/* do nothing for 4 cycles */
			(*p_mbox).MSGID.all = ((unsigned long)(m.cob_id) << 18);
			EINT;

			/* (*p_mbox).MSGCTRL.bit.DLC = m.len; */
			/* (*p_mbox).MSGCTRL.bit.RTR = m.rtr; */
			DINT;	/* workaround to avoid eCAN register access error (see silicon errata) */
			(*p_mbox).MSGCTRL.all = (unsigned long)(m.len + (m.rtr << 4));
			asm("	RPT #3 || NOP");	/* do nothing for 4 cycles */
			(*p_mbox).MSGCTRL.all = (unsigned long)(m.len + (m.rtr << 4));
			EINT;

			/* enable mailbox */
			ECanaShadow_CANME.all = ECanaRegs.CANME.all;
			ECanaShadow_CANME.all |= aux32;
			ECanaRegs.CANME.all = ECanaShadow_CANME.all;

			ECanaShadow_CANMDL.byte.BYTE0 = m.data[0];
			ECanaShadow_CANMDL.byte.BYTE1 = m.data[1];
			ECanaShadow_CANMDL.byte.BYTE2 = m.data[2];
			ECanaShadow_CANMDL.byte.BYTE3 = m.data[3];
			DINT;	/* workaround to avoid eCAN register access error (see silicon errata) */
			(*p_mbox).MDL.all = ECanaShadow_CANMDL.all;
			asm("	RPT #3 || NOP");	/* do nothing for 4 cycles */
			(*p_mbox).MDL.all = ECanaShadow_CANMDL.all;
			EINT;

			ECanaShadow_CANMDH.byte.BYTE4 = m.data[4];
			ECanaShadow_CANMDH.byte.BYTE5 = m.data[5];
			ECanaShadow_CANMDH.byte.BYTE6 = m.data[6];
			ECanaShadow_CANMDH.byte.BYTE7 = m.data[7];
			DINT;	/* workaround to avoid eCAN register access error (see silicon errata) */
			(*p_mbox).MDH.all = ECanaShadow_CANMDH.all;
			asm("	RPT #3 || NOP");	/* do nothing for 4 cycles */
			(*p_mbox).MDH.all = ECanaShadow_CANMDH.all;
			EINT;

			/* disable Time-Out to change time-out value */
			ECanaShadow_CANTOC.all = ECanaRegs.CANTOC.all;
			ECanaShadow_CANTOC.all &= ~aux32;
			ECanaRegs.CANTOC.all = ECanaShadow_CANTOC.all;
			while(ECanaRegs.CANTOC.all & aux32);	/* wait until Time-out is disabled */
			/* set time-out value as TSC + time-out */
			p_moto_reg = &ECanaMOTORegs.MOTO0 + i;		/* pointer to every Message-Object Time-Out Register */
			DINT;	/* workaround to avoid eCAN register access error (see silicon errata) */
			*p_moto_reg = ECanaRegs.CANTSC + CAN_MSG_TIME_OUT;
			asm("	RPT #3 || NOP");	/* do nothing for 4 cycles */
			*p_moto_reg = ECanaRegs.CANTSC + CAN_MSG_TIME_OUT;
			EINT;

			/* enable Time-Out */
			ECanaShadow_CANTOC.all = ECanaRegs.CANTOC.all;
			ECanaShadow_CANTOC.all |= aux32;
			ECanaRegs.CANTOC.all = ECanaShadow_CANTOC.all;

			/* send data in mailbox i */
			ECanaRegs.CANTRS.all = aux32;

			SET_YELLOW_CAN_LED;			/* turn on yellow CAN led */
		}
		else
		{
			_WARNINGmessage(0xfffb, 0x01, 0, "all tx can_mboxes full", 0, 0);
		}
	}
}


/*!	Task that manages the CAN bus state and the red CAN LED
*/
void manage_can_bus(void)
{
	static unsigned int i_blink = 0;
	union CANES_REG ECanaShadow_CANES;

	ECanaShadow_CANES.all = ECanaRegs.CANES.all;		/*Clear the Status and Error Register*/
	ECanaRegs.CANES.all = ECanaShadow_CANES.all;

	/* when warning level, bus-off or error-passive modes are entered, can_bus_state is updated in the ISR */
	if ((ECanaShadow_CANES.bit.BO == 0) && (ECanaShadow_CANES.bit.EP == 0)) {
		if(ECanaShadow_CANES.bit.EW == 1) {
			if (ATM_seti(&can_bus_state, WARNING_LEVEL) != WARNING_LEVEL) 	/* error passive mode has been left */
				_LOGmessage(0x0036,"CAN bus left ERROR_PASSIVE mode to WARNING_LEVEL", 0, 0);
		} else {
			if (ATM_seti(&can_bus_state, ERROR_ACTIVE) != ERROR_ACTIVE) 	/* returned to ERROR_ACTIVE mode */
				_LOGmessage(0x0037,"CAN bus returned to ERROR_ACTIVE mode", 0, 0);
		}
	}

	switch (can_bus_state) {
		case ERROR_ACTIVE:
			CLEAR_RED_CAN_LED;
			break;
		case WARNING_LEVEL:
			if (++i_blink >= 50) {
				TOGGLE_RED_CAN_LED;	/* blink red CAN led */
				i_blink = 0;
			}
			break;
		case ERROR_PASSIVE:
		case BUS_OFF:
			SET_RED_CAN_LED;
	}
}


/* =========================================================================== */
/*  End of SourceCode. */
/* =========================================================================== */

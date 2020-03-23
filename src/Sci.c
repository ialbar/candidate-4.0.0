/*!	\file Sci.c
	\brief Initialization for serial communications

\verbatim
*********************************************************************
* File: Sci.c
* Devices: TMS320F28XXX
* Author: Luis Jimenez.
* History:
*   07/11/06 - original
*********************************************************************
\endverbatim
*/

#include "DSP2833x_Device.h"
#include "amc.h"
#include "debug_sci.h"

#define BAUD_9600 		487
#define BAUD_115200 	80
#define BAUD_1171875	3


/*****************************************************************/
/*!	Initializes SCI A for serial comunications
*/
/*****************************************************************/

void InitSci(void)
{
	SciaRegs.SCICCR.all =0x07;   	/* 1 stop bit,  No loopback */
                                   	/* No parity,8 char bits, */
                                   	/* async mode, idle-line protocol */

	SciaRegs.SCICTL1.all =0x03;  	/* enable TX, RX, internal SCICLK, */
//                                  /* Disable RX ERR, SLEEP, TXWAKE */
//	SciaRegs.SCICTL1.all =0x02;  	/* enable TX, stay in Reset */

	SciaRegs.SCIHBAUD = BAUD_1171875 >> 8 ;	/* 115200 Baud ; LSPCLK = 37.5MHz */
	SciaRegs.SCILBAUD = BAUD_1171875 & 0x00FF;


	SciaRegs.SCICTL2.bit.TXINTENA = 0;		/* Enable SCI-Transmit-interrupt */
	SciaRegs.SCICTL2.bit.RXBKINTENA = 1;	/* Enable SCI-A-Receive-interrupt */

	SciaRegs.SCIFFTX.all = 0xE021;		/* bit 15 = 1 : relinquish from Reset */
										/* bit 14 = 1 : Enable FIFO Enhancement */
										/* bit 13 = 1 : Reset TX FIFO */
										/* bit 6 = 0 :  no effect (CLR TXFFINT-Flag if 1) */
										/* bit 5 = 1 :  enable TX FIFO match */
										/* bit 4-0 = 1 :  TX-ISR, if TX FIFO is 1 or less */

	SciaRegs.SCIFFRX.all = 0xE061;		/* Rx interrupt level = 1 */
	SciaRegs.SCICTL1.all = 0x23;		/* Relinquish SCI from Reset */

	SciaRegs.SCIFFTX.bit.TXFIFOXRESET=1;
	SciaRegs.SCIFFRX.bit.RXFIFORESET=1;

	/* Enable SCI_A_TX_INT in PIE */
	PieCtrlRegs.PIEIER9.bit.INTx2 = 1;
	/* Enable SCI_A_RX_INT in PIE */
	PieCtrlRegs.PIEIER9.bit.INTx1 = 1;

	/* Enable CPU INT 9 */
	IER |= 0x100;
}

#if 0
void InitSci(void)
{
	SciaRegs.SCICCR.all =0x07;   	/* 1 stop bit,  No loopback */
                                   	/* No parity,8 char bits, */
                                   	/* async mode, idle-line protocol */

//	SciaRegs.SCICTL1.all =0x03;  	/* enable TX, RX, internal SCICLK, */
//                                  /* Disable RX ERR, SLEEP, TXWAKE */
	SciaRegs.SCICTL1.all =0x02;  	/* enable TX, stay in Reset */

//	SciaRegs.SCIHBAUD = 487 >> 8 ;  /* 9600 Baud ; LSPCLK = 37.5MHz */
//	SciaRegs.SCILBAUD = 487 & 0x00FF;
	SciaRegs.SCIHBAUD = 40 >> 8 ;	/* 115200 Baud ; LSPCLK = 37.5MHz */
	SciaRegs.SCILBAUD = 40 & 0x00FF;
//	SciaRegs.SCIHBAUD = 80 >> 8 ;	/* 57600 Baud ; LSPCLK = 37.5MHz */
//	SciaRegs.SCILBAUD = 80 & 0x00FF;

	SciaRegs.SCICTL2.bit.TXINTENA = 1;		/* Enable SCI-Transmit-interrupt */
//	SciaRegs.SCICTL2.bit.RXBKINTENA = 1;	/* Enable SCI-A-Receive-interrupt */

	SciaRegs.SCIFFTX.all = 0xE021;		/* bit 15 = 1 : relinquish from Reset */
										/* bit 14 = 1 : Enable FIFO Enhancement */
										/* bit 13 = 1 : Reset TX FIFO */
										/* bit 6 = 0 :  no effect (CLR TXFFINT-Flag if 1) */
										/* bit 5 = 1 :  enable TX FIFO match */
										/* bit 4-0 = 1 :  TX-ISR, if TX FIFO is 1 or less */

//	SciaRegs.SCIFFRX.all = 0xE061;		/* Rx interrupt level = 1 */
	SciaRegs.SCICTL1.all = 0x22;		/* Relinquish SCI from Reset */

	/* Enable SCI_A_TX_INT in PIE */
	PieCtrlRegs.PIEIER9.bit.INTx2 = 1;
	/* Enable SCI_A_RX_INT in PIE */
//	PieCtrlRegs.PIEIER9.bit.INTx1 = 1;

	/* Enable CPU INT 9 */
	IER |= 0x100;
}
#endif

/*****************************************************************/
/*!	Sends a message using SCI port.
	The length of the message can't exceed 16 characters
*/
/*****************************************************************/
int SCI_send(char *message)
{
	int i, size, sent;

	size = strlen(message);

	DINT;		/* necessary to protect log_buff, buff_tosend, buff_towrite */
	if(size <= ((buff_tosend - buff_towrite - 1)%SERIAL_LOG_BUFFER_SIZE))
	{
		for(i=0;i<size;i++) {
			log_buff[buff_towrite] = message[i];		/* Copy character to the buffer */
			buff_towrite = (buff_towrite + 1)%SERIAL_LOG_BUFFER_SIZE;	/* increment pointer index */
		}
		sent = 1;
	} else sent = 0;
	EINT;

	SciaRegs.SCIFFTX.bit.TXFFINTCLR = 1;		/* clear tx interrupt flag for interrupt to occur */

	return sent;
}

/*****************************************************************/
/*!	Sends a message using SCI port.
	The length of the message can't exceed 16 characters
*/
/*****************************************************************/
int SCI_send_debug(unsigned char *message)
{

	int i, size, sent;
	static char test=0;
	size = FRAME_SCIDEBUG_SIZE_14BYTES;
	//DINT;		/* necessary to protect log_buff, buff_tosend, buff_towrite */
	if(size <= ((buff_tosend - buff_towrite - 1)%SERIAL_LOG_BUFFER_SIZE))
	{
		for(i=0;i<size;i++) {
			log_buff[buff_towrite] = message[i];		/* Copy character to the buffer */
			buff_towrite = (buff_towrite + 1)%SERIAL_LOG_BUFFER_SIZE;	/* increment pointer index */
		}
		sent = 1;
	} else sent = 0;
	//EINT;
	SciaRegs.SCIFFTX.bit.TXFFINTCLR = 1;		/* clear tx interrupt flag for interrupt to occur */
	return sent;
}


/*** end of file *****************************************************/

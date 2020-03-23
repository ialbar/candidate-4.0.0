/*!
 * \addtogroup i2c
 * \{
 */

/*!
 * \file      i2c.c
 * \brief     Source file of the I2C module for TMS320F28XXX devices
 * \author    Manuel Godoy
 * \version   V1R0.0
 * \copyright SEDECAL S.A.
 */

#include "DSP2833x_Device.h"
#include "i2c.h"
#include "string.h"
#include "fpga.h"
#include "amc.h"

/*!
 * According to datasheet, module clk mus be 7-12MHz. Set to 10MHz.
 * Module clk frec = ( I2C input clk / ( IPSC + 1 ) ) where I2C input clk = SYSCLK
 */
#define I2C_MODULE_CLK_MHz     10
#define I2C_PRESCALER          ( ( SYSCLK_MHz / I2C_MODULE_CLK_MHz ) - 1 )

/*!
 * IPSC = 0 => d = 7
 * IPSC = 1 => d = 6
 * IPSC > 1 => d = 5
 */
#if( I2C_PRESCALER > 1 )
	#define I2C_PARAM_D 5
#elif( I2C_PRESCALER == 1 )
	#define I2C_PARAM_D 6
#elif( I2C_PRESCALER == 0 )
	#define I2C_PARAM_D 7
#endif

/*!
 * Formula of the master clk:
 * Tmaster = Tmodule * ( ( ICCL + d ) + ( ICCH + d ) )
 * I2C divider has been considered as ( ( ICCL + d ) + ( ICCH + d ) )
 */
#define I2C_CLK_DIVIDER   ( I2C_MODULE_CLK_MHz*1000 / I2C_FRECUENCY_KHz )
#define I2C_CLK_HIGH      ( ( I2C_CLK_DIVIDER / 2 ) - I2C_PARAM_D )
#define I2C_CLK_LOW       ( I2C_CLK_DIVIDER - I2C_CLK_HIGH - 2*I2C_PARAM_D )


#define I2C_NO_ISRC       0x0000 //!< Interrupt source register (I2CISRC): None
#define I2C_ARB_ISRC      0x0001 //!< Interrupt source register (I2CISRC): Arbitration lost
#define I2C_NACK_ISRC     0x0002 //!< Interrupt source register (I2CISRC): No-akcnowledgment
#define I2C_ARDY_ISRC     0x0003 //!< Interrupt source register (I2CISRC): Register ready
#define I2C_RX_ISRC       0x0004 //!< Interrupt source register (I2CISRC): Receive data ready
#define I2C_TX_ISRC       0x0005 //!< Interrupt source register (I2CISRC): Transmit data ready
#define I2C_SCD_ISRC      0x0006 //!< Interrupt source register (I2CISRC): Stop Condition detected
#define I2C_AAS_ISRC      0x0007 //!< Interrupt source register (I2CISRC): Addressed as slave

#define I2C_CLR_AL_BIT    0x0001 //!< Status register (I2CSTR) Interrupt flag: Arbitration lost
#define I2C_CLR_NACK_BIT  0x0002 //!< Status register (I2CSTR) Interrupt flag: No-akcnowledgment
#define I2C_CLR_ARDY_BIT  0x0004 //!< Status register (I2CSTR) Interrupt flag: Register ready
#define I2C_CLR_RRDY_BIT  0x0008 //!< Status register (I2CSTR) Interrupt flag: Receive data ready
#define I2C_CLR_SCD_BIT   0x0020 //!< Status register (I2CSTR) Interrupt flag: Stop Condition detected

#define I2C_ST_STANDBY    0x00

#define I2C_BLOCK_SIZE    16     //!< Max number of bytes allowed to keep on FIFOs.

// Macro funtions
#define I2C_TXFIFO_RESET( )   I2caRegs.I2CFFTX.bit.TXFFRST   = 0;\
	                          I2caRegs.I2CFFTX.all           = 0x6000         //!< reset TX fifo
#define I2C_TXFIFO_INT_EN( )  I2caRegs.I2CFFTX.all          |= 0x0060
#define I2C_TXFIFO_INT_DIS( ) I2caRegs.I2CFFTX.bit.TXFFIENA  = 0

#define I2C_RXFIFO_RESET( )   I2caRegs.I2CFFRX.bit.RXFFRST   = 0;\
	                          I2caRegs.I2CFFRX.all           = 0x2040         //!< reset RX fifo
#define I2C_RXFIFO_INT_EN( )  I2caRegs.I2CFFRX.all          |= 0x0060
#define I2C_RXFIFO_INT_DIS( ) I2caRegs.I2CFFRX.bit.RXFFIENA  = 0

#define I2C_MODE_WR_STOP( )   I2caRegs.I2CMDR.all            = 0x2E20         //!< Write messages
#define I2C_MODE_WR_NOSTOP( ) I2caRegs.I2CMDR.all            = 0x2620         //!< Read setup messages
#define I2C_MODE_RD_STOP( )   I2caRegs.I2CMDR.all            = 0x2C20         //!< Read messages

/*!
 * \brief Typedef for message types
 */
typedef enum
{
	I2CMSG_WRITE = 0,  //!< Write message
	I2CMSG_READ_SETUP, //!< Write part of a write-read message (read master orders)
	I2CMSG_READ        //!< Receie message
}tI2CMsgType;

/*!
 * \brief Typedef for message information
 */
typedef struct
{
	bool        *trans_complete;  //!< Flag of transaction completed.
	Uint8       n_bytes;          //!< Number of bytes in the message.
	Uint8       remaining;        //!< Number of remaining bytes.
	Uint8       *buffer;          //!< Buffer to receive / transmit.
	tI2CMsgType rw;               //!< Type of message according to \ref tI2CMsgType.
	Uint16      sl_address;       //!< Slave address.
}tI2C_message;

/*!
 * \brief Struct for queue managment
 */
static struct
{
	Uint8        status;                      //!< Status of the queue
	Uint8        trans_index;                 //!< Index of the last message on the queue
	Uint8        queue_index;                 //!< Index of the processing message
	tI2C_message queue[I2C_MAX_TRANSACTIONS]; //!< Queue of messages
}__i2c_info;

/*!
 * \brief Returns if the I2c module is busy.
 */
bool I2CA_IsBusy( void )
{
	return I2caRegs.I2CSTR.bit.BB;
}

/*!
 * \brief Scans next message of the queue.
 * This interrupt has a complementary functionality with \ref I2CINT1A_ISR.
 * If the message in the queue is a write-style message (including read order messages)
 * the FIFO is filled with the message buffer. Between write messages and read setup,
 * the only difference is STOP bit. For read messages, just the start read
 * order is given.
 * \note inline function because is called by an interrupt.
 */
static inline tI2CError I2CA_Scan( void )
{
	Uint16 i;

	if( __i2c_info.queue_index == __i2c_info.trans_index ) return I2CER_NONE;

	I2caRegs.I2CSAR = __i2c_info.queue[__i2c_info.queue_index].sl_address;
	I2caRegs.I2CCNT = __i2c_info.queue[__i2c_info.queue_index].n_bytes;

	if( __i2c_info.queue[__i2c_info.queue_index].rw != I2CMSG_READ )
	{
		for (i=0; i < __i2c_info.queue[__i2c_info.queue_index].n_bytes && i < I2C_BLOCK_SIZE; i++ )
		{
			I2caRegs.I2CDXR = *( __i2c_info.queue[__i2c_info.queue_index].buffer + i );
		}
		__i2c_info.queue[__i2c_info.queue_index].remaining -= i;
		if( !__i2c_info.queue[__i2c_info.queue_index].remaining )
			I2C_TXFIFO_INT_DIS( );
		else
			I2C_TXFIFO_INT_EN( );

		if( __i2c_info.queue[__i2c_info.queue_index].rw == I2CMSG_WRITE )
			I2C_MODE_WR_STOP( );
		else if( __i2c_info.queue[__i2c_info.queue_index].rw == I2CMSG_READ_SETUP )
			I2C_MODE_WR_NOSTOP( );
	}else
	{
		if( __i2c_info.queue[__i2c_info.queue_index].remaining <= I2C_BLOCK_SIZE )
			I2C_RXFIFO_INT_DIS( );
		else
			I2C_RXFIFO_INT_EN( );

		I2C_MODE_RD_STOP( );
	}
	return I2CER_NONE;
}

tI2CError I2CA_Init( tI2CAddressFormat add_format )
{
	// Variables initialization
	__i2c_info.status           = I2C_ST_STANDBY;
	__i2c_info.trans_index      = 0;
	__i2c_info.queue_index      = 0;
	(void)memset( __i2c_info.queue, 0, sizeof(tI2C_message)*I2C_MAX_TRANSACTIONS );

	// Prescaler configuration
	EDIS;
	I2caRegs.I2CPSC.all = I2C_PRESCALER;
	I2caRegs.I2CCLKL    = I2C_CLK_LOW;
	I2caRegs.I2CCLKH    = I2C_CLK_HIGH;

	// Start and interrupt and FIFOs configuration
	I2caRegs.I2CMDR.bit.IRS  = 1;
	I2caRegs.I2CIER.all      = 0x0026;
	I2caRegs.I2CFFTX.all     = 0x6000;
	I2caRegs.I2CFFRX.all     = 0x202C;

	// Format parameter
	if( add_format != I2C_ADDR_FREE ) I2caRegs.I2CMDR.bit.XA = add_format;
	else I2caRegs.I2CMDR.bit.FDF = 1;

	// Enable I2C interrupt 1 in the PIE: Group 8 interrupts 1 and 2
	PieCtrlRegs.PIEIER8.bit.INTx1 = 1;
	PieCtrlRegs.PIEIER8.bit.INTx2 = 1;

	// Enable CPU INT8 which is connected to PIE group 8
	IER |= M_INT8;
	EINT;

	return I2CER_NONE;
}

tI2CError I2CA_WriteData( Uint16 slave_add, Uint8 *p_data, Uint8 n_bytes )
{
	// Errors caused by parameters
	if( !n_bytes )                    return I2CER_NONE;
	if( n_bytes > I2C_MAX_TX_BYTES)   return I2CER_OVERRUN;

	// Write message into the queue
	__i2c_info.queue[__i2c_info.trans_index].rw              = I2CMSG_WRITE;
	__i2c_info.queue[__i2c_info.trans_index].sl_address      = slave_add;
	__i2c_info.queue[__i2c_info.trans_index].n_bytes         = n_bytes;
	__i2c_info.queue[__i2c_info.trans_index].buffer          = p_data;
	__i2c_info.queue[__i2c_info.trans_index].remaining       = n_bytes;
	*__i2c_info.queue[__i2c_info.trans_index].trans_complete = false;        // dummy

	// Check queue overflow
	if( __i2c_info.trans_index == I2C_MAX_TRANSACTIONS - 1 ) __i2c_info.trans_index = 0;
	else __i2c_info.trans_index++;

	// Scan queue if bus is prepared for it, and scan
	if( I2caRegs.I2CMDR.bit.STP == 1 ) return I2CER_NOT_STOPPED;
	if( I2caRegs.I2CSTR.bit.BB  == 1 ) return I2CER_BUS_BUSY;
	I2CA_Scan();

	return I2CER_NONE;
}

tI2CError I2CA_SendReadData( Uint16 slave_add, Uint8 *p_data_w, Uint8 n_bytes_w,
		                                     Uint8 *p_data_r, Uint8 n_bytes_r, bool *p_flag )
{
	// Errors caused by parameters
	if( !n_bytes_w )                   return I2CER_NONE;
	if( n_bytes_w > I2C_MAX_TX_BYTES ) return I2CER_OVERRUN;
	if( !n_bytes_r )                   return I2CER_NONE;
	if( n_bytes_r > I2C_MAX_RX_BYTES ) return I2CER_OVERRUN;

	// Write (read setup) message
	__i2c_info.queue[__i2c_info.trans_index].rw              = I2CMSG_READ_SETUP;
	__i2c_info.queue[__i2c_info.trans_index].sl_address      = slave_add;
	__i2c_info.queue[__i2c_info.trans_index].n_bytes         = n_bytes_w;
	__i2c_info.queue[__i2c_info.trans_index].buffer          = p_data_w;
	__i2c_info.queue[__i2c_info.trans_index].remaining       = n_bytes_w;
	*__i2c_info.queue[__i2c_info.trans_index].trans_complete = false;

	// Check queue overflow
	if( __i2c_info.trans_index == I2C_MAX_TRANSACTIONS - 1 ) __i2c_info.trans_index = 0;
	else __i2c_info.trans_index++;

	// Scan queue if bus is prepared for it, and scan
	if( I2caRegs.I2CMDR.bit.STP == 1 ) return I2CER_NOT_STOPPED;
	if( I2caRegs.I2CSTR.bit.BB  == 1 ) return I2CER_BUS_BUSY;
	I2CA_Scan();

	// Read message
	__i2c_info.queue[__i2c_info.trans_index].rw              = I2CMSG_READ;
	__i2c_info.queue[__i2c_info.trans_index].sl_address      = slave_add;
	__i2c_info.queue[__i2c_info.trans_index].n_bytes         = n_bytes_r;
	__i2c_info.queue[__i2c_info.trans_index].buffer          = p_data_r;
	__i2c_info.queue[__i2c_info.trans_index].remaining       = n_bytes_r;
	*__i2c_info.queue[__i2c_info.trans_index].trans_complete = false;
	if( p_flag )
		__i2c_info.queue[__i2c_info.trans_index].trans_complete = p_flag;

	// Check queue overflow
	if( __i2c_info.trans_index == I2C_MAX_TRANSACTIONS - 1 ) __i2c_info.trans_index = 0;
	else __i2c_info.trans_index++;

	// Scan queue if bus is prepared for it, and scan
	if( I2caRegs.I2CMDR.bit.STP == 1 ) return I2CER_NOT_STOPPED;
	if( I2caRegs.I2CSTR.bit.BB  == 1 ) return I2CER_BUS_BUSY;
	I2CA_Scan();

	return I2CER_NONE;
}

/*!
 * \brief I2C main interrupt
 *
 * This interrupt has a complementary functionality with \ref I2CA_Scan.
 * - Stop Condition detected: If write message, just scan next message on queue,
 * because message has been processed. If read message, read FIFO to fill the
 * buffer and jump to next.
 * - Access register ready: If message is a read order, jump to next message.
 * - Not Acknowledge: Something was wrong. Clear bit but nothing to do on the queue.
*/
#ifdef OS_NONE
interrupt void I2CINT1A_ISR( void )
#else
void I2CINT1A_ISR( void )
#endif
{
	Uint16 int_source, i, first;

	int_source = I2caRegs.I2CISRC.bit.INTCODE;

	// Stop Condition Detected
	if( int_source == I2C_SCD_ISRC )
	{
		*__i2c_info.queue[__i2c_info.queue_index].trans_complete = true;

		// Write message: Increase index, check overflow and scan next
		if( __i2c_info.queue[__i2c_info.queue_index].rw == I2CMSG_WRITE )
		{
			I2C_TXFIFO_RESET( );
			if( __i2c_info.queue_index == I2C_MAX_TRANSACTIONS - 1 ) __i2c_info.queue_index = 0;
			else __i2c_info.queue_index++;
			I2CA_Scan();
		}
		// Read message: Read until the end of the message or FIFO, check overflow and scan next
		else if( __i2c_info.queue[__i2c_info.queue_index].rw == I2CMSG_READ )
		{
			first = __i2c_info.queue[__i2c_info.queue_index].n_bytes - __i2c_info.queue[__i2c_info.queue_index].remaining;
			for( i=0; i < __i2c_info.queue[__i2c_info.queue_index].n_bytes && i < I2C_BLOCK_SIZE; i++ )
			{
				*( __i2c_info.queue[__i2c_info.queue_index].buffer + first + i ) = I2caRegs.I2CDRR;
			}
			__i2c_info.queue[__i2c_info.queue_index].remaining = 0;
			I2C_RXFIFO_RESET( );
			if( __i2c_info.queue_index == I2C_MAX_TRANSACTIONS - 1 ) __i2c_info.queue_index = 0;
			else __i2c_info.queue_index++;
			I2CA_Scan();
		}
		I2caRegs.I2CSTR.all |= I2C_CLR_SCD_BIT; // clear condition
	}
	// Access registers ready
	else if( int_source == I2C_ARDY_ISRC )
	{
		if( __i2c_info.queue[__i2c_info.queue_index].rw == I2CMSG_READ_SETUP )
		{
			I2C_TXFIFO_RESET( );
			*__i2c_info.queue[__i2c_info.queue_index].trans_complete = true;
			if( __i2c_info.queue_index == I2C_MAX_TRANSACTIONS - 1 ) __i2c_info.queue_index = 0;
			else __i2c_info.queue_index++;
			I2CA_Scan();
		}
	}
	// No acknoledgment condition detected
	else if( int_source == I2C_NACK_ISRC )
	{
		I2caRegs.I2CMDR.bit.STP = 1;
		I2caRegs.I2CSTR.all = I2C_CLR_NACK_BIT;
	}else{}
	// else for an error or callback error
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;
}

/*!
 * \brief FIFO interrupt
 *
 * This interrupt is only used when a message is longer than 16 bytes. If so, FIFOs interrupts
 * continues the message reading or writing operation, taking next buffer to read/write.
 */
#ifdef OS_NONE
interrupt void I2CINT2A_ISR( void )
#else
void I2CINT2A_ISR( void )
#endif
{
	Uint16 first, rx_bytes, i;

	// Write or setup message
	if( I2caRegs.I2CFFTX.bit.TXFFINT )
	{
		if( __i2c_info.queue[__i2c_info.queue_index].rw != I2CMSG_READ )
		{
			first = __i2c_info.queue[__i2c_info.queue_index].n_bytes - __i2c_info.queue[__i2c_info.queue_index].remaining;
			for( i=0; i < __i2c_info.queue[__i2c_info.queue_index].n_bytes && i < I2C_BLOCK_SIZE; i++ )
			{
				I2caRegs.I2CDXR = *( __i2c_info.queue[__i2c_info.queue_index].buffer + first + i );
			}
			__i2c_info.queue[__i2c_info.queue_index].remaining -= i;
			if( !__i2c_info.queue[__i2c_info.queue_index].remaining )
				I2C_TXFIFO_INT_DIS( );
		}
		else I2C_TXFIFO_INT_DIS( );

		// clear interrupt flag
		I2caRegs.I2CFFTX.bit.TXFFINTCLR = 1;
	}

	// Read message
	if( I2caRegs.I2CFFRX.bit.RXFFINT )
	{
		if( __i2c_info.queue[__i2c_info.queue_index].rw == I2CMSG_READ )
		{
			// read number of available bytes
			rx_bytes = I2caRegs.I2CFFRX.bit.RXFFST;

			first = __i2c_info.queue[__i2c_info.queue_index].n_bytes - __i2c_info.queue[__i2c_info.queue_index].remaining;
			for( i=0; i < rx_bytes; i++ )
			{
				*( __i2c_info.queue[__i2c_info.queue_index].buffer + first + i ) = I2caRegs.I2CDRR;
			}
			__i2c_info.queue[__i2c_info.queue_index].remaining -= i;
			if( __i2c_info.queue[__i2c_info.queue_index].remaining <= I2C_BLOCK_SIZE )
				I2C_RXFIFO_INT_DIS( );
			else
				I2C_RXFIFO_INT_EN( );
		}
		else I2C_RXFIFO_INT_DIS( );   // disable interrupt

		// clear interrupt flag
		I2caRegs.I2CFFRX.bit.RXFFINTCLR = 1;
	}
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;
}



/*! \} */ //grouping

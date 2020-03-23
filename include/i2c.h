/*!
 * \defgroup i2c I2CDriver
 * \ingroup  drivers
 * \remark    Driver for just ONE I2C bus, as TMS320F28XXX supports.
 * \remark    Timming is not controled. User must know devices timming restrictions
 * \pre       SYSCLK_MHz and I2C_FRECUENCY_KHz must be defined in settings.h.
 * \todo      SLAVE mode. This driver just works in master mode
 * \todo      Free address mode not tested
 *
 * USAGE:
 * - Define a OS ( OS_NONE, OS_DSPBIOS, OS_SYSBIOS) in settings.h.
 * - Define I2C_FRECUENCY_KHz and I2C_MAX_TRANSACTIONS in settings.h
 * - Enable I2C clock in SysCtrl.c: SysCtrlRegs.PCLKCR0.bit.I2CAENCLK
 * - Remove I2CINT1A_ISR, I2CINT2A_ISR from DefaultIsr.c
 * - Use \ref I2CA_Init to initialize the driver.
 *
 * \{
 */

/*!
 * \file      i2c.h
 * \brief     Header file of the I2C module for TMS320F28XXX devices
 * \author    Manuel Godoy
 * \version   V1R0.0
 * \copyright SEDECAL S.A.
 */

#ifndef I2C_H_
#define I2C_H_

#include "settings.h"
#include "typedef.h"
#include "amc.h"

// Copy one of this clock definitions in your settings.h
//#define I2C_FRECUENCY_KHz     100
//#define I2C_FRECUENCY_KHz     200
//#define I2C_FRECUENCY_KHz     250
//#define I2C_FRECUENCY_KHz     333
//#define I2C_FRECUENCY_KHz     400

//#define I2C_MAX_TRANSACTIONS  8   // 4 devices can transmitt and receive at the same time
//#define I2C_MAX_TX_BYTES      50
//#define I2C_MAX_RX_BYTES      100

/*!
 * \brief enum of I2C errors
 */
typedef enum
{
	I2CER_NONE = 0,
	I2CER_TX_MAX_LENGTH,
	I2CER_OVERRUN,
	I2CER_BUS_BUSY,
	I2CER_NOT_STOPPED,
	I2CER_NACK_RD_NOTSENT,
	I2CER_ACK_NOTRECEIVED
} tI2CError;

/*!
 * \brief  enum of I2C address formats
 * \remark I2C_ADDR_FREE is not supported (V1R0.0)
 */
typedef enum
{
	I2C_ADDR_7BITS = 0,
	I2C_ADDR_10BITS,
	I2C_ADDR_FREE        //!< not supported yet.
}tI2CAddressFormat;

/*!
 * \brief Initializes I2C module.
 *
 * - Clocking module to \ref I2C_MODULE_CLK_MHz and to \ref I2C_FRECUENCY_KHz.
 * - Enable I2C module.
 * - Enable SCD, ARDY (register access ready) and NACK interrupts.
 * - Enable FIFOs.
 * - Addressing format.
 * - Free mode on breakpoints.
 * - Enable interrupts.
 */
tI2CError I2CA_Init        ( tI2CAddressFormat add_format );

/*!
 * \brief Sends a message through the I2C module A.
 * \param [in] slave_add Slave address. Supported 7-bits or 10-bits formats.
 * \param [in] p_data    Pointer to the bytes buffer of data to write.
 * \param [in] n_bytes   Number of bytes to transmit.
 * \return Error code according to \ref tI2CError.
 *
 * - This function checks if I2C bus is busy and abort if it is.
 * - Transmission is done by FIFO register.
 * - Enable of start, stop, master, transmitter bit
 */
tI2CError I2CA_WriteData   ( Uint16 slave_add, Uint8 *p_data, Uint8 n_bytes );

/*!
 * \brief Sends a read message through the I2C module A.
 * \param [in] slave_add Slave address. Supported 7-bits or 10-bits formats.
 * \param [in] p_data_w   Pointer to the buffer to write (if required by slave)
 * \param [in] n_bytes_w  Number of bytes to transmit in the \ref p_data_w buffer.
 * \param [in] p_data_r   Pointer to the buffer to read (value to be read)
 * \param [in] n_bytes_r  Number of bytes to read in the \ref p_data_r buffer.
 * \param [out] flag      pointer to bool, that message if recived.
 * \return Error code according to \ref tI2CError.
 * \note  p_data_w can be NULL, i.e if a slave doesn't need further info to send its data
 *
 * - Message is divided into a \ref I2CMSG_READ_SETUP and a \ref I2CMSG_READ messages
 * - This function checks if I2C bus is busy and abort if it is.
 * - Enable of start, non-stop, master, transmitter bit.
 * - Save reading parameters.
 * - Enable of start, stop, master, receiver bit.
*/

tI2CError I2CA_SendReadData( Uint16 slave_add, Uint8 *p_data_w, Uint8 n_bytes_w,
                             Uint8 *p_data_r, Uint8 n_bytes_r, bool *p_flag );

bool I2CA_IsBusy           ( void );

#endif /* I2C_H_ */


/*! \} */ //grouping

/*!
 * \defgroup M24xx32A Memory_24xx32A_I2C
 * \ingroup  generics
 * \remark   Timming is not controled. User must know devices timming restrictions
 * \remark   Write operations take up to 5 ms. Timming must be controled
 * \pre      WORD_MOST_SIGN_BYTE and WORD_LESS_SIGN_BYTE defined in \ref settings.h
 * \todo     Several items support not tested
 *
 * USAGE:
 * - Use \ref M24xx32AInit.
 * - It is highly recommended to use \ref I2C_EEPROM_PAGE to memory address
 * - Writting proccess takes the same time (<=5ms) for single byte up to a complete
 * page. For this reason, it is recommended to write complete pages.
 * - See the example below
 *
 * \code{.c}
 *  byte write_data[I2C_EEPROM_PAGE_SIZE];
 *  byte read_data[I2C_EEPROM_PAGE_SIZE];
 *
 *  M24xx32AInit      ( I2C_REAL_ADD_EEPROM, (t24xx32ACallbackWR)&I2cwrite,
 *			                                 (t24xx32ACallbackRD)&I2cread );
 *	M24xx32AWritePage ( I2C_EEPROM_PAGE(0), write_data );
 *	DelayUs           ( 5000L );
 *	M24xx32AReadPage  ( I2C_EEPROM_PAGE(0), read_data, &flag );
 *
 *	while( !flag ){};
 * \endcode
 * \{
 */

/*!
 * \file      24xx32a.h
 * \brief     Header file of the Microchip 24AA32A/24LC32A I2C EEPROM devices API
 * \author    Manuel Godoy
 * \version   V1R0.0
 * \copyright SEDECAL S.A
 *
 */


#ifndef M24XX32A_H_
#define M24XX32A_H_

#include "settings.h"
#include "typedef.h"

#define I2C_EEPROM_SIZE        128                                            //!< 24XX32A number of pages
#define I2C_EEPROM_PAGE_SIZE   32                                             //!< 24XX32A bytes per page
#define I2C_EEPROM_LAST_REG    I2C_EEPROM_PAGE_SIZE * I2C_EEPROM_SIZE - 1     //!< Last byte address of the memory
#define I2C_EEPROM_LAST_PAGE   I2C_EEPROM_PAGE_SIZE * ( I2C_EEPROM_SIZE - 1 ) //!< Last page address of the memory
#define I2C_EEPROM_PAGE( x )   I2C_EEPROM_PAGE_SIZE * x                       //!< Page x of the memory (from 0 to I2C_EEPROM_SIZE-1)

#define I2C_EEPROM_MIN_I2CADD  0x50  //!< Minimum I2C address of this module (SOT-23 must be this)
#define I2C_EEPROM_MAX_I2CADD  0x5E  //!< Maximum I2C address of this module ( A2=1; A1=1; A0=1 )

/*!
 * \brief enum of eeprom errors
 *
 * Positive error values are reserved for the I2C errors
 */
typedef enum
{
	I2CEEPROMER_NONE = 0,            //!< No error
	I2CEEPROMER_NOCB_DEFINED = -1,   //!< Any of the required callbacks are not defined
	I2CEEPROMER_WRONG_PAGE = -2,     //!< Address of the page
	I2CEEPROMER_END_OF_MEM = -3      //!< End of memory reached
}t24xx32AError;

typedef Uint8 (*t24xx32ACallbackWR)( Uint16 slave_add, Uint8 *p_data,   Uint8 n_bytes );
typedef Uint8 (*t24xx32ACallbackRD)( Uint16 slave_add, Uint8 *p_data_w, Uint8 n_bytes_w,
                                    Uint8 *p_data_r, Uint8 n_bytes_r, bool *p_flag );


/*!
 * \brief Init function of the 24xx32A API
 * \param [in] i2c_address  7-bit address 1010XXX according to chip select pins (see below)
 * \param [in] cb_write     Write function of the I2C or control module
 * \param [in] cb_read      Write-read function of the I2C or control module
 * \return This functions returns:
 * - t24xx32AError errors (negative values)
 * - I2C callbacks errors (reserved positive values for it)
 *
 * Address of the I2c memory is defined by chip select pins. The control byte of the memory is:
 * 1 0 1 0 A2 A1 A0 R/W where the last three bits are the chip select pins in order to use up
 * to 8 devices in the same bus. For the SOT-23 and chip scale packages, these pins are not
 * available, and those bits must be 0. So in this case, address is 0x50.
 *
 * Anyway, it is allowed to set an I2C address out of this range, taking into account that
 * callbacks could be a parser of a protocol or a generic function not necessarily I2c funtions.
 */

int M24xx32AInit      ( Uint16 address, t24xx32ACallbackWR cb_write, t24xx32ACallbackRD cb_read );

// ************Write functions***********

/*!
 * \brief Writes a byte into the memory
 * \param [in] address Memory address from 0 to \ref I2C_EEPROM_LAST_REG
 * \param [in] value    Value to be writen in this address
 *
 * \return This functions returns:
 * - t24xx32AError errors (negative values)
 * - I2C callbacks errors (reserved positive values for it)
 *
 * Writting proccess takes the same time (<=5ms) for single byte up to a complete
 * page. For this reason, it is recommended to write complete pages.
 */

int M24xx32AWriteByte ( Uint16 address, Uint8 value );

/*!
 * \brief Writes a page into the memory
 * \param [in] address Memory address from 0 to \ref I2C_EEPROM_LAST_PAGE
 * \param [in] vale    Value to be writen in this address
 *
 * \return This functions returns:
 * - t24xx32AError errors (negative values)
 * - I2C callbacks errors (reserved positive values for it)
 *
 * Writting proccess takes the same time (<=5ms) for single byte up to a complete
 * page. For this reason, it is recommended to write complete pages.
 *
 * \note   Use \ref I2C_EEPROM_PAGE to be sure you are using a correct page address
 * \remark Make sure that \ref data_w is at least \ref I2C_EEPROM_PAGE_SIZE long
 */

int M24xx32AWritePage ( Uint16 address, Uint8 *data_w );

// ************Read functions************

/*!
 * \brief Reads a byte from the memory
 * \param [in]  address Memory address from 0 to \ref I2C_EEPROM_LAST_REG
 * \param [in]  value   Value to be read from this address
 * \param [out] p_flag  Flag to know when read operation is done
 *
 * \return This functions returns:
 * - t24xx32AError errors (negative values)
 * - I2C callbacks errors (reserved positive values for it)
 */
int M24xx32AReadByte  ( Uint16 address, Uint8 *value, bool *p_flag );

/*!
 * \brief Reads a byte from the memory
 * \param [in]  address Memory address from 0 to \ref I2C_EEPROM_LAST_PAGE
 * \param [in]  value   Value to be read from this address
 * \param [out] p_flag  Flag to know when read operation is done
 *
 * \return This functions returns:
 * - t24xx32AError errors (negative values)
 * - I2C callbacks errors (reserved positive values for it)
 *
 * \remark Make sure that \ref data_r is at least \ref I2C_EEPROM_PAGE_SIZE long
 */

int M24xx32AReadPage  ( Uint16 address, Uint8 *data_r, bool *p_flag );

/*!
 * \brief Reads a byte from the memory
 * \param [in]  address Memory address from 0 to \ref I2C_EEPROM_LAST_REG
 * \param [in]  value   Value to be read from this address
 * \param [in]  n_bytes Number of bytes to be read
 * \param [out] p_flag  Flag to know when read operation is done
 *
 * \return This functions returns:
 * - t24xx32AError errors (negative values)
 * - I2C callbacks errors (reserved positive values for it)
 *
 * \remark Make sure that \ref data_r is at least n_bytes long
 */

int M24xx32AReadBuffer( Uint16 address, Uint8 *data_r, Uint8 n_bytes, bool *p_flag );

#endif /* 24XX32A_H_ */


/*! \} */ //grouping

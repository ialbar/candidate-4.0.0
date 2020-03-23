/*!
 * \addtogroup M24xx32A
 * \{
 */

/*!
 * \file      24xx32a.c
 * \brief     Source file of the Microchip 24AA32A/24LC32A I2C EEPROM devices API
 * \author    Manuel Godoy
 * \version   V1R0.0
 * \copyright SEDECAL S.A.
 * \remark    Timming is not controled. User must know devices timming restrictions
 * \pre       This API needs an I2C driver with write and write-read functions
 */
#include "amc.h"
#include "24xx32a.h"
#include <string.h>

#define I2C_EEPROM_HEADER_SIZE 2  //!< header message size
/*!
 * \brief Struct for eeprom information
 */
static struct
{
	Uint16 address;                 //!< Address of the 24xx32A device
	Uint8 msg[I2C_EEPROM_HEADER_SIZE+I2C_EEPROM_PAGE_SIZE];
	t24xx32ACallbackWR write_cb;  //!< Write callback
	t24xx32ACallbackRD read_cb;   //!< Read callback
}__eeprom_info;

int M24xx32AInit( Uint16 i2c_address, t24xx32ACallbackWR cb_write, t24xx32ACallbackRD cb_read )
{
	if( ( !cb_write ) || ( !cb_read ) ) return I2CEEPROMER_NOCB_DEFINED;

	__eeprom_info.address       = i2c_address;
	__eeprom_info.write_cb      = cb_write;
	__eeprom_info.read_cb       = cb_read;
	(void)memset( __eeprom_info.msg, 0,
			      sizeof(Uint8)*(I2C_EEPROM_HEADER_SIZE+I2C_EEPROM_PAGE_SIZE) );

	return I2CEEPROMER_NONE;
}

// ************Write functions***********
int M24xx32AWriteByte( Uint16 address, Uint8 value )
{
	if( address > I2C_EEPROM_LAST_REG ) return I2CEEPROMER_END_OF_MEM;

	__eeprom_info.msg[0] = WORD_MOST_SIGN_BYTE( address );
	__eeprom_info.msg[1] = WORD_LESS_SIGN_BYTE( address );
	__eeprom_info.msg[2] = value;

	if( !__eeprom_info.write_cb )
		return I2CEEPROMER_NOCB_DEFINED;
	else
		return __eeprom_info.write_cb( __eeprom_info.address,
				                       __eeprom_info.msg,
				                       I2C_EEPROM_HEADER_SIZE+1 );
}

int M24xx32AWritePage( Uint16 address, Uint8 *data_w )
{
	if( address > I2C_EEPROM_LAST_PAGE )
		return I2CEEPROMER_END_OF_MEM;
	if( address % I2C_EEPROM_PAGE_SIZE )
		return I2CEEPROMER_WRONG_PAGE;

	__eeprom_info.msg[0] = WORD_MOST_SIGN_BYTE( address );
	__eeprom_info.msg[1] = WORD_LESS_SIGN_BYTE( address );

	memcpy( __eeprom_info.msg + I2C_EEPROM_HEADER_SIZE, data_w, I2C_EEPROM_PAGE_SIZE );

	if( !__eeprom_info.write_cb )
		return I2CEEPROMER_NOCB_DEFINED;
	else
		return __eeprom_info.write_cb( __eeprom_info.address,
				                       __eeprom_info.msg,
				                       I2C_EEPROM_HEADER_SIZE+I2C_EEPROM_PAGE_SIZE );
}

// ************Read functions************
int M24xx32AReadByte( Uint16 address, Uint8 *value, bool *p_flag )
{
	if( address > I2C_EEPROM_LAST_REG )
		return I2CEEPROMER_END_OF_MEM;

	__eeprom_info.msg[0] = WORD_MOST_SIGN_BYTE( address );
	__eeprom_info.msg[1] = WORD_LESS_SIGN_BYTE( address );

	if( !__eeprom_info.read_cb )
		return I2CEEPROMER_NOCB_DEFINED;
	else
		return __eeprom_info.read_cb( __eeprom_info.address,
				                      __eeprom_info.msg, I2C_EEPROM_HEADER_SIZE,
				                      value, 1, p_flag );
}

int M24xx32AReadPage( Uint16 address, Uint8 *data_r, bool *p_flag )
{
	if( address > I2C_EEPROM_LAST_PAGE )
		return I2CEEPROMER_END_OF_MEM;
	if( address % I2C_EEPROM_PAGE_SIZE )
		return I2CEEPROMER_WRONG_PAGE;

	__eeprom_info.msg[0] = WORD_MOST_SIGN_BYTE( address );
	__eeprom_info.msg[1] = WORD_LESS_SIGN_BYTE( address );

	if( !__eeprom_info.read_cb )
		return I2CEEPROMER_NOCB_DEFINED;
	else
		return __eeprom_info.read_cb( __eeprom_info.address,
				                      __eeprom_info.msg, I2C_EEPROM_HEADER_SIZE,
	                                  data_r, I2C_EEPROM_PAGE_SIZE, p_flag );
}

int M24xx32AReadBuffer( Uint16 address, Uint8 *data_r, Uint8 n_bytes, bool *p_flag )
{
	if( ( address + (Uint16)n_bytes ) > I2C_EEPROM_LAST_REG )
		return I2CEEPROMER_END_OF_MEM;

	__eeprom_info.msg[0] = WORD_MOST_SIGN_BYTE( address );
	__eeprom_info.msg[1] = WORD_LESS_SIGN_BYTE( address );

	if( !__eeprom_info.read_cb )
		return I2CEEPROMER_NOCB_DEFINED;
	else
		return __eeprom_info.read_cb( __eeprom_info.address,
									  __eeprom_info.msg, I2C_EEPROM_HEADER_SIZE,
									  data_r, n_bytes, p_flag );
}


/*! \} */ //grouping

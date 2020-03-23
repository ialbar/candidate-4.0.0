/*!
 * \file store.h
 * \brief file to store configuration into FLASH memory
 */

#ifndef _STORE_H
#define _STORE_H

#include "24xx32a.h"

/*----defines----*/
#define STORE_BUFFER_LENGTH     256       /*!< size (16-bit words) of store buffer */
#define STORE_BUFFER16_LENGTH  (STORE_BUFFER_LENGTH*2)


/*!
 * \struct store_data
 * \brief struct to save direction of a variable
 */
typedef struct store_data
{
	void *data_ptr; /**< pointer to the configuration data*/
	Uint16 size;	/**< size in 16 bits' words */
}store_data;


/*--- Functions ---*/
void InitStore( void );
void save_configuration(void);
void load_configuration( void );
void clear_configuration(void);

int  get_configuration_from_eeprom(void);
void store_service_routine(void);

Uint16 store_get_status( void );
Uint16 store_get_fsm( void );
Uint16 store_get_nodeid( void );

/*! Store status */
enum
{
	STORE_INACTIVE   = 0x0000,
	STORE_WRITING    = 0x0001,
	STORE_READING    = 0x0002,
	STORE_ERROR      = 0x0003
};


/*! FSM states of the EEPROM accesses */
enum
{
	STORE_WRITE_START       = 0x0101,
	STORE_READ_START        = 0x0202,
	STORE_EEPROM_ACCESS     = 0x0303,
	STORE_EEPROM_WAIT_MESSAGE_PROCESSED = 0x0404,
	STORE_EEPROM_WAIT_BUSY  = 0x0505,
	STORE_WRITE_COMPLETED   = 0x0606,
	STORE_READ_COMPLETED    = 0x0707,
	STORE_EEPROM_ERROR      = 0x0808,
	STORE_PAGE_COMPLETED    = 0x0909,
	STORE_EEPROM_NODE_ERROR = 0x0A0A
};


#endif

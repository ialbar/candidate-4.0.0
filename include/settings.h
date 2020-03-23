/*!
 * \addtogroup dmc_ii
 * \{
 */

/*!
 * \file      settings.h
 * \brief     Header file for system definition. Included on drivers.
 * \author    Manuel Godoy
 * \version   V1R0.0
 * \copyright SEDECAL S.A.
 */


#ifndef SETTINGS_H_
#define SETTINGS_H_

// Operating System (UNCOMMENT ONE OF THE FOLLOWING)
#define OS_DSPBIOS

// Clocks
#define SYSCLK_MHz			 150
#define I2C_FRECUENCY_KHz    400
#define I2C_MAX_TRANSACTIONS 8    //!< 4 devices writing/reading at a time
#define I2C_MAX_TX_BYTES     34   //!< Memory page + address
#define I2C_MAX_RX_BYTES     32   //!< Memory page


#endif /* SETTINGS_H_ */

/*! \} */ //grouping

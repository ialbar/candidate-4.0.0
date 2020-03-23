/*!
 * \addtogroup dmc_ii
 * \{
 */

/*!
 * \file      typedef.h
 * \brief     Header file of Sedecal typedefs
 * \author    Manuel Godoy
 * \version   V1R0.0
 * \copyright SEDECAL S.A.
 * \attention Typedefs "by byte" doesn't work properly on DSPs
 * \remark    Use \ref WORD_MOST_SIGN_BYTE and \ref WORD_LESS_SIGN_BYTE
 */

#ifndef TYPEDEF_H_
#define TYPEDEF_H_

#include "DSP2833x_Device.h"

typedef unsigned char      bool;
typedef unsigned char      byte;
typedef unsigned int       word;
typedef unsigned long      dword;

// Typical function pointers
typedef void (*pfvoid) (void);         //!< pointer to a void return function, no params
typedef void (*pfvoid_pchar) (char *); //!< pointer to a void return function, char vector param
typedef void (*pfvoid_pbyte) (byte *); //!< pointer to a void return function, byte vector param

/*!
 * \brief Typedef to have the two words (MSW and LSW) from a double word
 */
typedef union {
	dword dw;     //!< double word
	word w[2];    //!< vector of words (MSW first, LSW second)
} dword_by_word;

/*********** MACROS FOR TYPES BY BYTES **********/
#define WORD_MOST_SIGN_BYTE( x ) (byte)( x >> 8 );     //!< Take the MSByte of a word
#define WORD_LESS_SIGN_BYTE( x ) (byte)( x % 0x0100 ); //!< Take the LSByte of a word

// Units and constants
#ifndef TRUE
#define TRUE	1
#endif

#ifndef FALSE
#define FALSE	0
#endif

#ifndef true
#define true	TRUE
#endif

#ifndef false
#define false	FALSE
#endif

#define	PERCENTAGE		100

#define SEG_TO_MS		1000
#define MS_TO_US		1000
#define MS_TO_NS		1000000
#define US_TO_NS		1000

// cm -> inches
#define CM_TO_INCHES	2.54
#define MM_TO_INCHES	25.4
#define INCHES_TO_CM	2.54
#define INCHES_TO_MM	25.4

// #PI
#define	PI				3.14
#define PI_100			314

#endif /* TYPEDEF_H_ */

/*! \} */ //grouping

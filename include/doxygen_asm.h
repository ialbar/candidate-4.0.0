/*!
 * \file  doxygen_asm.h
 * \brief Doxygen documentation for assembly code files
 * ****************************************************************************/

/* Doxygen documentation for assembly code files
================================================*/
/*!	\file CodeStartBranch.asm
	\brief Assembly functions that enable or disable Watchdog before function 'main'
*/

/*!	\file DelayUs.asm
	\brief Implements a time delay
*/

/*!	\file passwords.asm
	\brief Code required when using code security
*/

/*!	\file SetDBGIER.asm
	\brief Sets the DBGIER register (for realtime emulation)
*/

/*!	\file fir16.asm
	\brief 16 bit digital FIR filter
	
\verbatim
========================================================================

 File Name     : fir16.asm
 
 Originator    : Advanced Embeeded Control (AEC)
                 Texas Instruments Inc.
 
 Description   : This file contain source code for 16-bit FIR Filter
               
 Date          : 12/05/2002 (DD/MM/YYYY)
======================================================================
 
 Routine Name  : Generic Function      
 Routine Type  : C Callable
 
 Description   :
 void FIR16_calc(FIR16_handle) 
       
 This routine implements the non-recursive difference equation of an 
 all-zero filter(FIR), of order N. All the coefficients of all-zero 
 filter are assumed to be less than 1 in magnitude.
======================================================================

 Difference Equation :

       y(n)=H(0)*x(n)+H(1)*x(n-1)+H(2)*x(n-2)+....+H(N)*x(n-N)

      where
              y(n)=output sample of the filter at index n 
              x(n)=input sample of the filter at index n 

 Transfer Function :
                                  
              Y(z)                -1        -2               -N+1       -N
             ----- = h(0) + h(1) z  + h(2) z  + ... +h(N-1) z   + h(N) z    
              X(z)

     Network Diagram  : 
     dbuffer[0]          dbuffer[1]    dbuffer[2]    dbuffer[N}
     Input           -1  x(n-1)  -1    x(n-2)        x(n-N)
   x(n) >------o----z---->-o----z---->-o---  - ->- - o
               |           |           |             |
               |           |           |             |
               |           |           |             |
               v H(0)      v H(1)      v H(2)        v H(N)  
               |           |           |             |  
               |           |           |             |        output
               |---->-----(+)---->----(+)-- - -> -  (+)-----> y(n)    

       Symbols Used :
             H(0),H(1),H(2),...,H(N) : filter coefficients
            x(n-1),x(n-2),...,x(n-N) : filter states
                                x(n) : filter input 
                                y(n) : filter output
==============================================================================         
  Function Input: This function accepts the handle of the below structure

  typedef struct { 
      int *coeff_ptr;          --> Pointer to Filter co-efficient array
      int *dbuffer_ptr;        --> Delay buffer pointer 
      int cbindex;				--> Circular Buffer Index
      int order;               --> Order of the filter
      int input;               --> Input data
      int output;              --> Output data
      void (*init)(void *)     --> Pointer to init fun
      void (*calc)(void *);    --> Pointer to the calculation function
     }FIR16_handle;    
\endverbatim
*/

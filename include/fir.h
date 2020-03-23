/*!	\file fir.h
	\brief Header file containing  object definitions, proto type declaration and default object initializers for FIR Filter.

\verbatim
* File: fir.h
* Device: TMS320F28235
* Author: Advanced Embeeded Control (AEC)  Texas Instruments Inc.
* Description: Header file containing  object definitions, proto type declaration and default object initializers for FIR Filter.
\endverbatim
*/

#ifndef __FIR_H__
#define __FIR_H__
#define NULL    0
 
  
/*----------------------------------------------------------------
Define the structure of the FIRFILT_GEN Filter Module 
-----------------------------------------------------------------*/
typedef struct { 
    long *coeff_ptr;        /* Pointer to Filter coefficient */
    long * dbuffer_ptr;/* Delay buffer ptr              */
    int	cbindex;			/* Circular Buffer Index         */
    int order;              /* Order of the Filter           */
    int input;              /* Latest Input sample           */ 
    int output;             /* Filter Output                 */
    void (*init)(void *);   /* Ptr to Init funtion           */
    void (*calc)(void *);   /* Ptr to calc fn                */  
    }FIR16;    



           
/*---------------------------------------------------------------
Define a Handles for the Filter Modules
-----------------------------------------------------------------*/
typedef FIR16 	*FIR16_handle;

          
#define FIR16_DEFAULTS { (long *)NULL, \
             (long *)NULL,   \
             0,            \
             50,             \
             0,				\
             0,				\
             (void (*)(void *))FIR16_init,\
             (void (*)(void *))FIR16_calc}    


                                                       
/*-------------------------------------------------------------
 Prototypes for the functions
---------------------------------------------------------------*/
void FIR16_calc(void *);
void FIR16_init(void *);


/*********** Sample FIR Co-efficients **************************/
/* Even Order (50): LPF co-efficients for FIR16 module	*/
#define FIR16_LPF50 {\
			9839,-2219809,-1436900,853008,3340889,3668111,-896,-5963392,-8977456,-3669326,\
			8585216,18152991,13041193,-8257663,-30867258,-31522540,131,45285320,64028535,25231269,\
			-58654721,-124846025,-94830542,68157453,320667626,551550942}

			
/* Even Order (50): HPF co-efficients for FIR16 module	*/			
#define FIR16_HPF50 {\
			19646,55659,2290741,1640376,-2094815,-4259840,-1447,7011622,5440090,-6946006,\
			-13959169,64971,21102301,15728880,-19070655,-37027840,-214,53149590,39452755,-47841174,\
			-94765057,65471,153223136,129499161,-197853150,-647299072}

/* Even Order (50): HPF co-efficients for FIR16 module	*/
#define FIR16_BPF50 {\
			6543,65535,4581487,-1,-4124098,0,-2890,-1,10945715,-65536,\
			-27918337,65535,42139067,-1,-38075774,-65536,-427,65535,78839975,-1,\
			-189333505,0,306118593,-1,-395378619,-65536}
            
/* Even Order (50): BSF co-efficients for FIR16 module	*/            
#define FIR16_BSF50 {\
			26267,396754944,6054,-307232768,-4688,190054400,2900,-79167488,-1208,-65536,\
			-1,38141952,-64954,-42270721,-645,27983872,427,-11075584,-169,-65536,\
			-65537,4063231,62,-4587520,-65606,-1}

/* Odd Order (51): LPF co-efficients for FIR16 module	*/
#define FIR16_LPF51 {\
			-1956603,-2090327,-455862,2424384,4257913,2554383,-3145945,-8846556,-7928922,2163047,\
			15269587,18611676,3997368,-21495748,-35913445,-19726104,23527457,61276039,52756345,-14155825,\
			-99680218,-126287808,-29360092,189464569,447348704,621150178}

/* Odd Order (51): LPF co-efficients for FIR16 module	*/
#define FIR16_BPF51 {\
			-3010057,3272381,3469336,-3404284,-2749505,1242809,-1509064,5637229,11076189,-17432751,\
			-23789423,29032093,31718962,-30277149,-23264838,9567893,-11403531,39649449,74252374,-113704984,\
			-155647982,197132246,235208652,-266862540,-289603535,301465554}

#endif 


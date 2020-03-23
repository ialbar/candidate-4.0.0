
#ifndef DEBUG_SCI_H_
#define DEBUG_SCI_H_

//#define DEBUG_TEST_DATA

//#define DEBUG_CURRENT_LOOP_BLDC

#ifndef DEBUG_CURRENT_LOOP_BLDC
	#define DEBUG_POS_LOOP
	#define DEBUG_VEL_LOOP
	#define DEBUG_MANUAL_LOOP
#endif


#define FRAME_SCIDEBUG_SIZE_14BYTES 14
#define DEBUG_SCI_TO_DEBUG_LOOPS


typedef struct DebugSciData14bytes tDebugSciData14bytes;

extern void InitDataStructDebugSci(void);

extern void TickDebugSci14bytesPositionMode(void);

extern void TickDebugSci14bytesVelocityMode(void);

extern void TickDebugSci14bytesCurrentBLDC(void);

extern void TickDebugSci14bytesManualMode(void);

#endif //DEBUG_SCI_H_

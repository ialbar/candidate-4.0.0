#include "debug_sci.h"
#include "amc_od.h"
#include "amc.h"
#include "Gpio.h"
#include "IQmathLib.h"
#include "assisted_mode.h"
#include "position_mode.h"
#include "velocity_mode.h"
#include "peripherals.h"
#include "dc_bldc_current.h"
#include "manual_mode.h"
#include "quick_stop.h"

#define HEAD_FRAME 	 0x28; //"(" en ascii  40 in decimal
#define TAIL_FRAME 	 0x29; //")" en ascii  41 in decimal



struct DebugSciData14bytes
{
	unsigned char DataToSend[FRAME_SCIDEBUG_SIZE_14BYTES]; // Head frame/packet + u32 + Tail /frame/packet
};

tDebugSciData14bytes SciData14bytes;

static void SendData14bytes (unsigned long,unsigned long,unsigned long);

void InitDataStructDebugSci(void)
{

	unsigned short int i=0;
	for (i=0;i<FRAME_SCIDEBUG_SIZE_14BYTES;i++)
		SciData14bytes.DataToSend[i]=0;

}


void TickDebugSci14bytesManualMode(void)
{
	static unsigned long test=0;
	static unsigned long s1,s2,s3;
	float s3_f,s2_f,s1_f;


#if 1
	s1 = Position_demand_value;
	s2 = Position_actual_value;
	s3 = Velocity_actual_value;
#endif

#if 0
	s1_f = _IQtoF(manual_vel_control_pid.Ref);
	s2_f = _IQtoF(manual_vel_control_pid.Fdb);
	s3 	 = manual_mode_trans_dbg;
#endif


#if 0
	s1 = Debug_long5;
	s2 = Debug_long6;
	s3 = Debug_long3;
#endif

//	s1_f = _IQtoF(manual_pos_control_pid.Ref);
//	s2_f = _IQtoF(manual_pos_control_pid.Fdb);
//	s1 		= state_machine_manual_mode_dbg;
//	s2 		= manual_mode_trans_dbg;
//	s3 		= state_machine_manual_mode_dbg;
//	s3_f 	= _IQtoF(manual_vel_control_pid.Ki);

//    s1 = *((unsigned long*)&s1_f);
//    s2 = *((unsigned long*)&s2_f);
//    s3 = *((unsigned long*)&s3_f);


    SendData14bytes(s1,s2,s3);
}

void TickDebugSci14bytesPositionMode(void)
{
	static unsigned long test=0;
	static unsigned long s1,s2,s3;
	float s3_f,s2_f,s1_f;
#if 1
	s1 		= Position_demand_value;
	s2 		= Position_actual_value;
	s3 		= final_current;
#endif
#if 0
	s1_f 	= _IQtoF(current_control_pid.Ref);
	s2_f 	= _IQtoF(factor_current_down);
	s3_f 	= _IQtoF(current_control_pid.Out);

    s1 = *((unsigned long*)&s1_f);
    s2 = *((unsigned long*)&s2_f);
    s3 = *((unsigned long*)&s3_f);
#endif

    SendData14bytes(s1,s2,s3);
}



void TickDebugSci14bytesVelocityMode(void)
{
	static unsigned long test=0;
	static unsigned long s1,s2,s3;

	float s3_f,s2_f,s1_f;

#if 0
	s1 = Velocity_demand_value;
	s2 = Velocity_actual_value;
	s3 = final_current;//(short)(_IQtoIQ15(current_control_pid.Out));
#endif
#if 1
	s1_f 		= _IQtoF(pv_control_pid_mms.Ref);
	s2_f 		= _IQtoF(pv_control_pid_mms.Fdb);
	s3_f 		= _IQtoF(pv_control_pid_mms.Out);

	s1 = *((unsigned long*)&s1_f);
	s2 = *((unsigned long*)&s2_f);
	s3 = *((unsigned long*)&s3_f);
#endif

    SendData14bytes(s1,s2,s3);
}

void TickDebugSci14bytesCurrentBLDC(void)
{
	static unsigned long test=0;
	static unsigned long s1,s2,s3;
	float s3_f,s2_f,s1_f;

	s1_f 	= _IQtoF(current_control_pid.Ref);
	s2_f 	= _IQtoF(current_control_pid.Fdb);
	s3_f 	= _IQtoF(current_control_pid.Out);


    s1 = *((unsigned long*)&s1_f);
    s2 = *((unsigned long*)&s2_f);
    s3 = *((unsigned long*)&s3_f);

    SendData14bytes(s1,s2,s3);

}

static void SendData14bytes (unsigned long s1,unsigned long s2,unsigned long s3)
{
	SciData14bytes.DataToSend[0]=HEAD_FRAME;

	SciData14bytes.DataToSend[1]=(unsigned char)(s1&0x000000FF);
	SciData14bytes.DataToSend[2]=(unsigned char)((s1>>8)&0x00FF);
	SciData14bytes.DataToSend[3]=(unsigned char)((s1>>16)&0x00FF);
	SciData14bytes.DataToSend[4]=(unsigned char)((s1>>24)&0x00FF);

	SciData14bytes.DataToSend[5]=(unsigned char)(s2&0x000000FF);
	SciData14bytes.DataToSend[6]=(unsigned char)((s2>>8)&0x00FF);
	SciData14bytes.DataToSend[7]=(unsigned char)((s2>>16)&0x00FF);
	SciData14bytes.DataToSend[8]=(unsigned char)((s2>>24)&0x00FF);

	SciData14bytes.DataToSend[9]=(unsigned char)(s3&0x000000FF);
	SciData14bytes.DataToSend[10]=(unsigned char)((s3>>8)&0x00FF);
	SciData14bytes.DataToSend[11]=(unsigned char)((s3>>16)&0x00FF);
	SciData14bytes.DataToSend[12]=(unsigned char)((s3>>24)&0x00FF);

	SciData14bytes.DataToSend[13]=TAIL_FRAME;
#ifdef	DEBUG_SCI_TO_DEBUG_LOOPS
	if (flagDebugScia==0xAA)
    	SCI_send_debug(SciData14bytes.DataToSend);
#endif
}



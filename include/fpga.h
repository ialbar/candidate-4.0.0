/*
 * fpga.h
 *
 *  Created on: Aug 25, 2015
 *      Author: user
 */

#ifndef FPGA_H_
#define FPGA_H_


/* FPGA registers addresses */
#define FPGA_ADD_BOARD_ID0 			0x004000  //! Board Identifier. Ex: 3624
#define FPGA_ADD_BOARD_ID1 			0x004001  //! Board Version. Ex: 0002
#define FPGA_ADD_BOARD_ID2 			0x004002  //! Board Revision. Ex: 0041 --> A ASCII
#define FPGA_ADD_HDL_VER 			0x004003  //! HDL version. Ex: 1004 --> V1R0.04
#define FPGA_ADD_DNA0			    0x004004  //! FPGA DNA LSB
#define FPGA_ADD_DNA1			    0x004005  //! FPGA DNA
#define FPGA_ADD_DNA2			    0x004006  //! FPGA DNA
#define FPGA_ADD_DNA3			    0x004007  //! FPGA DNA MSB
#define FPGA_ADD_NODE_ID			0x004008  //! FPGA NODE ID from uSwitches
#define FPGA_ADD_CONF               0x004009  //! FPGA config register
                                          //! Bit 0,1: Working Mode (RD)
                                          //! Bit 2: Service HW (RD)
                                          //! Bit 3: Service SW (WR)
                                          //! Bit 4: Move Enable (RD) (In Safe and Service always ON)
										  //! Bit 5: Enable Remote (WR)
										  //! Bit 15: Request DSP reset (WR)
#define FPGA_ADD_WORKMODE_HEARTBEAT_CNT_LSB  0x00400A
#define FPGA_ADD_WORKMODE_HEARTBEAT_CNT_MSB  0x00400B
#define FPGA_ADD_NCE_CFG                     0x00400C
#define FPGA_ADD_NCE_MEASURE                 0x00400D
#define FPGA_ADD_ACOUSTIC_MODE               0x00400E
#define FPGA_ADD_MOVE_HEARTBEAT_CNT_LSB      0x00400F
#define FPGA_ADD_MOVE_HEARTBEAT_CNT_MSB      0x004010
#define FPGA_ADD_RESET_HEARTBEAT_CNT_LSB     0x004011
#define FPGA_ADD_RESET_HEARTBEAT_CNT_MSB     0x004012
#define FPGA_ADD_FIFO                        0x004013
#define FPGA_ADD_TRINQUETE_CFG               0x004014
#define FPGA_ADD_REMOTE_CONTROL_CODE         0x004015
#define FPGA_ADD_REMOTE_CONTROL_MASK_LSW     0x004016
#define FPGA_ADD_REMOTE_CONTROL_MASK_MSW     0x004017
#define FPGA_ADD_DAC_1                       0x004018
#define FPGA_ADD_DAC_2                       0x004019
#define FPGA_ADD_DAC_3                       0x00401A
#define FPGA_ADD_DAC_4                       0x00401B
#define FPGA_ADD_REPROG_CODE                 0x00401C
#define FPGA_ADD_FPGA_ERRORS                 0x00401E
#define FPGA_ADD_HDL_MBOOT_VER 			     0x00401F  //! HDL Multiboot version. Ex: 1004 --> V1R0.04
#define FPGA_ADD_DSP_BOOT_VERSION 			 0x004020
#define FPGA_ADD_DSP_BOOT_REVIEW 			 0x004021
#define FPGA_ADD_DSP_BOOT_SUBREVISION        0x004022
#define FPGA_ADD_DSP_BOOT_TAG                0x004023
//#define FPGA_ADD_CURRENT                     0x004020


/* FPGA configuration register bits */
#define FPGA_REG_CONF_WORKING_MODE_RD_desp        0
#define FPGA_REG_CONF_WORKING_MODE_RD_MASK   0x0003
#define FPGA_REG_CONF_SERVICE_HW_RD_desp          2
#define FPGA_REG_CONF_SERVICE_HW_RD_MASK     0x0004
#define FPGA_REG_CONF_SERVICE_SW_WR_desp          3
#define FPGA_REG_CONF_SERVICE_SW_WR_MASK     0x0008
#define FPGA_REG_CONF_MOVE_ENABLE_RD_MASK    0x0010
#define FPGA_REG_CONF_ENABLE_REMOTE_WR_MASK  0x0020
#define FPGA_REG_CONF_RESET_WR_MASK          0x8000

#define FPGA_REG_NCE_CFG_MODE_WR_desp          0
#define FPGA_REG_NCE_CFG_MODE_WR_MASK     0x0001
#define FPGA_REG_NCE_CFG_CHANNEL_WR_desp       1
#define FPGA_REG_NCE_CFG_CHANNEL_WR_MASK  0x0006

#define FPGA_REG_TRINQUETE_CFG_MODE_WR_desp       0
#define FPGA_REG_TRINQUETE_CFG_MODE_WR_MASK  0x0001 //!Bit 0: 1.Trinquete mode Disabled    0.Trinquete mode Enabled

#define FPGA_HEARTBEAT_WORKINGMODE_MASK        0x01
#define FPGA_HEARTBEAT_MASTER_COMM_MASK        0x02
#define FPGA_HEARTBEAT_ENABLE_MASK            (FPGA_HEARTBEAT_WORKINGMODE_MASK|FPGA_HEARTBEAT_MASTER_COMM_MASK)

#define FPGA_MOVE_WATCHDOG_TASK1_MASK         0x01
#define FPGA_MOVE_WATCHDOG_TASK2_MASK         0x02
#define FPGA_MOVE_WATCHDOG_ENABLE_MASK        (FPGA_MOVE_WATCHDOG_TASK1_MASK | FPGA_MOVE_WATCHDOG_TASK2_MASK)

#define FPGA_SYSTEM_WATCHDOG_TASK1_MASK        0x01
#define FPGA_SYSTEM_WATCHDOG_TASK2_MASK        0x02
#define FPGA_SYSTEM_WATCHDOG_ENABLE_MASK      (FPGA_SYSTEM_WATCHDOG_TASK1_MASK | FPGA_SYSTEM_WATCHDOG_TASK2_MASK)

#define FPGA_ENCODER_ERROR_RESET_MASK          0x03

/* FPGA Registers interface */
void InitFPGA( void );
void manageFPGA( void );
void Configure_FPGA_heartbeat( int mask, int value );
void Configure_FPGA_move_watchdog( int mask, int value );
void Configure_FPGA_system_watchdog( int mask, int value );
UNS32 FPGA_WriteConfigurationRegister( UNS16 mask, UNS16 val );
UNS32 FPGA_WriteGenericRegister( UNS16 mask, UNS16 val, UNS32 address);
void ReadFPGARegisters( void );
void WriteFPGARegisters( void );

void FPGA_SetNCE( int mode, int channel );
void FPGA_SetTrinquete( int mode );
void FPGA_SetService( int val );

//UNS16 FPGA_current_value(UNS16 val);

extern int FPGA_initialized;

#endif /* FPGA_H_ */

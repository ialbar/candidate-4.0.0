/*!	\file canopen.c
	\brief Callbacks and other application-dependant functions to handle CanOpen stack

\verbatim
*********************************************************************
* File: canopen.c
* Devices: TMS320F28XXX
* Author: Luis Jimenez.
* History:
*   02/02/07 - original
*********************************************************************
\endverbatim
*/

#include "DSP2833x_Device.h"

#include "amc.h"
#include "amc_od.h"
#include "store.h"
#include "fpga.h"
#include "tag.h"
#include "canopen_amc.h"

/*! Neccesary to initialize the node */
s_BOARD AmcBoard = {"0", CAN_SPEED};		/* 250kbps */

/*!	Initializes CanOpen stack
*/
void InitCanopen(void)
{
	if(strcmp(AmcBoard.baudrate, "none")){

		amc_od_Data.heartbeatError = amc_heartbeatError;
		amc_od_Data.initialisation = amc_initialisation;
		amc_od_Data.preOperational = amc_preOperational;
		amc_od_Data.operational = amc_operational;
		amc_od_Data.stopped = amc_stopped;
		amc_od_Data.post_sync = amc_post_sync;
		amc_od_Data.post_TPDO = amc_post_TPDO;
		amc_od_Data.storeODSubIndex = amc_storeODSubIndex;

		canOpen(&AmcBoard,&amc_od_Data);
	}
	// APedroso: 10/8/2016 forzamos la inicializaci�n de las variables est�ticas al m�dulo timer.c
	InitTimerCanFestival();

	StartTimerLoop(&InitNode);			// Start timer thread

	// initialize drive state
	setDriveState((unsigned short *)&Device_status_word, NOT_READY_TO_SWITCH_ON);
	// activate drive comunication
	setDriveState((unsigned short *)&Device_status_word, SWITCH_ON_DISABLED);
}


/*!	Init CANOpen node
*/
void InitNode(CO_Data* d, UNS32 id)
{
	RegisterSetODentryCallBack(&amc_od_Data, 0x1016, 1, &On_HeartbeatTimeUpdate);
	RegisterSetODentryCallBack(&amc_od_Data, 0x1017, 0, &On_HeartbeatTimeUpdate);
	RegisterSetODentryCallBack(&amc_od_Data, 0x6040, 0, &On_ControlwordUpdate);
	RegisterSetODentryCallBack(&amc_od_Data, 0x6060, 0, &On_ModesOfOperationUpdate);
	RegisterSetODentryCallBack(&amc_od_Data, 0x2021, 0, &On_ApplySensorsConfigurationUpdate);
	RegisterSetODentryCallBack(&amc_od_Data, 0x2230, 0, &On_PowerBoardType);
	RegisterSetODentryCallBack(&amc_od_Data, 0x608C, 0, &On_VelocityDimensionIndexUpdate);
	RegisterSetODentryCallBack(&amc_od_Data, 0x608E, 0, &On_AccelerationDimensionIndexUpdate);
	RegisterSetODentryCallBack(&amc_od_Data, 0x60FF, 0, &On_TargetVelocityUpdate);
	RegisterSetODentryCallBack(&amc_od_Data, 0x607A, 0, &On_TargetPositionUpdate);
	RegisterSetODentryCallBack(&amc_od_Data, 0x6073, 0, &On_CurrentLimitsUpdate);
	RegisterSetODentryCallBack(&amc_od_Data, 0x6075, 0, &On_CurrentLimitsUpdate);
	RegisterSetODentryCallBack(&amc_od_Data, 0x200B, 0, &On_CurrentLimitsUpdate);
	RegisterSetODentryCallBack(&amc_od_Data, 0x5FFF, 0, &On_ResetNodeUpdate);
	RegisterSetODentryCallBack(&amc_od_Data, 0x607C, 0, &On_HomeoffsetUpdate);
	RegisterSetODentryCallBack(&amc_od_Data, 0x60C1, 1, &On_InterpolationDataRecordUpdate);
	RegisterSetODentryCallBack(&amc_od_Data, 0x60C4, 6, &On_InterpolationDataConfigurationUpdate);
	RegisterSetODentryCallBack(&amc_od_Data, 0x5FFE, 0, &On_LaunchAMCLoaderUpdate);
	RegisterSetODentryCallBack(&amc_od_Data, 0x5FFD, 0, &On_Store_configuration);
	RegisterSetODentryCallBack(&amc_od_Data, 0xFF06, 1, &On_CurrentControlParameterUpdate);
	RegisterSetODentryCallBack(&amc_od_Data, 0xFF06, 2, &On_CurrentControlParameterUpdate);
	RegisterSetODentryCallBack(&amc_od_Data, 0xFF06, 3, &On_CurrentControlParameterUpdate);
	RegisterSetODentryCallBack(&amc_od_Data, 0xFF06, 4, &On_CurrentControlParameterUpdate);
	RegisterSetODentryCallBack(&amc_od_Data, 0xFF06, 5, &On_CurrentControlParameterUpdate);
	RegisterSetODentryCallBack(&amc_od_Data, 0xFF06, 10, &On_CurrentControlParameterUpdate);
	RegisterSetODentryCallBack(&amc_od_Data, 0xFF17, 3, &On_FPGAHeartbeatCounterLSBRegisterUpdate);
	RegisterSetODentryCallBack(&amc_od_Data, 0xFF17, 4, &On_FPGAHeartbeatCounterMSBRegisterUpdate);
	RegisterSetODentryCallBack(&amc_od_Data, 0xFF18, 2, &On_FPGAConfigurationRegisterUpdate);
	RegisterSetODentryCallBack(&amc_od_Data, 0xFF1B, 0, &On_FPGAAcousticModeRegisterUpdate);
	RegisterSetODentryCallBack(&amc_od_Data, 0xFF1C, 0, &On_WorkingModeUpdate);
	RegisterSetODentryCallBack(&amc_od_Data, 0x2120, 1, &On_AssistedCurrentDemandUpdate);
	RegisterSetODentryCallBack(&amc_od_Data, 0xFF32, 0, &On_LaunchFpgaLoaderUpdate);

	if(AmcBoard.baudrate) {
		/* Defining the node Id */
		setNodeId(&amc_od_Data, nodeID);
		/* init */
		setState(&amc_od_Data, Initialisation);
	}

	/* copy software version to the OD */
	publishSoftwareVersion();

	/* As char variables are 16 bits I make manually 1's all 8 most significant bits of negative numbers */
	Position_notation_index = (unsigned char)(((Position_notation_index + 128)%256) - 128);
	Velocity_notation_index = (unsigned char)(((Velocity_notation_index + 128)%256) - 128);
	Acceleration_notation_index = (unsigned char)(((Acceleration_notation_index + 128)%256) - 128);
}


/**********************************************************************
* Functions of the amc object dictionary declared in file canopen_amc.h
**********************************************************************/


/*! Function called by CanOpen stack when a heartbeat is not received after specified timeout
	\param heartbeatID ID of the node that caused heartbeat error
*/
void amc_heartbeatError(CO_Data* d, UNS8 heartbeatID)
{
	_LOGmessage(0x0116,"amc_heartbeatError", 0, 0);

	//resetNode();  //can NOT be done because configuration is lost
	setFault(FAULT_HEARTBEAT);
	QueueFault(FAULT_HEARTBEAT);

	_ERRORmessage(0x8130, 0x10, heartbeatID, "amc_heartbeatError %d", heartbeatID, 0);
    //JAF: Slave doesn't send PDOs when the Master doesn't send Heartbeats.
	PDOStop(d);
}

/*! Function called by CanOpen after initialisation */
void amc_initialisation(CO_Data* d)
{
	_LOGmessage(0x0003,"amc_initialisation", 0, 0);

	/* make some checks before automatically entering preOperational state */
	while(!ZeroCurrent_set)
	{
		LeaveMutex();
		TSK_sleep(2);	/* wait 1 ms */
		EnterMutex();

		if (CLK_getltime() > 200) {	/* limit wait time to 100 ms */
			_LOGmessage(0x0034,"ZeroCurrent initilized manually to null value", 0, 0);
			ZeroCurrent_motor = 0;  //683;
			ZeroCurrent_motor2 = 0;
			ZeroCurrent_set = 1;
		}
	}
}

/*! Function called by CanOpen when preOperational state is reached */
void amc_preOperational(CO_Data* d)
{
	_LOGmessage(0x0004,"amc_preOperational", 0, 0);
}

/*! Function called by CanOpen when Operational state is reached */
void amc_operational(CO_Data* d)
{
	int i;

	_LOGmessage(0x0005,"amc_operational", 0, 0);
	DeQueueFault(FAULT_HEARTBEAT);
//	DeQueueFault(FAULT_PASSIVE_CAN);
//  DeQueueFault(FAULT_BUSOFF_CAN);
	/*! According to CanFestival:
	 *  - OnChange PDOs are sent independently of the state. These PDOs can not be stopped
	 *  - Periodic PDOs are sent once the system reaches operational state. These PDOs can be stopped
	 *  The servo has 4 PDOs:
	 *  - PDO1 and PDO2 are configured to be sent on change
	 *  - PDO3 and PDO4 are configured to be sent periodically
	 *  So here only OnChange PDOs (PDO1 and PDO2) are sent to update the master with the current value
	 */
	for( i=0; i<1; i++ )
	{
		if( ! buildPDO( &amc_od_Data, i, &amc_od_Data.PDO_status[i].last_message ) )
			canSend( amc_od_Data.canHandle, &amc_od_Data.PDO_status[i].last_message );
	}

	//Dictionary callbacks have already been called
	FPGA_initialized = 1;
}

/*! Function called by CanOpen when CanOpen stack is stopped */
void amc_stopped(CO_Data* d)
{
	_LOGmessage(0x0006,"amc_stopped", 0, 0);
}

/*! Function called by CanOpen when a sync message has been post */
void amc_post_sync(CO_Data* d)
{
//	_LOGmessage(0x0007,"amc_post_sync", 0, 0);
}

/*! Function called by CanOpen when a TPDO message has been post */
void amc_post_TPDO(CO_Data* d)
{
//	_LOGmessage(0x0008,"amc_post_TPDO", 0, 0);
}

/*! Function called by CanOpen to store a value of the OD */
void amc_storeODSubIndex(CO_Data* d, UNS16 wIndex, UNS8 bSubindex)
{
	/*TODO :
	 * - call getODEntry for index and subindex,
	 * - save content to file, database, flash, nvram, ...
	 *
	 * To ease flash organisation, index of variable to store
	 * can be established by scanning d->objdict[d->ObjdictSize]
	 * for variables to store.
	 *
	 * */
	_LOGmessage(0x0009,"amc_storeODSubIndex : %4x %2x", wIndex, bSubindex);
}


/**********************************************************************
* CanOpen Object Dictionary callbacks
**********************************************************************/
/*! Callback called by CanOpen when ProducerHeartbeatTime or ConsumerHeartbeatTime OD entry are modified */
UNS32 On_HeartbeatTimeUpdate(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex)
{
	_LOGmessage(0x0115,"On_HeartbeatTimeUpdate", 0, 0);
	heartbeatStop(d);
	heartbeatInit(d);
    return 0;
}

/*! Callback called by CanOpen when Controlword OD entry is modified */
UNS32 On_ControlwordUpdate(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex)
{
	drive_state old_state;
	control_command cmd;

//	if( (FPGA_configuration_rd&3) != WORKING_MODE_OPERATIONAL )
//		return 0;
	old_state = getDriveState(Device_status_word);
	cmd = getCommand(Device_control_word);
	processCommand(cmd, old_state);

	return 0;
}

/*! Callback called by CanOpen when ModesOfOperation OD entry is modified */
UNS32 On_ModesOfOperationUpdate(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex)
{
//	if( (FPGA_configuration_rd&3) != WORKING_MODE_OPERATIONAL )
//		return 0;
	ATM_seti(&change_mode, 1);	/* mode of operation has to be changed */
	ATM_seti(&init_mode, 1);	/* mode of operation has to be initialized */
	return 0;
}


/*! Callback called by CanOpen when TargetVelocity OD entry is modified */
UNS32 On_TargetVelocityUpdate(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex)
{
	long target;

	/* update velocity target only if profile velocity mode and Operation_Enable state */
	if( (Modes_of_operation_display == OPERATION_MODE_VELOCITY) &&
		(getDriveState(Device_status_word) == OPERATION_ENABLE))
	{
		target = (long)Target_velocity;
		if(!MBX_post(&pv_newtarget_mbox, &target, 0)) _WARNINGmessage(0xfffa, 0x01, 0x00, "pv_newtarget_mbox full", 0, 0);
	}
	return 0;
}


/*! Callback called by CanOpen when VelocityDimension OD entry is modified (DEPRECATED)*/
UNS32 On_VelocityDimensionIndexUpdate(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex)
{
	_LOGmessage(0x000A,"Changes to Velocity Dimension have no effect", 0, 0);
	return 0;
}


/*! Callback called by CanOpen when AccelerationDimension OD entry is modified (DEPRECATED)*/
UNS32 On_AccelerationDimensionIndexUpdate(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex)
{
	_LOGmessage(0x000B,"Changes to Velocity Acceleration have no effect", 0, 0);
	return 0;
}


/*! Callback called by CanOpen when TargetPosition OD entry is modified */
UNS32 On_TargetPositionUpdate(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex)
{
	long target;

	/* update position target only if profile position mode and Operation_Enable state */
	if( (Modes_of_operation_display == OPERATION_MODE_POSITION) &&
		(getDriveState(Device_status_word) == OPERATION_ENABLE))
	{
		/* set new target only if New set-point bit is set */
		if(Device_control_word & NEW_SET_POINT)
		{
			/* set target if absolute or relative reference */
			target = (Device_control_word & ABS_REL)? (long)Target_position + (long)Position_actual_value : (long)Target_position;
			if(Device_control_word & CHANGE_IMMEDIATELY)
			{
				if(!MBX_post(&pp_newtarget_mbox, &target, 0))
					_WARNINGmessage(0xfff1, 0x20, 0x0000, "pp_newtarget_mbox full", 0, 0);
			} else
			{
				if(!MBX_post(&pp_nexttarget_mbox, &target, 0))
					_WARNINGmessage(0xfff2, 0x20, 0x0000, "pp_nexttarget_mbox full", 0, 0);
			}
		}
	}
	return 0;
}

/*! Callback called by CanOpen when Power_board OD entry is modified */
UNS32 On_PowerBoardType(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex)
{
	configurePowerBoardValues();
	return 0;
}

void CheckAndApplyMotorConfiguration( void )
{
	if (Apply_sensors_configuration == 1)
	{
		if (checkPositioningSensorsConfig() && checkFactors())
		{
			configurePositioningSensors();
			configurePowerBoardValues();
			updateFactors();
			Sensors_configuration_active = 1;
		}
		else
		{
			Sensors_configuration_active = 0;
		}
	}
	else
	{
		Sensors_configuration_active = 0;
	}
}

/*! Callback called by CanOpen when Apply_encoder_configuration OD entry is modified */
UNS32 On_ApplySensorsConfigurationUpdate(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex)
{
	CheckAndApplyMotorConfiguration();
	return 0;
}


/*! Callback called by CanOpen when Max_current or Motor_rated_current are modified. This function sets current_limit (mA) according to them */
UNS32 On_CurrentLimitsUpdate(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex)
{
	long tmp;

	tmp = Motor_rated_current * Max_current;
	tmp /= 1000;
	current_limit = minimum(tmp, Servo_rated_current);

	tmp = Motor_rated_current * Max_acceleration_current;
	tmp /= 1000;
	current_limit_acc = minimum(tmp, Servo_rated_current);

	return 0;
}


/*! Callback called by CanOpen when Reset_node is modified. */
UNS32 On_ResetNodeUpdate(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex)
{
	if(Node_reset)
		#ifdef _DEBUG
			slaveSendBootUp(d);
		#else
			resetNode();	/* reset the DSP if 1 */
		#endif
	return 0;
}


/*! Callback called by CanOpen when Home_offset value is modified */
UNS32 On_HomeoffsetUpdate(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex)
{
	new_redundancy((void *)NULL);
	//redundancy_test(1, (void *)NULL);
	//hybrid_redundancy_test(1, (void *)NULL);

	return 0;
}


/*! Callback called by CanOpen when there is a new data in Interpolation_data_record (subindex1) */
UNS32 On_InterpolationDataRecordUpdate(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex)
{
	if ((ip_available < IP_BUFFER_SIZE) && (Interpolation_data_configuration_Buffer_clear == 1)) {
		ip_buffer[ip_iwrite].pos = Interpolation_data_record[0];
		ip_buffer[ip_iwrite].vel = Interpolation_data_record[1];
		ip_iwrite = (ip_iwrite + 1)%IP_BUFFER_SIZE;
		++ip_available;
	} else _WARNINGmessage(0xffe4, 0x80, 0, "Interpolation buffer full, data ignored", 0, 0);

	return 0;
}


/*! Callback called by CanOpen when Interpolation_configuration_update is modified (only when buffer is cleared) */
UNS32 On_InterpolationDataConfigurationUpdate(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex)
{
	switch(unsused_bSubindex) {
		case 6:		/* buffer clear */
			if (Interpolation_data_configuration_Buffer_clear == 0) {
				ip_iwrite = 0;
				ip_iread = 0;
				ip_available = 0;
			}
			break;
		default:
		  break;
	}
	return 0;
}


/*! Callback called when Launch_AMC_loader is modified to launch AMC-loader application */
UNS32 On_LaunchAMCLoaderUpdate(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex)
{
	if (Launch_AMC_loader == 0x55AA) launch_amc_loader();
	return 0;
}

/*! Callback called when Launch_Fpga_loader is modified to launch AMC-loader application to update Flash's FPGA*/
UNS32 On_LaunchFpgaLoaderUpdate(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex)
{
	volatile unsigned *pointer;
	if (Launch_FPGA_Loader == 0x1122)
	{
		pointer = (unsigned *)FPGA_ADD_REPROG_CODE;
		(*pointer) = (UNS16) (0x0088);
		launch_fpga_loader();
	}
	return 0;
}


/*! Callback called when Store_configuration is modified to save configuration
 *  into conf_buffer. From LSB to MSB:
 *	If "save" is writen, configuration is saved.
 *  If "clr" is writen, only zeros are saved in configuration flash memory.
 */
UNS32 On_Store_configuration(CO_Data* d,
							 const indextable * unsused_indextable,
							 UNS8 unsused_bSubindex)
{
	/* Save configuration if "save" in ASCII received */
	if (Store_configuration == 0x65766173) save_configuration();
	else if (Store_configuration == 0x00726C63) clear_configuration();
	return 0;
}

/*! Callback called by CanOpen when Current_Control_Parameter is modified (only when buffer is cleared) */
UNS32 On_CurrentControlParameterUpdate(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex)
{
	switch(unsused_bSubindex)
	{
		case 1:
			Kp = _IQdiv_Current(_IQCurrent(Current_control_parameter_set_Kp),_IQCurrent(Current_control_parameter_set_Divisor));
			break;
		case 2:
			Ki = _IQdiv_Current(_IQCurrent(Current_control_parameter_set_Ki),_IQCurrent(Current_control_parameter_set_Divisor));
			break;
		case 3:
			Kc = _IQdiv_Current(_IQCurrent(Current_control_parameter_set_Kc),_IQCurrent(Current_control_parameter_set_Divisor));
			break;
		case 4:
			Kff = _IQdiv_Current(_IQCurrent(Current_control_parameter_set_Kff),_IQCurrent(Current_control_parameter_set_Divisor));
			break;
		case 5:
			Kp =  _IQdiv_Current(_IQCurrent(Current_control_parameter_set_Kp ),_IQCurrent(Current_control_parameter_set_Divisor));
			Ki =  _IQdiv_Current(_IQCurrent(Current_control_parameter_set_Ki ),_IQCurrent(Current_control_parameter_set_Divisor));
			Kc =  _IQdiv_Current(_IQCurrent(Current_control_parameter_set_Kc ),_IQCurrent(Current_control_parameter_set_Divisor));
			Kff = _IQdiv_Current(_IQCurrent(Current_control_parameter_set_Kff),_IQCurrent(Current_control_parameter_set_Divisor));
			break;
		case 10:
			if(Update_current_control_parameter_refs == 1)
			{
				Id_ref_off = _IQCurrent(Current_control_parameter_set_Id_ref_off);
				Iq_ref_off = _IQCurrent(Current_control_parameter_set_Iq_ref_off);
				Ud_ref_off = _IQCurrent(Current_control_parameter_set_Ud_ref_off);
				Uq_ref_off = _IQCurrent(Current_control_parameter_set_Uq_ref_off);
				Update_current_control_parameter_refs = 0;
			}
			break;
		default:
			break;
	}
	return 0;
}






UNS32 On_FPGAConfigurationRegisterUpdate (CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex)
{
	volatile unsigned *pointer2;

	if( !FPGA_initialized ) return 0;
	pointer2 = (unsigned *)FPGA_ADD_CONF;
	(*pointer2) = (UNS16) FPGA_configuration_wr;

	return 0;
}

UNS32 On_FPGAHeartbeatCounterLSBRegisterUpdate (CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex)
{
	volatile unsigned *pointer;

	if( !FPGA_initialized ) return 0;
	pointer = (unsigned *)FPGA_ADD_WORKMODE_HEARTBEAT_CNT_LSB;
	(*pointer) = (UNS16) FPGA_HB_counter_lsb_wr;
	return 0;
}

UNS32 On_FPGAHeartbeatCounterMSBRegisterUpdate (CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex)
{
	volatile unsigned *pointer;

	if( !FPGA_initialized ) return 0;
	pointer = (unsigned *)FPGA_ADD_WORKMODE_HEARTBEAT_CNT_MSB;
	(*pointer) = (UNS16) FPGA_HB_counter_msb_wr;
	return 0;
}

UNS32 On_FPGAAcousticModeRegisterUpdate (CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex)
{
	volatile unsigned *pointer;

	if( !FPGA_initialized ) return 0;
	pointer = (unsigned *)FPGA_ADD_ACOUSTIC_MODE;
	(*pointer) = (UNS16) FPGA_acoustic_mode_wr;
	return 0;
}

UNS32 On_WorkingModeUpdate (CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex)
{
	if( !FPGA_initialized ) return 0;
	switch( Working_mode_wr )
	{
		case WORKING_MODE_SAFE:
			Configure_FPGA_heartbeat( FPGA_HEARTBEAT_WORKINGMODE_MASK, 0x00 );
			break;

		case WORKING_MODE_SERVICE:
			FPGA_WriteConfigurationRegister( FPGA_REG_CONF_SERVICE_SW_WR_MASK, 0 );
			Configure_FPGA_heartbeat( FPGA_HEARTBEAT_WORKINGMODE_MASK, FPGA_HEARTBEAT_WORKINGMODE_MASK );
			break;

		case WORKING_MODE_OPERATIONAL:
			FPGA_WriteConfigurationRegister( FPGA_REG_CONF_SERVICE_SW_WR_MASK, FPGA_REG_CONF_SERVICE_SW_WR_MASK );
			Configure_FPGA_heartbeat( FPGA_HEARTBEAT_WORKINGMODE_MASK, FPGA_HEARTBEAT_WORKINGMODE_MASK );
			break;
	}
	return 0;
}


/*! Callback called by CanOpen when Assisted_mode_current_demand OD entry is modified */
UNS32 On_AssistedCurrentDemandUpdate(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex)
{
	return 0;
}
/*===========================================================================
	End of SourceCode.
===========================================================================*/

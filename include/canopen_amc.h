/*!	\file canopen_amc.h
	\brief Header file containing  definitions of callbacks called from CanFestival.
*/

#include "amc_od.h"

void amc_heartbeatError(CO_Data* d, UNS8);

UNS8 amc_canSend(Message *);

void amc_initialisation(CO_Data* d);
void amc_preOperational(CO_Data* d);
void amc_operational(CO_Data* d);
void amc_stopped(CO_Data* d);

void amc_post_sync(CO_Data* d);
void amc_post_TPDO(CO_Data* d);
void amc_storeODSubIndex(CO_Data* d, UNS16 wIndex, UNS8 bSubindex);

void InitCanopen();
void InitNode(CO_Data* d, UNS32 id);
UNS32 On_HeartbeatTimeUpdate(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex);
UNS32 On_ControlwordUpdate(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex);
UNS32 On_ModesOfOperationUpdate(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex);
UNS32 On_ApplySensorsConfigurationUpdate(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex);
UNS32 On_PowerBoardType(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex);
UNS32 On_VelocityDimensionIndexUpdate(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex);
UNS32 On_AccelerationDimensionIndexUpdate(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex);
UNS32 On_TargetVelocityUpdate(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex);
UNS32 On_TargetPositionUpdate(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex);
UNS32 On_CurrentLimitsUpdate(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex);
UNS32 On_ResetNodeUpdate(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex);
UNS32 On_HomeoffsetUpdate(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex);
UNS32 On_InterpolationDataRecordUpdate(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex);
UNS32 On_InterpolationDataConfigurationUpdate(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex);
UNS32 On_LaunchAMCLoaderUpdate(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex);
UNS32 On_Store_configuration(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex);
UNS32 On_Store_configuration(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex);
UNS32 On_CurrentControlParameterUpdate(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex);
UNS32 On_FPGAConfigurationRegisterUpdate (CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex);
UNS32 On_FPGAHeartbeatCounterLSBRegisterUpdate (CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex);
UNS32 On_FPGAHeartbeatCounterMSBRegisterUpdate (CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex);
UNS32 On_FPGAAcousticModeRegisterUpdate (CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex);
UNS32 On_WorkingModeUpdate (CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex);
UNS32 On_AssistedCurrentDemandUpdate(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex);
UNS32 On_LaunchFpgaLoaderUpdate (CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex);

void updateFactors();
int dimensionalOrder(unsigned long *num, unsigned long *den, int index_num, int index_div);

void CheckAndApplyMotorConfiguration( void );


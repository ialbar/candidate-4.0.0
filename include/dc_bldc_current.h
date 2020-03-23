#ifndef DC_BLDC_CURRENT_MODE_H_
#define DC_BLDC_CURRENT_MODE_H_


#include "amc.h"
#include "pid_reg.h"
#include "stdint.h"

extern PIDREG current_control_pid;	/*!< PID struct for current control */

extern void set_Kp_current_control_dc_bldc(uint16_t Kp,uint16_t divisor);
extern void set_Ki_current_control_dc_bldc(uint16_t Kp,uint16_t divisor);
extern void set_Kd_current_control_dc_bldc(uint16_t Kp,uint16_t divisor);
extern void set_Kc_current_control_dc_bldc(uint16_t Kp,uint16_t divisor);

extern void init_current_controller_dc_bldc(void);
extern void init_current_controller_dc_bldc_from_assited_mode(void);
extern _iq current_controller_dc_bldc(_iq, _iq);

#endif  /* end DC_BLDC_CURRENT_MODE_H_ definition */

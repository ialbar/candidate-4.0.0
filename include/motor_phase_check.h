/*!	\file motor_phase_check.h
	\brief Definitions of the motor phase check module.
*/
#ifndef MOTOR_PHASE_CHECK_H_
#define MOTOR_PHASE_CHECK_H_

/*! possible device states */
typedef enum {
	IDLE,						/*!< initial state */
	CHECKING_PHASE_1,			/*!< checking first two phases */
	WAITING_ZEROCURRENT,		/*!< wait until current is zero */
	CHECKING_PHASE_2,			/*!< checking third phase */
	CHECKED_ERROR,				/*!< error occurred during the check */
	CHECKED_OK,					/*!< motor phases checked */
	SKIP_CHECK					/*!< skip the motor phases check */
} motor_phase_check_state;

extern void motor_phase_check(char);
extern motor_phase_check_state get_motor_phase_check_state(void);
extern void set_motor_phase_check_start(void);
extern void apply_motor_phase_check(void);
extern unsigned char motor_phase_check_running(void);
extern void skip_motor_phase_checking(void);

#endif //MOTOR_PHASE_CHECK_H_

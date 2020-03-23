/*!	\file interpolated_mode.h
	\brief Header file containing declarations for Interpolated Position Mode
*/

#ifndef _INTERPOLATED_MODE_H_
#define _INTERPOLATED_MODE_H_

/*! interpolated mode states */
typedef enum {
	IP_INITIALIZING,		/*!< The axle is stopping */
	IP_READY,				/*!< The servo is waiting for a "enable ip mode" command */
	IP_ACTIVE				/*!< Interpolation mode is active, the axle is moving */
} ip_state_t;

/* defines for argument "initial" of ip_trajectory_generator */
#define IP_INITIAL_TRAJECTORY 1		/*!< Initial trajectory */
#define IP_NORMAL_TRAJECTORY 0		/*!< Normal trajectory */
/* defines for return value of ip_trajectory_generator */
#define IP_TRAJ_OK 0		/*!< Trajectory created successfully */
#define IP_TRAJ_END -1	/*!< Last point of trajectory reached */
#define IP_TRAJ_NOPOINTS_ERR -2	/*!< Not enough points for the trajectory */
#define IP_TRAJ_START_ERR -3	/*!< Error in the start point of the trajectory (velocity not null or position different from current */
#define IP_TRAJ_POS_ERR -4	/*!< Error in the position of the trajectory points */
#define IP_TRAJ_VEL_ERR -5	/*!< Error in the velocity of a time slice of the trajectory (maybe > Max_profile_velocity) */
#define IP_TRAJ_ACC_ERR -6	/*!< Error in the acceleration of a time slice of the trajectory (maybe > Max_acceleration) */
#define IP_TRAJ_CONF_ERR -7 /*!< Error in the configuration of the iterpolation mode */
/* defines for "start" argument in ip_stop_trajectory */
#define IP_START_FROM_CURRENT 0	/*!< The stop trajectory generated will start from current state */
#define IP_START_FROM_DEMAND 1	/*!< The stop trajectory generated will start from demand state */


/*! struct that defines trajectories in Interpolated Position Mode 

Between two fixed points of the trajectory the most general expression of the demanded position will be:
\verbatim x = a*(t-t0)^3 + b*(t-t0)^2 + c*(t-t0) + d \endverbatim
This is equivalent to a movement with constant jerk (time derivative of the acceleration):
\verbatim x = 1/6*jerk*(t-t0)^3 + 1/2*acc_0*(t-t0)^2 + vel_0*(t-t0) + pos_0 \endverbatim
Parameters a, b, c and d are used for cubic splines interpolation, but only c and d are used for linear interplation (a = b = 0)
*/
typedef  struct {
	long long t0;		/*!< Init time of the time slice between two fixed points */
	long long t1;		/*!< End time of the time slice between two fixed points */
	long a;				/*!< jerk/6 */
	long b;				/*!< initial acceleration/2 */
	long c;				/*!< initial velocity */
	long d;				/*!< initial position */
	} interpolated_trajectory_struct;	/*!< x = x = a*(t-t0)^3 + b*(t-t0)^2 + c*(t-t0) + d */

	
/*! Struct to define everty point specified by the master in the Interpolated position mode (in external units)*/
typedef struct {
	long pos;	/*!< Position [position units] */
	long vel;	/*!< Velocity (unused in linear interpolation mode) [velocity units] */
	} ip_point;
	
/*! Size of the buffer that stores the fixed points in the Interpolated position mode */
#define IP_BUFFER_SIZE 256


/* extern variables */
extern unsigned int ip_iread;			/*!< Index of the next element in the ip_buffer to be read */
extern unsigned int ip_iwrite;		/*!< Index of the next element in the ip_buffer to be written */
extern unsigned int ip_available;	/*!< Number of points available in the interpolation buffer */
extern ip_point ip_buffer[IP_BUFFER_SIZE];	/*! buffer for fixed points in the Interpolated position mode */


/* function declarations */
void ip_mode_operation(motion_state_struct *state, char init);
void ip_initialize(interpolated_trajectory_struct *trajectory, motion_state_struct *state);
void ip_stop_trajectory(interpolated_trajectory_struct *trajectory, motion_state_struct *state, char start);
long ip_trajectory_point(interpolated_trajectory_struct *trajectory, long long current_time);
unsigned int ip_get_period_time(unsigned char units, char index);
char ip_trajectory_generator(unsigned char initial, interpolated_trajectory_struct *trajectory, long long current_time, motion_state_struct *state);
void ip_trajectory_interval_cubic(interpolated_trajectory_struct *trajectory, unsigned int time, ip_point *point_a, ip_point *point_b);
void ip_trajectory_interval_linear(interpolated_trajectory_struct *trajectory, unsigned int time, ip_point *point_a, ip_point *point_b);
char ip_traj_check_cubic(char start, unsigned int time, ip_point *point_a, ip_point *point_b);
char ip_traj_check_linear(char start, unsigned int time, ip_point *point_a, ip_point *point_b);
char ip_configure(void);
void ip_status_flags(long int position, long int demand, ip_state_t ip_state, motion_state_struct *state, int reset);
long ip_trajectory_vel(interpolated_trajectory_struct *trajectory, long long current_time);
void ip_current_limit(interpolated_trajectory_struct *trajectory);

#endif  /* end _INTERPOLATED_MODE_H_ definition */

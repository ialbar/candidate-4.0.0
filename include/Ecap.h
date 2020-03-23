/*!
 * \file  Ecap.h
 * \brief Header file of the functions for the capture modules.
 * ****************************************************************************/

/* Functions to initilialize capture modules */
void InitECapture(void);
void InitECap4PWMSensor( unsigned short resolution, unsigned char polarity );

/* Functions to be called by an interrupt to get cap values (see DefaultISR.c)*/
void pwm_in1_capture   ( void );
void pwm_in2_capture   ( void );
void pwm_in3_capture   ( void );
void pwm_sensor_capture( void );

/* Functions to calculate values from capture data */
unsigned short calculate_pwm_in1_value   ( void );
unsigned short calculate_pwm_in2_value   ( void );
unsigned short calculate_pwm_in3_value   ( void );
void calculate_pwm_sensor_value( unsigned short *value, unsigned long *per );

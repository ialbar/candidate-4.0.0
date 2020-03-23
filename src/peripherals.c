/*!	\file peripherals.c
	\brief Functions related to peripherals configuration and encoder configuration modes

\verbatim
*********************************************************************
* File: peripheral.c
* Devices: TMS320F28234
* Authors: Luis Jimenez & Alvaro Garcia.
* History:
*   06/06/2007 - original
*   21/10/2009 - rewritten for AMC2
*********************************************************************

// TODO: description of peripherals config

\endverbatim
*/


#include "amc.h"
#include "DSP2833x_Device.h"
#include "peripherals.h"
#include "fpga.h"

/* local function declarations */
int manageGPI(void);
void manageGPO(void);
void manageAnalog(void);
int manageGPIO(void);
void manageEnc(void);
void manageHall(void);
void manageResolver(void);
void manageHybrid(void);
void managePWM(void);
long get_Enc1(char params);
long get_Enc2(char params);
long get_AN1(char params);
long get_AN2(char params);
long get_AN3(char params);
long get_AN4(char params);
long get_PWM_encoder(char params);
long get_MAG_encoder(char params);
long get_resolver_acc(char params);
long get_hybrid(char params);
unsigned get_AN_vpower_24vslim(void);
short get_AN_current_24vslim(void);
short get_AN_null_current2(void);
short get_AN_current2_24vslim(void);
short get_AN_temp_24vslim(void);
unsigned get_AN_vpower_300Vpowerboard(void);
unsigned get_AN_vpower_300V_C_powerboard(void);
unsigned get_AN_vpower_300V_02_powerboard(void);
unsigned get_AN_vpower_24V_02_powerboard(void);
short get_AN_current_300Vpowerboard(void);
short get_AN_current2_300Vpowerboard(void);
short get_AN_current_300V_C_powerboard( void );
short get_AN_current2_300V_C_powerboard(void);
short get_AN_current_24V_03_powerboard( void );
short get_AN_current2_24V_03_powerboard(void);
short get_AN_temp_300Vpowerboard(void);
short get_AN_temp_300V_C_powerboard(void);
short get_AN_temp_KTY_motor_sensor( void );
unsigned get_AN_vpower_emuboard(void);
short get_AN_current_emuboard(void);
short get_AN_temp_emuboard(void);
unsigned get_PWM_voltage(void);
short get_PWM_current(void);
short get_PWM_temp(void);
int get_temperature_from_table(unsigned ntcADC, const adc_temp_table_t *table);
long l_get_null(char init);
short s_get_null(void);
unsigned short us_get_null(void);
unsigned us_int_get_null(void);
int manage_shift_reg(unsigned bits);
int manage_non_contacting_abs_enc( void);

int checkP_ENC(UNS8 enc);
int checkP_ANALOG(UNS8 aninput);
int checkP_PWM_ENC(void);
int checkP_MAG_ENC(void);
int checkP_ZERO(UNS8 Encoder_configuration_ElementX);
char used_by_control(char peripheral);

short calculate_resolver_value(void);
unsigned calculate_hybrid_value(void);


extern int homing_zeroed;

short Vgain_num, Vgain_den;

const adc_temp_table_t adcTable_amc1 = { 30,
                                         {149, 169, 192, 220, 251, 287, 330, 379, 437, 504, 582, 673, 778, 899, 1038, 1195,\
                                          1372, 1568, 1782, 2013, 2255, 2504, 2755, 3001, 3234, 3451, 3646, 3817, 3963, 4086},
                                         {1250, 1200, 1150, 1100, 1050, 1000, 950, 900, 850,\
                                          800, 750, 700, 650, 600, 550, 500, 450, 400, 350, 300,\
                                          250, 200, 150, 100, 50, 0, -50, -100, -150, -200}};
const adc_temp_table_t temp_table_24vslim = { 30,
                                              {185, 207, 233, 262, 296, 335, 380, 432, 491, 559, 636, 725,\
                                               827, 942, 1073, 1219,1382, 1561, 1756, 1965, 2185, 2414, 2645,\
                                               2875, 3098, 3310, 3505, 3682, 3838, 3973},
                                              {1250, 1200, 1150, 1100, 1050, 1000, 950, 900, 850,\
                                               800, 750, 700, 650, 600, 550, 500, 450, 400, 350, 300,\
                                               250, 200, 150, 100, 50, 0, -50, -100, -150, -200}};

//const adc_temp_table_t temp_table_300v = { 5, //TODO: Verificar Medida NTC
//					   {    0, 1024, 2048, 3072, 4095},
//					   { 1500, 1000,  750,  500,  250}};
const adc_temp_table_t temp_table_300v = { 14,
					 { 2068, 2136, 2199, 2263, 2333, 2391, 2457, 2522, 2585, 2650,\
					   3161, 3677, 3870, 4180},
					 { 1200, 1120, 1080, 1040,  990,  950,  900,  860,  830,  790,\
					    600,  440,  380,  300}};
const adc_temp_table_t temp_table_300v_C = { 4, //TODO: Verificar Medida NTC
					   { 1171, 2183, 2982, 3770},  //ADC counts
					   {  250,  600,  900, 1180}}; //��C x 10
const adc_temp_table_t temp_table_kty_motor = {   5,
											  {   0, 623, 751, 1390, 4095},
											  {-1220,  0, 250, 1500, 6800} };

long (*position_counter)(char init) = l_get_null;   /*! Pointer to the function that will return the position */
long (*velocity_counter)(char init) = l_get_null;   /*! Pointer to the function that will return the velocity */
long (*abs_pos_counter)(char init) = l_get_null;    /*! Pointer to the function that will return an absolute position value */
long (*red_pos_counter)(char init) = l_get_null;    /*! Pointer to the function that will return a redundancy position value */
long (*hyb_pos_counter)(char init) = l_get_null;    /*! Pointer to the function that will return a redundancy position value */

unsigned (*get_vpower) ( void ) = us_int_get_null;
short (*get_current)( void ) = s_get_null;
short (*get_current2)( void ) = s_get_null;
short (*get_temp)   (  void  ) = s_get_null;
short (*get_motor_temp)  ( void) = s_get_null;

long velocity_counter_filter =0;

short int vel_calc=0;

/*****************************************************************/
/*!	Checks encoder configuration mode
	\return '1' if everything is ok, '0' if not
*/
/*****************************************************************/
int checkPositioningSensorsConfig(void)
{
	char ok = 1;

	/* check velocity sensor */
	ok &= 1;

	/* check position sensor */
	ok &= 1;

	/* check additional position sensor */
	ok &= 1;

	/* analog sensors can only be used in external units */

	return ok;
}


/*****************************************************************/
/*!	Checks that all the necessary parameters for
	position/velocity/acceleration factors are configured
	\return '1' if everything is ok, '0' if not
*/
/*****************************************************************/
int checkFactors(void)
{
	if(!Gear_ratio_Motor_revolutions || !Gear_ratio_Shaft_revolutions)
	{
		_WARNINGmessage(0xffee, 0x20, 0x6091, "Wrong value in OD, index: %x", 0x6091, 0);
		return 0;
	}

	/* As char variables are 16 bits I make manually 1's all 8 most significant bits of negative numbers */
	Position_notation_index = (unsigned char)(((Position_notation_index + 128)%256) - 128);
	Velocity_notation_index = (unsigned char)(((Velocity_notation_index + 128)%256) - 128);
	Acceleration_notation_index = (unsigned char)(((Acceleration_notation_index + 128)%256) - 128);

	/* Only meters (0x01), radians (0x10), and degrees (0x41) are allowed */
	if (!(Position_dimension_index == 0x01) && !(Position_dimension_index == 0x10) && !(Position_dimension_index == 0x41))
	{
		_WARNINGmessage(0xffee, 0x20, 0x608A, "Wrong value in OD, index: %x", 0x608A, 0);
		return 0;
	}

	/* if radians are used Feed_constant is set as 2*pi */
	if (Position_dimension_index == 0x10)
	{
		Feed_constant_Feed = 6283;		/* (2 * pi * 1000) */
		Feed_constant_Shaft_revolutions = 1;
		dimensionalOrder(&Feed_constant_Feed, &Feed_constant_Shaft_revolutions, -3, (int)Position_notation_index);
	}

	/* if degrees are used Feed_constant is set as 360 */
	if (Position_dimension_index == 0x41)
	{
		Feed_constant_Feed = 3600;		/* (360 * 10) */
		Feed_constant_Shaft_revolutions = 1;
		dimensionalOrder(&Feed_constant_Feed, &Feed_constant_Shaft_revolutions, -1, (int)Position_notation_index);
	}

	if(!Feed_constant_Feed || !Feed_constant_Shaft_revolutions)
	{
		_WARNINGmessage(0xffee, 0x20, 0x6092, "Wrong value in OD, index: %x", 0x6092, 0);
		return 0;
	}

	switch(Motor_type) {
		case 0x0002:	/* DC motor */
			if(!((DC_motor_polarity == 0) || (DC_motor_polarity == 1))) {
				_WARNINGmessage(0xffee, 0x80, 0x200A, "Wrong value in OD, index: %x", 0x200A, 0);
				return 0;
			}
			break;
		case 0x000B:	/* Brushless motor */
			if(!((Brushless_sequence_type == 1) || (Brushless_sequence_type == 2) || (Brushless_sequence_type == 3))) {
				_WARNINGmessage(0xffee, 0x80, 0x2009, "Wrong value in OD, index: %x", 0x2009, 0);
				return 0;
			}
			break;
		case 0x000A:   /* Brushless AC motor */
			if(!((BLAC_Motor_Polarity == 1) || (BLAC_Motor_Polarity == 2))) {
				_WARNINGmessage(0xffee, 0x80, 0xFF04, "Wrong value in OD, index: %x", 0xFF04, 0);
				return 0;
			}
			break;
		case 0xFFFF:	/* no motor */
			break;		/* nothing to check */
		default:
			_WARNINGmessage(0xffee, 0x20, 0x6402, "Wrong value in OD, index: %x", 0x6402, 0);
			return 0;
			//break;
	}

	if(!Position_control_margin)
	{
		_WARNINGmessage(0xffee, 0x80, 0x2007, "Wrong value in OD, index: %x", 0x2007, 0);
		return 0;
	}

	if(!Velocity_control_margin)
	{
		_WARNINGmessage(0xffee, 0x80, 0x2006, "Wrong value in OD, index: %x", 0x2006, 0);
		return 0;
	}

	{
		int i, j;

		for(i=0;i<4;i++)
		{
			//if Position control parameter divisor or Position_control_margin is not configured, use default ones
			if( !Position_control_dataset[1+(i*5)] || !Position_control_dataset[1+(i*6)] )
			{
				for(j=0;j<5;j++)
				{
					Position_control_dataset[1+(i*6)+j] = Position_control_parameter_set[j];
				}
				Position_control_dataset[1+(i*6)+5] = Position_control_margin;
			}
		}
	}

	{
		long max_travel = labs(Software_position_limit_Max_position_limit-Software_position_limit_Min_position_limit);
		pp_set_max_travel( max_travel );
	}

	return 1;	/* if no problem found */
}

/*****************************************************************/
/*!	Function that configures position/velocity counters
*/
/*****************************************************************/
void configurePositioningSensors(void)
{
	switch(Velocity_sensor_Peripheral) {
		case P_ENC1:
			velocity_counter = get_Enc1;
			break;
		case P_ENC2:
			velocity_counter = get_Enc2;
			break;
		case P_AN1:
			velocity_counter = get_AN1;
			break;
		case P_AN2:
			velocity_counter = get_AN2;
			break;
		case P_AN3:
			velocity_counter = get_AN3;
			break;
		case P_AN4:
			velocity_counter = get_AN4;
			break;
		case P_MAG_ENC:
			velocity_counter = get_MAG_encoder;
			break;
		case P_PWM_ENC:
			velocity_counter = get_PWM_encoder;
			break;
		case P_RESOLVER:
			velocity_counter = get_resolver_acc;
			break;
		case P_HYBRID:
			velocity_counter = get_hybrid;
			break;
		default:
			_WARNINGmessage(0xffed, 0x80, 0x2020, "Wrong velocity sensor", 0, 0);
			break;
	}
	velocity_counter(INITIALIZE);

	switch(Position_sensor_Peripheral) {
		case P_ENC1:
			position_counter = get_Enc1;
			break;
		case P_ENC2:
			position_counter = get_Enc2;
			break;
		case P_AN1:
			position_counter = get_AN1;
			break;
		case P_AN2:
			position_counter = get_AN2;
			break;
		case P_AN3:
			position_counter = get_AN3;
			break;
		case P_AN4:
			position_counter = get_AN4;
			break;
		case P_MAG_ENC:
			position_counter = get_MAG_encoder;
			break;
		case P_PWM_ENC:
			position_counter = get_PWM_encoder;
			break;
		case P_RESOLVER:
			position_counter = get_resolver_acc;
			break;
		case P_HYBRID:
			position_counter = get_hybrid;
			break;
		default:
			_WARNINGmessage(0xffed, 0x80, 0x2021, "Wrong position sensor", 0, 0);
			break;
	}
	position_counter(INITIALIZE);

	switch(Absolute_pos_sensor_Peripheral) {
		case P_ENC1:
			abs_pos_counter = get_Enc1;
			break;
		case P_ENC2:
			abs_pos_counter = get_Enc2;
			break;
		case P_AN1:
			abs_pos_counter = get_AN1;
			break;
		case P_AN2:
			abs_pos_counter = get_AN2;
			break;
		case P_AN3:
			abs_pos_counter = get_AN3;
			break;
		case P_AN4:
			abs_pos_counter = get_AN4;
			break;
		case P_MAG_ENC:
			abs_pos_counter = get_MAG_encoder;
			break;
		case P_PWM_ENC:
			abs_pos_counter = get_PWM_encoder;
			break;
		case P_RESOLVER:
			abs_pos_counter = get_resolver_acc;
			break;
		case P_HYBRID:
			abs_pos_counter = get_hybrid;
			break;
		default:
			_WARNINGmessage(0xffed, 0x80, 0x2022, "Wrong absolute position sensor", 0, 0);
			break;
	}
	abs_pos_counter(INITIALIZE);

	switch(Hybrid_analog_sel)
	{
			case 0:
				red_pos_counter = get_AN1;
				break;
			case 1:
				red_pos_counter = get_AN2;
				break;
			default:
				_WARNINGmessage(0xffed, 0x80, 0x2022, "Wrong redundancy position sensor", 0, 0);
				break;
	}
	red_pos_counter(INITIALIZE);

	if(used_by_control(P_HYBRID))
	{
		hyb_pos_counter = get_hybrid;
	}
	hyb_pos_counter(INITIALIZE);
}

/*!
 * \brief Function that configures peripherals to get values from Power Board
 * ****************************************************************************/
void configurePowerBoardValues( void )
{
	switch( Power_board )
	{
	case NO_BOARD:   /* fictitious in which Pvoltage, current and temp are transmitted as pwms */
				get_vpower     = us_int_get_null;
				get_current    = s_get_null;
				get_current2   = s_get_null;
				get_temp       = s_get_null;
				get_motor_temp = s_get_null;
			break;
		case A3625_01_BOARD:    /* 24v slim power board with measurement of Idc */
				get_vpower     = get_AN_vpower_24vslim;
				get_current    = get_AN_current_24vslim;
				get_current2   = get_AN_null_current2;
				get_temp       = get_AN_temp_24vslim;
				get_motor_temp = s_get_null;
				Half_Period = HALF_PERIOD_17KHz;
				InitSWEPwmModules();
			break;
		case A3625_02_BOARD:   /* 24v slim power board with measurement of Iu & Iw  */
				get_vpower     = get_AN_vpower_24vslim;
				get_current    = get_AN_current_24vslim; //TODO: adapt the current measurement
				get_current2   = get_AN_current2_24vslim;
				get_temp       = get_AN_temp_24vslim;
				get_motor_temp = s_get_null;
				Half_Period = HALF_PERIOD_17KHz;
				InitSWEPwmModules();
			break;
		case A3625_03_BOARD:
			    get_vpower     = get_AN_vpower_24V_02_powerboard;
			    get_current    = get_AN_current_24V_03_powerboard;//get_AN_current_24vslim; //TODO: adapt the current measurement
			    get_current2   = get_AN_current2_24V_03_powerboard;//get_AN_current2_24vslim;
			    get_temp       = get_AN_temp_24vslim;
			    get_motor_temp = s_get_null;
			    Half_Period = HALF_PERIOD_17KHz;
			    InitSWEPwmModules();
			break;
		case A3666_01_BOARD:   /* 320v power board with measurement of Iu, Iw and Igbt Temperature */
				get_vpower     = get_AN_vpower_300Vpowerboard;
				get_current    = get_AN_current_300Vpowerboard;
				get_current2   = get_AN_current2_300Vpowerboard;
				get_temp       = get_AN_temp_300Vpowerboard; //TODO: configure the analog optocoupler to get gain 1:1 & table
				get_motor_temp = get_AN_temp_KTY_motor_sensor; //TODO: configure with the motor type or especific OD-entry
				Half_Period = HALF_PERIOD_16KHz;
				InitSWEPwmModules();
				break;
		case A3666_01_C_BOARD:   /* 320v power board with measurement of Iu, Iw, Udc and IGBT Temperature  */
				get_vpower     = get_AN_vpower_300V_C_powerboard;
				get_current    = get_AN_current_300V_C_powerboard;
				get_current2   = get_AN_current2_300V_C_powerboard;
				get_temp       = get_AN_temp_300V_C_powerboard; //TODO: configure the analog optocoupler to get gain 1:1 & table
				get_motor_temp = get_AN_temp_KTY_motor_sensor; //TODO: configure with the motor type or especific OD-entry
				Half_Period = HALF_PERIOD_14KHz;
				InitSWEPwmModules();
				#ifndef _DEBUG
				break;
				#endif
		case A3666_02_BOARD:   /* 320v power board with measurement of Iu, Iw, Udc and IGBT Temperature  */
				get_vpower     = get_AN_vpower_300V_02_powerboard;
				get_current    = get_AN_current_300V_C_powerboard;
				get_current2   = get_AN_current2_300V_C_powerboard;
				get_temp       = get_AN_temp_300V_C_powerboard; //TODO: configure the analog optocoupler to get gain 1:1 & table
				get_motor_temp = get_AN_temp_KTY_motor_sensor; //TODO: configure with the motor type or especific OD-entry
				Half_Period = HALF_PERIOD_14KHz;
				InitSWEPwmModules();
				#ifndef _DEBUG
				break;
				#endif
		case EMUL_BOARD:   /* Emulation board  */
				get_vpower     = get_AN_vpower_emuboard;
				get_current    = get_AN_current_emuboard;
				get_current    = get_AN_null_current2;
				get_temp       = get_AN_temp_emuboard;
				get_motor_temp = s_get_null;
			break;
		default:
			break;
	}
	_LOGmessage(0x0054,"Power board config n:%d", Power_board, 0);

	if(ZeroCurrent_motor==0)
	{
		switch( Power_board )
		{
			case A3625_01_BOARD:
			case A3625_02_BOARD:
				ZeroCurrent_motor = 683;
				break;
			case A3625_03_BOARD:
				//ZeroCurrent_motor = 1079; // 0.8V
				ZeroCurrent_motor = 2095;
				break;
			case A3666_01_BOARD:
				/* A3666-01-A: ZeroCurrent_motor = 3413; */
				/* A3666-01-B theorical: ZeroCurrent_motor = 2208;*/
				/* A3666-01-B empirical:*/
				ZeroCurrent_motor = 1856;
				break;
			case A3666_01_C_BOARD:
			case A3666_02_BOARD:
				ZeroCurrent_motor = 2020;
				break;
			default:
				break;
		}
		_LOGmessage(0x0034,"ZeroCurrent initilized to theorical value", 0, 0);
	}
	if(ZeroCurrent_motor2==0)
	{
		switch( Power_board )
		{
			case A3625_01_BOARD:
			case A3625_02_BOARD:
				ZeroCurrent_motor2 = 683;
				break;
			case A3625_03_BOARD:
				//ZeroCurrent_motor2 = 1079;
				ZeroCurrent_motor2 = 2095;
				break;
			case A3666_01_BOARD:
				ZeroCurrent_motor2 = 1856;
				break;
			case A3666_01_C_BOARD:
			case A3666_02_BOARD:
				ZeroCurrent_motor2 = 2020;
				break;
			default:
				break;
		}
		_LOGmessage(0x0034,"ZeroCurrent2 initilized to theorical value", 0, 0);
	}


}

/*****************************************************************/
/*!	Updates all the factors affected by a change in dimensional OD entries of the OD
*/
/*****************************************************************/
void updateFactors()
{
	unsigned int position_resolution;
	unsigned int velocity_resolution;

	/*!
	********************************************************************
		in some cases velocity encoder units are used as internal units
		while position is used when connected "behind" (not usual)
		- position is converted to internal units using pos2int_factor
		- velocity is converted to internal units using vel2int_factor
	********************************************************************
	*/

	position_resolution = get_position_resolution(Position_sensor_Peripheral);
	velocity_resolution = get_velocity_resolution(Velocity_sensor_Peripheral);
	get_pos_num_den(Position_sensor_Place, Velocity_sensor_Place, position_resolution, velocity_resolution, &Position_factor_Numerator, &Position_factor_Feed_constant);
	get_pos_vel_factors(Position_sensor_Place, Velocity_sensor_Place, position_resolution, velocity_resolution);

	/* reduce fractions of factors */
	fractionReduction(&pos2int_factor_num, &pos2int_factor_den);
	fractionReduction(&vel2int_factor_num, &vel2int_factor_den);
	fractionReduction(&Position_factor_Numerator, &Position_factor_Feed_constant);

	/* Update Velocity_encoder_factor */
	Velocity_encoder_factor_Numerator = Position_factor_Numerator;
	Velocity_encoder_factor_Divisor = Position_factor_Feed_constant;
	dimensionalOrder(&Velocity_encoder_factor_Numerator, &Velocity_encoder_factor_Divisor, (int)Velocity_notation_index, (int)Position_notation_index);

	/* Update Velocity_factor_1 (converts motor rpm to vel external units) */
	Velocity_factor_1_Numerator = Feed_constant_Feed * Gear_ratio_Shaft_revolutions;
	Velocity_factor_1_Divisor = 60 * Feed_constant_Shaft_revolutions * Gear_ratio_Motor_revolutions;
	fractionReduction(&Velocity_factor_1_Numerator, &Velocity_factor_1_Divisor);
	dimensionalOrder(&Velocity_factor_1_Numerator, &Velocity_factor_1_Divisor, (int)Velocity_notation_index, (int)Position_notation_index);

	/* Update Acceleration_factor */
	Acceleration_factor_Numerator = Position_factor_Numerator;
	Acceleration_factor_Divisor = Position_factor_Feed_constant;
	dimensionalOrder(&Acceleration_factor_Numerator, &Acceleration_factor_Divisor, (int)Acceleration_notation_index, (int)Position_notation_index);

	/* set motor type */
	switch(Motor_type) {
		case 0x0002:
			motor = (DC_motor_polarity)? DC_MOTOR_b : DC_MOTOR_a;
			break;
		case 0x000A:
			switch(BLAC_Motor_Polarity) {
				case 1:
					motor = BLAC_MOTOR_a;
					break;
				case 2:
					motor = BLAC_MOTOR_b;
					break;
				default:
					motor = NO_MOTOR;
					break;
			}
			break;
		case 0x000B:
			switch(Brushless_sequence_type) {
				case 1:
					motor = BLDC_MOTOR_a;
					break;
				case 2:
					motor = BLDC_MOTOR_b;
					break;
				case 3:
					motor = BLDC_MOTOR_c;
					break;
				default:
					motor = NO_MOTOR;
					break;
			}
			break;
		case 0xFFFF:
			motor = NO_MOTOR;
			break;
		default:
			motor = NO_MOTOR;
			break;
	}

	/* Set pos_error_normFactor  and vel_normFactor */
	pos_error_normFactor = Position_control_margin;
	vel_normFactor = Velocity_control_margin;
}


/*****************************************************************/
/*!	Function that applies a dimensional correction to a fraction with
	different dimensional order between numerator and denominator

This function does not only multiply numerator and denominator by their
corresponding power of 10, but also tries to divide the opposite member
of the fraction in order to avoid overflow

	\param num Pointer to the numerator of the fraction (will be modified)
	\param den Pointer to the denominator of the fraction (will be modified)
	\param index_num Numerator's power of 10
	\param index_div Denominator's power of 10
	\return 1 if correct, 0 if dimensional indexes are too different
*/
/*****************************************************************/
int dimensionalOrder(unsigned long *num, unsigned long *den, int index_num, int index_div)
{
	int diff_order;
	int i;
	unsigned long *multiply, *no_multiply;

	if(index_num == index_div) return 1;

	if(index_num > index_div)
	{
		diff_order = index_num - index_div;
		multiply = num;
		no_multiply = den;
	} else
	{
		diff_order =  index_div - index_num;
		multiply = den;
		no_multiply = num;
	}

	if(diff_order > 6)
	{
		_WARNINGmessage(0xffeb, 0x80, 0x0000, "Too different dimension indexes", 0, 0);
		return 0;
	}
	for(i = 0;i<diff_order;i++)
	{
		if(*no_multiply%10 == 0) *no_multiply /= 10;
		else *multiply *= 10;
	}
	return 1;
}


/*****************************************************************/
/*!	 Reduces fractions
*/
/*****************************************************************/
void fractionReduction(unsigned long *num, unsigned long *den)
{
	while((*num%10 == 0) && (*den%10 == 0) && *num && *den)
	{
		*num /= 10;
		*den /= 10;
	}

	while((*num%2 == 0) && (*den%2 == 0) && *num && *den)
	{
		*num /= 2;
		*den /= 2;
	}

	while((*num%3 == 0) && (*den%3 == 0) && *num && *den)
	{
		*num /= 3;
		*den /= 3;
	}

	while((*num%5 == 0) && (*den%5 == 0) && *num && *den)
	{
		*num /= 5;
		*den /= 5;
	}
}


/*****************************************************************/
/*!	Gives the motion state of the system (position, velocity and time)
	\param state Returns the motion state
*/
/*****************************************************************/
void getMotionState(motion_state_struct *state)
{
	static unsigned long htime = 0;
	unsigned long old_htime;
	//static unsigned int new_velocity_counter = 0;
	unsigned int old_velocity_counter;
	long long tmp;


	old_htime = htime;
	htime = CLK_gethtime();

	manage_AN1();
	manage_AN2();
	manage_NCE();
	manage_Enc1();
	manage_Enc2();
	manage_Resolver();

	//state->position = position_counter(SAVED_PARAMS);
	state->pos_cntr = position_counter(SAVED_PARAMS);

	state->pos_prev = state->pos_cntr;


	//old_velocity_counter = new_velocity_counter;
	//new_velocity_counter = velocity_counter((velocity_counter == position_counter)? SAVED_RESULT : SAVED_PARAMS);
	old_velocity_counter = state->vel_cntr;
	state->vel_cntr = velocity_counter((velocity_counter == position_counter)? SAVED_RESULT : SAVED_PARAMS);
	tmp = (long long)(CLK_countspms() * 1000) * (int)(state->vel_cntr - old_velocity_counter);
	vel_calc = (short int)(state->vel_cntr - old_velocity_counter);
	tmp /= (htime - old_htime);
	state->vel_prev = tmp;

	state->abs_pos_cntr = abs_pos_counter(((abs_pos_counter == position_counter) ||
			(abs_pos_counter == velocity_counter))? SAVED_RESULT : SAVED_PARAMS);

	state->red_pos_cntr = red_pos_counter(SAVED_PARAMS);
	Debug_long1 = red_pos_counter(SAVED_PARAMS);
	state->hyb_pos_cntr = hyb_pos_counter(SAVED_PARAMS);
	Debug_long2 = hyb_pos_counter(SAVED_PARAMS);
	state->time = getltime();		/* set current time */
}


/*****************************************************************/
/*!	Function that reads input peripherals and writes output ones
*/
/*****************************************************************/
void managePeripherals(void)
{
	int send_tpdo2 = 0;

	manageFPGA();
	send_tpdo2 |= manageGPI();
	manageGPO();
	manageAnalog();
	send_tpdo2 |= manageGPIO();
	manageEnc();
	manageHall();
	manageResolver();
	manageHybrid();
	managePWM();

	if (send_tpdo2)
	{
		/* send txPDO2 (PDO index 1) if changed */
//		sendOnePDOevent( &amc_od_Data, 1 );
		if( ! buildPDO( &amc_od_Data, 1, &amc_od_Data.PDO_status[1].last_message ) )
			canSend( amc_od_Data.canHandle, &amc_od_Data.PDO_status[1].last_message );
	}
}


/*****************************************************************/
/*!	Function that reads GPIs
	\return 1 if value has to be sent in a TPDO
*/
/*****************************************************************/
int manageGPI(void)
{
	static unsigned char old_value = 0;
	unsigned char value;
	volatile unsigned *pointer= (unsigned *)FPGA_ADD_CONF;
	int ret = 0;

	/* read inputs */
	value = GPI1 + (GPI2 << 1) + (GPI3 << 2) + (GPI4 << 3);
	if( ((UNS16)(*pointer)) & FPGA_REG_CONF_SERVICE_HW_RD_MASK )
		value |= 0x40;
	if( SAFETY == SAFETY_ALLOW )
		value |= 0x80;
	value ^= GPI_Polarity;

	/* publish value in the OD */
	GPI_Value = value;

	/* compare with last value to check if it has changed */
	if((value ^ old_value) & GPI_Interrupt_mask)	/* only if interrupt mask active */
		ret = 1;
	else
		ret = 0;

	/* save value */
	old_value = value;

	{
		static UNS16 old=0; //this variable is used to manage a fsm
		volatile unsigned *pointer = (unsigned *)FPGA_ADD_ACOUSTIC_MODE;
		unsigned int limiting_faults = (unsigned int)(FAULT_IGBT_ISR | FAULT_MOTOR_PHASE | FAULT_VPOWER | FAULT_OVERCURRENT);

		if( (FPGA_mode== WORKING_MODE_SAFE) &&
			Enable_remote_wr && ((GPI_Value & 0x08)==0) &&
			!isFaultActive(limiting_faults) && homing_zeroed )
		{
			if( (old & 0x80) != 0x80 )		//First step: Change to mode=POSITION and state=OPERATION_ENABLE
			{
				old = (*pointer);
				old |= 0x80;
				(*pointer) = 2;

				ATM_seti(&change_mode, 1);	/* mode of operation has to be changed */
				Modes_of_operation = OPERATION_MODE_POSITION;
				setDriveState((unsigned short *)&Device_status_word, OPERATION_ENABLE);
			}
			else if( (old & 0x40) != 0x40 )	//Second step: Set target demand
				// TODO: assure mode and state are the desired ones
			{
				old |= 0x40;
				Profile_velocity = 500;
				Target_position = Software_position_limit_Min_position_limit;
				Device_control_word &= ~HALT_MASKBIT;

				if(!MBX_post(&pp_newtarget_mbox, &Target_position, 0))
					_WARNINGmessage(0xfff1, 0x20, 0x0000, "pp_newtarget_mbox full", 0, 0);
			}
		}
		else
		{
			if( (old & 0x80) == 0x80 )
			{
				old &= ~0x80;
				old &= ~0x40;
				(*pointer) = (UNS16)old;

				// TODO: Change to state=QUICK_STOP_ACTIVE as in the other places
				setDriveState((unsigned short *)&Device_status_word, QUICK_STOP_ACTIVE);
			}
		}
	}
	return ret;
}


/*****************************************************************/
/*!	Function that writes GPOs
*/
/*****************************************************************/
void manageGPO(void)
{
	if (GPO & 0x001) SET_GPO1;
	else CLEAR_GPO1;

	if ((GPO >> 1) & 0x001) SET_GPO2;
	else CLEAR_GPO2;

	if ((GPO >> 2) & 0x001) SET_GPO3;
	else CLEAR_GPO3;

	if ((GPO >> 3) & 0x001) SET_GPO4;
	else CLEAR_GPO4;
}


/*****************************************************************/
/*!	Function that reads analog inputs
*/
/*****************************************************************/
void manageAnalog(void)
{
	/* read AN1 */
	AnalogIn1_Value = get_AN1(used_by_control(P_AN1)?SAVED_PARAMS:INIT_PARAMS);

	/* read AN2 */
	AnalogIn2_Value = get_AN2(used_by_control(P_AN2)?SAVED_PARAMS:INIT_PARAMS);

	/* read AN3 */
	AnalogIn3_Value = get_AN3(used_by_control(P_AN3)?SAVED_PARAMS:INIT_PARAMS);

	/* read AN4 */
	AnalogIn4_Value = get_AN4(used_by_control(P_AN4)?SAVED_PARAMS:INIT_PARAMS);
}


/*****************************************************************/
/*!	Function that reads the inputs  and set the outputs of the GPIO
	\return 1 if value has to be sent in a TPDO
*/
/*****************************************************************/
int manageGPIO_2I2O(void)
{
	static unsigned char initialized = 0;
	static unsigned char old_value = 0;
	unsigned char value;
	int ret = 0;

	if( !initialized )
	{
		FPGA_SetNCE( 0, 0 );
		FPGA_SetTrinquete( 0 );
		initialized = 1;
	}

	/* read 2 inputs */
	value = GPIO_I1 + (GPIO_I2 << 1);

	/* publish value in the OD */
	GPIO_Value = value;

	/* compare with last value to check if it has changed */
	if( (value ^ old_value) )	/* only if interrupt mask active */
		ret = 1;
	else
		ret = 0;

	/* save value */
	old_value = value;

	/* set 2 outputs */
	if (GPIO_Property_1 & 0x001) SET_GPIO_O1;
	else CLEAR_GPIO_O1;
	if (GPIO_Property_1 & 0x002) SET_GPIO_O2;
	else CLEAR_GPIO_O2;

	return ret;
}


int manage_nce_sensor( int model )
{
	static unsigned char initialized = 0;
	volatile unsigned *pointer;
	int ret = 0;

	if( !initialized )
	{
		FPGA_SetNCE( 0, model );
		FPGA_SetTrinquete( 0 );
		initialized = 1;
	}
	else
	{
		FPGA_SetNCE( 1, model );
		pointer = (unsigned *)FPGA_ADD_NCE_MEASURE;
		FPGA_NCE_measure_rd = (UNS16) (*pointer);
		if (FPGA_NCE_measure_rd&0x8000){
			if(!(isFaultActive(FAULT_NCE_MEASURE)))
			{
				_ERRORmessage(0xFF08, 0x80, 0x0000, "NCE measure error", 0, 0);
				setFault(FAULT_NCE_MEASURE);
			}
			QueueFault(FAULT_NCE_MEASURE);
		}
		else{
			if(isFaultActive(FAULT_NCE_MEASURE)){
		      DeQueueFault(FAULT_NCE_MEASURE);
			}
		}
	}
	return ret;
}



int isIronCableBroken( void )    //TODO: confirm the activation level
{
	return (GPIO_I1 == 1);
}

/*!	Trinquete management
 * GPIO_I1: input for iron cable break
 * GPIO_I2: output to activate the trinquete
 * In Trinquete mode enabled, the FPGA maps the GPIO_I2 state into the second brake output (24V) */
int manage_trinquete( void )
{
	static unsigned char initialized = 0;
	static unsigned char old_value = 0;
	static unsigned char old_iron_cable = 0xFF;
	unsigned char iron_cable;
	unsigned char value;
	int ret = 0;

	if( !initialized )
	{
		FPGA_SetNCE( 0, 0 );
		FPGA_SetTrinquete( 1 );
		initialized = 1;
		return ret;
	}

	FPGA_SetTrinquete( 1 );

	/* read 2 inputs */
	value = GPIO_I1 + (GPIO_I2 << 1);

	/* publish value in the OD */
	GPIO_Value = value;

	/* compare with last value to check if it has changed */
	if( (value ^ old_value) )	/* only if interrupt mask active */
		ret = 1;
	else
		ret = 0;

	/* save value */
	old_value = value;

	/* check input 1: iron cable break value */
	if( (iron_cable = isIronCableBroken()) )
	{
		GPIO_Property_1 |= 0x01;
	}
	else if( !(GetQueuedFault() & FAULT_IRON_CABLE_BREAK))
	{
		GPIO_Property_1 &= ~0x01;
	}
	if( old_iron_cable != iron_cable )
	{
		if( iron_cable )
		{
			if(!(isFaultActive(FAULT_IRON_CABLE_BREAK)))
			{
				_ERRORmessage(0x1616, 0x00, 0x0000, "IronCable break", 0, 0);
				setFault(FAULT_IRON_CABLE_BREAK);
			}
			QueueFault(FAULT_IRON_CABLE_BREAK);
		}
	}
	old_iron_cable = iron_cable;

	/* check slippage */
	if( isFaultActive(FAULT_REDUNDANCY) | (GetQueuedWarning() & FAULT_REDUNDANCY)| isFaultActive(FAULT_VEL_LIMIT) )
	{
		GPIO_Property_1 |= 0x02;
	}
	else
	{
		GPIO_Property_1 &= ~0x02;
	}

	/* set output 1: activate trinquete because of iron cable break */
	if (GPIO_Property_1 & 0x01) SET_GPIO_O1;
	else CLEAR_GPIO_O1;

	/* set output 2: activate trinquete because of slippage */
	if (GPIO_Property_1 & 0x02) SET_GPIO_O2;
	else CLEAR_GPIO_O2;

	if((GPIO_O1 | GPIO_O2) & (GPIO_I2))
	{
		QueueWarning(FAULT_RATCHET);
	}
	else
	{
		DeQueueWarning(FAULT_RATCHET);
	}

	return ret;
}


/*****************************************************************/
/*!	Function that manages the peripheral connected in GPIO
*/
/*****************************************************************/
int manageGPIO(void)
{
	unsigned ret = 0;

	switch(GPIO_Type)
	{
		case GPIO_2I2O:
			ret = manageGPIO_2I2O();
			break;
		case GPIO_MAG:	/* Renishaw magnetic encoder */
			/* read only if not already read for position/velocity */
			GPIO_Value = get_MAG_encoder(used_by_control(P_MAG_ENC)?SAVED_RESULT:INIT_PARAMS);
			break;
		case GPIO_8b_SHIFT:   /* 8-bit shift register */
			ret = manage_shift_reg(8);
			break;
		case GPIO_16b_SHIFT:  /* 16-bit shift register */
			ret = manage_shift_reg(16);
			break;
		case GPIO_32b_SHIFT:  /* 32-bit shift register */
			ret = manage_shift_reg(32);
			break;
		case GPIO_NC_ABS_ENCODER: /* Bourns EMS22A non contact absolute encoder */
			ret = manage_non_contacting_abs_enc();
			break;
		case GPIO_UNUSED:
			break;   /* do nothing */
		case GPIO_12b_NCE_SENSOR:
			ret = manage_nce_sensor(1);
			break;
		case GPIO_TRINQUETE:
			ret = manage_trinquete();
			break;
		case GPIO_12b_NCE_SENSOR_TRINQUETE:
			manage_nce_sensor(1);
			ret = manage_trinquete();
			break;
		default:
			_WARNINGmessage(0xffee, 0x20, 0x2207, "Wrong value in OD, index: %x", 0x2207, 0);
			GPIO_Type = GPIO_UNUSED;   /* disable GPIO */
			break;
	}
	return ret;
}


/*****************************************************************/
/*!	Function that shows encoder values
*/
/*****************************************************************/
void manageEnc(void)
{
	/* read encoder 1 */
	Encoder1_Value = get_Enc1(used_by_control(P_ENC1)?SAVED_PARAMS:INIT_PARAMS);

	/* read encoder 2 */
	Encoder2_Value = get_Enc2(used_by_control(P_ENC2)?SAVED_PARAMS:INIT_PARAMS);

	if (used_by_control(P_ENC1)){
		if (FPGA_errors&0x0004){
			if(!(isFaultActive(FAULT_ENCODER)))
			{
				_ERRORmessage(0xFF05, 0x80, 0x0000, "Encoder error", 0, 0);
				setFault(FAULT_ENCODER);
			}
			QueueFault(FAULT_ENCODER);
		}
		else{
			if(isFaultActive(FAULT_ENCODER)){
				DeQueueFault(FAULT_ENCODER);
			}
		}
	}
	if (used_by_control(P_ENC2)){
		if (FPGA_errors&0x0008){
			if(!(isFaultActive(FAULT_ENCODER)))
			{
				_ERRORmessage(0xFF05, 0x80, 0x0000, "Encoder error", 0, 0);
				setFault(FAULT_ENCODER);
			}
			QueueFault(FAULT_ENCODER);
		}
		else{
			if(isFaultActive(FAULT_ENCODER)){
				DeQueueFault(FAULT_ENCODER);
			}
		}
	}
}


/*****************************************************************/
/*!	Function that shows Hall effect sensors value
*/
/*****************************************************************/
void manageHall(void)
{
	// Hall_sensor_Polarity is IGNORED

	/* read hall sensors inputs from GPIO50-52 */
	Hall_sensor_Value = ((GpioDataRegs.GPBDAT.all & 0x001D0000) >> 18)^7;	/* bits 0,1,2 = H1,H2,H3 */
}


/*****************************************************************/
/*!	Function that shows resolver position
*/
/*****************************************************************/
void manageResolver(void)
{
	Resolver_Value = calculate_resolver_value();

	Resolver_Acc_value = get_resolver_acc(used_by_control(P_RESOLVER)?SAVED_PARAMS:INIT_PARAMS);
}
void manageHybrid(void)
{
	Hybrid_Value = get_hybrid(INIT_PARAMS);
	//Hybrid_Value = get_hybrid(used_by_control(P_RESOLVER)?SAVED_PARAMS:INIT_PARAMS);
}

/*****************************************************************/
/*!	Function that shows PWM sensor
*/
/*****************************************************************/
void managePWM(void)
{
	static unsigned char initialized = 0;

	if (!initialized && !used_by_control(P_PWM_ENC) && PWM_sensor_Resolution)
	{
		InitECap4PWMSensor( PWM_sensor_Resolution, ( GPI_Polarity & GPI2_POLARITY_MASK ) >> 1 );
		initialized = 1;
	}
	calculate_pwm_sensor_value(&PWM_sensor_Value, &PWM_sensor_Period);
}



/*****************************************************************/
/*!	Function that reads encoder 1
	\return Enc1 value
*/
/*****************************************************************/
long get_Enc1(char params)
{
	static char polarity = 0;
	static long result = 0;

	switch(params) {
		case SAVED_PARAMS:
			break;
		case INITIALIZE:
		case INIT_PARAMS:
			polarity = Encoder1_Polarity;
			break;
		case SAVED_RESULT:
			return result;
			//break;
		default:   /* unimplemented call mode for this function */
			_WARNINGmessage(0xffda, 0x80, 0x0000, "Internal software error", 0, 0);
			return 0;
			//break;
	}

	result = EQep1Regs.QPOSCNT;
	if(polarity) result = -result;
	return result;
}


/*****************************************************************/
/*!	Function that reads encoder 2
	\return Enc2 value
*/
/*****************************************************************/
long get_Enc2(char params)
{
	static char polarity = 0;
	static long result = 0;

	switch(params) {
		case SAVED_PARAMS:
			break;
		case INITIALIZE:
		case INIT_PARAMS:
			polarity = Encoder2_Polarity;
			break;
		case SAVED_RESULT:
			return result;
			//break;
		default:   /* unimplemented call mode for this function */
			_WARNINGmessage(0xffda, 0x80, 0x0000, "Internal software error", 0, 0);
			return 0;
			//break;
	}

	result = EQep2Regs.QPOSCNT;
	if(polarity) result = -result;
	return result;
}


/*****************************************************************/
/*!	Function that reads analog input 1 and applies gain/offset
	\return Analog input 1 value
*/
/*****************************************************************/
long get_AN1(char params)
{
	static short int gain_num = 0;
	static short int gain_div = 1;
	static long offset = 0;
	static long result = 0;
	long long aux;

	switch(params) {
		case SAVED_PARAMS:
			break;
		case INITIALIZE:
		case INIT_PARAMS:
			gain_num = AnalogIn1_Gain_num;
			gain_div = AnalogIn1_Gain_div;
			offset = AnalogIn1_Offset;
			break;
		case SAVED_RESULT:
			return result;
			//break;
		default:   /* unimplemented call mode for this function */
			_WARNINGmessage(0xffda, 0x80, 0x0000, "Internal software error", 0, 0);
			return 0;
			//break;
	}

	aux = (long long)an1 * gain_num;
	result = (aux / gain_div) + offset;
	// Debug_long5 = result;
	return result;
}


/*****************************************************************/
/*!	Function that reads analog input 2 and applies gain/offset
	\return Analog input 2 value
*/
/*****************************************************************/
long get_AN2(char params)
{
	static short int gain_num = 0;
	static short int gain_div = 2;
	static long offset = 0;
	static long result = 0;
	long long aux;

	switch(params) {
		case SAVED_PARAMS:
			break;
		case INITIALIZE:
		case INIT_PARAMS:
			gain_num = AnalogIn2_Gain_num;
			gain_div = AnalogIn2_Gain_div;
			offset = AnalogIn2_Offset;
			break;
		case SAVED_RESULT:
			return result;
			//break;
		default:   /* unimplemented call mode for this function */
			_WARNINGmessage(0xffda, 0x80, 0x0000, "Internal software error", 0, 0);
			return 0;
			//break;
	}

	aux = (long long)an2 * gain_num;
	result = (aux / gain_div) + offset;
	return result;
}


/*****************************************************************/
/*!	Function that reads analog input 3 and applies gain/offset
	\return Analog input 3 value
*/
/*****************************************************************/
long get_AN3(char params)
{
	static short int gain_num = 0;
	static short int gain_div = 3;
	static long offset = 0;
	static long result = 0;
	long long aux;

	switch(params) {
		case SAVED_PARAMS:
			break;
		case INITIALIZE:
		case INIT_PARAMS:
			gain_num = AnalogIn3_Gain_num;
			gain_div = AnalogIn3_Gain_div;
			offset = AnalogIn3_Offset;
			break;
		case SAVED_RESULT:
			return result;
			//break;
		default:   /* unimplemented call mode for this function */
			_WARNINGmessage(0xffda, 0x80, 0x0000, "Internal software error", 0, 0);
			return 0;
			//break;
	}

	aux = (long long)an3 * gain_num;
	result = (aux / gain_div) + offset;
	return result;
}


/*****************************************************************/
/*!	Function that reads analog input 4 and applies gain/offset
	\return Analog input 4 value
*/
/*****************************************************************/
long get_AN4(char params)
{
	static short int gain_num = 0;
	static short int gain_div = 4;
	static long offset = 0;
	static long result = 0;
	long long aux;

	switch(params) {
		case SAVED_PARAMS:
			break;
		case INITIALIZE:
		case INIT_PARAMS:
			gain_num = AnalogIn4_Gain_num;
			gain_div = AnalogIn4_Gain_div;
			offset = AnalogIn4_Offset;
			break;
		case SAVED_RESULT:
			return result;
			//break;
		default:   /* unimplemented call mode for this function */
			_WARNINGmessage(0xffda, 0x80, 0x0000, "Internal software error", 0, 0);
			return 0;
			//break;
	}

	aux = (long long)an4 * gain_num;
	result = (aux / gain_div) + offset;
	return result;
}


/*****************************************************************/
/*!	Function that reads a PWM output encoder (connected in GPI2)
	\return PWM encoder value
*/
/*****************************************************************/
long get_PWM_encoder(char params)
{
	static unsigned short resolution = 0;
	static unsigned char polarity = 0;
	static long result = 0;
	static unsigned int last_value = 0;
	unsigned short value = 0;
	short diff;

	switch(params) {
		case SAVED_PARAMS:
			break;
		case INITIALIZE:
			resolution = PWM_sensor_Resolution;
			polarity   = ( GPI_Polarity & GPI2_POLARITY_MASK ) >> 1;
			InitECap4PWMSensor( resolution, polarity );
//#define PWMENC_ZERO_AT_START
#ifndef PWMENC_ZERO_AT_START
			last_value = 0;
#endif
			result = 0;
			break;
		case SAVED_RESULT:
			return result;
			//break;
		case INIT_PARAMS:
			resolution = PWM_sensor_Resolution;
			polarity   = ( GPI_Polarity & GPI2_POLARITY_MASK ) >> 1;
			InitECap4PWMSensor( resolution, polarity );
			break;
		default:   /* unimplemented call mode for this function */
			_WARNINGmessage(0xffda, 0x80, 0x0000, "Internal software error", 0, 0);
			return 0;
			//break;
	}

	/* value is in range [0, resolution - 1] */
	calculate_pwm_sensor_value(&value, 0);

#ifdef PWMENC_ZERO_AT_START
	/* initialize last_value if INITIALIZE */
	if (params == INITIALIZE) last_value = value;
#endif

	diff = (value - last_value);

	/* value and last_value are in [0, resolution - 1] range so diff is in [-(resolution-1), resolution-1] */
	/* but an increment of -(resolution-1) corresponds to +1 an an increment of (resolution-1) is -1 */
	if (diff < -(resolution/2)) diff += resolution;
	if (diff >= (resolution/2)) diff -= resolution;

	result += diff;
	last_value = value;
	return result;
}



/*****************************************************************/
/*!	Function that return accumulated resolver value in tenths of degree
	\param params Indicates if params have to be initialized
	\return Resolver accumulated value (tenths of degree)
*/
/*****************************************************************/
long get_resolver_acc(char params)
{
	static unsigned char polarity = 0;
	static long result = 0;
	static long last_value = 0;
	short value = 0;
	short diff;

	switch(params) {
		case SAVED_PARAMS:
			break;
		case INITIALIZE:
			polarity = Resolver_Acc_polarity;
#define RESOLVER_ZERO_AT_START
#ifndef RESOLVER_ZERO_AT_START
			last_value = 0;
#endif
			result = 0;
			break;
		case SAVED_RESULT:
			return result;
			//break;
		case INIT_PARAMS:
			polarity = Resolver_Acc_polarity;
			break;
		default:   /* unimplemented call mode for this function */
			_WARNINGmessage(0xffda, 0x80, 0x0000, "Internal software error", 0, 0);
			return 0;
			//break;
	}

	value = calculate_resolver_value();

#ifdef RESOLVER_ZERO_AT_START
	/* initialize last_value if INITIALIZE */
	if (params == INITIALIZE) last_value = value;
#endif

	diff = ( value - last_value );
	if(polarity) diff = -diff;

	/* value and last_value are in [-1800, 1800] range so diff is in [-3600, 3600] */
	/* but an increment of -3599 corresponds to +1 an an increment of 3599 is -1 */
	if (diff < -1800) diff += 3600;
	if (diff >= 1800) diff -= 3600;

	result += diff;
	last_value = value;



	return result;
}
/*****************************************************************/
/*!	Function that reads the hybrid sensor
	\return Hybrid value
*/
/*****************************************************************/
long get_hybrid(char params)
{
	static short int gain_num = 0;
	static short int gain_div = 1;
	static long offset = 0;
	static long result = 0;

	switch(params) {
		case SAVED_PARAMS:
			break;
		case INITIALIZE:
		case INIT_PARAMS:
			gain_num = AnalogHybrid_Gain_num;
			gain_div = AnalogHybrid_Gain_div;
			offset	 = AnalogHybrid_Offset;
			break;
		case SAVED_RESULT:
			return result;
			//break;
		default:   /* unimplemented call mode for this function */
			_WARNINGmessage(0xffda, 0x80, 0x0000, "Internal software error", 0, 0);
			return 0;
			//break;
	}

	calculate_hybrid_value(); // return analog_hybrid //pasarle las variables de init config?
	result = (long)(analog_hybrid - offset) * gain_num / gain_div;
	Debug_long4 = result;
	return result;
}

/*!
 * \brief  Function that reads value of V power from analog input
 * \return
 * ****************************************************************************/
#pragma CODE_SECTION(get_AN_vpower_24vslim, "criticalFuncs")
unsigned get_AN_vpower_24vslim( void )
{
	unsigned int intermediate_vpower;

	/* return Vpower (24v) in mV */
	/* Correct analog optocoupler gain */
	intermediate_vpower = ((unsigned long)Analog_optocoupler_gain * analog_vpower * 524) >> 19;
	/*! 123/8 converts adc measurements in voltage in mV at Vpower (considering 1/21 resistive divisor before analog optocoupler) */
	return (123 * (unsigned long)intermediate_vpower) >> 3;
}


/*!
 * \brief  Function that reads value of Motor current from analog input
 * \return
 * ****************************************************************************/
#pragma CODE_SECTION(get_AN_current_24vslim, "criticalFuncs")
short get_AN_current_24vslim( void )
{
	int incr_v_curr;

	incr_v_curr = analog_motorcurrent - ZeroCurrent_motor;
	if (incr_v_curr < 0) incr_v_curr = 0;
	return (short)(incr_v_curr * 4); 		/* current in mA = 3,959 * (adc - 0.5v) */
}

#pragma CODE_SECTION(get_AN_null_current2, "criticalFuncs")
short get_AN_null_current2( void )
{
	return (short)(- (current1 >> 1));
}

#pragma CODE_SECTION(get_AN_current2_24vslim, "criticalFuncs")
short get_AN_current2_24vslim( void )
{
	int incr_v_curr;

	incr_v_curr = analog_motorcurrent2 - ZeroCurrent_motor;
	if (incr_v_curr < 0) incr_v_curr = 0;
	return (short)(incr_v_curr * 4); 		/* current in mA = 3,959 * (adc - 0.5v) */
}

/*!
 * \brief  Function that reads value of temperature from analog input
 * \return
 * ****************************************************************************/
short get_AN_temp_24vslim( void )
{
	return get_temperature_from_table(analog_ntc, &temp_table_24vslim);
}

/*!
 * \brief  Function that reads value of V power from analog input
 * \return
 * ****************************************************************************/
#pragma CODE_SECTION(get_AN_vpower_300Vpowerboard, "criticalFuncs")
unsigned get_AN_vpower_300Vpowerboard( void )
{
	/* return Vpower in hundreds of Volts (return 30000 = 300 Vdc) */
	//return (unsigned)( 1419 * ((unsigned long)an4-10)  >> 5 );
	return (unsigned)( 1419 * ((unsigned long)analog_vpower2-10)  >> 5 );
}

#pragma CODE_SECTION(get_AN_vpower_300V_C_powerboard, "criticalFuncs")
unsigned get_AN_vpower_300V_C_powerboard( void )
{
	/* return Vpower in hundreds of Volts (return 30000 = 300 Vdc) */

	// Use 1/100 in Divisor of UDC
	// Relation 60K4 / 100K = 0.604 (=3/5) in HCNR200
	// Relation 2/3.5 in AN4 input of A3624-01
	// ADC: 3V = 4095 counts
	// ADC_counts = 0.01 x 0.604 x 2/3.5 * 4095/3 = 4.7112
	// Udc(V) = Adc_counts / 4.7112 = Adc_counts x 0.21226015

	//return (unsigned)( 2717 * ((unsigned long)analog_vpower2)  >> 7 );

	//Correct with optocoupler gain
	//return (unsigned)( 11 * ((unsigned long)Analog_optocoupler_gain * analog_vpower2)  >> 9 ); //0.021484375
	return (unsigned)( 87 * ((unsigned long)Analog_optocoupler_gain * analog_vpower2)  >> 12 );  //0.021240234375
}

#pragma CODE_SECTION(get_AN_vpower_300V_02_powerboard, "criticalFuncs")
unsigned get_AN_vpower_300V_02_powerboard( void )
{
	unsigned long intermediate_vpower;

	/* return Vpower in hundreds of Volts (return 30000 = 300 Vdc) */

	// Use 1/100 in Divisor of UDC
	// Relation 60K4 / 100K = 0.604 (=3/5) in HCNR200
	// ADC: 3V = 4095 counts
	// ADC_counts = 0.01 x 0.604 x 4095/3 = 8.2446
	// Udc(V) = Adc_counts / 8.2446 = Adc_counts x 0.12129
	//12.129

	//return (unsigned)( 97 * ((unsigned long)analog_vpower2)  >> 3 );

	/* Correct analog optocoupler gain (Divide by 1000)*/
	intermediate_vpower = ((unsigned long)Analog_optocoupler_gain * analog_vpower2 * 524) >> 19;
	/*! x12.129 converts adc measurements in voltage in hundreds of Volts at Vpower  */
	return (97 * (unsigned long)intermediate_vpower) >> 3;


}

#pragma CODE_SECTION(get_AN_vpower_24V_02_powerboard, "criticalFuncs")
unsigned get_AN_vpower_24V_02_powerboard( void )
{
	/* return Vpower in hundreds of Volts (return 2400 = 24 Vdc) */

	// Use 1/6 in Divisor of UDC
	// Relation 100K / 160K = 0.625 in HCNR200
	// ADC: 3V = 4096 counts
	// ADC_counts = 0.166 x 0.625 x (4096/3) = 142 (counts / V)
	// Udc(V) = Adc_counts / 142 = Adc_counts x 0.00703
	// 0.000703 with no correct factor (1000) in Analog_optocoupler_gain
	// 0.703 = 90 / 128 -> Udc in hundreds of Volts -> Adc_counts x 90 / (2^7)


	return (unsigned)( 90 * ((unsigned long)analog_vpower2)  >> 7 );

	//Correct with optocoupler gain
	//return (unsigned)( 11 * ((unsigned long)Analog_optocoupler_gain * analog_vpower2)  >> 9 ); //0.021484375
	//return (unsigned)( 87 * ((unsigned long)Analog_optocoupler_gain * analog_vpower2)  >> 12 );  //0.021240234375
}

/*!
 * \brief  Function that reads value of Motor current from analog input
 * \return
 * ****************************************************************************/
#pragma CODE_SECTION(get_AN_current_300Vpowerboard, "criticalFuncs")
short get_AN_current_300Vpowerboard( void )
{
	/* A3666-01-A: current in mA = 4,852 * (adc - 2.5v) */
	//return (short)((analog_motorcurrent - ZeroCurrent_motor) * 5);

	/* A3666-01-B theorical: current in mA = 3.139315 * (adc - 2.5V * 0.64706) */
	//return (short)( 201 * ((long)analog_motorcurrent - (long)ZeroCurrent_motor) >> 6);
	/* A3666-01-B empirical (out resistance of AD710): current in mA = 8.9185 * (adc - 1.36V) */
	return (short)( 571 * ((long)analog_motorcurrent - (long)ZeroCurrent_motor) >> 6);
}

#pragma CODE_SECTION(get_AN_current2_300Vpowerboard, "criticalFuncs")
short get_AN_current2_300Vpowerboard( void )
{
	return (short)( 571 * ((long)analog_motorcurrent2 - (long)ZeroCurrent_motor2) >> 6);
}
#pragma CODE_SECTION(get_AN_current_300V_C_powerboard, "criticalFuncs")
short get_AN_current_300V_C_powerboard( void )
{
	/* Return current in mA */

	// Use Hall sensor 151 mV/A
	// Adaptation= minus 1.02 V

	/* A3666-01-C current in mA = 4.85166 * (adc - 2020) */
	return (short)( (1242 * ((long)analog_motorcurrent - (long)ZeroCurrent_motor)) >> 8);
}

#pragma CODE_SECTION(get_AN_current2_300V_C_powerboard, "criticalFuncs")
short get_AN_current2_300V_C_powerboard( void )
{
	return (short)( (1242 * ((long)analog_motorcurrent2 - (long)ZeroCurrent_motor2)) >> 8);
}

#pragma CODE_SECTION(get_AN_current_24V_03_powerboard, "criticalFuncs")
short get_AN_current_24V_03_powerboard( void )
{
	/* Return current in mA */

	// Use Hall sensor 151 mV/A

	/* A3625-03-C current in mA = 4.85 * (adc - 1079)  */
	//return (short) (485*((long)analog_motorcurrent-(long)ZeroCurrent_motor)/100);
	return (short) ((621*((long)analog_motorcurrent-(long)ZeroCurrent_motor)) >> 7);
}

#pragma CODE_SECTION(get_AN_current2_24V_03_powerboard, "criticalFuncs")
short get_AN_current2_24V_03_powerboard( void )
{
	//return (short)( 6617 * ((long)analog_motorcurrent - (long)ZeroCurrent_motor) >> 10);
	//return (short) (485*((long)analog_motorcurrent2-(long)ZeroCurrent_motor2)/100);
	return (short) ((621*((long)analog_motorcurrent2-(long)ZeroCurrent_motor2)) >> 7);
}

/*!
 * \brief  Function that reads value of temperature from analog input
 * \return
 * ****************************************************************************/
short get_AN_temp_300Vpowerboard( void )
{
	return get_temperature_from_table(analog_ntc, &temp_table_300v);
	//return 240;
}

short get_AN_temp_300V_C_powerboard( void )
{
	return get_temperature_from_table(analog_ntc, &temp_table_300v_C);
	//return 240;
}

/*!
 * \brief  Function that reads value of temperature from analog input
 * \return
 * ****************************************************************************/
short get_AN_temp_KTY_motor_sensor( void )
{
	return get_temperature_from_table(thermistor, &temp_table_kty_motor);
	//return 240;
}

/*!
 * \brief  Function that emulate the behavioral of a board
 * \return
 * ****************************************************************************/
unsigned get_AN_vpower_emuboard( void )
{
	/* return Vpower in mV */
	return 2400;
}

/*!
 * \brief  Function that emulate the behavioral of a board
 * \return
 * ****************************************************************************/
#pragma CODE_SECTION(get_AN_current_emuboard, "criticalFuncs")
short get_AN_current_emuboard( void )
{
	/* current in mA */
	return 0;
}

/*!
 * \brief  Function that emulate the behavioral of a board
 * \return
 * ****************************************************************************/
short get_AN_temp_emuboard( void )
{
	/* temperature (in tenths of degrees Centigrade) */
	return 240;
}


/*!
 * \brief Gets the value from the pwm sensor of Power Voltage.
 * \return
 * ****************************************************************************/
unsigned get_PWM_voltage( void )
{
	return calculate_pwm_in1_value();
}


/*!
 * \brief Gets the value from the pwm sensor of Power Voltage.
 * \return
 * ****************************************************************************/
#pragma CODE_SECTION(get_PWM_current, "criticalFuncs")
short get_PWM_current( void )
{
	return calculate_pwm_in2_value();
}


/*!
 * \brief Gets the value from the pwm sensor of Power Voltage.
 * \return
 * ****************************************************************************/
short get_PWM_temp( void )
{
	return calculate_pwm_in3_value();
}


/*****************************************************************/
/*!	Function that reads a magnetic encoder in GPIO
	\return Magnetic encoder value value
*/
/*****************************************************************/
long get_MAG_encoder(char params)
{
	static unsigned char resolution_bits = 0, polarity = 0;
	static long result = 0;
	unsigned int i;
	int value = 0, increment;
	static int last_value = 0;
	unsigned int half_res, resolution;

	switch(params) {
		case SAVED_PARAMS:
			break;
		case INITIALIZE:
			resolution_bits = GPIO_Property_1;
			polarity = GPIO_Property_2;
			result = 0;
//#define MAG_ZERO_AT_START_POS
#ifndef MAG_ZERO_AT_START_POS
			last_value = 0;
#endif
			break;
		case SAVED_RESULT:
			return result;
			//break;
		case INIT_PARAMS:
			resolution_bits = GPIO_Property_1;
			polarity = GPIO_Property_2;
			break;
		default:   /* unimplemented call mode for this function */
			_WARNINGmessage(0xffda, 0x80, 0x0000, "Internal software error", 0, 0);
			return 0;
			//break;
	}

	if ( (resolution_bits >= 8) && (resolution_bits <= 13) )
	{
		DINT;
		CLEAR_GPIO_O1;
		asm ("   RPT #2 || NOP");
		for(i = 0x01 << resolution_bits; i>1; i>>=1)
		{
			SET_GPIO_O1;
			if (GPIO_I1) value |= i;    /* Reading bit */
			asm("	RPT #5 || NOP");    /* do nothing for 14 cycles */
			CLEAR_GPIO_O1;                /* clock and inverted changed */
			asm("	RPT #2 || NOP");    /* do nothing for 14 cycles */
		}
		SET_GPIO_O1;
		if (GPIO_I1) value |= i;        /* Reading bit */
		EINT;

		/*We get only the bits needed */
		value &= 0xffff >> (16 - resolution_bits);

//		/* make tmp a signed value (fill left with 1's if negative) */
//		half_res = 2^(resolution_bits-1);
//		resolution = 2^resolution_bits;
//		value = ((value + half_res) % resolution) - half_res;

		/* invert if polarity is '1' */
		value = (polarity)? -value : value;

#ifdef MAG_ZERO_AT_START_POS
		/* initialize last_value if INITIALIZE */
		if (params == INITIALIZE) last_value = value;
#endif

		/* increment accumulated result */
		half_res = 2^(resolution_bits-1);
		resolution = 2^resolution_bits;
		increment = ((polarity)? (last_value - value) : (value - last_value));
		increment &= resolution - 1;
		increment = ((increment + half_res) % resolution) - half_res;
		result += increment;

		/* save value for next cycle */
		last_value = value;
	} else {
		result = 0;
		GPIO_Type = GPIO_UNUSED;
		_WARNINGmessage(0xffee, 0x20, 0x2207, "Wrong value in OD, index: %x", 0x2207, 0);
	}

	return result;
}


/*****************************************************************/
/*!	Converts voltage in NTC (in ADC units) in temperature (in tenths of degrees Centigrade) by interpolation in a table */
/*****************************************************************/
int get_temperature_from_table(unsigned ntcADC, const adc_temp_table_t *table)
{
	int i;

	if (!table) return 0;   /* invalid struct pointer */

	for(i=0; i<table->n; ++i)
	{
		if (ntcADC < table->adcTable[i]) break;
	}

	if(i == 0) return interpolate(ntcADC, table->adcTable[0], table->adcTable[1], table->tempTable[0], table->tempTable[1]);
	else if(i==table->n) return interpolate(ntcADC, table->adcTable[table->n-2], table->adcTable[table->n-1], table->tempTable[table->n-2], table->tempTable[table->n-1]);
	else return interpolate(ntcADC, table->adcTable[i-1], table->adcTable[i], table->tempTable[i-1], table->tempTable[i]);
}

/*****************************************************************/
/*!	Function that checks if a peripheral is used for motor control
	\return '1' if used, '0' if not
*/
/*****************************************************************/
char used_by_control(char peripheral)
{
	if( ( Velocity_sensor_Peripheral == peripheral ) ||
	    ( Position_sensor_Peripheral == peripheral ) ||
	    ( Absolute_pos_sensor_Peripheral == peripheral ) ) return 1;
	else return 0;
}



/*!	Function that always returns 0. Used as initial value of pointer functions
	\return '0' */
long l_get_null(char init) { return 0; }


/*!	Function that always returns 0. Used as initial value of pointer functions
	\return '0' */
short s_get_null( void ) { return 0; }


/*!	Function that always returns 0. Used as initial value of pointer functions
	\return '0' */
unsigned short us_get_null( void ) { return 0; }

/*!	Function that always returns 0. Used as initial value of pointer functions
	\return '0' */
unsigned us_int_get_null( void ) { return 0; }


/*   Function that calculates resolver value from accumulated_sin and accumulated_cos,
	  the fitered values of sine and cosine resolver
	\return resolver value in [0,3600] range
	*/
short calculate_resolver_value(void)
{
	_iq res_angle;
	long long tmp;
	/* resolver_angle is in [-pi, pi] range */
	/* 3600/(2*pi) = 572.9578 = 1.1190582 << 9 */
	/* tmp = _IQmpy(_IQ(1.1190581936), resolver_angle); */
	/* value = (int)(tmp + 256 >> (GLOBAL_Q - 9)); */

	/* res_angle is in [0, 1] range */
	/* 3600 = 7.03125 * 512 = 7.03125 << 9 */
	//res_angle = _IQatan2PU((accumulated_sin + 4) >> 3, (accumulated_cos + 4) >> 3);
	res_angle = _IQatan2PU(accumulated_sin, accumulated_cos);
	tmp = _IQmpy(_IQ(7.03125), res_angle);
	return(tmp + 256 >> (GLOBAL_Q - 9));
}

/*
	Compound the hybrid 16 bit measurement
*/

unsigned calculate_hybrid_value(void)
{
	UNS32 pot  = 0;
	UNS32 pot_count  = 0;
	UNS32 factor  = 1;
	UNS32 factor2 = 1;
	UNS32 NCE_count = 0;
	UNS32 NCE_count_comp = 0;
	UNS32 Npot = 12;
	UNS32 Nhyb = 12;
	UNS32 Hyb_counts = 0;

	Hyb_counts = 1<<Nhyb;

	pot = Hybrid_analog_sel == 0? an1 : an2; 																// Select analog input
	pot_count = Pot_Polarity == 0? pot : ((1<<Npot) - 1) - pot; 											// Compensate sign polarity Potentiometer
	NCE_count = NCE_Polarity==0? FPGA_NCE_measure_rd : ((Hyb_counts-1) - FPGA_NCE_measure_rd) & 0x0000FFF; 	// Compensate sign polarity NCE

	factor  = (pot_count * Hybrid_n_turns + (1<<(Npot-1))) >> Npot; 	// rounded division
	factor2 = (pot_count * Hybrid_n_turns) >> Npot;						// floor division

	NCE_count_comp = (NCE_count + ((Hyb_counts - AnalogNCE_Offset) & 0x0000FFF)) % (Hyb_counts-1); // NCE counter with offset correction

	//Total value in internal counts of the hybrid sensor
	if (NCE_count_comp > 3*((1<<(Nhyb-2))))
	{
		analog_hybrid = NCE_count_comp + ((factor - 1) * (Hyb_counts-1));
	}
	else
	{
		if(NCE_count_comp < (1<<(Nhyb-2)))
		{
			analog_hybrid = NCE_count_comp + (factor * (Hyb_counts-1));
		}
		else
		{
			analog_hybrid = NCE_count_comp + (factor2 * (Hyb_counts-1));
		}
	}

	Debug_long3 = analog_hybrid;
	return analog_hybrid;
	//Hybrid_measure = (analog_hybrid - AnalogHybrid_Offset) * AnalogHybrid_Gain_num / AnalogHybrid_Gain_div; // Measure of hybrid sensor in tenths of mm
}

unsigned new_calculate_hybrid_value(void)
{
	UNS32 pot  = 0;
	UNS32 pot_count  = 0;
	UNS32 factor  = 1;
	UNS32 factor2 = 1;
	UNS32 NCE_count = 0;
	UNS32 NCE_count_comp = 0;
	UNS32 Npot = 12;
	UNS32 Nhyb = 12;
	UNS32 Hyb_counts = 0;

	Hyb_counts = 1<<Nhyb;

	pot = Hybrid_analog_sel == 0? an1 : an2; 																// Select analog input
	pot_count = Pot_Polarity == 0? pot : ((1<<Npot) - 1) - pot; 											// Compensate sign polarity Potentiometer
	NCE_count = NCE_Polarity==0? FPGA_NCE_measure_rd : ((Hyb_counts-1) - FPGA_NCE_measure_rd) & 0x0000FFF; 	// Compensate sign polarity NCE

	factor  = (pot_count * Hybrid_n_turns + (1<<(Npot-1))) >> Npot; 	// rounded division
	factor2 = (pot_count * Hybrid_n_turns) >> Npot;						// floor division

	//NCE_count_comp = (NCE_Count + (Hyb_counts - AnalogHybrid_Offset)) & (Hyb_counts - 1);
	NCE_count_comp = (NCE_count + ((Hyb_counts - AnalogNCE_Offset) & 0x0000FFF)) % (Hyb_counts-1); // NCE counter with offset correction

	//Total value in internal counts of the hybrid sensor
	if (NCE_count_comp > 3*((1<<(Nhyb-2))))
	{
		analog_hybrid = NCE_count_comp + ((factor - 1) * (Hyb_counts-1));
	}
	else
	{
		if(NCE_count_comp < (1<<(Nhyb-2)))
		{
			analog_hybrid = NCE_count_comp + (factor * (Hyb_counts-1));
		}
		else
		{
			analog_hybrid = NCE_count_comp + (factor2 * (Hyb_counts-1));
		}
	}

	Debug_long3 = analog_hybrid;
	return analog_hybrid;
	//Hybrid_measure = (analog_hybrid - AnalogHybrid_Offset) * AnalogHybrid_Gain_num / AnalogHybrid_Gain_div; // Measure of hybrid sensor in tenths of mm
}


#pragma CODE_SECTION(manage_shift_reg, "criticalFuncs")
/*   Function that reads a 8-bit, 16-bit or 32-bit shift register using GPIO_O1 as CLK,
	  GPIO_O2 as LATCH and GPIO_I1 as serial input
	\return 1 if value has to be sent in a TPDO
	*/
int manage_shift_reg(unsigned bits)
{
	unsigned long value = 0, i;
	unsigned aux;
	static unsigned long old_value = 0;
	int ret = 0;

	//DINT;   /* it's not necessary to disable interrupts */
	/* latch inputs */
	CLEAR_GPIO_O2;
	asm ("   RPT #9 || NOP");
	SET_GPIO_O2;
	//asm ("   RPT #9 || NOP");

	for (i=0; i<bits; ++i) {
		CLEAR_GPIO_O1;
		value <<= 1;
		//aux = GPIO_I1;
		aux = GPIO_I1_word;
		SET_GPIO_O1;
		aux >>= GPIO_I1_bit;
		aux &= 0x00000001;
		aux ^= 0x00000001;
		value |= aux;
	}

	//EINT;

	/* Invert polarity according to GPIO_Property_1 */
	value ^= GPIO_Property_1 ;
	GPIO_Value = value;

	/* compare with last value to check if it has changed */
	if((value ^ old_value) & GPIO_Property_2)	/* only if interrupt mask active */
		ret = 1;
	else
		ret = 0;

	/* save last value */
	old_value = value;

	return ret;
}


//#pragma CODE_SECTION(manage_nc_abs_enc, "criticalFuncs")
/*   Function that reads a 16-bit shift register using GPIO_O2 as CLK,
	  GPIO_O1 as CHIP_SELECT and GPIO_I1 as serial input
	\return 1 if value has to be sent in a TPDO
*/
int manage_non_contacting_abs_enc( void)
{
	unsigned int data = 0, i, aux, value;
	static unsigned int old_value = 0;
	int ret = 0;

	//DINT;   /* it's not necessary to disable interrupts */
	/* Chip select */
	CLEAR_GPIO_O1;
	asm ("   RPT #50 || NOP");

	for (i=0; i<16; ++i) {
		CLEAR_GPIO_O2;
		data = data << 1;
		asm ("   RPT #200 || NOP");
		SET_GPIO_O2;
		asm ("   RPT #200 || NOP");
		aux = GpioDataRegs.GPADAT.bit.GPIO9; //GPIO1
		aux &= 0x00000001;
		data |= aux;
		if (i==9) value = data;
	}

	asm ("   RPT #50 || NOP");
	SET_GPIO_O1;
	//EINT;

	/* Invert polarity according to GPIO_Property_1 */
	value &= 0x03FF;
	value ^= 0x03FF;
	//AMQ: Valor del encoder leido en 80 useg, con error del bit menos significativo
	//Resolucion : 360/1023 = 0.35 grado

	//TODO: generar una estructura para leer los bits y acceder directamente al dato y
	// a los bits de error. Recordar invertir todo la estructura. Meter la lectura en una
	// tarea de baja prioridad y escribir el dato de 16 bits con acceso unico
	// ATM_seti(&position, NCEnc.Dato);

	GPIO_Value = value;

	/* compare with last value to check if it has changed */
	if((value ^ old_value) & GPIO_Property_2)	/* only if interrupt mask active */
		ret = 1;
	else
		ret = 0;

	/* save last value */
	old_value = value;

	return ret;
}



/* FPGA Registers interface ****************************/
void manageFPGA( void )
{
	if(	(GetQueuedFault() &  FAULT_HEARTBEAT) )
	{
		_LOGmessage(0x0113,"manageFPGA:FAULT_HEARTBEAT", 0, 0);
		Configure_FPGA_heartbeat( FPGA_HEARTBEAT_ENABLE_MASK, 0x00 );
		//Configure_FPGA_move_watchdog(FPGA_MOVE_WATCHDOG_HEARTBEAT_MASK, 0);
	}
	else
	{
		_LOGmessage(0x0114,"manageFPGA:FAULT_HEARTBEAT", 0, 0);
		Configure_FPGA_heartbeat( FPGA_HEARTBEAT_MASTER_COMM_MASK, FPGA_HEARTBEAT_MASTER_COMM_MASK );
		//Configure_FPGA_move_watchdog(FPGA_MOVE_WATCHDOG_HEARTBEAT_MASK, FPGA_MOVE_WATCHDOG_HEARTBEAT_MASK);
	}
	WriteFPGARegisters();
	ReadFPGARegisters();
}

static int FPGA_heartbeat_enable = 0;
static int FPGA_move_watchdog_enable  = 0;
static int FPGA_system_watchdog_enable = 0;
int FPGA_initialized = 0;

void Configure_FPGA_heartbeat( int mask, int value )
{
	volatile unsigned *pointer;
	int old;

	old = FPGA_heartbeat_enable;
	FPGA_heartbeat_enable = ((FPGA_heartbeat_enable &~mask)|(mask&value));
	if( old ^ FPGA_heartbeat_enable)
	{
		if( FPGA_heartbeat_enable == FPGA_HEARTBEAT_ENABLE_MASK )
		{
			_LOGmessage(0x0117,"reload FPGA_heartbeat", 0, 0);
			pointer = (unsigned *)FPGA_ADD_WORKMODE_HEARTBEAT_CNT_LSB;
			(*pointer) = (UNS16) FPGA_HB_counter_lsb_wr;

			pointer = (unsigned *)FPGA_ADD_WORKMODE_HEARTBEAT_CNT_MSB;
			(*pointer) = (UNS16) FPGA_HB_counter_msb_wr;
		}
		else
		{
			_LOGmessage(0x0118,"reset FPGA_heartbeat", 0, 0);
			pointer = (unsigned *)FPGA_ADD_WORKMODE_HEARTBEAT_CNT_LSB;
			(*pointer) = (UNS16) 0;

			pointer = (unsigned *)FPGA_ADD_WORKMODE_HEARTBEAT_CNT_MSB;
			(*pointer) = (UNS16) 0;
		}
	}
}


void Configure_FPGA_move_watchdog( int mask, int value )
{
	FPGA_move_watchdog_enable = ((FPGA_move_watchdog_enable &~mask)|(mask&value));
}


void Configure_FPGA_system_watchdog( int mask, int value )
{
	FPGA_system_watchdog_enable = ((FPGA_system_watchdog_enable &~mask)|(mask&value));
}


void ReadFPGARegisters( void )
{
	volatile unsigned *pointer, *pointer2;

	pointer = (unsigned *)FPGA_ADD_CONF;
	FPGA_configuration_rd = (UNS16)(*pointer);
	FPGA_mode = FPGA_configuration_rd & FPGA_REG_CONF_WORKING_MODE_RD_MASK;

	pointer = (unsigned *)FPGA_ADD_NODE_ID;
	FPGA_node = (UNS8)(*pointer);

//	pointer = (unsigned *)FPGA_ADD_WORKMODE_HEARTBEAT_CNT_LSB;
//	FPGA_HB_counter_lsb_rd = (UNS16)  (*pointer);
//	pointer = (unsigned *)FPGA_ADD_WORKMODE_HEARTBEAT_CNT_MSB;
//	FPGA_HB_counter_msb_rd = (UNS16) (*pointer);

	pointer = (unsigned *)FPGA_ADD_NCE_MEASURE;
	FPGA_NCE_measure_rd = (UNS16) (*pointer);

	pointer = (unsigned *)FPGA_ADD_REMOTE_CONTROL_CODE;
	FPGA_Remote_control_code = (UNS16) (*pointer);

	pointer  = (unsigned *)FPGA_ADD_REMOTE_CONTROL_MASK_LSW;
	pointer2 = (unsigned *)FPGA_ADD_REMOTE_CONTROL_MASK_MSW;
	FPGA_Remote_control_mask = (UNS32)(((UNS32)(*pointer2)<<16) | (UNS32)(*pointer));

	pointer = (unsigned *)FPGA_ADD_FPGA_ERRORS;
	FPGA_errors = (UNS16) (*pointer);

	//pointer = (unsigned *)FPGA_ADD_CURRENT;
	//FPGA_current = (UNS16) (*pointer);
}


void WriteFPGARegisters( void )
{
	volatile unsigned *pointer;
	UNS16 enc_err_reset;

	if( FPGA_heartbeat_enable == FPGA_HEARTBEAT_ENABLE_MASK )
	{
		pointer = (unsigned *)FPGA_ADD_WORKMODE_HEARTBEAT_CNT_LSB;
		(*pointer) = (UNS16) FPGA_HB_counter_lsb_wr;

		pointer = (unsigned *)FPGA_ADD_WORKMODE_HEARTBEAT_CNT_MSB;
		(*pointer) = (UNS16) FPGA_HB_counter_msb_wr;
	}

	if( FPGA_move_watchdog_enable == FPGA_MOVE_WATCHDOG_ENABLE_MASK )
	{
		pointer = (unsigned *)FPGA_ADD_MOVE_HEARTBEAT_CNT_LSB;
		(*pointer) = (UNS16) FPGA_HB_move_counter_lsb_wr;

		pointer = (unsigned *)FPGA_ADD_MOVE_HEARTBEAT_CNT_MSB;
		(*pointer) = (UNS16) FPGA_HB_move_counter_msb_wr;
	}
	Configure_FPGA_move_watchdog(FPGA_MOVE_WATCHDOG_ENABLE_MASK, 0);

	if( FPGA_system_watchdog_enable == FPGA_SYSTEM_WATCHDOG_ENABLE_MASK )
	{
		pointer = (unsigned *)FPGA_ADD_RESET_HEARTBEAT_CNT_LSB;
		(*pointer) = (UNS16) FPGA_HB_reset_counter_lsb_wr;

		pointer = (unsigned *)FPGA_ADD_RESET_HEARTBEAT_CNT_MSB;
		(*pointer) = (UNS16) FPGA_HB_reset_counter_msb_wr;
	}
	Configure_FPGA_system_watchdog(FPGA_SYSTEM_WATCHDOG_ENABLE_MASK, 0);

	if( Enable_remote_wr )
	{
		FPGA_WriteConfigurationRegister(FPGA_REG_CONF_ENABLE_REMOTE_WR_MASK, FPGA_REG_CONF_ENABLE_REMOTE_WR_MASK );
	}
	else
	{
		FPGA_WriteConfigurationRegister(FPGA_REG_CONF_ENABLE_REMOTE_WR_MASK, 0 );
	}

	//Remote control code
	pointer = (unsigned *)FPGA_ADD_REMOTE_CONTROL_CODE;
	(*pointer) = (UNS16) FPGA_Remote_control_code;

	pointer = (unsigned *)FPGA_ADD_REMOTE_CONTROL_MASK_LSW;
	(*pointer) = (UNS16) (FPGA_Remote_control_mask & 0xFFFF);

	pointer = (unsigned *)FPGA_ADD_REMOTE_CONTROL_MASK_MSW;
    (*pointer) = (UNS16) (FPGA_Remote_control_mask>>16);

    enc_err_reset = (UNS16) ((Encoder2_Error_Reset<<1) | Encoder1_Error_Reset);
    FPGA_WriteGenericRegister(FPGA_ENCODER_ERROR_RESET_MASK, enc_err_reset, FPGA_ADD_FPGA_ERRORS);

	// si quitamos el callback descomentar esto
	//pointer = (unsigned *)FPGA_ADD_ACOUSTIC_MODE;
	//(*pointer) = (UNS16) FPGA_acoustic_mode_wr;

	// DAC code
/*	pointer = (unsigned *)FPGA_ADD_DAC_1;
	(*pointer) = (UNS16) FPGA_DAC_1;

	pointer = (unsigned *)FPGA_ADD_DAC_2;
    (*pointer) = (UNS16) FPGA_DAC_2;

    pointer = (unsigned *)FPGA_ADD_DAC_3;
	(*pointer) = (UNS16) FPGA_DAC_3;

	pointer = (unsigned *)FPGA_ADD_DAC_4;
	(*pointer) = (UNS16) FPGA_DAC_4;
	*/
}
/* ************************** FPGA Registers interface */


/*!	Write the FPGA configuration register */
UNS32 FPGA_WriteConfigurationRegister( UNS16 mask, UNS16 val )
{
	UNS16 FPGA_intf_reg;
	volatile unsigned *pointer;

	pointer = (unsigned *)FPGA_ADD_CONF;
	FPGA_intf_reg = (UNS16) (*pointer);

	FPGA_intf_reg = ((FPGA_intf_reg & ~mask) | (mask & val) );
	(*pointer) = (UNS16) FPGA_intf_reg;

	return 0;
}

/*!	Write the FPGA configuration register */
UNS32 FPGA_WriteGenericRegister( UNS16 mask, UNS16 val, UNS32 address)
{
	UNS16 FPGA_bridge_reg;
	volatile unsigned *pointer;

	pointer = (unsigned *)address;
	FPGA_bridge_reg = (UNS16) (*pointer);

	FPGA_bridge_reg = ((FPGA_bridge_reg & ~mask) | (mask & val) );
	(*pointer) = (UNS16) FPGA_bridge_reg;

	return 0;
}

/*!	Start/Reset the NCE (Non-contacting encoder) sensor mapped in to the specified channel */
void FPGA_SetNCE( int mode, int channel )
{
	volatile unsigned *pointer = (unsigned *)FPGA_ADD_NCE_CFG;

	channel &= 3;
	mode    &= 1;
	(*pointer) = (UNS16)((mode<<FPGA_REG_NCE_CFG_MODE_WR_desp) |
						(channel<<FPGA_REG_NCE_CFG_CHANNEL_WR_desp));
}

/*!	Enable/Disable the trinquete support feature */
void FPGA_SetTrinquete( int mode )
{
	volatile unsigned *pointer = (unsigned *)FPGA_ADD_TRINQUETE_CFG;

	mode &= 1;
	(*pointer) = (UNS16)((mode<<FPGA_REG_TRINQUETE_CFG_MODE_WR_desp));
}

void InitFPGA( void )
{
	volatile unsigned *pointer;

	pointer = (unsigned *)FPGA_ADD_CONF;
	(*pointer) = (UNS16) 0x08;

	//Mode Safe
	pointer = (unsigned *)FPGA_ADD_WORKMODE_HEARTBEAT_CNT_LSB;
	(*pointer) = (UNS16) 0;
	pointer = (unsigned *)FPGA_ADD_WORKMODE_HEARTBEAT_CNT_MSB;
	(*pointer) = (UNS16) 0;

	//NCE
	pointer = (unsigned *)FPGA_ADD_NCE_CFG;
	(*pointer) = (UNS16) 0;

	//Acoustic
	pointer = (unsigned *)FPGA_ADD_ACOUSTIC_MODE;
	(*pointer) = (UNS16) 0;

	//Reset Move Watchdog
	pointer = (unsigned *)FPGA_ADD_MOVE_HEARTBEAT_CNT_LSB;
	(*pointer) = (UNS16) 0;
	pointer = (unsigned *)FPGA_ADD_MOVE_HEARTBEAT_CNT_MSB;
	(*pointer) = (UNS16) 0;

}

/*UNS16 FPGA_current_value(UNS16 val)
{
	UNS16 current_value_int = 0;
	UNS32 current_value = 0;
	UNS16 final = 0;
	UNS16 sign = 0;
	UNS16 mask = 0x1FFF;

    //I(mA) = (x-4096)*1000/80 = (x-4096)*12,5;

	sign = (mask & val)>>12;
    if (sign == 1){
	current_value_int = (UNS32)((mask & val)-4096); //y=x-4096;
    }
    else
    {
      current_value_int = 4096-(mask & val);
    }
	FPGA_final_current = (UNS32)current_value_int;
    current_value = (((UNS32)current_value_int)<<3)+ (((UNS32)current_value_int)<<2) + (((UNS32)current_value_int)>>1);//I(mA)= y*12,5 = y*8+y*4+y*0.5 //
    FPGA_final_current_2 = current_value;
    final = (UNS16)(current_value & 0x0000FFFF);

	return final;
}*/

unsigned int get_position_resolution(int sensor){

	unsigned int position_encoder_resolution;
	/* get position_encoder_resolution */
	switch(sensor) {
		case P_ENC1:
			position_encoder_resolution = Encoder1_Resolution;
			break;
		case P_ENC2:
			position_encoder_resolution = Encoder2_Resolution;
			break;
		case P_AN1:
		case P_AN2:
		case P_AN3:
		case P_AN4:
			position_encoder_resolution = 1;
			break;
		case P_MAG_ENC:
			position_encoder_resolution = 0x01 << GPIO_Property_1;
			break;
		case P_PWM_ENC:
			position_encoder_resolution = 0x01 << PWM_sensor_Resolution;
			break;
		case P_RESOLVER:
			position_encoder_resolution = 3600;
			break;
		case P_HYBRID:
			break;
		default:
			_WARNINGmessage(0xffed, 0x80, 0x2021, "Wrong position sensor", 0, 0);
			break;
	}
	return position_encoder_resolution;
}

unsigned int get_velocity_resolution(int sensor){

	unsigned int velocity_encoder_resolution;
	switch(sensor) {
			case P_ENC1:
				velocity_encoder_resolution = Encoder1_Resolution;
				break;
			case P_ENC2:
				velocity_encoder_resolution = Encoder2_Resolution;
				break;
			case P_AN1:
			case P_AN2:
			case P_AN3:
			case P_AN4:
				velocity_encoder_resolution = 1;
				break;
			case P_MAG_ENC:
				velocity_encoder_resolution = 0x01 << GPIO_Property_1;
				break;
			case P_PWM_ENC:
				velocity_encoder_resolution = 0x01 << PWM_sensor_Resolution;
				break;
			case P_RESOLVER:
				velocity_encoder_resolution = 3600;
				break;
			case P_HYBRID:
				break;
			default:
				_WARNINGmessage(0xffed, 0x80, 0x2020, "Wrong velocity sensor", 0, 0);
				break;
		}
	return velocity_encoder_resolution;
}

void get_pos_num_den(unsigned int pos_place, unsigned int vel_place, unsigned int pos_res, unsigned int vel_res, unsigned long *num, unsigned long *den){
	/* get Position_factor, pos2int_factor and vel2int_factor */
	switch(vel_place) {
	case PLACE_BACK:   /* internal units = velocity units */
		*num = vel_res * Gear_ratio_Motor_revolutions * Feed_constant_Shaft_revolutions;
		*den = Gear_ratio_Shaft_revolutions * Feed_constant_Feed;
		break;
	case PLACE_FRONT:
		switch(pos_place) {
		case PLACE_BACK:   /* internal units = position units */
			*num = pos_res * Gear_ratio_Motor_revolutions * Feed_constant_Shaft_revolutions;
			*den = Gear_ratio_Shaft_revolutions * Feed_constant_Feed;
			break;
		case PLACE_FRONT:  /* internal units = velocity units */
			*num = vel_res * Feed_constant_Shaft_revolutions;
			*den = Feed_constant_Feed;
			break;
		case PLACE_EXT:    /* internal units = velocity units */
			*num = vel_res * Feed_constant_Shaft_revolutions;
			*den = Feed_constant_Feed;
			break;
		}
		break;
	case PLACE_EXT:   /* internal units = position units */
		switch(pos_place) {
		case PLACE_BACK:
			*num = pos_res * Gear_ratio_Motor_revolutions * Feed_constant_Shaft_revolutions;
			*den = Gear_ratio_Shaft_revolutions * Feed_constant_Feed;
			break;
		case PLACE_FRONT:
			*num = pos_res * Feed_constant_Shaft_revolutions;
			*den = Feed_constant_Feed;
			break;
		case PLACE_EXT:    /* all units are the same */
			*num = 1;
			*den = 1;
			break;
		}
		break;
		}
}

void get_pos_vel_factors(int pos_place, int vel_place, unsigned int pos_res, unsigned int vel_res){
	/* get Position_factor, pos2int_factor and vel2int_factor */
	switch(vel_place) {
	case PLACE_BACK:   /* internal units = velocity units */
		vel2int_factor_num = 1;
		vel2int_factor_den = 1;
		switch(pos_place) {
		case PLACE_BACK:
			pos2int_factor_num = vel_res;
			pos2int_factor_den = pos_res;
			break;
		case PLACE_FRONT:
			pos2int_factor_num = vel_res * Gear_ratio_Motor_revolutions;
			pos2int_factor_den = pos_res * Gear_ratio_Shaft_revolutions;
			break;
		case PLACE_EXT:
			pos2int_factor_num = vel_res * Gear_ratio_Motor_revolutions * Feed_constant_Shaft_revolutions;
			pos2int_factor_den = Gear_ratio_Shaft_revolutions * Feed_constant_Feed;
			break;
		}
		break;
		case PLACE_FRONT:
			switch(pos_place) {
			case PLACE_BACK:   /* internal units = position units */
				pos2int_factor_num = 1;
				pos2int_factor_den = 1;
				vel2int_factor_num = pos_res * Gear_ratio_Motor_revolutions;
				vel2int_factor_den = vel_res * Gear_ratio_Shaft_revolutions;
			break;
			case PLACE_FRONT:  /* internal units = velocity units */
				vel2int_factor_num = 1;
				vel2int_factor_den = 1;
				pos2int_factor_num = vel_res;
				pos2int_factor_den = pos_res;
				break;
			case PLACE_EXT:    /* internal units = velocity units */
				vel2int_factor_num = 1;
				vel2int_factor_den = 1;
				pos2int_factor_num = vel_res * Feed_constant_Shaft_revolutions;
				pos2int_factor_den = Feed_constant_Feed;
				break;
			}
			break;
			case PLACE_EXT:   /* internal units = position units */
				pos2int_factor_num = 1;
				pos2int_factor_den = 1;
				switch(pos_place) {
				case PLACE_BACK:
					vel2int_factor_num = pos_res * Gear_ratio_Motor_revolutions * Feed_constant_Shaft_revolutions;
					vel2int_factor_den = Gear_ratio_Shaft_revolutions * Feed_constant_Feed;
					break;
				case PLACE_FRONT:
					vel2int_factor_num = pos_res * Feed_constant_Shaft_revolutions;
					vel2int_factor_den = Feed_constant_Feed;
					break;
				case PLACE_EXT:    /* all units are the same */
					vel2int_factor_num = 1;
					vel2int_factor_den = 1;
				break;
				}
				break;
	}
}

long int2ext_place(long internal_pos, long offset, int pos_per, int pos_place, int vel_per, int vel_place)
{
	long long tmp;
	unsigned long numerator;
	unsigned long denominator;
	unsigned int vel_res;
	unsigned int pos_res;

	vel_res = get_velocity_resolution(vel_per);
	pos_res = get_position_resolution(pos_per);
	get_pos_num_den(pos_place, vel_place, pos_res, vel_res, &numerator, &denominator);

	tmp = (long long)internal_pos * denominator;
	if(tmp >= 0) tmp += (numerator >> 1); else tmp -= (long long)(numerator >> 1);
	tmp /= numerator;
	tmp = (Polarity & 0x80) ? -tmp : tmp;
	return tmp + offset;		/* [position units] */
}

void manage_AN1(void)
{
	if (AN1_used_for_position){
		AN1_position = get_AN1(INIT_PARAMS);
		AN1_counts = an1;
	}
	else
	{
		AN1_position = 0;
		AN1_counts = 0;
	};
}

void manage_AN2(void)
{
	if (AN2_used_for_position){
		AN2_position = get_AN2(INIT_PARAMS);
		AN2_counts = an2;
	}
	else
	{
		AN2_position = 0;
		AN2_counts = 0;
	};
}

void manage_Enc1(void)
{
	if (Enc1_used_for_position){
		Enc1_position = int2ext_place(get_Enc1(INIT_PARAMS),absolute_sensor, P_ENC1, Enc1_place, Velocity_sensor_Peripheral, Velocity_sensor_Place);
		Enc1_counts = get_Enc1(INIT_PARAMS);
	}
	else
	{
		Enc1_position = 0;
		Enc1_counts = 0;
	};
}

void manage_Enc2(void)
{
	if (Enc2_used_for_position){
		Enc2_position = int2ext_place(get_Enc2(INIT_PARAMS),absolute_sensor, P_ENC2, Enc2_place, Velocity_sensor_Peripheral, Velocity_sensor_Place);
		Enc2_counts = get_Enc2(INIT_PARAMS);
	}
	else
	{
		Enc2_position = 0;
		Enc2_counts = 0;
	};
}

void manage_NCE(void)
{
	if (NCE_used_for_position){
		NCE_position = get_hybrid(INIT_PARAMS);
		NCE_counts = FPGA_NCE_measure_rd;
	}
	else
	{
		NCE_position = 0;
		NCE_counts = 0;
	};
}

void manage_Resolver(void)
{
	if (Resolver_used_for_position){
		Resolver_position = int2ext_place(get_resolver_acc(INIT_PARAMS),absolute_sensor, P_RESOLVER, Resolver_place, Velocity_sensor_Peripheral, Velocity_sensor_Place);
		Resolver_counts = get_resolver_acc(INIT_PARAMS);
	}
	else
	{
		Resolver_position = 0;
		Resolver_counts = 0;
	};
}

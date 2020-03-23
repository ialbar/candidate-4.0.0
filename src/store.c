/*!
 * \file store.c
 * \brief File containing functions to store data into/from EEPROM memory
 */

/*--------includes------------*/
#include "amc.h"
#include "i2c.h"
#include "fpga.h"
#include "store.h"

#define I2C_EEPROM_ATTEMPTS   100 //100ms

#pragma DATA_SECTION( __store_array, "storedata" );
/*!
 * \var store_array
 * \brief Array to save configuration data
 *
 * This array contains store_data structs containing pointers and size
 * of the configuration parameters from the Object Dictionary.
 *
 * First register MUST contain eeprom_nodeid
 */
static Uint16 eeprom_nodeid = 0;

store_data __store_array[] = {
{ (void*) &eeprom_nodeid,                                2 },
{ (void*) &Power_board,                                  1 },
{ (void*) &Position_dimension_index,                     1 },
{ (void*) &Position_notation_index,                      1 },
{ (void*) &Velocity_notation_index,                      1 },
{ (void*) &Acceleration_notation_index,                  1 },
{ (void*) &Gear_ratio_Motor_revolutions,                 4 },
{ (void*) &Gear_ratio_Shaft_revolutions,                 4 },
{ (void*) &Feed_constant_Feed,                           4 },
{ (void*) &Feed_constant_Shaft_revolutions,              4 },
{ (void*) &Max_profile_velocity,                         4 },
{ (void*) &Max_acceleration,                             4 },
{ (void*) &Max_deceleration,                             4 },
{ (void*) &Profile_velocity,                             4 },
{ (void*) &Profile_acceleration,                         4 },
{ (void*) &Profile_deceleration,                         4 },
{ (void*) &Quick_stop_deceleration,                      4 },
{ (void*) &Quick_stop_option_code,                       2 },
{ (void*) &Home_offset,                                  4 },
{ (void*) &Software_position_limit_Min_position_limit,   4 },
{ (void*) &Software_position_limit_Max_position_limit,   4 },
{ (void*) &Max_motor_speed,                              4 },
{ (void*) &Motor_rated_current,                          4 },
{ (void*) &Max_current,                                  2 },
{ (void*) &Max_acceleration_current,                     2 },
{ (void*) &Following_error_window,                       4 },
{ (void*) &Following_error_time_out,                     2 },
{ (void*) &Position_window,                              4 },
{ (void*) &Position_window_time,                         2 },
{ (void*) &Velocity_window,                              2 },
{ (void*) &Velocity_window_time,                         2 },
{ (void*) &Velocity_threshold,                           2 },
{ (void*) &Velocity_threshold_time,                      2 },
{ (void*) &Max_slippage,                                 4 },
{ (void*) &Velocity_control_margin,                      4 },
{ (void*) &Position_control_margin,                      4 },
{ (void*) &AnalogIn1_Gain_num,                           2 },
{ (void*) &AnalogIn1_Gain_div,                           2 },
{ (void*) &AnalogIn1_Offset,                             4 },
{ (void*) &AnalogIn2_Gain_num,                           2 },
{ (void*) &AnalogIn2_Gain_div,                           2 },
{ (void*) &AnalogIn2_Offset,                             4 },
{ (void*) &AnalogIn3_Gain_num,                           2 },
{ (void*) &AnalogIn3_Gain_div,                           2 },
{ (void*) &AnalogIn3_Offset,                             4 },
{ (void*) &AnalogIn4_Gain_num,                           2 },
{ (void*) &AnalogIn4_Gain_div,                           2 },
{ (void*) &AnalogIn4_Offset,                             4 },
{ (void*) &GPIO_Type,                                    4 },
{ (void*) &GPIO_Property_1,                              4 },
{ (void*) &GPIO_Property_2,                              4 },
{ (void*) &Encoder1_Polarity,                            1 },
{ (void*) &Encoder1_Resolution,                          2 },
{ (void*) &Encoder2_Polarity,                            1 },
{ (void*) &Encoder2_Resolution,                          2 },
{ (void*) &Velocity_sensor_Peripheral,                   1 },
{ (void*) &Velocity_sensor_Place,                        1 },
{ (void*) &Position_sensor_Peripheral,                   1 },
{ (void*) &Position_sensor_Place,                        1 },
{ (void*) &Absolute_pos_sensor_Peripheral,               1 },
{ (void*) &Absolute_pos_sensor_Place,                    1 },
{ (void*) &Power_voltage_limit[0],                       2 },
{ (void*) &Power_voltage_limit[1],                       2 },
{ (void*) &Motor_type,                                   1 },
{ (void*) &Brushless_sequence_type,                      1 },
{ (void*) &DC_motor_polarity,                            1 },
{ (void*) &Polarity,                                     1 },
{ (void*) &Brake1_Mode,                                  1 },
{ (void*) &Brake2_Mode,                                  1 },
{ (void*) &Axle_brake_Polarity,                          1 },
{ (void*) &Axle_brake_Position,                          1 },
{ (void*) &Axle_brake_Delay,                             2 },
{ (void*) &Axle_clutch_Polarity,                         1 },
{ (void*) &Axle_clutch_Position,                         1 },
{ (void*) &Axle_clutch_Delay,                            2 },
{ (void*) &Axle_clutch_Needed_to_brake,                  1 },
{ (void*) &Velocity_control_parameter_set[0],            2 },
{ (void*) &Velocity_control_parameter_set[2],            2 },
{ (void*) &Velocity_control_parameter_set[3],            2 },
{ (void*) &Velocity_control_parameter_set[4],            2 },
{ (void*) &Velocity_control_parameter_set[5],            2 },
{ (void*) &Position_control_parameter_set[0],            2 },
{ (void*) &Position_control_parameter_set[1],            2 },
{ (void*) &Position_control_parameter_set[2],            2 },
{ (void*) &Position_control_parameter_set[3],            2 },
{ (void*) &Position_control_parameter_set[4],            2 },
{ (void*) &Modes_of_operation,                           1 },
{ (void*) &Enable_redundancy,                            1 },
{ (void*) &Control_effort_offset,                        2 },
{ (void*) &Quick_stop_offset,                            4 },
{ (void*) &Quick_ramp_time,                              4 },
{ (void*) &GPI_Polarity,                                 1 },
{ (void*) &GPI_Interrupt_mask,                           1 },
{ (void*) &Detents_list_D1,                              4 },
{ (void*) &Detents_list_D2,                              4 },
{ (void*) &Detents_list_D3,                              4 },
{ (void*) &Detents_list_D4,                              4 },
{ (void*) &Detents_list_D5,                              4 },
{ (void*) &Detents_list_D6,                              4 },
{ (void*) &Detents_list_D7,                              4 },
{ (void*) &Detents_list_D8,                              4 },
{ (void*) &Detents_config_Max_input_distance,            4 },
{ (void*) &Detents_config_Max_output_distance,           4 },
{ (void*) &Detents_config_Max_velocity,                  4 },
{ (void*) &Detents_config_Max_duty,                      2 },
{ (void*) &Detents_config_Skip_distance,                 4 },
{ (void*) &Detents_config_Enable_detents,                2 },
{ (void*) &Analog_optocoupler_gain,                      2 },
{ (void*) &Temperature_limit,                            2 },
{ (void*) &Motor_Temperature_limit,                      2 },
{ (void*) &Enable_Zero_Pwm_Window,                       1 },
{ (void*) &Enable_Ramp_PWM_mode,                         1 },
{ (void*) &BLAC_Vect_Control,                            1 },
{ (void*) &Current_control_parameter_set_Iq_ref_off,     4 },
{ (void*) &FPGA_HB_counter_lsb_wr,                       2 },
{ (void*) &FPGA_HB_counter_msb_wr,                       2 },
{ (void*) &FPGA_HB_move_counter_lsb_wr,                  2 },
{ (void*) &FPGA_HB_move_counter_msb_wr,                  2 },
{ (void*) &FPGA_HB_reset_counter_lsb_wr,                 2 },
{ (void*) &FPGA_HB_reset_counter_msb_wr,                 2 },
{ (void*) &Enable_remote_wr,                             1 },
{ (void*) &FPGA_Remote_control_code,                     2 },
{ (void*) &FPGA_Remote_control_mask,                     4 },
{ (void*) &NCE_Polarity,                                 1 },
{ (void*) &Pot_Polarity,                                 1 },
{ (void*) &AnalogNCE_Offset,                             4 },
{ (void*) &AnalogHybrid_Gain_num,                        2 },
{ (void*) &AnalogHybrid_Gain_div,                        2 },
{ (void*) &AnalogHybrid_Offset,                          4 },
{ (void*) &Hybrid_n_turns,                               1 },
{ (void*) &Hybrid_analog_sel,                            1 },
{ (void*) &Hybrid_redundancy_error,                      2 },
{ (void*) &Current_control_parameter_set_Kp,             4 },
{ (void*) &Current_control_parameter_set_Ki,             4 },
{ (void*) &Current_control_parameter_set_Kc,             4 },
{ (void*) &Current_control_parameter_set_Kff,            4 },
{ (void*) &Current_control_parameter_set_Divisor,        4 },
{ (void*) &Update_current_control_parameter_refs,        1 },
{ (void*) &Assisted_mode_current_demand,                 2 },
{ (void*) &Assisted_mode_conf_Analog_input,              1 },
{ (void*) &Assisted_mode_conf_Center,                    2 },
{ (void*) &Assisted_mode_conf_Threshold,                 2 },
{ (void*) &Assisted_mode_conf_Window_time,               2 },
{ (void*) &Assisted_mode_conf_Gain_numerator,            2 },
{ (void*) &Assisted_mode_conf_Gain_divisor,              2 },
{ (void*) &Assisted_mode_conf_Max_velocity,              4 },
{ (void*) &Assisted_mode_conf_Kp,                        2 },
{ (void*) &Assisted_mode_conf_Ki,                        2 },
{ (void*) &Assisted_mode_conf_Kc,                        2 },
{ (void*) &Assisted_mode_conf_control_parameter_Divisor, 2 },
{ (void*) &Assisted_mode_conf_filter,                    1 },
{ (void*) &Assisted_mode_conf_ramp,                      2 },
{ (void*) &Assisted_mode_control_margin,                 2 },
{ (void*) &Assisted_mode_pos_Kp,                         2 },
{ (void*) &Assisted_mode_pos_Ki,                         2 },
{ (void*) &Assisted_mode_pos_Kd,                         2 },
{ (void*) &Assisted_mode_pos_Kc,                         2 },
{ (void*) &Assisted_mode_pos_Kff,                        2 },
{ (void*) &Assisted_mode_pos_control_parameter_Divisor,  2 },
{ (void*) &Assisted_mode_debug_position,                 4 },
{ (void*) &Assisted_mode_debug_velocity,                 4 },
{ (void*) &Assisted_mode_dentent_active,                 1 },
{ (void*) &Assisted_mode_error_filter,                   1 },
{ (void*) &Outofbound_position_limit_Min,             4 },
{ (void*) &Outofbound_position_limit_Max,             4 },
{ 0,0 } /* Force a NULL pointer at the array's end. */
};


static UNS16 store_status = STORE_INACTIVE;
static UNS16 store_fsm = 0;
static bool puto_flag = 0;
static Uint8 page = 0;
static int delay=0;
static int attempts = 0;

/*!
 * \var conf_buffer
 * \brief variable that contains all of the configuration data.
 * conf_buffer has configuration data consecutively saved and ready
 * to save into EEPROM memory.
 * Last 2 bytes are reserved for checksum
 */
static UNS16 conf_buffer16[STORE_BUFFER16_LENGTH];
static Uint8 data_page[I2C_EEPROM_PAGE_SIZE];


/* Local functions *************************************/
static int  fill_conf_buffer(void);
static int  erase_conf_buffer(void);
//static void display_conf_buffer(void);
//static void display_data_page(void);
static int  write_od_from_eeprom(void);
static int  M24xx32AMessageProcessed( void );
/* ****************************************************/

/*!
 * \brief Returns the NodeId stored in EEPROM
 * \return NodeId stored in EEPROM
 */
Uint16 store_get_nodeid( void )
{
	return eeprom_nodeid;
}


/*!
 * \brief Returns the Store status
 * \return Store Status
 */
Uint16 store_get_status( void )
{
	return store_status;
}


/*!
 * \brief Returns the Store FSM state
 * \return Store FSM state
 */
Uint16 store_get_fsm( void )
{
	return store_fsm;
}

/*!
 * \brief Initializes the EEPROM callbacks to I2C driver
 */
void InitStore( void )
{
	eeprom_nodeid = (UNS16)FPGA_node;
	M24xx32AInit( I2C_EEPROM_MIN_I2CADD, (t24xx32ACallbackWR)&I2CA_WriteData, (t24xx32ACallbackRD)&I2CA_SendReadData );
}


/*!
 * \brief Function to copy configuration into EEPROM memory
 *
 * This function:
 * \li Copies configuration data into buffer
 * \li Sets the Store status to WRITE
 * \li Initializes the FSM WRITE state
 */
void save_configuration( void )
{
	Store_configuration = 0x79737562;   /* "busy" in ASCII */
	eeprom_nodeid = (UNS16)FPGA_node;
	//if( store_status == STORE_INACTIVE )
	{
		if( !fill_conf_buffer() )
		{
			//display_conf_buffer();
			store_fsm = (UNS16)(STORE_WRITE_START);
			store_status = STORE_WRITING;
		}
	}
}


/*!
 * \brief Function to load configuration from EEPROM memory
 *
 * This function:
 * \li Sets the Store status to READ
 * \li Initializes the FSM READ state
 */
void load_configuration( void )
{
//	if((store_status == STORE_INACTIVE))
	{
		store_fsm = (UNS16)(STORE_READ_START);
		store_status = STORE_READING;
	}
}


/*!
 * \brief Function to clear the EEPROM memory
 *
 * This function:
 * \li Clears buffer
 * \li Sets the Store status to WRITE
 * \li Initializes the FSM WRITE state
 */
void clear_configuration()
{
	Store_configuration = 0x79737562;   /* "busy" in ASCII */
//	if( store_status == STORE_INACTIVE )
	{
		if( !erase_conf_buffer() )
		{
			store_fsm = (UNS16)(STORE_WRITE_START);
			store_status = STORE_WRITING;
		}
	}
}


/*!
 * \brief Writes the data to store into the configuration buffer
 * \return 0 if ok. 1 if data size if above available size or memory set error
 */
static int fill_conf_buffer(void)
{
	unsigned int i=0, j=0, len=0;
	UNS16 checksum;

	// check size
	i=0;
	while ( __store_array[i].data_ptr != NULL )
	{
		if( __store_array[i].size == 4 ) len+= 2;  //UNS8 and UNS16 occupy 1 register, UNS32 occupies 2
		else len +=1;
		i++;
	}
	if( len > (STORE_BUFFER16_LENGTH-1) )  //checksum is written in the last one
		return 1;

	//write data into the configuration buffer
	i=0;
	while ( __store_array[i].data_ptr != NULL )
	{
		if( __store_array[i].size == 1 )
		{
			conf_buffer16[j] = (UNS16)*((UNS8*)__store_array[i].data_ptr);
		}
		else if( __store_array[i].size == 2 )
		{
			conf_buffer16[j] = (UNS16)*((UNS8*)__store_array[i].data_ptr);
		}
		else if( __store_array[i].size == 4 )
		{
			conf_buffer16[j] = (UNS16)((*((UNS32*)__store_array[i].data_ptr) >> 0) & 0xFFFF);
			j++;
			conf_buffer16[j] = (UNS16)((*((UNS32*)__store_array[i].data_ptr) >>16) & 0xFFFF);
		}
		i++;
		j++;
	}

	// compute checksum
	for(i=0, checksum=0; i<(STORE_BUFFER16_LENGTH-1); i++)
	{
		checksum += conf_buffer16[i];
	}
	checksum = -checksum;

	//copy checksum
	conf_buffer16[STORE_BUFFER16_LENGTH-1] = checksum;

	return 0;
}


/*!
 * \brief Resets the configuration buffer and sets and incorrect checksum
 * \return 0 if ok
 */
static int erase_conf_buffer(void)
{
	int i;
	UNS16 checksum = 0x5A5A;

	for( i=0; i< STORE_BUFFER16_LENGTH-1; i++ )
		conf_buffer16[i] = 0;

	// write checksum so it gives an error
	conf_buffer16[STORE_BUFFER16_LENGTH-1] = checksum;

	return 0;
}


/*!
 * \brief Writes into the FPGA register the configuration buffer data for debugging
  */
/*static void display_conf_buffer(void)
{
	int j;
	volatile UNS16 *pointer = (UNS16 *)FPGA_ADD_FIFO;

	for( j=0; j<STORE_BUFFER16_LENGTH; j++ )
	{
		(*pointer) = (UNS16)(conf_buffer16[j]);
	}
}*/


/*!
 * \brief Writes into the FPGA register the configuration buffer data for debugging
  */
/*static void display_data_page(void)
{
	int j;
	volatile unsigned *pointer = (unsigned *)FPGA_ADD_FIFO;

	for( j=0; j<I2C_EEPROM_PAGE_SIZE; j++ )
	{
		(*pointer) = (UNS8)data_page[j];
	}
}*/


/*!
 * \brief Writes the store data with the values of the configuration buffer
 * \return error status: 0(no error)/1(error)
 */
static int write_od_from_eeprom(void)
{
	volatile unsigned *pointer = (unsigned *)FPGA_ADD_FIFO;
	int i=0, j=0;
	UNS16 checksum;

	// verify checksum
	for(i=0, checksum=0; i<STORE_BUFFER16_LENGTH; ++i)
	{
		checksum += conf_buffer16[i];
	}

	if(checksum == 0)
	{
		i=0;
		j=0;
		while ( __store_array[i].data_ptr != NULL )
		{
			if( __store_array[i].size == 1 )
			{
				*((UNS8*)__store_array[i].data_ptr) = (UNS8)(conf_buffer16[j]);
				(*pointer) = (UNS16)(conf_buffer16[j]);
			}
			else if( __store_array[i].size == 2 )
			{
				UNS16 aux = (conf_buffer16[j]);
				*((UNS16*)__store_array[i].data_ptr) = (UNS16)(aux);
				(*pointer) = (UNS16)(aux);
			}
			else if( __store_array[i].size == 4 )
			{
				UNS32 aux = (UNS32)((UNS32)((conf_buffer16[j+1])&0xFFFF)<<16) | (UNS32)(conf_buffer16[j]&0xFFFF);
				(*pointer) = (UNS16) (conf_buffer16[j]);
				(*pointer) = (UNS16) (conf_buffer16[j+1]);
				j++;
				*((UNS32*)__store_array[i].data_ptr) = (UNS32)(aux);
			}
			i++;
			j++;
		}
		return 0;
	}
	return 1;
}


/*!
 * \brief Used to check if the I2C command has been sent
 * \return 1(finished)/0(busy)
 */

static int M24xx32AMessageProcessed( void )
{
	return( (int)puto_flag );
}



/*!
 * \brief Function to be called every ms to handle EEPROM accesses
*/
void store_service_routine(void)
{
	int reg=0;

	if (store_status == STORE_INACTIVE)
	{
		return;
	}

	if(store_status == STORE_WRITING)
	{
		if( store_fsm == STORE_WRITE_START )
		{
			page = 0;
			attempts = 0;
			store_fsm = (UNS16)(STORE_EEPROM_ACCESS);
		}
		else if( store_fsm == STORE_EEPROM_ACCESS )
		{
			reg = page * 16;
			{
				int i;
				for( i=0; i<16; i++ )
				{
					data_page[(i*2)]   = (conf_buffer16[reg+i] >> 0) & 0xFF;
					data_page[(i*2)+1] = (conf_buffer16[reg+i] >> 8) & 0xFF;
				}
			}
			//display_data_page();
			if( !M24xx32AWritePage ( I2C_EEPROM_PAGE(page), data_page ) )
			{
				attempts = 0;
//store_fsm = (UNS16)(STORE_EEPROM_WAIT_MESSAGE_PROCESSED);
				store_fsm = (UNS16)(STORE_EEPROM_WAIT_BUSY);
			}
			else
			{
				if( ++attempts == I2C_EEPROM_ATTEMPTS )
				{
					attempts = 0;
					store_fsm = (UNS16)(STORE_EEPROM_ERROR);
					store_status = (UNS16)STORE_INACTIVE;
				}
			}
		}
//      NOTA: este estado no es valido para la release
//		http://svn.sedecal.com/software/Embedded%20Drivers/TMS320F28XXX/I2C/tags/V1R0.0
//      porque no utiliza el puto_flag
//		else if( store_fsm == STORE_EEPROM_WAIT_MESSAGE_PROCESSED )
//		{
//			if( M24xx32AMessageProcessed() )
//			{
//				attempts = 0;
//				delay = 0;
//				store_fsm = (UNS16)(STORE_EEPROM_WAIT_BUSY);
//			}
//			else
//			{
//				if( ++attempts == I2C_EEPROM_ATTEMPTS )
//				{
//					attempts = 0;
//					store_fsm = (UNS16)(STORE_EEPROM_ERROR);
//					store_status = (UNS16)STORE_INACTIVE;
//				}
//			}
//		}
		else if( store_fsm == STORE_EEPROM_WAIT_BUSY )
		{
			if( !I2CA_IsBusy() )
			{
				if( ++delay == 10 )
				{
					delay = 0;
					if( ((page+1)*16) < STORE_BUFFER16_LENGTH )
					{
						page++;
						attempts = 0;
						store_fsm = (UNS16)(STORE_PAGE_COMPLETED);
						store_fsm = (UNS16)(STORE_EEPROM_ACCESS);
					}
					else
					{
						store_fsm = (UNS16)(STORE_PAGE_COMPLETED);
						store_fsm = (UNS16)(STORE_WRITE_COMPLETED);
						store_status = STORE_INACTIVE;
					}
				}
			}
			else
			{
				if( ++attempts == I2C_EEPROM_ATTEMPTS )
				{
					attempts = 0;
					store_fsm = (UNS16)(STORE_EEPROM_ERROR);
					store_status = (UNS16)STORE_INACTIVE;
				}
			}
		}
	}
	else if(store_status == STORE_READING)
	{
		if( store_fsm == STORE_READ_START )
		{
			page = 0;
			attempts = 0;
			store_fsm = (UNS16)(STORE_EEPROM_ACCESS);
		}
		else if( store_fsm == STORE_EEPROM_ACCESS )
		{
			puto_flag = 0;
			if( !M24xx32AReadPage( I2C_EEPROM_PAGE(page), data_page, &puto_flag ) )
			{
				attempts = 0;
				store_fsm = (UNS16)(STORE_EEPROM_WAIT_MESSAGE_PROCESSED);
			}
			else
			{
				if( ++attempts == I2C_EEPROM_ATTEMPTS )
				{
					attempts = 0;
					store_fsm = (UNS16)(STORE_EEPROM_ERROR);
					store_status = (UNS16)STORE_INACTIVE;
				}
			}
		}
		else if( store_fsm == STORE_EEPROM_WAIT_MESSAGE_PROCESSED )
		{
			if( M24xx32AMessageProcessed() )
			{
				attempts = 0;
				delay = 0;
				store_fsm = (UNS16)(STORE_EEPROM_WAIT_BUSY);
			}
			else
			{
				if( ++attempts == I2C_EEPROM_ATTEMPTS )
				{
					attempts = 0;
					store_fsm = (UNS16)(STORE_EEPROM_ERROR);
					store_status = (UNS16)STORE_INACTIVE;
				}
			}
		}
		else if( store_fsm == STORE_EEPROM_WAIT_BUSY )
		{
			if( !I2CA_IsBusy() )
			{
				if( ++delay == 10 )
				{
					delay = 0;
					reg = page * 16;
					{
						int i;
						for( i=0; i<16; i++ )
						{
							conf_buffer16[reg+i]  = data_page[i*2];
							conf_buffer16[reg+i] |= (data_page[(i*2)+1] << 8);
						}
					}

					if(((page+1)*16) < STORE_BUFFER16_LENGTH )
					{
						page++;
						attempts = 0;
						store_fsm = (UNS16)(STORE_PAGE_COMPLETED);
						store_fsm = (UNS16)(STORE_EEPROM_ACCESS);
					}
					else
					{
						store_fsm = (UNS16)(STORE_PAGE_COMPLETED);
//						display_conf_buffer();
						if( nodeID == conf_buffer16[0] )  //first register MUST contain eeprom_nodeid
						{
							store_fsm = (UNS16)(STORE_READ_COMPLETED);
							write_od_from_eeprom();
						}
						else
						{
							store_fsm = (UNS16)(STORE_EEPROM_NODE_ERROR);
						}
						store_status = STORE_INACTIVE;
					}
				}
			}
			else
			{
				if( ++attempts == I2C_EEPROM_ATTEMPTS )
				{
					attempts = 0;
					store_fsm = (UNS16)(STORE_EEPROM_ERROR);
					store_status = (UNS16)STORE_INACTIVE;
				}
			}
		}
	}
}

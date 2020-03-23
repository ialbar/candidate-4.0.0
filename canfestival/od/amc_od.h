/* File generated by gen_cfile.py. Should not be modified. */

#ifndef AMC_OD_H
#define AMC_OD_H

#include "data.h"

/* Prototypes of function provided by object dictionnary */
UNS32 amc_od_valueRangeTest (UNS8 typeValue, void * value);
const indextable * amc_od_scanIndexOD (UNS16 wIndex, UNS32 * errorCode, ODCallback_t **callbacks);

/* Master node data struct */
extern CO_Data amc_od_Data;
extern INTEGER32 Velocity_demand_value_in_increments;		/* Mapped at index 0x2000, subindex 0x00*/
extern INTEGER16 PWM_reference;		/* Mapped at index 0x2001, subindex 0x00*/
extern INTEGER32 Velocity_following_error;		/* Mapped at index 0x2002, subindex 0x00*/
extern INTEGER16 Control_effort_offset;		/* Mapped at index 0x2003, subindex 0x00*/
extern UNS32 Velocity_control_margin;		/* Mapped at index 0x2006, subindex 0x00*/
extern UNS32 Position_control_margin;		/* Mapped at index 0x2007, subindex 0x00*/
extern UNS16 Servo_rated_current;		/* Mapped at index 0x2008, subindex 0x00*/
extern UNS8 Brushless_sequence_type;		/* Mapped at index 0x2009, subindex 0x00*/
extern UNS8 DC_motor_polarity;		/* Mapped at index 0x200A, subindex 0x00*/
extern UNS16 Max_acceleration_current;		/* Mapped at index 0x200B, subindex 0x00*/
extern UNS8 Apply_sensors_configuration;		/* Mapped at index 0x2021, subindex 0x00*/
extern UNS8 Sensors_configuration_active;		/* Mapped at index 0x2022, subindex 0x00*/
extern UNS8 Enable_redundancy;		/* Mapped at index 0x2024, subindex 0x00*/
extern UNS8 Enable_hybrid_redundancy;		/* Mapped at index 0xFF34, subindex 0x01*/
extern UNS16 Power_voltage;		/* Mapped at index 0x2030, subindex 0x00*/
extern INTEGER16 Analog_optocoupler_gain;		/* Mapped at index 0x2031, subindex 0x00*/
extern UNS16 Temperature;		/* Mapped at index 0x2032, subindex 0x00*/
extern UNS32 Motor_current;		/* Mapped at index 0x2033, subindex 0x00*/
extern UNS16 Power_voltage_limit[2];		/* Mapped at index 0x203A, subindex 0x01 - 0x02 */
extern UNS16 Temperature_limit;		/* Mapped at index 0x203C, subindex 0x00*/
extern UNS16 Analog_input_Resolver_in1;		/* Mapped at index 0x2040, subindex 0x01 */
extern UNS16 Analog_input_Resolver_in2;		/* Mapped at index 0x2040, subindex 0x02 */
extern UNS16 Analog_input_An1;		/* Mapped at index 0x2040, subindex 0x03 */
extern UNS16 Analog_input_An2;		/* Mapped at index 0x2040, subindex 0x04 */
extern UNS16 Analog_input_An3;		/* Mapped at index 0x2040, subindex 0x05 */
extern UNS16 Analog_input_An4;		/* Mapped at index 0x2040, subindex 0x06 */
extern UNS16 Analog_input_Thermistor;		/* Mapped at index 0x2040, subindex 0x07 */
extern UNS16 Analog_input_NTC;		/* Mapped at index 0x2040, subindex 0x08 */
extern UNS16 Analog_input_Motorcurrent;		/* Mapped at index 0x2040, subindex 0x09 */
extern UNS16 Analog_input_Ref1v5;		/* Mapped at index 0x2040, subindex 0x0A */
extern UNS16 Analog_input_Vpower;		/* Mapped at index 0x2040, subindex 0x0B */
extern UNS16 Analog_input_Motorcurrent2;		/* Mapped at index 0x2040, subindex 0x0C */
extern UNS8 Axle_brake_Start;		/* Mapped at index 0x2100, subindex 0x01 */
extern UNS8 Axle_brake_Position;		/* Mapped at index 0x2100, subindex 0x02 */
extern UNS8 Axle_brake_Polarity;		/* Mapped at index 0x2100, subindex 0x03 */
extern UNS16 Axle_brake_Delay;		/* Mapped at index 0x2100, subindex 0x04 */
extern UNS8 Brake1_Mode;		/* Mapped at index 0x2101, subindex 0x01 */
extern UNS8 Brake1_Start;		/* Mapped at index 0x2101, subindex 0x02 */
extern UNS16 Brake1_Fixed_duty_1;		/* Mapped at index 0x2101, subindex 0x03 */
extern UNS16 Brake1_Fixed_duty_2;		/* Mapped at index 0x2101, subindex 0x04 */
extern UNS16 Brake1_Time_duty1;		/* Mapped at index 0x2101, subindex 0x05 */
extern UNS16 Brake1_Duty_cycle;		/* Mapped at index 0x2101, subindex 0x06 */
extern UNS8 Brake2_Mode;		/* Mapped at index 0x2102, subindex 0x01 */
extern UNS8 Brake2_Start;		/* Mapped at index 0x2102, subindex 0x02 */
extern UNS16 Brake2_Fixed_duty_1;		/* Mapped at index 0x2102, subindex 0x03 */
extern UNS16 Brake2_Fixed_duty_2;		/* Mapped at index 0x2102, subindex 0x04 */
extern UNS16 Brake2_Time_duty1;		/* Mapped at index 0x2102, subindex 0x05 */
extern UNS16 Brake2_Duty_cycle;		/* Mapped at index 0x2102, subindex 0x06 */
extern UNS8 Brake2_Inhibit_safety;		/* Mapped at index 0x2102, subindex 0x07 */
extern UNS8 Axle_clutch_Start;		/* Mapped at index 0x2103, subindex 0x01 */
extern UNS8 Axle_clutch_Position;		/* Mapped at index 0x2103, subindex 0x02 */
extern UNS8 Axle_clutch_Polarity;		/* Mapped at index 0x2103, subindex 0x03 */
extern UNS16 Axle_clutch_Delay;		/* Mapped at index 0x2103, subindex 0x04 */
extern UNS8 Axle_clutch_Needed_to_brake;		/* Mapped at index 0x2103, subindex 0x05 */
extern INTEGER32 Detents_list_D1;		/* Mapped at index 0x2110, subindex 0x01 */
extern INTEGER32 Detents_list_D2;		/* Mapped at index 0x2110, subindex 0x02 */
extern INTEGER32 Detents_list_D3;		/* Mapped at index 0x2110, subindex 0x03 */
extern INTEGER32 Detents_list_D4;		/* Mapped at index 0x2110, subindex 0x04 */
extern INTEGER32 Detents_list_D5;		/* Mapped at index 0x2110, subindex 0x05 */
extern INTEGER32 Detents_list_D6;		/* Mapped at index 0x2110, subindex 0x06 */
extern INTEGER32 Detents_list_D7;		/* Mapped at index 0x2110, subindex 0x07 */
extern INTEGER32 Detents_list_D8;		/* Mapped at index 0x2110, subindex 0x08 */
extern INTEGER32 Detents_list_D9;		/* Mapped at index 0x2110, subindex 0x09 */
extern INTEGER32 Detents_list_D10;		/* Mapped at index 0x2110, subindex 0x0A */
extern INTEGER32 Detents_list_D11;		/* Mapped at index 0x2110, subindex 0x0B */
extern INTEGER32 Detents_list_D12;		/* Mapped at index 0x2110, subindex 0x0C */
extern INTEGER32 Detents_list_D13;		/* Mapped at index 0x2110, subindex 0x0D */
extern INTEGER32 Detents_list_D14;		/* Mapped at index 0x2110, subindex 0x0E */
extern INTEGER32 Detents_list_D15;		/* Mapped at index 0x2110, subindex 0x0F */
extern INTEGER32 Detents_list_D16;		/* Mapped at index 0x2110, subindex 0x10 */
extern UNS16 Detents_config_Enable_detents;		/* Mapped at index 0x2111, subindex 0x01 */
extern UNS8 Detents_config_Leave;		/* Mapped at index 0x2111, subindex 0x02 */
extern UNS32 Detents_config_Max_velocity;		/* Mapped at index 0x2111, subindex 0x03 */
extern UNS32 Detents_config_Max_input_distance;		/* Mapped at index 0x2111, subindex 0x04 */
extern UNS16 Detents_config_Max_duty;		/* Mapped at index 0x2111, subindex 0x05 */
extern INTEGER32 Detents_config_Active_detent;		/* Mapped at index 0x2111, subindex 0x06 */
extern UNS32 Detents_config_Skip_distance;		/* Mapped at index 0x2111, subindex 0x07 */
extern UNS32 Detents_config_Max_output_distance;			/* Mapped at index 0x2111, subindex 0x08 */
extern UNS32 Detents_Position_Window_factor_change_Ki;		/* Mapped at index 0x2111, subindex 0x09 */
extern UNS16 Position_Detent_window;						/* Mapped at index 0x2111, subindex 0x0A */
extern UNS16 Position_Detent_window_time;					/* Mapped at index 0x2111, subindex 0x0B */
extern UNS16 Detents_Position_Window_factor_change_Ki_window_time;	/* Mapped at index 0x2111, subindex 0x0C */
extern INTEGER16 Assisted_mode_current_demand;		/* Mapped at index 0x2120, subindex 0x01 */
extern UNS8 Assisted_mode_conf_Analog_input;		/* Mapped at index 0x2120, subindex 0x02 */
extern INTEGER16 Assisted_mode_conf_Center;			/* Mapped at index 0x2120, subindex 0x03 */
extern UNS16 Assisted_mode_conf_Threshold;			/* Mapped at index 0x2120, subindex 0x04 */
extern UNS16 Assisted_mode_conf_Window_time;		/* Mapped at index 0x2120, subindex 0x05 */
extern INTEGER16 Assisted_mode_conf_Gain_numerator;	/* Mapped at index 0x2120, subindex 0x06 */
extern INTEGER16 Assisted_mode_conf_Gain_divisor;	/* Mapped at index 0x2120, subindex 0x07 */
extern UNS32 Assisted_mode_conf_Max_velocity;		/* Mapped at index 0x2120, subindex 0x08 */
extern UNS16 Assisted_mode_conf_Kp;					/* Mapped at index 0x2120, subindex 0x09 */
extern UNS16 Assisted_mode_conf_Ki;					/* Mapped at index 0x2120, subindex 0x0A */
extern UNS16 Assisted_mode_conf_Kc;					/* Mapped at index 0x2120, subindex 0x0B */
extern UNS16 Assisted_mode_conf_control_parameter_Divisor;	/* Mapped at index 0x2120, subindex 0x0C */
extern UNS8 Assisted_mode_conf_filter;		/* Mapped at index 0x2120, subindex 0x0D */
extern UNS16 Assisted_mode_conf_ramp;					/* Mapped at index 0x2120, subindex 0x0E */
extern UNS16 Assisted_mode_control_margin;					/* Mapped at index 0x2120, subindex 0x0F */

extern UNS8 Safety;		/* Mapped at index 0x2200, subindex 0x00*/
extern UNS8 GPI_Polarity;		/* Mapped at index 0x2201, subindex 0x01 */
extern UNS8 GPI_Interrupt_mask;		/* Mapped at index 0x2201, subindex 0x02 */
extern UNS8 GPI_Value;		/* Mapped at index 0x2201, subindex 0x03 */
extern UNS8 GPO;		/* Mapped at index 0x2202, subindex 0x00*/
extern INTEGER16 AnalogIn1_Gain_num;		/* Mapped at index 0x2203, subindex 0x01 */
extern INTEGER16 AnalogIn1_Gain_div;		/* Mapped at index 0x2203, subindex 0x02 */
extern INTEGER32 AnalogIn1_Offset;		/* Mapped at index 0x2203, subindex 0x03 */
extern INTEGER32 AnalogIn1_Value;		/* Mapped at index 0x2203, subindex 0x04 */
extern INTEGER16 AnalogIn2_Gain_num;		/* Mapped at index 0x2204, subindex 0x01 */
extern INTEGER16 AnalogIn2_Gain_div;		/* Mapped at index 0x2204, subindex 0x02 */
extern INTEGER32 AnalogIn2_Offset;		/* Mapped at index 0x2204, subindex 0x03 */
extern INTEGER32 AnalogIn2_Value;		/* Mapped at index 0x2204, subindex 0x04 */
extern INTEGER16 AnalogIn3_Gain_num;		/* Mapped at index 0x2205, subindex 0x01 */
extern INTEGER16 AnalogIn3_Gain_div;		/* Mapped at index 0x2205, subindex 0x02 */
extern INTEGER32 AnalogIn3_Offset;		/* Mapped at index 0x2205, subindex 0x03 */
extern INTEGER32 AnalogIn3_Value;		/* Mapped at index 0x2205, subindex 0x04 */
extern INTEGER16 AnalogIn4_Gain_num;		/* Mapped at index 0x2206, subindex 0x01 */
extern INTEGER16 AnalogIn4_Gain_div;		/* Mapped at index 0x2206, subindex 0x02 */
extern INTEGER32 AnalogIn4_Offset;		/* Mapped at index 0x2206, subindex 0x03 */
extern INTEGER32 AnalogIn4_Value;		/* Mapped at index 0x2206, subindex 0x04 */
extern INTEGER32 GPIO_Type;		/* Mapped at index 0x2207, subindex 0x01 */
extern INTEGER32 GPIO_Property_1;		/* Mapped at index 0x2207, subindex 0x02 */
extern INTEGER32 GPIO_Property_2;		/* Mapped at index 0x2207, subindex 0x03 */
extern INTEGER32 GPIO_Value;		/* Mapped at index 0x2207, subindex 0x04 */
extern UNS8 Encoder1_Polarity;		/* Mapped at index 0x2208, subindex 0x01 */
extern UNS8 Encoder1_Error_Reset;		/* Mapped at index 0x2208, subindex 0x02 */
extern UNS16 Encoder1_Resolution;		/* Mapped at index 0x2208, subindex 0x03 */
extern INTEGER32 Encoder1_Value;		/* Mapped at index 0x2208, subindex 0x04 */
extern UNS8 Encoder2_Polarity;		/* Mapped at index 0x2209, subindex 0x01 */
extern UNS8 Encoder2_Error_Reset;		/* Mapped at index 0x2209, subindex 0x02 */
extern UNS16 Encoder2_Resolution;		/* Mapped at index 0x2209, subindex 0x03 */
extern INTEGER32 Encoder2_Value;		/* Mapped at index 0x2209, subindex 0x04 */
extern UNS8 Hall_sensor_Polarity;		/* Mapped at index 0x220A, subindex 0x01 */
extern UNS8 Hall_sensor_Value;		/* Mapped at index 0x220A, subindex 0x02 */
extern INTEGER16 Resolver_Value;		/* Mapped at index 0x220B, subindex 0x01 */
extern UNS8 Resolver_Acc_polarity;		/* Mapped at index 0x220B, subindex 0x02 */
extern INTEGER32 Resolver_Acc_value;		/* Mapped at index 0x220B, subindex 0x03 */
extern UNS8 Limit_switches_Lower_limit;		/* Mapped at index 0x220C, subindex 0x01 */
extern UNS8 Limit_switches_Upper_limit;		/* Mapped at index 0x220C, subindex 0x02 */
extern UNS8 Zero_Input;		/* Mapped at index 0x220D, subindex 0x01 */
extern INTEGER32 Zero_Position;		/* Mapped at index 0x220D, subindex 0x02 */
extern UNS8 Zero_Side;		/* Mapped at index 0x220D, subindex 0x03 */
extern UNS16 PWM_sensor_Resolution;		/* Mapped at index 0x220E, subindex 0x01 */
extern UNS16 PWM_sensor_Value;		/* Mapped at index 0x220E, subindex 0x02 */
extern UNS32 PWM_sensor_Period;		/* Mapped at index 0x220E, subindex 0x03 */
extern UNS8 Velocity_sensor_Peripheral;		/* Mapped at index 0x2220, subindex 0x01 */
extern UNS8 Velocity_sensor_Place;		/* Mapped at index 0x2220, subindex 0x02 */
extern UNS8 Position_sensor_Peripheral;		/* Mapped at index 0x2221, subindex 0x01 */
extern UNS8 Position_sensor_Place;		/* Mapped at index 0x2221, subindex 0x02 */
extern UNS8 Absolute_pos_sensor_Peripheral;		/* Mapped at index 0x2222, subindex 0x01 */
extern UNS8 Absolute_pos_sensor_Place;		/* Mapped at index 0x2222, subindex 0x02 */
extern UNS8 Power_board;		/* Mapped at index 0x2230, subindex 0x00*/
extern UNS32 Circle_Radius;		/* Mapped at index 0x3001, subindex 0x01 */
extern UNS16 Circle_Step_time;		/* Mapped at index 0x3001, subindex 0x02 */
extern UNS8 Circle_Axis;		/* Mapped at index 0x3001, subindex 0x03 */
extern INTEGER8 Build_revision[10];		/* Mapped at index 0x5001, subindex 0x00*/
extern UNS32 Store_configuration;		/* Mapped at index 0x5FFD, subindex 0x00*/
extern UNS16 Launch_AMC_loader;		/* Mapped at index 0x5FFE, subindex 0x00*/
extern UNS8 Node_reset;		/* Mapped at index 0x5FFF, subindex 0x00*/
extern UNS16 Device_control_word;		/* Mapped at index 0x6040, subindex 0x00*/
extern UNS16 Device_status_word;		/* Mapped at index 0x6041, subindex 0x00*/
extern INTEGER16 Quick_stop_option_code;		/* Mapped at index 0x605A, subindex 0x00*/
extern INTEGER16 Fault_reaction_option_code;		/* Mapped at index 0x605E, subindex 0x00*/
extern INTEGER8 Modes_of_operation;		/* Mapped at index 0x6060, subindex 0x00*/
extern INTEGER8 Modes_of_operation_display;		/* Mapped at index 0x6061, subindex 0x00*/
extern INTEGER32 Position_demand_value;		/* Mapped at index 0x6062, subindex 0x00*/
extern INTEGER32 Position_actual_value_in_increments;		/* Mapped at index 0x6063, subindex 0x00*/
extern INTEGER32 Position_actual_value;		/* Mapped at index 0x6064, subindex 0x00*/
extern UNS32 Following_error_window;		/* Mapped at index 0x6065, subindex 0x00*/
extern UNS16 Following_error_time_out;		/* Mapped at index 0x6066, subindex 0x00*/
extern UNS32 Position_window;		/* Mapped at index 0x6067, subindex 0x00*/
extern UNS16 Position_window_time;		/* Mapped at index 0x6068, subindex 0x00*/
extern INTEGER32 Velocity_demand_value;		/* Mapped at index 0x606B, subindex 0x00*/
extern INTEGER32 Velocity_actual_value;		/* Mapped at index 0x606C, subindex 0x00*/
extern UNS16 Velocity_window;		/* Mapped at index 0x606D, subindex 0x00*/
extern UNS16 Velocity_window_time;		/* Mapped at index 0x606E, subindex 0x00*/
extern UNS16 Velocity_threshold;		/* Mapped at index 0x606F, subindex 0x00*/
extern UNS16 Velocity_threshold_time;		/* Mapped at index 0x6070, subindex 0x00*/
extern UNS16 Max_current;		/* Mapped at index 0x6073, subindex 0x00*/
extern UNS32 Motor_rated_current;		/* Mapped at index 0x6075, subindex 0x00*/
extern INTEGER16 Current_actual_value;		/* Mapped at index 0x6078, subindex 0x00*/
extern UNS32 DC_link_circuit_voltage;		/* Mapped at index 0x6079, subindex 0x00*/
extern INTEGER32 Target_position;		/* Mapped at index 0x607A, subindex 0x00*/
extern INTEGER32 Home_offset;		/* Mapped at index 0x607C, subindex 0x00*/
extern INTEGER32 Software_position_limit_Min_position_limit;		/* Mapped at index 0x607D, subindex 0x01 */
extern INTEGER32 Software_position_limit_Max_position_limit;		/* Mapped at index 0x607D, subindex 0x02 */
extern UNS8 Polarity;		/* Mapped at index 0x607E, subindex 0x00*/
extern UNS32 Max_profile_velocity;		/* Mapped at index 0x607F, subindex 0x00*/
extern UNS32 Max_motor_speed;		/* Mapped at index 0x6080, subindex 0x00*/
extern UNS32 Profile_velocity;		/* Mapped at index 0x6081, subindex 0x00*/
extern UNS32 Profile_acceleration;		/* Mapped at index 0x6083, subindex 0x00*/
extern UNS32 Profile_deceleration;		/* Mapped at index 0x6084, subindex 0x00*/
extern UNS32 Quick_stop_deceleration;	/* Mapped at index 0x6085, subindex 0x01*/
extern INTEGER32 Quick_stop_offset;		/* Mapped at index 0x6085, subindex 0x02*/
extern UNS32 Quick_ramp_time;			/* Mapped at index 0x6085, subindex 0x03*/
extern UNS16 Quick_mode_change_time;	/* Mapped at index 0x6085, subindex 0x04*/
extern UNS8 Current_ramp_quick_stop;	/* Mapped at index 0x6085, subindex 0x05*/
extern INTEGER8 Position_notation_index;		/* Mapped at index 0x6089, subindex 0x00*/
extern UNS8 Position_dimension_index;		/* Mapped at index 0x608A, subindex 0x00*/
extern INTEGER8 Velocity_notation_index;		/* Mapped at index 0x608B, subindex 0x00*/
extern UNS8 Velocity_dimension_index;		/* Mapped at index 0x608C, subindex 0x00*/
extern INTEGER8 Acceleration_notation_index;		/* Mapped at index 0x608D, subindex 0x00*/
extern UNS8 Acceleration_dimension_index;		/* Mapped at index 0x608E, subindex 0x00*/
extern UNS32 Gear_ratio_Motor_revolutions;		/* Mapped at index 0x6091, subindex 0x01 */
extern UNS32 Gear_ratio_Shaft_revolutions;		/* Mapped at index 0x6091, subindex 0x02 */
extern UNS32 Feed_constant_Feed;		/* Mapped at index 0x6092, subindex 0x01 */
extern UNS32 Feed_constant_Shaft_revolutions;		/* Mapped at index 0x6092, subindex 0x02 */
extern UNS32 Position_factor_Numerator;		/* Mapped at index 0x6093, subindex 0x01 */
extern UNS32 Position_factor_Feed_constant;		/* Mapped at index 0x6093, subindex 0x02 */
extern UNS32 Velocity_encoder_factor_Numerator;		/* Mapped at index 0x6094, subindex 0x01 */
extern UNS32 Velocity_encoder_factor_Divisor;		/* Mapped at index 0x6094, subindex 0x02 */
extern UNS32 Velocity_factor_1_Numerator;		/* Mapped at index 0x6095, subindex 0x01 */
extern UNS32 Velocity_factor_1_Divisor;		/* Mapped at index 0x6095, subindex 0x02 */
extern UNS32 Acceleration_factor_Numerator;		/* Mapped at index 0x6097, subindex 0x01 */
extern UNS32 Acceleration_factor_Divisor;		/* Mapped at index 0x6097, subindex 0x02 */
extern INTEGER8 Homing_method;		/* Mapped at index 0x6098, subindex 0x00*/
extern UNS32 Homing_speeds_Speed_during_search_for_switch;		/* Mapped at index 0x6099, subindex 0x01 */
extern UNS32 Homing_speeds_speed_during_search_for_zero;		/* Mapped at index 0x6099, subindex 0x02 */
extern INTEGER32 Homing_acceleration;		/* Mapped at index 0x609A, subindex 0x00*/
extern INTEGER16 Interpolation_sub_mode_select;		/* Mapped at index 0x60C0, subindex 0x00*/
extern UNS32 Interpolation_data_record[2];		/* Mapped at index 0x60C1, subindex 0x01 - 0x02 */
extern UNS8 Interpolation_time_period_Interpolation_time_units;		/* Mapped at index 0x60C2, subindex 0x01 */
extern INTEGER8 Interpolation_time_period_Interpolation_time_index;		/* Mapped at index 0x60C2, subindex 0x02 */
extern UNS8 Interpolation_sync_definition_Synchronize_on_group;		/* Mapped at index 0x60C3, subindex 0x01 */
extern UNS8 Interpolation_sync_definition_ip_sync_every_n_event;		/* Mapped at index 0x60C3, subindex 0x02 */
extern UNS32 Interpolation_data_configuration_Maximum_buffer_size;		/* Mapped at index 0x60C4, subindex 0x01 */
extern UNS32 Interpolation_data_configuration_Actual_buffer_size;		/* Mapped at index 0x60C4, subindex 0x02 */
extern UNS8 Interpolation_data_configuration_Buffer_organization;		/* Mapped at index 0x60C4, subindex 0x03 */
extern UNS16 Interpolation_data_configuration_Buffer_position;		/* Mapped at index 0x60C4, subindex 0x04 */
extern UNS8 Interpolation_data_configuration_Size_of_data_record;		/* Mapped at index 0x60C4, subindex 0x05 */
extern UNS8 Interpolation_data_configuration_Buffer_clear;		/* Mapped at index 0x60C4, subindex 0x06 */
extern UNS32 Max_acceleration;		/* Mapped at index 0x60C5, subindex 0x00*/
extern UNS32 Max_deceleration;		/* Mapped at index 0x60C6, subindex 0x00*/
extern INTEGER32 Following_error_actual_value;		/* Mapped at index 0x60F4, subindex 0x00*/
extern INTEGER32 Max_slippage;		/* Mapped at index 0x60F8, subindex 0x00*/
extern INTEGER32 Control_effort;		/* Mapped at index 0x60FA, subindex 0x00*/

extern INTEGER32 Position_demand_value_in_increments;		/* Mapped at index 0x60FC, subindex 0x00*/
extern INTEGER32 Target_velocity;		/* Mapped at index 0x60FF, subindex 0x00*/
extern UNS16 Motor_type;		/* Mapped at index 0x6402, subindex 0x00*/
extern INTEGER8 Motor_catalog_number[10];		/* Mapped at index 0x6403, subindex 0x00*/
extern INTEGER8 Motor_manufacturer[10];		/* Mapped at index 0x6404, subindex 0x00*/
extern UNS32 Supported_drive_mode;		/* Mapped at index 0x6502, subindex 0x00*/

//AMQ new entries in the Dictionary
extern INTEGER16 Debug_word_1;
extern INTEGER16 Debug_word_2;
extern INTEGER16 Debug_word_3;
extern INTEGER16 Debug_word_4;
extern INTEGER16 Debug_word_5;
extern INTEGER16 Debug_word_6;
extern INTEGER16 Debug_word_7;
extern INTEGER16 Debug_word_8;
extern UNS8 Enable_Ramp_PWM_mode;
extern UNS16 Motor_Temperature;
extern UNS16  Motor_Temperature_limit;
extern INTEGER16 Motor_current_Iu;
extern INTEGER16 Motor_current_Iv;
extern INTEGER16 Motor_current_Iw;
extern UNS8  BLAC_Motor_Poles;
extern UNS8  BLAC_Motor_Polarity;
extern UNS8  BLAC_Vect_Control;
/*extern INTEGER16 PWM_Motor_Velocity; //Deprecated */
extern UNS32 Current_control_parameter_set_Kp;
extern UNS32 Current_control_parameter_set_Ki;
extern UNS32 Current_control_parameter_set_Kc;
extern UNS32 Current_control_parameter_set_Kff;
extern UNS32 Current_control_parameter_set_Divisor;
extern INTEGER32 Current_control_parameter_set_Id_ref_off;
extern INTEGER32 Current_control_parameter_set_Iq_ref_off;
extern INTEGER32 Current_control_parameter_set_Ud_ref_off;
extern INTEGER32 Current_control_parameter_set_Uq_ref_off;
extern UNS8 Update_current_control_parameter_refs;
extern UNS8 Enable_Fixed_Freq;
extern INTEGER32 Fixed_Freq;
extern UNS8  Enable_Recovery_Faults;
extern UNS8 Enable_Zero_Pwm_Window;
extern INTEGER16 Zero_Pwm_Window;
extern INTEGER16 Limited_Duty;
extern INTEGER16 PWM_Out_Time_Mean;
extern INTEGER16 PWM_Out_Time_Max;
extern INTEGER16 PWM_Out_Time_Min;
extern INTEGER16 PWM_Out_Time_Max_Mean;
extern INTEGER16 PWM_Out_Time_Min_Mean;
extern INTEGER16 PWM_Out_Time_Limit;
extern INTEGER16 PWM_Period_Mean;
extern INTEGER16 PWM_Period_Max;
extern INTEGER16 PWM_Period_Min;
extern INTEGER16 PWM_Period_Max_Mean;
extern INTEGER16 PWM_Period_Min_Mean;
extern INTEGER16 PWM_Period_Limit_Max;
extern INTEGER16 PWM_Period_Limit_Min;
extern INTEGER16 Motor_current_Id;
extern INTEGER16 Motor_current_Iq;
extern INTEGER16 Motor_current_absI;
extern INTEGER32 Debug32_word_1;
extern INTEGER32 Debug32_word_2;
extern INTEGER32 Debug32_word_3;
extern INTEGER32 Debug32_word_4;
extern INTEGER16 Current_control_variables_Ud_ref;
extern INTEGER16 Current_control_variables_Uq_ref;
extern INTEGER16 Current_control_variables_duty_a;
extern INTEGER16 Current_control_variables_duty_b;
extern INTEGER16 Current_control_variables_duty_c;
extern INTEGER16 Current_control_variables_EPwm1_CMPA;
extern INTEGER16 Current_control_variables_EPwm2_CMPA;
extern INTEGER16 Current_control_variables_EPwm3_CMPA;
extern UNS16 Current_control_variables_st_debug_index;
extern UNS16 Current_control_variables_debug_index;
extern INTEGER16 Current_control_variables_debug_var1;
extern INTEGER16 Current_control_variables_debug_var2;
extern INTEGER16 Current_control_variables_debug_var3;
extern INTEGER16 Current_control_variables_debug_var4;
extern INTEGER16 Current_control_variables_debug_var5;
extern INTEGER16 Current_control_variables_debug_var6;
extern UNS8 Current_control_variables_time_enable;
extern UNS8 WD_Debug_variable;

extern UNS16 HDL_version;
extern UNS16 DNA_id0;
extern UNS16 DNA_id1;
extern UNS16 DNA_id2;
extern UNS16 DNA_id3;
extern UNS16 Board_id0;
extern UNS16 Board_id1;
extern UNS16 Board_id2;
extern UNS16 FPGA_HB_counter_lsb_rd;
extern UNS16 FPGA_HB_counter_msb_rd;
extern UNS16 FPGA_HB_counter_lsb_wr;
extern UNS16 FPGA_HB_counter_msb_wr;
extern UNS16 FPGA_configuration_rd;
extern UNS16 FPGA_configuration_wr;
extern UNS8  FPGA_node;
extern UNS16 FPGA_NCE_measure_rd;
extern UNS8  FPGA_acoustic_mode_wr;

extern UNS16 Launch_FPGA_Loader;
extern UNS16 FPGA_SPI_REPROG;

extern UNS8 Debug_Sci_Choose_Method;

extern UNS8  Working_mode_wr;
extern UNS8  Enable_remote_wr;

extern UNS16 FPGA_HB_move_counter_lsb_wr;
extern UNS16 FPGA_HB_move_counter_msb_wr;
extern UNS16 FPGA_HB_reset_counter_lsb_wr;
extern UNS16 FPGA_HB_reset_counter_msb_wr;
extern UNS8  Load_eeprom;

extern UNS32     Analog_input_Hybrid;
extern UNS8      NCE_Polarity;
extern UNS8      Pot_Polarity;
extern UNS32     AnalogNCE_Offset;
extern INTEGER16 AnalogHybrid_Gain_num;
extern INTEGER16 AnalogHybrid_Gain_div;
extern INTEGER32 AnalogHybrid_Offset;
extern UNS8      Hybrid_n_turns;
extern UNS8      Hybrid_analog_sel;
extern INTEGER32 Hybrid_Value;
extern UNS16     Hybrid_redundancy_error;

extern INTEGER32 Debug_long1;
extern INTEGER32 Debug_long2;
extern INTEGER32 Debug_long3;
extern INTEGER32 Debug_long4;
extern INTEGER32 Debug_long5;
extern INTEGER32 Debug_long6;
extern UNS32 Error_statusword;
extern UNS32 Warning_statusword;

extern UNS16 FPGA_Remote_control_code;
extern UNS32 FPGA_Remote_control_mask;
extern UNS16 FPGA_mode;

extern UNS16 svn_revision;
extern UNS16 svn_mixed;
extern UNS16 svn_mods;

extern UNS16 fw_version;
extern UNS16 fw_review;
extern UNS16 fw_subrevision;
extern UNS16 fw_tag;

extern UNS16 pp_init_time;
extern UNS16 pv_init_time;

extern UNS16 FPGA_DAC_1;
extern UNS16 FPGA_DAC_2;
extern UNS16 FPGA_DAC_3;
extern UNS16 FPGA_DAC_4;

extern UNS16 Assisted_mode_pos_Kp;
extern UNS16 Assisted_mode_pos_Ki;
extern UNS16 Assisted_mode_pos_Kd;
extern UNS16 Assisted_mode_pos_Kc;
extern UNS16 Assisted_mode_pos_Kff;
extern UNS16 Assisted_mode_pos_control_parameter_Divisor;
extern INTEGER32 Assisted_mode_debug_position;
extern INTEGER32 Assisted_mode_debug_velocity;
extern UNS8 Assisted_mode_dentent_active;
extern UNS8 Assisted_mode_error_filter;

extern INTEGER32 Outofbound_position_limit_Min;
extern INTEGER32 Outofbound_position_limit_Max;

extern UNS16 Slow_vel_Kp;
extern UNS16 Slow_vel_Ki;
extern UNS16 Slow_vel_Kd;
extern UNS16 Slow_vel_Kc;
extern UNS16 Slow_vel_control_parameter_Divisor;
extern UNS16 Slow_vel_control_margin;
extern UNS16 Slow_vel_saturation;
extern UNS16 Slow_vel_feedforward;
extern UNS16 High_vel_Ki_min;

extern UNS16 PWM_Check_effort;

extern UNS16 FPGA_errors;

//extern UNS16 FPGA_current;
//extern UNS32 FPGA_final_current;
//extern UNS32 FPGA_final_current_2;

extern UNS16 HDL_MBOOT_version;

extern UNS16 COMSP_version;
extern UNS16 COMSP_review;
extern UNS16 COMSP_subrevision;

extern UNS16 Dsp_boot_version;
extern UNS16 Dsp_boot_review;
extern UNS16 Dsp_boot_subrevision;
extern UNS16 Dsp_boot_tag;

extern UNS16 Position_control_parameter_set[5];		/* Mapped at index 0x60FB, subindex 0x01 - 0x05 */
extern UNS16 Position_control_dataset[25];
extern UNS16 Position_Velocity_control_parameter_set[5];
extern UNS16 Position_Velocity_control_dataset[25];
extern UNS16 Position_Current_control_parameter_set[5];
extern UNS16 Position_Current_control_dataset[25];
extern UNS16 Manual_Detent_control_parameters[5];
extern UNS16 Position_Manual_control_parameters[5];

extern UNS32 Max_demand_velocity;

extern UNS16 Velocity_Detent_control_parameters[6];

extern UNS16 Velocity_control_parameter_set[6];		/* Mapped at index 0x60F9, subindex 0x01 - 0x06 */
extern UNS16 Velocity_control_dataset[29];
extern UNS16 Velocity_Current_control_parameter_set[5];
extern UNS16 Velocity_Current_control_dataset[25];
extern UNS16 Velocity_Manual_control_parameters[5];
extern UNS16 Current_Manual_control_parameters[5];

extern UNS32 OverCurrent_percent_error_window;
extern UNS16 OverCurrent_error_time_out;

extern UNS16 Max_sat_current;
extern UNS16 Min_sat_current;
extern UNS16 Max_sat_pwm_duty;
extern UNS16 Min_sat_pwm_duty;
extern UNS16 Control_effort_current_debug;

extern UNS8 AN1_used_for_position;
extern UNS8 AN1_used_for_velocity;
extern UNS8 AN1_place;
extern INTEGER32 AN1_counts;
extern INTEGER32 AN1_position;
extern INTEGER32 AN1_velocity;

extern UNS8 AN2_used_for_position;
extern UNS8 AN2_used_for_velocity;
extern UNS8 AN2_place;
extern INTEGER32 AN2_counts;
extern INTEGER32 AN2_position;
extern INTEGER32 AN2_velocity;

extern UNS8 Enc1_used_for_position;
extern UNS8 Enc1_used_for_velocity;
extern UNS8 Enc1_place;
extern INTEGER32 Enc1_counts;
extern INTEGER32 Enc1_position;
extern INTEGER32 Enc1_velocity;

extern UNS8 Enc2_used_for_position;
extern UNS8 Enc2_used_for_velocity;
extern UNS8 Enc2_place;
extern INTEGER32 Enc2_counts;
extern INTEGER32 Enc2_position;
extern INTEGER32 Enc2_velocity;

extern UNS8 NCE_used_for_position;
extern UNS8 NCE_used_for_velocity;
extern UNS8 NCE_place;
extern INTEGER32 NCE_counts;
extern INTEGER32 NCE_position;
extern INTEGER32 NCE_velocity;

extern UNS8 Resolver_used_for_position;
extern UNS8 Resolver_used_for_velocity;
extern UNS8 Resolver_place;
extern INTEGER32 Resolver_counts;
extern INTEGER32 Resolver_position;
extern INTEGER32 Resolver_velocity;

extern UNS8 New_redundancy_error_enable;
extern UNS32 New_redundancy_error_window;
extern UNS32 New_redundancy_error_timeout;
extern UNS32 New_redundancy_max_err;

extern UNS32 Overlimit_offset_error;

extern UNS32 FAULT_POS_FOLLOWING_ERROR_TIMEOUT;
extern UNS32 FAULT_VEL_FOLLOWING_ERROR_TIMEOUT;
extern UNS32 FAULT_REDUNDANCY_ERROR_TIMEOUT;
extern UNS32 FAULT_HYBRID_REDUNDANCY_ERROR_TIMEOUT;
extern UNS32 FAULT_IRON_CABLE_ERROR_TIMEOUT;
extern UNS32 FAULT_VEL_LIMIT_ERROR_TIMEOUT;
extern UNS32 FAULT_HALL_ERROR_TIMEOUT;
extern UNS32 FAULT_RATCHET_ERROR_TIMEOUT;
extern UNS32 FAULT_OVERCURRENT_ERROR_TIMEOUT;

extern UNS32 Max_detent_velocity;



#define MINIMUM_MODE_CHANGE_TIME 50

#endif // AMC_OD_H

/*
 * redundancy.c
 *
 *  Created on: Nov 14, 2019
 *      Author: user
 */

#include "amc.h"
#include "redundancy.h"


long long tmp_red;



static unsigned int max(unsigned int num1, unsigned int num2, unsigned int num3, unsigned int num4, unsigned int num5);
unsigned int get_max_error(void);

/*****************************************************************/
/*!	function that checks the redundancy between the different encoders or
	potentiometers to fix errors.
	\param init '1' if it will be initialized in next cycle, '0' for normal test
	\param state Cinematic state of the system
	\return 1 if error, 0 if OK
*/
/*****************************************************************/
int redundancy_test(int init, motion_state_struct *state)
{
	static short initialize = 0;
	static long redundant_counter_abs = 0, redundant_abs_pos_offset = 0;
	//static long redundant_counter_abs = 0,redundant_counter_vel = 0, redundant_vel_offset = 0, redundant_abs_pos_offset = 0;//redundant_counter_hybrid = 0, redundant_hybrid_offset = 0;
	//long long temp = 0;
	//unsigned long tmp_uns;
	int err_value = 0;

	if(init)	/* For initialization mode */
	{
		initialize = 1;
		return 0;
	}

	if(initialize)
	{
		//redundant_vel_offset = state->vel_cntr - Position_actual_value_in_increments;
		redundant_abs_pos_offset = state->abs_pos_cntr - Position_actual_value;
		//redundant_hybrid_offset = 0; 													//TODO
		initialize = 0;
		return 0;
	}

	if(state) /* For normal redundancy test */
	{
		if( !Sensors_configuration_active )
			return 0;

		if( !homing_zeroed )
			return 0;

		/* If a potentiometer is used as an absolute position encoder */
		if(abs_pos_counter != position_counter)
		{
			/* For the potentiometer comparison it is used 2*Following_error_window due to the potentiometer low resolution */
			if(labs(state->abs_pos_cntr - Position_actual_value - redundant_abs_pos_offset) > 2*Following_error_window)
			{
				redundant_counter_abs++;
			}
			else
			{
				if(--redundant_counter_abs < 0)
					redundant_counter_abs =0; /* If it doesn't make a mistake, then redundant_counter decreases*/
			}
		}

		//if(velocity_counter != position_counter)
		//{
			// Calculates the difference
		//	tmp_uns = labs(state->vel_cntr - Position_actual_value_in_increments - redundant_vel_offset);
			// convert to external units
		//	temp = (long long)tmp_uns * Position_factor_Feed_constant;
		//	temp /= Position_factor_Numerator;//	 temp is the redundant encoder value in external units
		//	if(temp > Following_error_window)
		//	{
		//		redundant_counter_vel++;
		//	}
		//	else
		//	{
		//		if(--redundant_counter_vel < 0)
		//			redundant_counter_vel =0;
		//	}
		//}

		/* If the hybrid sensor is used*/
		/*if(used_by_control(10))
		{
			if(labs(state->hyb_pos_cntr - state->red_pos_cntr - redundant_hybrid_offset) > Hybrid_redundancy_error)
			{
				redundant_counter_hybrid++;
			}
			else
			{
				if(--redundant_counter_hybrid < 0)
					redundant_counter_hybrid =0; // If it doesn't make a mistake, then redundant_counter decreases
			}
		}*/

		/* When redundant_counter's are higher than a defined value, the function returns 1 to warn about it*/
		//if(redundant_counter_abs >= REDUNDANCY_ERROR_FILTER || redundant_counter_vel >= REDUNDANCY_ERROR_FILTER)
	    if(redundant_counter_abs >= REDUNDANCY_ERROR_FILTER)
		{
			//err_value = (redundant_counter_vel > redundant_counter_abs)?1:2;
			//redundant_counter_vel = redundant_counter_abs = 0;
	    	err_value = 2;
	    	redundant_counter_abs = 0;
			return err_value; //Returns 1 for encoder fault and 2 for potentiometer faults
		}
		/*if(redundant_counter_hybrid >= REDUNDANCY_ERROR_FILTER)
		{
			redundant_counter_hybrid = 0;
			return 3; //Returns 3 for hybrid fault
		}*/
	}
	return 0;
}

/*****************************************************************/
/*!	function that checks the redundancy between the different encoders or
	potentiometers to fix errors.
	\param init '1' if it will be initialized in next cycle, '0' for normal test
	\param state Cinematic state of the system
	\return 1 if error, 0 if OK
*/
/*****************************************************************/
int hybrid_redundancy_test(int init, motion_state_struct *state)
{
	static short initialize = 0;
	static long redundant_counter_hybrid = 0, redundant_hybrid_offset = 0;
	unsigned long tmp_uns;

	if(init)	/* For initialization mode */
	{
		initialize = 1;
		return 0;
	}

	if(initialize)
	{
		redundant_hybrid_offset = 0; 													//TODO
		initialize = 0;
		return 0;
	}

	if(state) /* For normal redundancy test */
	{
		if( !Sensors_configuration_active )
			return 0;

		if( !homing_zeroed )
			return 0;

		/* If the hybrid sensor is used*/
		//if(used_by_control(10))
		//{
			tmp_uns = labs(state->hyb_pos_cntr - state->red_pos_cntr - redundant_hybrid_offset);
			//Debug_long6 = tmp_uns;

			if( tmp_uns > Hybrid_redundancy_error)
			{
				redundant_counter_hybrid++;
			}
			else
			{
				if(--redundant_counter_hybrid < 0)
					redundant_counter_hybrid =0; // If it doesn't make a mistake, then redundant_counter decreases
			}
		//}

		if(redundant_counter_hybrid >= REDUNDANCY_ERROR_FILTER)
		{
			redundant_counter_hybrid = 0;
			return 3; //Returns 3 for hybrid fault
		}
	}
	return 0;
}


unsigned int max(unsigned int num1, unsigned int num2, unsigned int num3, unsigned int num4, unsigned int num5)
{
    unsigned int result = 0;
    if (num1 >= num2 && num1 >= num3 && num1 >= num4 && num1 >= num5){
        result = num1;
    } else if (num2 >= num1 && num2 >= num3 && num2 >= num4 && num2 >= num5){
        result = num2;
    } else if (num3 >= num1 && num3 >= num2 && num3 >= num4 && num3 >= num5){
        result = num3;
    } else if (num4 >= num1 && num4 >= num2 && num4 >= num3 && num4 >= num5){
        result = num4;
    } else if (num5 >= num1 && num5 >= num2 && num5 >= num3 && num5 >= num4){
        result = num5;
    }
    return result;
}

unsigned int get_max_error(void)
{
	unsigned int err_value_an1_an2 = 0;
	unsigned int err_value_an1_enc1 = 0;
	unsigned int err_value_an1_enc2 = 0;
	unsigned int err_value_an1_nce = 0;
	unsigned int err_value_an1_resolver = 0;
	unsigned int err_value_an2_enc1 = 0;
	unsigned int err_value_an2_enc2 = 0;
	unsigned int err_value_an2_nce = 0;
	unsigned int err_value_an2_resolver = 0;
	unsigned int err_value_enc1_enc2 = 0;
	unsigned int err_value_enc1_nce = 0;
	unsigned int err_value_enc1_resolver = 0;
	unsigned int err_value_enc2_nce = 0;
	unsigned int err_value_enc2_resolver = 0;
	unsigned int err_value_nce_resolver = 0;
	unsigned int group1_err = 0;
	unsigned int group2_err = 0;
	unsigned int group3_err = 0;
	unsigned int group4_err = 0;
	unsigned int group5_err = 0;
	unsigned int max_err_value = 0;

	if (AN1_used_for_position){
		if (AN2_used_for_position){
			err_value_an1_an2 = labs(labs(AN1_position)-labs(AN2_position));
		}
		if (Enc1_used_for_position){
			err_value_an1_enc1 = labs(labs(AN1_position)-labs(Enc1_position));
		}
		if (Enc2_used_for_position){
			err_value_an1_enc2 = labs(labs(AN1_position)-labs(Enc2_position));
		}
		if (NCE_used_for_position){
			err_value_an1_nce = labs(labs(AN1_position)-labs(NCE_position));
		}
		if (Resolver_used_for_position){
			err_value_an1_resolver = labs(labs(AN1_position)-labs(Resolver_position));
		}
	}
	else{
		err_value_an1_an2 = 0;
		err_value_an1_enc1 = 0;
		err_value_an1_enc2 = 0;
		err_value_an1_nce = 0;
		err_value_an1_resolver = 0;
	}
	if (AN2_used_for_position){
		if (Enc1_used_for_position){
			err_value_an2_enc1 = labs(labs(AN2_position)-labs(Enc1_position));
		}
		if (Enc2_used_for_position){
			err_value_an2_enc2 = labs(labs(AN2_position)-labs(Enc2_position));
		}
		if (NCE_used_for_position){
			err_value_an2_nce = labs(labs(AN2_position)-labs(NCE_position));
		}
		if (Resolver_used_for_position){
			err_value_an2_resolver = labs(labs(AN2_position)-labs(Resolver_position));
		}
	}
	else{
		err_value_an2_enc1 = 0;
		err_value_an2_enc2 = 0;
		err_value_an2_nce = 0;
		err_value_an2_resolver = 0;
	}
	if (Enc1_used_for_position){
		if (Enc2_used_for_position){
			err_value_enc1_enc2 = labs(labs(Enc1_position)-labs(Enc2_position));
		}
		if (NCE_used_for_position){
			err_value_enc1_nce = labs(labs(Enc1_position)-labs(NCE_position));
		}
		if (Resolver_used_for_position){
			err_value_enc1_resolver = labs(labs(Enc1_position)-labs(Resolver_position));
		}
	}
	else{
		err_value_enc1_enc2 = 0;
		err_value_enc1_nce = 0;
		err_value_enc1_resolver = 0;
	}
	if (Enc2_used_for_position){
		if (NCE_used_for_position){
			err_value_enc2_nce = labs(labs(Enc2_position)-labs(NCE_position));
		}
		if (Resolver_used_for_position){
			err_value_enc2_resolver = labs(labs(Enc2_position)-labs(Resolver_position));
		}
	}
	else{
		err_value_enc2_nce = 0;
		err_value_enc2_resolver = 0;
	}
	if (NCE_used_for_position){
		if (Resolver_used_for_position){
			err_value_nce_resolver = labs(labs(NCE_position)-labs(Resolver_position));
		}
	}
	else{
		err_value_nce_resolver = 0;
	}

	if	((Axle_clutch_Position != 0) &&
		((Modes_of_operation_display==OPERATION_MODE_MANUAL)  ||
		(Modes_of_operation_display==OPERATION_MODE_MANUAL_2)))
	{
		err_value_an1_enc1 = 0;
		err_value_an1_enc2 = 0;
		err_value_an2_enc1 = 0;
		err_value_an2_enc2 = 0;
		err_value_enc1_enc2 = 0;
		err_value_enc1_nce = 0;
		err_value_enc1_resolver = 0;
		err_value_enc2_nce = 0;
		err_value_enc2_resolver = 0;
	}

	group1_err = max(err_value_an1_an2, err_value_an1_enc1, err_value_an1_enc2, err_value_an1_nce, err_value_an1_resolver);
	group2_err = max(err_value_an2_enc1, err_value_an2_enc2, err_value_an2_nce, err_value_an2_resolver, 0);
	group3_err = max(err_value_enc1_enc2, err_value_enc1_nce, err_value_enc1_resolver, 0, 0);
    group4_err = max(err_value_enc2_nce, err_value_enc2_resolver, 0, 0, 0);
    group5_err = err_value_nce_resolver;

    max_err_value = max(group1_err, group2_err, group3_err, group4_err, group5_err);

	return max_err_value;
}

void new_redundancy(motion_state_struct *state )
{
	static short new_redundancy = 0;

	if(state) /* For normal redundancy test */
	{
		if( !Sensors_configuration_active )
			return;

		if( !homing_zeroed )
			return;

		New_redundancy_max_err = get_max_error();

		if(New_redundancy_error_enable){
			if(labs(New_redundancy_max_err) > New_redundancy_error_window)
			{
				if(!new_redundancy)
				{
					new_redundancy = state->time;		/* Start time counting if not started */
				}
				tmp_red = (long long)(state->time - new_redundancy) * 1000;
				tmp_red /= lcounts_p_s;
				if(tmp_red > New_redundancy_error_timeout)
				{
//					if(!(isFaultActive(FAULT_REDUNDANCY)))
//					{
//						_ERRORmessage(0xFF07, 0x80, 0x0000, "Position redundancy error", 0, 0);
//						setFault(FAULT_REDUNDANCY);
//					}
//					QueueFault(FAULT_REDUNDANCY);
					QueueWarning(FAULT_REDUNDANCY);

				}
				//			else
				//			{
				//				DeQueueFault(FAULT_REDUNDANCY);
				//				DeQueueWarning(FAULT_REDUNDANCY);
				//			}
			}
			else
			{
				new_redundancy = 0;			/* stop time counting */
				//			DeQueueFault(FAULT_REDUNDANCY);
				DeQueueWarning(FAULT_REDUNDANCY);
			}
		}
		else
		{
			//		DeQueueFault(FAULT_REDUNDANCY);
			DeQueueWarning(FAULT_REDUNDANCY);
		}
	}
}

void check_redundancy( motion_state_struct *state )
{
	static short i_redundancy = 0;

	/* Redundancy test every REDUNDANCY_N_CYCLES */
	if( Enable_redundancy )
	{
		if( redundancy_test(0, state) )
		{
			++i_redundancy;
		}

		i_redundancy = CLAMP( i_redundancy, 0, REDUNDANCY_N_CYCLES );

		if( i_redundancy == REDUNDANCY_N_CYCLES )
		{
//			if(!(isFaultActive(FAULT_REDUNDANCY)))
//			{
//				/* Set the Fault flag and the error information. Additional info */
//				_ERRORmessage(0xFF07, 0x80, 0x0000, "Position redundancy error", 0, 0);
//				setFault(FAULT_REDUNDANCY);
//			}
//			QueueFault(FAULT_REDUNDANCY);
			QueueWarning(FAULT_REDUNDANCY);
			i_redundancy = 0;
		}
		else if( i_redundancy == 0 )
		{
			//		DeQueueFault(FAULT_REDUNDANCY);
		}
	}
	else
	{
		i_redundancy = 0;
		DeQueueFault(FAULT_REDUNDANCY);
		DeQueueWarning(FAULT_REDUNDANCY);
	}
}

void hybrid_check_redundancy( motion_state_struct *state )
{
	static short i_redundancy = 0;

	/* Redundancy test every REDUNDANCY_N_CYCLES */
	if( Enable_hybrid_redundancy )
	{
		if( hybrid_redundancy_test(0, state) )
		{
			++i_redundancy;
		}

		i_redundancy = CLAMP( i_redundancy, 0, REDUNDANCY_N_CYCLES );

		if( i_redundancy == REDUNDANCY_N_CYCLES )
		{
//			if(!(isFaultActive(FAULT_HYBRID_REDUNDANCY)))
//			{
//				/* Set the Fault flag and the error information. Additional info */
//				_ERRORmessage(0xFF0B, 0x80, 0x0000, "Hybrid redundancy error", 0, 0);
//				setFault(FAULT_HYBRID_REDUNDANCY);
//			}
//			QueueFault(FAULT_HYBRID_REDUNDANCY);
			QueueWarning(FAULT_HYBRID_REDUNDANCY);
			i_redundancy = 0;
		}
		else if( i_redundancy == 0 )
		{
			//			DeQueueFault(FAULT_HYBRID_REDUNDANCY);
		}
	}
	else
	{
		i_redundancy = 0;
		DeQueueFault(FAULT_HYBRID_REDUNDANCY);
		DeQueueWarning(FAULT_HYBRID_REDUNDANCY);
	}
}

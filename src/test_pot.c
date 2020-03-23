#if 0

			//if (NCE_count < NCE_count_ant)
		    if (int_nce_pol == 1)
			{
				//if (Pot_Polarity == 0)
		    	if (int_pot_pol == 0)
					factor_update = factor_floor;
				else
					factor_update = Hybrid_n_turns-factor_floor;
			}

		    else
			{
				//if (Pot_Polarity == 0)
		    	if (int_pot_pol == 0)
					factor_update = factor_floor;
				else
					factor_update = Hybrid_n_turns-factor_floor;
			}

	    	if (int_pot_pol == 0)
				factor_update = factor_floor;
			else
				factor_update = Hybrid_n_turns-factor_floor;



	//if (NCE_Polarity == 0)
	if (int_nce_pol == 0)
		analog_hybrid = NCE_count + factor_update*4096;
	else
		analog_hybrid = 4095-NCE_count + factor_update*4096;

#endif
#if 0
	int_nce_pol = 0;
	if (int_nce_pol == 0)
	{
		if (NCE_count < NCE_count_ant)
		{
			  factor_update = factor_floor;
#if 0
			if (factor_floor == 10)
			  factor_update = factor_floor+1;
			else
			{
			  factor_update = factor_floor;
			  flag_test =1000;
			}
#endif
		}
		else
			  flag_test =0;

	}
	else
	{
		if (NCE_count > NCE_count_ant)
			factor_update = factor_floor;
	}
#endif
/*
 * test_pot.c
 *
 *  Created on: Feb 6, 2020
 *      Author: user
 */



#include "constants.c"
#include "math.c"

int * get_proximity_values( int side, int led )
{
	static int prox_values[4];
	int i;
	int max;
	int max_i = -1;

	switch( side )
	{
		case LEFT:
			prox_values[0] = e_get_prox(7);
			prox_values[1] = e_get_prox(6);
			prox_values[2] = e_get_prox(5);
			prox_values[3] = e_get_prox(4);
			
			return prox_values;
			/*max = prox_values[0];
			for( i = 1; i < sizeof(prox_values); i++ )
			{	
				int new_max = fmax( max, prox_values[i] );
				if( new_max != max ) 
				{
					max = new_max;
					max_i = i;
				}
			}
	
			if( max_i >= 0 )
			{
				e_led_clear();
				e_set_led( 3 + max_i, 1 );
			}

			return max;
			*/	
		case RIGHT:
			prox_values[0] = e_get_prox(0);
			prox_values[1] = e_get_prox(1);
			prox_values[2] = e_get_prox(2);
			prox_values[3] = e_get_prox(3);
			
			return prox_values;
	}
}
			/*max = prox_values[0];
			for( i = 1; i < sizeof(prox_values); i++ )
			{	
				int new_max = fmax( max, prox_values[i] );
				if( new_max != max ) 
				{
					max = new_max;
					max_i = i;
				}
			}
			
			if( max_i >= 0 )
			{
				e_set_led( max_i, 1 );
			}

			return max;
	}
	return 0;
}
*/

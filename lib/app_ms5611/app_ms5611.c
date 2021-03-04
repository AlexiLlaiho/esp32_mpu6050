#include "app_ms5611.h"

/*The MS5611-01BA has only five basic commands: 
1.    Reset 
2.    Read PROM (128 bit of calibration words) 
3.    D1 conversion  
4.    D2 conversion 
5.    Read ADC result (24 bit pressure / temperature) 
*/
void task_ms5611(void *ignore)
{
	float temp_val = 0.0;
	float press_val = 0.0;
	float stp_press_val = 0.0; //start point pressure value
	float alt = 0, alt_present = 0, a = 0.26;
	    
	ms5611_init();
	ms5611_is_connected();
	ms5611_set_resolution(ms5611_resolution_osr_4096);	
	ms5611_reset();		
	ms5611_read_temperature_and_pressure(&temp_val, &stp_press_val);

    while(1) 
	{		
		ms5611_read_temperature_and_pressure(&temp_val, &press_val);		
		alt_present = getAltitude(press_val, stp_press_val);
		alt = alt_post_filter(alt, alt_present, a);	
		printf("Pressure = %f \n", alt);	
	}
	vTaskDelete(NULL);

}
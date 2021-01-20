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
    //ESP_LOGD(tag, ">> ms5611");

    while(1) 
	{		
		
		vTaskDelay(25/portTICK_PERIOD_MS);		
	}
	vTaskDelete(NULL);

}
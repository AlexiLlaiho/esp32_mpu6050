#include "app_imu.h"

char buffer[200];
int level = 0;
int n = 0;
uint16_t cycle_i = 0;
extern int16_t w_alt;
int16_t ac_x = 0, ac_y = 0, ac_z = 0, gy_x = 0, gy_y = 0, gy_z = 0;
int16_t tqx = 0, tqy = 0, tqz = 0;
uint8_t atmo_i = 0;

void app_imu(void *ignore)
{
	gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
	sd_card_init();
	check_a_file();   
	mpu60xx_preconf();
    qmc5883_test();
    hmc5883l_init();
	
    while(1) 
	{	
        Get_Data_Accelerometer(&ac_x, &ac_y, &ac_z);
        Get_Data_Gyro(&gy_x, &gy_y, &gy_z);	
        qmc5883_data(&tqx, &tqy, &tqz);
				
		n = sprintf(buffer, "Num: %u, Acc: %i, %i, %i; Gyro: %i, %i, %i; Mag: %i, %i, %i; Pres: %i \n",
																		cycle_i, 
																		ac_x, ac_y, ac_z, 
																		gy_x, gy_y, gy_z, 
																		tqx, tqy, tqz,
																		w_alt);
				
		if (cycle_i < 7600)
		{
			write_file_anv(buffer);
			gpio_set_level(GPIO_NUM_2, level);
			level = !level;
			printf("cycle_i = %u", cycle_i);
		}
		else
		{
			gpio_set_level(GPIO_NUM_2, 0);
			vTaskDelete(NULL);
		}			
		vTaskDelay(10/portTICK_PERIOD_MS);
		++cycle_i;
	}
	vTaskDelete(NULL);
}


  
  
  
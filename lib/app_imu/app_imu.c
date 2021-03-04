#include "app_imu.h"


void app_imu(void *ignore)
{
	char buffer [200];   
	float temp_val = 0.0;
	float press_val = 0.0;
	float stp_press_val = 0.0; //start point pressure value
	float alt = 0, alt_present = 0, a = 0.24;
	int level = 0;
	int n = 0;
	int16_t w_alt = 0;
	int16_t ac_x = 0, ac_y = 0, ac_z = 0, gy_x = 0, gy_y = 0, gy_z = 0;
    int16_t tqx = 0, tqy = 0, tqz = 0;
	uint8_t atmo_i = 0;

	gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
	sd_card_init();
	check_a_file();
	i2c_idf_init();    
	mpu60xx_preconf();
    qmc5883_test();
    hmc5883l_init();
	ms5611_is_connected();
	ms5611_set_resolution(ms5611_resolution_osr_4096);	
	ms5611_reset();		
	ms5611_read_temperature_and_pressure(&temp_val, &stp_press_val);

    while(1) 
	{	
        Get_Data_Accelerometer(&ac_x, &ac_y, &ac_z);
        Get_Data_Gyro(&gy_x, &gy_y, &gy_z);	
        qmc5883_data(&tqx, &tqy, &tqz);
		if(atmo_i == 5)
		{
			atmo_i = 0;
			ms5611_read_temperature_and_pressure(&temp_val, &press_val);		
			alt_present = getAltitude(press_val, stp_press_val);
			alt = alt_post_filter(alt, alt_present, a);
			w_alt = (int16_t)(alt * 1000);						
		}				
		n = sprintf(buffer, "Acc: %i, %i, %i; Gyro: %i, %i, %i; Mag: %i, %i, %i; Pres: %i \n", 
																		ac_x, ac_y, ac_z, 
																		gy_x, gy_y, gy_z, 
																		tqx, tqy, tqz,
																		w_alt);
		write_file_anv(buffer);
		++atmo_i;
		gpio_set_level(GPIO_NUM_2, level);
		level = !level;
		vTaskDelay(25/portTICK_PERIOD_MS);			
	}
	vTaskDelete(NULL);
}


  
  
  
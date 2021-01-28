#include "app_imu.h"


void app_imu(void *ignore)
{
    int16_t ac_x = 0, ac_y = 0, ac_z = 0, gy_x = 0, gy_y = 0, gy_z = 0;
    int16_t tqx = 0, tqy = 0, tqz = 0;
	float temp_val = 0.0;
	float press_val = 0.0;
	float stp_press_val = 0.0; //start point pressure value

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
		// ms5611_read_temperature_and_pressure(&temp_val, &press_val);
		// printf("Pressure = %f \n", getAltitude(press_val, stp_press_val));
        printf("Acc: %i, %i, %i; Gyro: %i, %i, %i; Mag: %i, %i, %i \n", ac_x, ac_y, ac_z, gy_x, gy_y, gy_z, tqx, tqy, tqz);	
        vTaskDelay(250/portTICK_PERIOD_MS);	
	}
	vTaskDelete(NULL);

}
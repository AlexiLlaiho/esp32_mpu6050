#include "app_mpu6050.h"

float convertRawAcceleration(int aRaw);
float convertRawGyro(int gRaw);
void GPIO_Conf(void);
void I2C_Conf(void);
void MPU6050_Conf(uint8_t Reg_Addr, int Value);
void Alpha_Betta_Filter(int16_t AcX, int16_t AcY, int16_t AcZ, int16_t GyX, int16_t GyY, int16_t GyZ);
uint8_t data[14];

int16_t accel_x, accel_y, accel_z;
int16_t gyro_x, gyro_y, gyro_z;
int Pin_Level = 0;
float roll, pitch, heading;
float convert_ax, convert_ay, convert_az, convert_gx, convert_gy, convert_gz;
double Angle_GX, Angle_GY, Angle_GZ; 

void task_mpu6050(void *ignore) {

	GPIO_Conf();
	ESP_LOGD(tag, ">> mpu6050");
	I2C_Conf();
	
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	if (cmd == NULL){
		printf("Memory_allocated \n");
		while(1);
	}
	MPU6050_Conf(0x1A, 0x04); //Preconfigured DLPF
	vTaskDelay(200/portTICK_PERIOD_MS);
	MPU6050_Conf(0x1B, 0x18); //Preconfigured Gyro_Range
	vTaskDelay(200/portTICK_PERIOD_MS);
	MPU6050_Conf(0x1C, 0x10); //Preconfigured Gyro_Range
	vTaskDelay(200/portTICK_PERIOD_MS);

	cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1));
	i2c_master_write_byte(cmd, MPU6050_ACCEL_XOUT_H, 1);
	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1));
	i2c_master_write_byte(cmd, MPU6050_PWR_MGMT_1, 1);
	i2c_master_write_byte(cmd, 0, 1);
	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	Madgwick();

	while(1) {
		// Tell the MPU6050 to position the internal register pointer to register
		// MPU6050_ACCEL_XOUT_H.
		cmd = i2c_cmd_link_create();
		ESP_ERROR_CHECK(i2c_master_start(cmd));
		ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1));
		ESP_ERROR_CHECK(i2c_master_write_byte(cmd, MPU6050_ACCEL_XOUT_H, 1));
		ESP_ERROR_CHECK(i2c_master_stop(cmd));
		ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS));
		i2c_cmd_link_delete(cmd);


		i2c_set_timeout(I2C_NUM_0, 400000);
	
		cmd = i2c_cmd_link_create();
		ESP_ERROR_CHECK(i2c_master_start(cmd));
		ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_READ, 1));

		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data,   0)); // X-High
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+1, 0)); // X-Low
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+2, 0)); // Y-High
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+3, 0)); // Y-Low
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+4, 0)); // Z-High
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+5, 1)); // Z-Low
		ESP_ERROR_CHECK(i2c_master_stop(cmd));
		ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS));
		i2c_cmd_link_delete(cmd);

		accel_x = (data[0] << 8) | data[1];
		accel_y = (data[2] << 8) | data[3];
		accel_z = (data[4] << 8) | data[5];
		// float f_accel_x = accel_x;
		// float f_accel_y = accel_y;
		// float f_accel_z = accel_z;
		// printf("Ax = %f  Ay = %f  Az = %f ", f_accel_x, f_accel_y, f_accel_z);
	
		cmd = i2c_cmd_link_create();
		ESP_ERROR_CHECK(i2c_master_start(cmd));
		ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1));
		ESP_ERROR_CHECK(i2c_master_write_byte(cmd, MPU6050_GYRO_XOUT_H, 1));
		ESP_ERROR_CHECK(i2c_master_stop(cmd));
		ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS));
		i2c_cmd_link_delete(cmd);

		cmd = i2c_cmd_link_create();
		ESP_ERROR_CHECK(i2c_master_start(cmd));
		ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_READ, 1));

		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+8, 0));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+9, 0));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+10, 0));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+11, 0));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+12, 0));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+13, 1));

		ESP_ERROR_CHECK(i2c_master_stop(cmd));
		ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS));
		i2c_cmd_link_delete(cmd);

		gyro_x = ((data[8] << 8) | data[9]) + 400;
		gyro_y = ((data[10] << 8) | data[11]) + 310;
		gyro_z = ((data[12] << 8) | data[13]) - 0;
		// float f_gyro_x = gyro_x;
		// float f_gyro_y = gyro_y;
		// float f_gyro_z = gyro_z;
		// printf("  Gx = %f  Gy = %f Gz = %f \n", f_gyro_x, f_gyro_y, f_gyro_z);

		// convert_ax = convertRawAcceleration(accel_x);
    	// convert_ay = convertRawAcceleration(accel_y);
    	// convert_az = convertRawAcceleration(accel_z);
    	// convert_gx = convertRawGyro(gyro_x);
    	// convert_gy = convertRawGyro(gyro_y);
    	// convert_gz = convertRawGyro(gyro_z);

		// updateIMU(f_gyro_x, f_gyro_y, f_gyro_z, f_accel_x, f_accel_y, f_accel_z);
		// updateIMU(convert_ax, convert_ay, convert_az, convert_gx, convert_gy, convert_gz);
		// roll = getRoll();
    	// pitch = getPitch();
    	// heading = getYaw();
		// printf ("Roll: %f, Pitch: %f, Yaw: %f \n", roll, pitch, heading);
		Alpha_Betta_Filter(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);

		gpio_set_level(GPIO_NUM_19, Pin_Level);
		Pin_Level = !Pin_Level;

		vTaskDelay(13/portTICK_PERIOD_MS);
	}

	vTaskDelete(NULL);
} 

float convertRawAcceleration(int aRaw) {
  // +2g -> 16384 LSB/g
  // +4g -> 8192 LSB/g
  // +8g -> 4096 LSB/g
  // +16g -> 2048 LSB/g
  float a = aRaw / 1;
  return a;
}

float convertRawGyro(int gRaw) {
  // +250 -> 131  LSB/grad/s
  // +500 -> 65.5 LSB/grad/s
  // +1000 -> 32.8 LSB/grad/s
  // +2000 -> 16.4 LSB/grad/s
  
  float g = gRaw / 16.4;
  return g;
}

void Alpha_Betta_Filter(int16_t AcX, int16_t AcY, int16_t AcZ, int16_t GyX, int16_t GyY, int16_t GyZ){
//   double K = 0.01;
//   double Old_GX = Angle_GX;
//   double Old_GY = Angle_GY;
  float Acc_XZ = (atan2(convertRawAcceleration(AcX), convertRawAcceleration(AcZ) ) ); // Angle is computing by accelerometer data
  float Acc_YZ = (atan2(convertRawAcceleration(AcY), convertRawAcceleration(AcZ) ) ); // Angle is computing by accelerometer data
  printf("%f   %f  \n", Acc_XZ, Acc_YZ); 
//   Angle_GX = convertRawGyro(GyX) * 0.02;
//   Angle_GY = convertRawGyro(GyY) * 0.02;
//   Angle_GZ = Angle_GZ + convertRawGyro(GyZ) * 0.02;

//   Angle_GX = ((0.1) *(Old_GX + Angle_GX) + (Acc_XZ * 0.002));
//   Angle_GY = ((0.1) *(Old_GY + Angle_GY) + (Acc_YZ * 0.002));
//   printf("%f   %f  \n", Angle_GX, Angle_GY); 
}

void GPIO_Conf(){
	gpio_config_t GPIO_Conf;
	GPIO_Conf.pin_bit_mask = GPIO_SEL_19;
	GPIO_Conf.mode = GPIO_MODE_OUTPUT;
	GPIO_Conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	GPIO_Conf.pull_up_en = GPIO_PULLUP_ENABLE;
	GPIO_Conf.intr_type = GPIO_INTR_DISABLE;
	gpio_config(&GPIO_Conf);
}

void I2C_Conf(){
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = PIN_SDA;
	conf.scl_io_num = PIN_CLK;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 100000;
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

}

void MPU6050_Conf(uint8_t Reg_Addr, int Value){
		i2c_cmd_handle_t constr = i2c_cmd_link_create();
		ESP_ERROR_CHECK(i2c_master_start(constr));
		ESP_ERROR_CHECK(i2c_master_write_byte(constr, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1));
		ESP_ERROR_CHECK(i2c_master_write_byte(constr, Reg_Addr, 1));
		ESP_ERROR_CHECK(i2c_master_write_byte(constr, Value, 1));
		ESP_ERROR_CHECK(i2c_master_stop(constr));
		ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, constr, 1000/portTICK_PERIOD_MS));
		i2c_cmd_link_delete(constr);
}




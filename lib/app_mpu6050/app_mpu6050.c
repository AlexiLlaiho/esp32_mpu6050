#include "app_mpu6050.h"

double convertRawAcceleration(int16_t aRaw);
double convertRawGyro(int16_t gRaw);
void GPIO_Conf(void);
void Get_Data_Accelerometer(void);
void Get_Data_Gyro(int16_t* Axes_X, int16_t* Axes_Y, int16_t* Axes_Z);
void I2C_Conf(void);
void MPU6050_Conf(uint8_t Reg_Addr, int Value);
void Alpha_Betta_Filter(int16_t AcX, int16_t AcY, int16_t AcZ, int16_t GyX, int16_t GyY, int16_t GyZ);
void Only_Read_One_Byte(uint8_t Reg_Number);
int row_data_massive(int16_t *mas_name, int16_t *ac_x, int16_t *ac_y, int16_t *ac_z, int16_t *g_x, int16_t *g_y, int16_t *g_z);
uint8_t switch_massives(uint16_t *iterator);

uint8_t data[14];
uint8_t Set_Data[3];
int16_t accel_x, accel_y, accel_z;
int16_t gyro_x, gyro_y, gyro_z;
int16_t Gyro_Data;
int16_t calib_gyro_x, calib_gyro_y, calib_gyro_z;
int Pin_Level = 0;
int Settings_1;
float roll, pitch, heading;
float convert_ax, convert_ay, convert_az, convert_gx, convert_gy, convert_gz;
double Angle_GX = 0; 
double Angle_GY = 0;
double Angle_GZ = 0; 
char Strp_Aq = 0;
double K = 0.02;
double Old_GX;
double Old_GY;
double exponent = 2.0;
int16_t Xa_Calib[100]; /* data */
int16_t Ya_Calib[100]; /* data */
int16_t Za_Calib[100]; /* data */
int16_t Xg_Calib[100]; /* data */
int16_t Yg_Calib[100]; /* data */
int16_t Zg_Calib[100]; /* data */
int16_t* pSensor_Xg; 
int16_t* pSensor_Yg; 
int16_t* pSensor_Zg; //pointers
float Divider = 10;
float calib_gyro_xf, calib_gyro_yf, calib_gyro_zf;
int16_t mpu_data_array_0[1200], mpu_data_array_1[1200], mpu_data_array_3[1200];
uint16_t *p_array_0 = &mpu_data_array_0, *p_array_1 = &mpu_data_array_1;
uint16_t mpu_array_lenght = sizeof(mpu_data_array_0) / sizeof(mpu_data_array_0[0]);
bool massive_1_flag = false;
bool massive_2_flag = false;

void task_mpu6050(void *ignore)
{
	GPIO_Conf();
	ESP_LOGD(tag, ">> mpu6050");
	I2C_Conf();
	Only_Read_One_Byte(0x75);
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	if (cmd == NULL){
		printf("Memory_allocated \n");
		while(1);
	}
	
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
	
	MPU6050_Conf(0x1A, 0x04); //Preconfigured DLPF	
	MPU6050_Conf(0x1B, 0x18); //Preconfigured Accelerometer_Range	
	MPU6050_Conf(0x1C, 0x10); //Preconfigured Gyro_Range
	
	pSensor_Xg = &Xg_Calib[0]; //передаем указателю нулевой адрес нулевого элемента массива
	pSensor_Yg = &Yg_Calib[0];
	pSensor_Zg = &Zg_Calib[0];

	for(uint8_t i = 0; i < 10; i++)
	{
		Get_Data_Gyro(pSensor_Xg + i, pSensor_Yg + i, pSensor_Zg + i);
		calib_gyro_x += *(pSensor_Xg + i);
		calib_gyro_y += *(pSensor_Yg + i);
		calib_gyro_z += *(pSensor_Zg + i);
		// printf("%d %d %d \n", Xg_Calib[i], Yg_Calib[i], Zg_Calib[i]);
		vTaskDelay(1000/portTICK_PERIOD_MS);
	}	
 
	calib_gyro_xf = calib_gyro_x / Divider;
	calib_gyro_yf = calib_gyro_y / Divider;
	calib_gyro_zf = calib_gyro_z / Divider;
	printf("Calibration Result: \n");
	printf("%f %f %f \n", calib_gyro_xf, calib_gyro_yf, calib_gyro_zf);
	vTaskDelay(5000/portTICK_PERIOD_MS);

	uint16_t n = 0;
	while(1) 
	{		
		Get_Data_Accelerometer();
		Get_Data_Gyro(&gyro_x, &gyro_y, &gyro_z);
		Alpha_Betta_Filter(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);
		if(switch_massives(&n) != 1)
			n++;
		gpio_set_level(GPIO_NUM_19, Pin_Level);
		Pin_Level = !Pin_Level;
		vTaskDelay(150/portTICK_PERIOD_MS);		
	}
	vTaskDelete(NULL);
} 

double convertRawAcceleration(int16_t aRaw) 
{
  // +2g -> 16384 LSB/g
  // +4g -> 8192 LSB/g
  // +8g -> 4096 LSB/g
  // +16g -> 2048 LSB/g
  double Acc_Preset = 4096;
  double a = aRaw / Acc_Preset;
  return a;
}

double convertRawGyro(int16_t gRaw) 
{
  // +250 -> 131  LSB/grad/s
  // +500 -> 65.5 LSB/grad/s
  // +1000 -> 32.8 LSB/grad/s
  // +2000 -> 16.4 LSB/grad/s
  double Gyro_Preset = 16.4;
  double g = gRaw / Gyro_Preset;
  return g;
}

void Alpha_Betta_Filter(int16_t AcX, int16_t AcY, int16_t AcZ, int16_t GyX, int16_t GyY, int16_t GyZ)
{
  double d_AcX = AcX;
  double d_AcY = AcY;
  double d_AcZ = AcZ;
  double AcYZ = (sqrt( pow(d_AcY, exponent) + pow(d_AcZ, exponent) ) );
  double Alpha = (atan2( d_AcX, AcYZ )) * 57.295 ; //http://bitaks.com/resources/inclinometer/content.html
  double AcXZ = (sqrt( pow(d_AcX, exponent) + pow(d_AcZ, exponent) ) );
  double Beta = (atan2( d_AcY, AcXZ )) * 57.295 + 1.0;
  double AcXY = (sqrt( pow(d_AcX, exponent) + pow(d_AcY, exponent) ) );
  double Tetta = (atan2( d_AcZ, AcXY )) * 57.295 ;

  if (Strp_Aq == 0) 
  {
	Old_GX = Beta;
	Old_GY = Alpha;
	Strp_Aq = 1;
  }
  else 
  {
	Old_GX = Angle_GX;
	Old_GY = Angle_GY;
  }	
 
  Angle_GX =  Old_GX + convertRawGyro(GyX) * 0.02; // 0.02 = 50Hz
  Angle_GY =  Old_GY + convertRawGyro(GyY) * 0.02;
  Angle_GZ = convertRawGyro(GyZ) * 0.02;

  Angle_GX = ((1 - K) *( Angle_GX ) + (Beta * K));
  Angle_GY = ((1 - K) *( Angle_GY ) + (Alpha * K));
  //printf("%f   %f  %f \n", Angle_GX, Angle_GY, Angle_GZ); 
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

void MPU6050_Conf(uint8_t Reg_Addr, int Value)
{
	i2c_cmd_handle_t constr = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(constr));
	ESP_ERROR_CHECK(i2c_master_write_byte(constr, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1));
	ESP_ERROR_CHECK(i2c_master_write_byte(constr, Reg_Addr, 1));
	ESP_ERROR_CHECK(i2c_master_write_byte(constr, Value, 1));
	ESP_ERROR_CHECK(i2c_master_stop(constr));
	ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, constr, 1000 / portTICK_PERIOD_MS));
	i2c_cmd_link_delete(constr);
}

void Only_Read_One_Byte(uint8_t Reg_Number){

	i2c_cmd_handle_t constr = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(constr));
	ESP_ERROR_CHECK(i2c_master_write_byte(constr, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1));
	ESP_ERROR_CHECK(i2c_master_write_byte(constr, Reg_Number, 1));
	ESP_ERROR_CHECK(i2c_master_stop(constr));
	ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, constr, 1000 / portTICK_PERIOD_MS));
	i2c_cmd_link_delete(constr);

	uint8_t Mein_Name_ist;

	constr = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(constr));
	ESP_ERROR_CHECK(i2c_master_write_byte(constr, (I2C_ADDRESS << 1) | I2C_MASTER_READ, 1)); //start adress from CONFIG_REG
	ESP_ERROR_CHECK(i2c_master_read_byte(constr, &Mein_Name_ist, 1));
	ESP_ERROR_CHECK(i2c_master_stop(constr));
	ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, constr, 1000 / portTICK_PERIOD_MS));
	i2c_cmd_link_delete(constr);

	printf("Reg_Number = %d  Value = %d \n", Reg_Number, Mein_Name_ist);
	vTaskDelay(1000 / portTICK_PERIOD_MS);	
}

void Get_Data_Accelerometer()
{
	// Tell the MPU6050 to position the internal register pointer to register
		// MPU6050_ACCEL_XOUT_H.
		i2c_cmd_handle_t cmd = i2c_cmd_link_create();
		cmd = i2c_cmd_link_create();
		ESP_ERROR_CHECK(i2c_master_start(cmd));
		ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1));
		ESP_ERROR_CHECK(i2c_master_write_byte(cmd, MPU6050_ACCEL_XOUT_H, 1));
		ESP_ERROR_CHECK(i2c_master_stop(cmd));
		ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS));
		i2c_cmd_link_delete(cmd);

		i2c_set_timeout(I2C_NUM_0, 400000); //- it gives a possible to get data
	
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
}

void Get_Data_Gyro(int16_t* Axes_X, int16_t* Axes_Y, int16_t* Axes_Z)
{
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
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

	*Axes_X = ((data[8] << 8) | data[9]);// + 400;
	*Axes_Y = ((data[10] << 8) | data[11]);// + 310;
	*Axes_Z = ((data[12] << 8) | data[13]);// - 0;
}

int row_data_massive(int16_t *mas_name, int16_t *ac_x, int16_t *ac_y, int16_t *ac_z, int16_t *g_x, int16_t *g_y, int16_t *g_z)
{
	int n = 1;
	*mas_name = *ac_x;
	*(mas_name + 1) = *ac_y;
	*(mas_name + 2) = *ac_z;
	*(mas_name + 3) = *g_x;
	*(mas_name + 4) = *g_y;
	*(mas_name + 5) = *g_z;

	return n;
}

uint8_t switch_massives(uint16_t *iterator)
{
	uint16_t m_pointer = 6;
	uint8_t state = 0;
	m_pointer *= *iterator; // calculate a new shift
	if (m_pointer <= mpu_array_lenght)
	{
		if (!massive_1_flag || massive_2_flag)
		{			
			printf(" m_pointer = %u \n", m_pointer);
			if (m_pointer == mpu_array_lenght)
			{
				massive_1_flag = true;
				printf("First massive is full! \n");
				massive_2_flag = false;
				*iterator = 0;
				state = 1;
			}
			printf("Writing in mpu_data_array_0[m_pointer] \n");
			row_data_massive(&mpu_data_array_0[m_pointer], &accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y, &gyro_z);
		}
		else
		{			
			printf(" m_pointer = %u \n", m_pointer);
			if (m_pointer == mpu_array_lenght)
			{
				massive_2_flag = true;
				printf("Second massive is full! \n");
				massive_1_flag = false;
				*iterator = 0;
				state = 1;
			}
			printf("Writing in mpu_data_array_1[m_pointer] \n");
			row_data_massive(&mpu_data_array_1[m_pointer], &accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y, &gyro_z);
		}
	}
	return state;
}
#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include <driver/i2c.h>
#include <esp_log.h>
#include <freertos/task.h>
#include <math.h>
#include "MadgwickAHRS.h"

#include "sdkconfig.h"

#define PIN_SDA 21
#define PIN_CLK 22
#define I2C_ADDRESS 0x68 // I2C address of MPU6050

#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_GYRO_XOUT_H 0x43

static char tag[] = "mpu6050";

#undef ESP_ERROR_CHECK
#define ESP_ERROR_CHECK(x)   do { esp_err_t rc = (x); if (rc != ESP_OK) { ESP_LOGE("err", "esp_err_t = %d", rc); assert(0 && #x);} } while(0);

void task_mpu6050( void *ignore );
void vTask2( void *pvParameters );
void mcpwm_example_servo_control(void *arg);

/* Глобальная переменная для хранения приоритета Задачи 2 */
xTaskHandle xTask2Handle;

void task_mpu6050(void *ignore) {

	uint8_t data[14];

	float accel_x, accel_y, accel_z;
	float gyro_x, gyro_y, gyro_z;
	int Pin_Level = 0;
	float roll, pitch, heading;

	gpio_config_t GPIO_Conf;
	GPIO_Conf.pin_bit_mask = GPIO_SEL_19;
	GPIO_Conf.mode = GPIO_MODE_OUTPUT;
	GPIO_Conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	GPIO_Conf.pull_up_en = GPIO_PULLUP_ENABLE;
	GPIO_Conf.intr_type = GPIO_INTR_DISABLE;
	gpio_config(&GPIO_Conf);
	
	ESP_LOGD(tag, ">> mpu6050");

	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = PIN_SDA;
	conf.scl_io_num = PIN_CLK;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 100000;
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

	i2c_cmd_handle_t cmd;
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
		
		//i2c_master_read(cmd, data, sizeof(data), 1);
		ESP_ERROR_CHECK(i2c_master_stop(cmd));
		ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS));
		i2c_cmd_link_delete(cmd);

		accel_x = (data[0] << 8) | data[1];
		accel_y = (data[2] << 8) | data[3];
		accel_z = (data[4] << 8) | data[5];

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

		gyro_x = (data[8] << 8) | data[9];
		gyro_y = (data[10] << 8) | data[11];
		gyro_z = (data[12] << 8) | data[13];
				

		updateIMU(gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z);
		roll = getRoll();
    	pitch = getPitch();
    	heading = getYaw();
		//printf ("Roll: %f, Pitch: %f, Yaw: %f \n", roll, pitch, heading);
		
		gpio_set_level(GPIO_NUM_19, Pin_Level);
		Pin_Level = !Pin_Level;

		vTaskDelay(13/portTICK_PERIOD_MS);
	}

	vTaskDelete(NULL);
} 

esp_err_t event_handler(void *ctx, system_event_t *event)
{
    return ESP_OK;
}

void vTask2(void *pvParameters) {
	int level = 0;
	for (;;) {
		gpio_set_level(GPIO_NUM_2, level);
		level = !level;
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
	vTaskDelete( NULL);
}

void app_main(void) {
	printf("Hello_WOLRD!");
	gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
	xTaskCreatePinnedToCore(&vTask2, "vTask2", 2048, NULL, 5, NULL, 0);
	xTaskCreatePinnedToCore(&task_mpu6050, "task_mpu6050", 2048, NULL, 1, NULL, 0);
	xTaskCreatePinnedToCore(&mcpwm_example_servo_control, "mcpwm_example_servo_control", 2048, NULL, 4, NULL, 0);

	ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
}


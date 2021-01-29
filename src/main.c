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
#include "app_blink_led.h"
#include "app_mpu6050.h"
#include "sdkconfig.h"
#include "sd_card.h"
#include "qmc5883.h"
#include "app_ms5611.h"
#include "app_imu.h"

#undef ESP_ERROR_CHECK
#define ESP_ERROR_CHECK(x)   do { esp_err_t rc = (x); if (rc != ESP_OK) { ESP_LOGE("err", "esp_err_t = %d", rc); assert(0 && #x);} } while(0);

void mcpwm_example_servo_control(void *arg);

xTaskHandle xTask2Handle; /* Глобальная переменная для хранения приоритета Задачи 2 */

esp_err_t event_handler(void *ctx, system_event_t *event)
{
    return ESP_OK;
}

void app_main(void) 
{
	//gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
	//xTaskCreatePinnedToCore(&vTask2, "vTask2", 1024, NULL, 5, NULL, 0);
	// xTaskCreatePinnedToCore(&task_mpu6050, "task_mpu6050", 4096, NULL, 1, NULL, 0);
	// xTaskCreatePinnedToCore(&task_qmc5883l, "task_qmc5883l", 4096, NULL, 1, NULL, 0);
	// xTaskCreatePinnedToCore(&task_write_file, "task_write_file", 8192, NULL, 3, NULL, 1);
	xTaskCreatePinnedToCore(&task_ms5611, "task_ms5611", 8192, NULL, 3, NULL, 1);
	// xTaskCreatePinnedToCore(&app_imu, "app_main_task", 8192, NULL, 3, NULL, 1);
	
	ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
}


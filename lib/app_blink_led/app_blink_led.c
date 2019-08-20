#include "app_blink_led.h"

void vTask2(void *pvParameters) {
    int level = 0;
	for (;;) {
		gpio_set_level(GPIO_NUM_2, level);
		level = !level;
		vTaskDelay(500 / portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);	
}
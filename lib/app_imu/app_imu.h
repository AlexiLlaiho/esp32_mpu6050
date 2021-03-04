/*!
 *******************************************************************************
 * @file    .h
 * @author  
 * @version V1.0.0.0
 * @date    20.01.2018
 * @brief   
 * @details 
 *******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT </center></h2>
 *******************************************************************************  
 */

#ifndef __APP_IMU_H
#define __APP_IMU_H

#include "freertos/FreeRTOS.h"
#include <freertos/task.h>
#include "driver/gpio.h"
// #include "driver/i2c.h"
#include "esp_log.h"
#include <math.h>
#include "i2c_driver.h"
#include "app_mpu6050.h"
#include "qmc5883.h"
#include "ms5611.h"
#include "sd_card.h"

#ifdef __cplusplus
extern "C" {
#endif
    
    void app_imu(void *ignore);   


#ifdef __cplusplus
}
#endif

#endif
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

#ifndef __APP_MS5611_H
#define __APP_MS5611_H

#include "freertos/FreeRTOS.h"
#include <freertos/task.h>
#include "esp_log.h"
#include "MadgwickAHRS.h"
#include <math.h>
#include "i2c_driver.h"
#include "ms5611.h"

#ifdef __cplusplus
extern "C" {
#endif
    
    void task_ms5611(void *ignore);

#ifdef __cplusplus
}
#endif

#endif
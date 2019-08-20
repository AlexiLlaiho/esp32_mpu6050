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

#ifndef __APP_MPU6050_H
#define __APP_MPU6050_H

#include "freertos/FreeRTOS.h"
#include <freertos/task.h>
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "MadgwickAHRS.h"

#define PIN_SDA 21
#define PIN_CLK 22
#define I2C_ADDRESS 0x68 // I2C address of MPU6050

#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_GYRO_XOUT_H 0x43

static char tag[] = "mpu6050";

#ifdef __cplusplus
extern "C" {
#endif

void task_mpu6050(void *ignore);

#ifdef __cplusplus
}
#endif

#endif
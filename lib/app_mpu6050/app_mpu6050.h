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
// #include "driver/gpio.h"
// #include "driver/i2c.h"
#include "esp_log.h"
#include "MadgwickAHRS.h"
#include <math.h>
#include "i2c_driver.h"

#define PIN_SDA 21
#define PIN_CLK 22
#define I2C_ADDRESS 0x68 // I2C address of MPU6050

#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_GYRO_XOUT_H 0x43

typedef enum
{
    DLPF_CFG_0 = 0,
    DLPF_CFG_1,
    DLPF_CFG_2,
    DLPF_CFG_3,
    DLPF_CFG_4,
    DLPF_CFG_5
} DLPF;

typedef enum{
FULL_SCALE_RANGE_250 = 0,
FULL_SCALE_RANGE_500,
FULL_SCALE_RANGE_1000,
FULL_SCALE_RANGE_2000
}FS_SEL;

static char tag[] = "mpu6050";
extern bool massive_1_flag;
extern bool massive_2_flag;
extern uint16_t mpu_array_lenght;
extern uint16_t *p_array_0, *p_array_1;

#ifdef __cplusplus
extern "C" {
#endif
    
    void task_mpu6050(void *ignore);
    void mpu60xx_preconf(void);
    void Get_Data_Accelerometer(int16_t *acc_x, int16_t *acc_y, int16_t *acc_z);
    void Get_Data_Gyro(int16_t* Axes_X, int16_t* Axes_Y, int16_t* Axes_Z);
    void Alpha_Betta_Filter(int16_t AcX, int16_t AcY, int16_t AcZ, int16_t GyX, int16_t GyY, int16_t GyZ);


#ifdef __cplusplus
}
#endif

#endif
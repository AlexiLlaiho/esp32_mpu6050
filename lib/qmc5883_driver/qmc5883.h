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

#ifndef __QMC5885_H
#define __QMC5885_H

#include <driver/i2c.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/gpio.h"
#include <math.h>

#include "sdkconfig.h"

#ifdef __cplusplus
extern "C"
{
#endif
    void gpio_conf();
    void i2c_setup(void);
    void hmc5883l_init(void);
    uint8_t qmc5883_test(void);
    void qmc5883_data(void);

#ifdef __cplusplus
}
#endif

#endif
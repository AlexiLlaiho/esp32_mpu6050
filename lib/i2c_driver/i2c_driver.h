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

#ifndef __I2C_DRIVER_H
#define __I2C_DRIVER_H

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define PIN_SDA 21
#define PIN_CLK 22

struct i2c_master_packet {
		uint8_t address;
		uint8_t data_length;
		uint8_t data[6];
	};

enum status_code{
    STATUS_ERR_OVERFLOW,
    STATUS_OK
};

#ifdef __cplusplus
extern "C"
{
#endif
    void i2c_idf_init(void);
	void i2c_master_init(void);
	void i2c_write_addr(uint8_t i2c_device_addr, uint8_t mdata, uint8_t ldata);
	void i2c_read_data(uint8_t i2c_device_addr, uint8_t ldata);
	enum status_code i2c_master_write_packet_wait(struct i2c_master_packet *p);
	enum status_code i2c_master_read_packet_wait(struct i2c_master_packet *r);

#ifdef __cplusplus
}
#endif

#endif //__I2C_DRIVER_H
#include "i2c_driver.h"

void i2c_idf_init()
{
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

void i2c_write_addr(uint8_t i2c_device_addr, uint8_t i2c_device_reg)
{
    i2c_cmd_handle_t cmd;
    cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    i2c_master_write_byte(cmd, (i2c_device_addr << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, i2c_device_reg, 1); //0x00 = "Data Output X MSB Register"
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

void i2c_read_data(uint8_t i2c_device_addr)
{
    uint8_t data[] = {0, 0, 0, 0, 0, 0};
    i2c_cmd_handle_t  cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_device_addr << 1) | I2C_MASTER_READ, 1);
    i2c_master_read_byte(cmd, data, 0);     //"Data Output X MSB Register"
    i2c_master_read_byte(cmd, data + 1, 0); //"Data Output X LSB Register"
    i2c_master_read_byte(cmd, data + 2, 0); //"Data Output Z MSB Register"
    i2c_master_read_byte(cmd, data + 3, 0); //"Data Output Z LSB Register"
    i2c_master_read_byte(cmd, data + 4, 0); //"Data Output Y MSB Register"
    i2c_master_read_byte(cmd, data + 5, 1); //"Data Output Y LSB Register "
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

enum status_code
{
    i2c_ok,
    i2c_error
};

void i2c_master_init()
{
    i2c_idf_init();
}

enum status_code i2c_master_write_packet_wait(struct i2c_master_packet *transfer)
{
    /* Do the transfer */

}

enum status_code i2c_master_read_packet_wait(struct i2c_master_packet *read_transfer)
{
    
}
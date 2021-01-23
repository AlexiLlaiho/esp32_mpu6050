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
    ESP_ERROR_CHECK( i2c_param_config(I2C_NUM_0, &conf) );
    ESP_ERROR_CHECK( i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0) );
}

void i2c_write_addr(uint8_t i2c_device_addr, uint8_t mdata, uint8_t ldata)
{
    i2c_cmd_handle_t cmd;
    cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK( i2c_master_start(cmd) );
    vTaskDelay(5000/portTICK_PERIOD_MS);
    ESP_ERROR_CHECK( i2c_master_write_byte(cmd, (i2c_device_addr << 1) | I2C_MASTER_WRITE, 1) );
    ESP_ERROR_CHECK( i2c_master_write(cmd, mdata, ldata, 1) ); 
    ESP_ERROR_CHECK( i2c_master_stop(cmd) );
    ESP_ERROR_CHECK( i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS) );
    i2c_cmd_link_delete(cmd);
}

void i2c_read_data(uint8_t i2c_device_addr, uint8_t ldata)
{
    uint8_t data[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    i2c_cmd_handle_t  cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_device_addr << 1) | I2C_MASTER_READ, 1);
    for(uint8_t i = 0; i < ldata; i++) 
    {
        if(i != (ldata - 1))
            i2c_master_read_byte(cmd, (data + i), 0);
        else
            i2c_master_read_byte(cmd, (data + i), 1);       
    }
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

void i2c_master_init()
{
    i2c_idf_init();
}   

enum status_code i2c_master_write_packet_wait(struct i2c_master_packet *p)
{   
    printf("ADDr = %u, Data = %p, Length = %u \n", p->address, p->data, p->data_length);
    vTaskDelay(1000/portTICK_PERIOD_MS);
    i2c_write_addr(p->address, p->data, p->data_length);
    return STATUS_OK;
}

enum status_code i2c_master_read_packet_wait(struct i2c_master_packet *r)
{
    i2c_read_data(r->address, r->data_length);
    return 1;    
}
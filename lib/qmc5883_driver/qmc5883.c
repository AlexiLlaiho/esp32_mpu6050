#include "qmc5883.h"

#define PIN_SDA 21
#define PIN_CLK 22

#define I2C_ADDRESS 0x0D

/* Register numbers */
#define QMC5883L_X_LSB 0x00
#define QMC5883L_X_MSB 0x01
#define QMC5883L_Y_LSB 0x02
#define QMC5883L_Y_MSB 0x03
#define QMC5883L_Z_LSB 0x04
#define QMC5883L_Z_MSB 0x05
#define QMC5883L_STATUS 0x06
#define QMC5883L_TEMP_LSB 0x07
#define QMC5883L_TEMP_MSB 0x08
#define QMC5883L_CONFIG 0x09
#define QMC5883L_CONFIG2 0x0A
#define QMC5883L_RESET 0x0B
#define QMC5883L_RESERVED 0x0C
#define QMC5883L_CHIP_ID 0x0D
#define QMC5883L_MODE_50HZ 0x55
#define QMC5883L_ROL_PNT 0x40

static char tag[] = "hmc5883l";
uint8_t data[6];

#undef ESP_ERROR_CHECK
#define ESP_ERROR_CHECK(x)                         \
    do                                             \
    {                                              \
        esp_err_t rc = (x);                        \
        if (rc != ESP_OK)                          \
        {                                          \
            ESP_LOGE("err", "esp_err_t = %d", rc); \
            assert(0 && #x);                       \
        }                                          \
    } while (0);

void i2c_setup()
{
    ESP_LOGD(tag, ">> hmc5883l");
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

void hmc5883l_init()
{
    //Set value in "Configuration Register 1"
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    uint8_t settings[] = {0x09, 0x1D};

    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1));    
    ESP_ERROR_CHECK(i2c_master_write(cmd, settings, 2, 1));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);

    uint8_t settings_2[] = {0x0B, 0x01};
    //Set value in "Configuration Register 2"
    cmd = i2c_cmd_link_create();   
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1);    
    i2c_master_write(cmd, settings_2, 2, 1); // the I2C data pointer automatically rolls between 00H ~ 06H
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

void qmc5883_test()
{
    uint8_t test_q[] = {0x0D, 0x00, 0xFF};
    i2c_cmd_handle_t constr = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(constr));
    ESP_ERROR_CHECK(i2c_master_write_byte(constr, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1));
    ESP_ERROR_CHECK(i2c_master_write(constr, test_q, 1, 1));
    ESP_ERROR_CHECK(i2c_master_stop(constr));
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, constr, 1000 / portTICK_PERIOD_MS));
    i2c_cmd_link_delete(constr);

    uint8_t Mein_Name_ist;

    constr = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(constr));
    ESP_ERROR_CHECK(i2c_master_write_byte(constr, (I2C_ADDRESS << 1) | I2C_MASTER_READ, 1)); //start adress from CONFIG_REG
    ESP_ERROR_CHECK(i2c_master_read_byte(constr, &Mein_Name_ist, 1));
    ESP_ERROR_CHECK(i2c_master_stop(constr));
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, constr, 1000 / portTICK_PERIOD_MS));
    i2c_cmd_link_delete(constr);

    printf("Value = %d \n", Mein_Name_ist);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}

void qmc5883_data(int16_t *q_x, int16_t *q_y, int16_t *q_z)
{
    i2c_cmd_handle_t cmd;
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, QMC5883L_X_LSB, 1); //0x00 = "Data Output X MSB Register"
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    //Read values for X, Y and Z
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_READ, 1);
    i2c_master_read_byte(cmd, data, 0);     //"Data Output X MSB Register"
    i2c_master_read_byte(cmd, data + 1, 0); //"Data Output X LSB Register"
    i2c_master_read_byte(cmd, data + 2, 0); //"Data Output Z MSB Register"
    i2c_master_read_byte(cmd, data + 3, 0); //"Data Output Z LSB Register"
    i2c_master_read_byte(cmd, data + 4, 0); //"Data Output Y MSB Register"
    i2c_master_read_byte(cmd, data + 5, 1); //"Data Output Y LSB Register "
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    *q_x = data[1] << 8 | data[0];
    *q_y = data[3] << 8 | data[2];
    *q_z = data[5] << 8 | data[4];
    // int angle = atan2((double)q_y, (double)q_x) * (180 / 3.14159265) + 180; // angle in degrees
    // printf("angle: %d, x: %d, y: %d, z: %d \n", angle, x, y, z);    
}

void task_qmc5883l(void *ignore)
{  
    int16_t tqx = 0, tqy = 0, tqz = 0;  
    i2c_idf_init();
    qmc5883_test();
    hmc5883l_init();

    while (1)
    {
        qmc5883_data(&tqx, &tqy, &tqz);
        vTaskDelay(25 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}
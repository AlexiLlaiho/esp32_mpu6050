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

void gpio_conf()
{
    gpio_config_t GPIO_Conf;
    GPIO_Conf.pin_bit_mask = GPIO_SEL_19;
    GPIO_Conf.mode = GPIO_MODE_OUTPUT;
    GPIO_Conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    GPIO_Conf.pull_up_en = GPIO_PULLUP_ENABLE;
    GPIO_Conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&GPIO_Conf);
}

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
    i2c_cmd_handle_t cmd;
    //Set value in "Configuration Register 1"
    cmd = i2c_cmd_link_create();
    if (cmd == NULL)
    {
        printf("Memory_allocated \n");
        while (1);
    }
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, QMC5883L_CONFIG, 1); // 0x02 = "Mode register"
    i2c_master_write_byte(cmd, QMC5883L_MODE_50HZ, 1); // 0x00 = "Continuous-Measurement Mode"
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    //Set value in "Configuration Register 2"
    cmd = i2c_cmd_link_create();
    if (cmd == NULL)
    {
        printf("Memory_allocated \n");
        while (1);
    }
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, QMC5883L_CONFIG2, 1);
    i2c_master_write_byte(cmd, QMC5883L_ROL_PNT, 1); // the I2C data pointer automatically rolls between 00H ~ 06H
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);    
}

uint8_t qmc5883_test()
{
    //Test type "Who_am_I?"
    i2c_cmd_handle_t constr;
    uint8_t name = 0;
    constr = i2c_cmd_link_create();
    if (constr == NULL)
    {
        printf("Memory_allocated \n");
        while (1);
    }
    ESP_ERROR_CHECK(i2c_master_start(constr));
    ESP_ERROR_CHECK(i2c_master_write_byte(constr, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1));
    ESP_ERROR_CHECK(i2c_master_write_byte(constr, QMC5883L_CHIP_ID, 1)); //0x0D = "ChipIDRegister"
    ESP_ERROR_CHECK(i2c_master_stop(constr));
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, constr, 100 / portTICK_PERIOD_MS));
    i2c_cmd_link_delete(constr);

    //Get data from chip identification register. It returns 0xFF
    constr = i2c_cmd_link_create();
    if (constr == NULL)
    {
        printf("Memory_allocated \n");
        while (1);
    }
    ESP_ERROR_CHECK(i2c_master_start(constr));
    ESP_ERROR_CHECK(i2c_master_write_byte(constr, (I2C_ADDRESS << 1) | I2C_MASTER_READ, 1));
    i2c_master_read_byte(constr, &name, 0);
    ESP_ERROR_CHECK(i2c_master_stop(constr));
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, constr, 100 / portTICK_PERIOD_MS));
    i2c_cmd_link_delete(constr);
    printf("name is = %u", name);
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    
    return name; //if we get a response type 0xFF, it means OK
}

void qmc5883_data()
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

    short x = data[1] << 8 | data[0];
    short z = data[3] << 8 | data[2];
    short y = data[5] << 8 | data[4];
    int angle = atan2((double)y, (double)x) * (180 / 3.14159265) + 180; // angle in degrees
    ESP_LOGD(tag, "angle: %d, x: %d, y: %d, z: %d", angle, x, y, z);
}
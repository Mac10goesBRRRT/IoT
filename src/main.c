#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "driver/i2c.h"

#define I2C_MASTER_PORT 0         // port number
#define I2C_MASTER_SDA_IO 21      // GPIO data
#define I2C_MASTER_SCL_IO 22      // GPIO clock
#define I2C_MASTER_FREQ_HZ 400000 // I2C frequency

#define ACK 1
#define NACK 0

i2c_config_t conf = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = I2C_MASTER_SDA_IO,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_io_num = I2C_MASTER_SCL_IO,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = I2C_MASTER_FREQ_HZ};

uint8_t i2c_register_read(
    i2c_port_t port_num,
    uint8_t dev_addr,
    uint8_t reg_addr);

static esp_err_t i2c_register_write(
    i2c_port_t port_num,
    uint8_t dev_addr,
    uint8_t reg_addr,
    uint8_t data);

static esp_err_t i2c_multi_register_read(
    i2c_port_t port_num,
    uint8_t dev_addr,
    uint8_t reg_addr,
    uint8_t *data,
    size_t size);

uint8_t lux_read_sensor_id();
 u_int16_t lux_read_sensor_value(u_int8_t reg, u_int8_t addr);

void read_write_i2c (void * pvParameters);

void app_main()
{
    i2c_param_config(I2C_MASTER_PORT, &conf);
    i2c_driver_install(I2C_MASTER_PORT, conf.mode, 0, 0, 0);
    //printf("0x%x\n",i2c_register_read(I2C_MASTER_PORT, 0x29, 0x0));
    i2c_register_write(I2C_MASTER_PORT, 0x29, 0x0, 0x3);
    i2c_register_write(I2C_MASTER_PORT, 0x29, 0x1, 0b00010010);
    printf("beep boop hell hier: %d\n", lux_read_sensor_value(0x29,0));
    /*TaskHandle_t xtask_main = NULL;
    xTaskCreate(&read_write_i2c, "readWritei2c", 4000, NULL, tskIDLE_PRIORITY, &xtask_main);
    configASSERT(xtask_main);*/
}

void read_write_i2c (void * pvParameters){
    for(;;){
        // printf("0x%x\n",lux_read_sensor_id());
        //printf("0x%x\n", i2c_register_read(I2C_MASTER_PORT, 0x29, 0xA));
        //printf("0x%x\n", i2c_register_read(I2C_MASTER_PORT, 0x29, 0xC));
        //i2c_register_write(I2C_MASTER_PORT,0x29,0x2,0x0);
        //printf("0x%x\n", i2c_register_read(I2C_MASTER_PORT, 0x29, 0xD));
        //uint8_t data[2];
        //i2c_multi_register_read(I2C_MASTER_PORT,0x29,0xC,&data,2);
        //printf("0x%d%d\n", data[0],data[1]);
        printf("beep boop hell hier: %d\n", lux_read_sensor_value(0x29,0));
        //lux_read_sensor_value(1, 0x29);
        vTaskDelay(200); //2 Sekunden
    }
}


uint8_t i2c_register_read(
    i2c_port_t port_num,
    uint8_t dev_addr,
    uint8_t reg_addr)
{
    uint8_t data;
    // port_num I2C Port Num, dev_addr Slave I2C, reg_addr register I2C
    reg_addr += 128;
    i2c_cmd_handle_t cmd;
    uint8_t addr = dev_addr << 1;

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr, ACK);
    i2c_master_write_byte(cmd, reg_addr, ACK);
    i2c_master_start(cmd);
    addr += 1;
    i2c_master_write_byte(cmd, addr, ACK);
    i2c_master_read_byte(cmd, &data, NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, 10);
    i2c_cmd_link_delete(cmd);

    return data;
}

uint8_t lux_read_sensor_id()
{
    for (uint8_t i = 0; i < 0xFF; i++)
    {
        uint8_t data = i2c_register_read(I2C_MASTER_PORT, i, 0xA);
        if (data == 0x50)
        {
            return i;
        }
    }
    return -1;
}

static esp_err_t i2c_register_write(
    i2c_port_t port_num,
    uint8_t dev_addr,
    uint8_t reg_addr,
    uint8_t data)
{
    //port_num i2c port, dev_addr Slave addr
    reg_addr += 128;
    i2c_cmd_handle_t cmd;
    uint8_t addr = dev_addr << 1;
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr, ACK);
    i2c_master_write_byte(cmd, reg_addr, ACK);
    i2c_master_write_byte(cmd, data, ACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, 10);
    i2c_cmd_link_delete(cmd);

    return 0;
}

static esp_err_t i2c_multi_register_read(
    i2c_port_t port_num,
    uint8_t dev_addr,
    uint8_t reg_addr,
    uint8_t *data,
    size_t size){
    // port_num I2C Port Num, dev_addr Slave I2C, reg_addr register I2C
    reg_addr += 128;
    i2c_cmd_handle_t cmd;
    uint8_t addr = dev_addr << 1;
    cmd = i2c_cmd_link_create();
    //Start + Slave Addr + Write
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr, ACK);
    //Register Addr
    i2c_master_write_byte(cmd, reg_addr, ACK);
    //Start + Slave Addr + Read
    addr += 1;
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr, ACK);
    //Jetzt lesen
    i2c_master_read(cmd, data, size, NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, 10);
    i2c_cmd_link_delete(cmd);

    return 0;
    }

    u_int16_t lux_read_sensor_value(u_int8_t addr, u_int8_t reg){
        reg = reg*2 + 0xC;
        uint8_t data[2];
        i2c_multi_register_read(I2C_MASTER_PORT, addr, reg, &data, 2);
        u_int16_t value = data[1];
        value = value << 8;
        value = value | data[0];
        return value;
    }
    
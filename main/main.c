#include <stdio.h>
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ina219.h"

void app_main(void)
{
    i2c_master_bus_handle_t bus_handle;
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = -1,
        .scl_io_num = 5,
        .sda_io_num = 6,
        .glitch_ignore_cnt = 7,
    };
    i2c_new_master_bus(&i2c_bus_config,&bus_handle);

    ina219_config_t ina219_config={
        .mode=INA219_MODE_CIRC_SHUNT_BUS,
        .shunt_adc_mode=INA219_ADC_12BITS,
        .bus_adc_mode=INA219_ADC_12BITS,
        .gain=INA219_PGA_GAIN_NO_DIV,
        .bus_range=INA219_BUS_VOL_RANGE_12V,

        .clk_speed=400*1000,
        .dev_addr=0x40,
        .r_shunt_ohm=0.01f,
        .i_max_a=3.5f
    };

    ina219_dev_t dev=NULL;
    ina219_init(dev,bus_handle,&ina219_config);

    float power;
    while(1){
        ina219_read(dev,INA219_VAL_POW,&power);
        printf("%f\n",power);
        esp_rom_delay_us(100);
    }
}

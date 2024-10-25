#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"

typedef enum{
    INA219_BUS_VOL_RANGE_12V = 0x00,
    INA219_BUS_VOL_RANGE_24V = 0x01
}ina219_bus_voltage_range_t;

typedef enum{
    INA219_PGA_GAIN_NO_DIV = 0x00,
    INA219_PGA_GAIN_DIV_2 = 0x01,
    INA219_PGA_GAIN_DIV_4 = 0x02,
    INA219_PGA_GAIN_DIV_8 = 0x03,
}ina219_pga_gain_t;

typedef enum{
    INA219_ADC_9BITS =  0x00,
    INA219_ADC_10BITS = 0x01,
    INA219_ADC_11BITS = 0x02,
    INA219_ADC_12BITS = 0x03,

    INA219_ADC_12BITS_1_SAMPLE = 0x08,
    INA219_ADC_12BITS_2_SAMPLE = 0x09,
    INA219_ADC_12BITS_4_SAMPLE = 0x0A,
    INA219_ADC_12BITS_8_SAMPLE = 0x0B,
    INA219_ADC_12BITS_16_SAMPLE = 0x0C,
    INA219_ADC_12BITS_32_SAMPLE = 0x0D,
    INA219_ADC_12BITS_64_SAMPLE = 0x0E,
    INA219_ADC_12BITS_128_SAMPLE = 0x0F
}ina219_adc_mode_t;

typedef enum{
    INA219_MODE_POWERDOWN =     0x00,
    INA219_MODE_TRIG_SHUNT =    0x01,
    INA219_MODE_TRIG_BUS =      0x02,
    INA219_MODE_TRIG_SHUNT_BUS =0x03,
    INA219_MODE_ADC_OFF =       0x04,
    INA219_MODE_CIRC_SHUNT =    0x05,
    INA219_MODE_CIRC_BUS =      0x06,
    INA219_MODE_CIRC_SHUNT_BUS =0x07
}ina219_mode_t;

typedef enum{
    INA219_VAL_BUS_VOL = 0x00,
    INA219_VAL_SHUNT_VOL = 0x01,
    INA219_VAL_CUR = 0x02,
    INA219_VAL_POW = 0x03
}ina219_value_t;

typedef struct{
    uint32_t    clk_speed;
    uint8_t     dev_addr;

    float       r_shunt_ohm;
    float       i_max_a;

    ina219_bus_voltage_range_t  bus_range;
    ina219_pga_gain_t           gain;
    ina219_adc_mode_t           bus_adc_mode;
    ina219_adc_mode_t           shunt_adc_mode;
    ina219_mode_t               mode;

}ina219_config_t;

typedef void* ina219_dev_t;

esp_err_t ina219_init(ina219_dev_t dev, i2c_master_bus_handle_t handle, const ina219_config_t *config);
esp_err_t ina219_deinit(ina219_dev_t dev);
esp_err_t ina219_read(ina219_dev_t dev, ina219_value_t type, float *val);
esp_err_t ina219_trig_bus(ina219_dev_t dev, ina219_adc_mode_t mode);
esp_err_t ina219_trig_shunt(ina219_dev_t dev, ina219_adc_mode_t mode);

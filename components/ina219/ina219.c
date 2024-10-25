#include <math.h>
#include <string.h>
#include "machine/endian.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "ina219_regs.h"
#include "ina219.h"

#define INA219_ACQUIRE_TIMEOUT_MS                   10

const static char *TAG="ina219";

typedef union{
#if BYTE_ORDER == LITTLE_ENDIAN
    struct{
        uint16_t mode:3;
        uint16_t sadc:4;
        uint16_t badc:4;
        uint16_t pg:2;
        uint16_t brng:1;
        uint16_t reserve:1;
        uint16_t rst:1; 
    };
#else
    struct{
        uint16_t rst:1; 
        uint16_t reserve:1;
        uint16_t brng:1;
        uint16_t pg:2;
        uint16_t badc:4;
        uint16_t sadc:4;
        uint16_t mode:3;
    };
#endif
    uint16_t val;
}ina219_config_reg_t;

typedef struct{
    i2c_master_dev_handle_t dev_handle;
    SemaphoreHandle_t lock;

    float lsb_bus;
    float lsb_shunt;
    float lsb_cur;
    float lsb_pow;

    uint16_t shunt_mask;

    uint8_t cache[3];
}ina219_dev_s;

typedef ina219_dev_s *ina219_dev_p; 

static inline esp_err_t INA219_LOCK(ina219_dev_p ina219_dev){
    if(xSemaphoreTake(ina219_dev->lock,pdMS_TO_TICKS(INA219_ACQUIRE_TIMEOUT_MS))==pdTRUE)
        return ESP_OK;
    else
        return ESP_ERR_TIMEOUT;
}

static inline esp_err_t INA219_UNLOCK(ina219_dev_p ina219_dev){
    return xSemaphoreGive(ina219_dev->lock);
}

static inline esp_err_t ina219_write_reg(ina219_dev_p ina219_dev, uint8_t addr, uint16_t val){
    esp_err_t ret=ESP_OK;
    ESP_GOTO_ON_ERROR(INA219_LOCK(ina219_dev),err,TAG,"fail to lock device");
    memcpy(ina219_dev->cache,&addr,1);
#if BYTE_ORDER == LITTLE_ENDIAN
    val=__builtin_bswap16(val);
#endif
    memcpy(ina219_dev->cache+1,&val,2);
    ESP_GOTO_ON_ERROR(i2c_master_transmit(ina219_dev->dev_handle,ina219_dev->cache,3,portMAX_DELAY),err,TAG,"fail to transmit");
    INA219_UNLOCK(ina219_dev);
err:
    return ret;
}

static inline esp_err_t ina219_read_reg(ina219_dev_p ina219_dev, uint8_t addr, uint16_t *val){
    esp_err_t ret=ESP_OK;
    ESP_GOTO_ON_ERROR(INA219_LOCK(ina219_dev),err,TAG,"fail to lock device");
    ESP_GOTO_ON_ERROR(i2c_master_transmit_receive(ina219_dev->dev_handle,&addr,1,(uint8_t *)val,2,portMAX_DELAY),err,TAG,"fail to receive");
#if BYTE_ORDER == LITTLE_ENDIAN
    *val=__builtin_bswap16(*val);
#endif
    INA219_UNLOCK(ina219_dev);
err:
    return ret;
}

esp_err_t ina219_init(ina219_dev_t dev, i2c_master_bus_handle_t handle, const ina219_config_t *config){
    esp_err_t ret=ESP_OK;
    dev=calloc(1,sizeof(ina219_dev_s));
    ESP_GOTO_ON_FALSE(dev,ESP_ERR_NO_MEM,err,TAG,"fail to calloc ina219_dev_s");

    ina219_dev_s *ina219_dev=dev;

    float bus_vol_max=0.0f;
    float shunt_vol_max=0.0f;

    switch(config->bus_range){
        case INA219_BUS_VOL_RANGE_12V: bus_vol_max=12.0f; break;
        case INA219_BUS_VOL_RANGE_24V: bus_vol_max=24.0f; break;
        default: 
            ret=ESP_ERR_INVALID_ARG;
            goto err;
            break;
    }

    switch (config->gain)
    {
        case INA219_PGA_GAIN_NO_DIV: shunt_vol_max=0.04f; ina219_dev->shunt_mask=0x8FFF; break;
        case INA219_PGA_GAIN_DIV_2: shunt_vol_max=0.08f; ina219_dev->shunt_mask=0x9FFF; break;
        case INA219_PGA_GAIN_DIV_4: shunt_vol_max=0.16f; ina219_dev->shunt_mask=0xBFFF; break;
        case INA219_PGA_GAIN_DIV_8: shunt_vol_max=0.32f; ina219_dev->shunt_mask=0xFFFF; break;
        default: 
            ret=ESP_ERR_INVALID_ARG;
            goto err;
            break;
    }

    switch(config->bus_adc_mode){
        case INA219_ADC_9BITS: ina219_dev->lsb_bus=bus_vol_max/512.0f; break;
        case INA219_ADC_10BITS: ina219_dev->lsb_bus=bus_vol_max/1024.0f; break;
        case INA219_ADC_11BITS: ina219_dev->lsb_bus=bus_vol_max/2048.0f; break;
        case INA219_ADC_12BITS: 
        case INA219_ADC_12BITS_1_SAMPLE:
        case INA219_ADC_12BITS_2_SAMPLE:
        case INA219_ADC_12BITS_4_SAMPLE:
        case INA219_ADC_12BITS_8_SAMPLE:
        case INA219_ADC_12BITS_16_SAMPLE:
        case INA219_ADC_12BITS_32_SAMPLE:
        case INA219_ADC_12BITS_64_SAMPLE:
        case INA219_ADC_12BITS_128_SAMPLE: ina219_dev->lsb_bus=bus_vol_max/4096.0f; break;
        default: 
            ret=ESP_ERR_INVALID_ARG;
            goto err;
            break;
    }

    switch(config->shunt_adc_mode){
        case INA219_ADC_9BITS: ina219_dev->lsb_shunt=shunt_vol_max/512.0f; break;
        case INA219_ADC_10BITS: ina219_dev->lsb_shunt=shunt_vol_max/1024.0f; break;
        case INA219_ADC_11BITS: ina219_dev->lsb_shunt=shunt_vol_max/2048.0f; break;
        case INA219_ADC_12BITS: 
        case INA219_ADC_12BITS_1_SAMPLE:
        case INA219_ADC_12BITS_2_SAMPLE:
        case INA219_ADC_12BITS_4_SAMPLE:
        case INA219_ADC_12BITS_8_SAMPLE:
        case INA219_ADC_12BITS_16_SAMPLE:
        case INA219_ADC_12BITS_32_SAMPLE:
        case INA219_ADC_12BITS_64_SAMPLE:
        case INA219_ADC_12BITS_128_SAMPLE: ina219_dev->lsb_shunt=shunt_vol_max/4096.0f; break;
        default: 
            ret=ESP_ERR_INVALID_ARG;
            goto err;
            break;
    }

    ina219_dev->lsb_cur=config->i_max_a/32768.0f;
    ina219_dev->lsb_pow=20.0f*ina219_dev->lsb_cur;
    
    i2c_device_config_t cfg={
        .dev_addr_length=I2C_ADDR_BIT_7,
        .device_address=config->dev_addr,
        .scl_speed_hz=config->clk_speed
    };
    ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(handle,&cfg,&(ina219_dev->dev_handle)),err,TAG,"fail to register device");
    
    ina219_dev->lock=xSemaphoreCreateBinary();
    xSemaphoreGive(ina219_dev->lock);

    ESP_GOTO_ON_ERROR(ina219_write_reg(ina219_dev,INA219_REG_CONFIG,0x8000),err,TAG,"fail to reset");
    vTaskDelay(pdMS_TO_TICKS(50));
    
    ina219_config_reg_t reg_val={
        .rst=0,
        .brng=config->bus_range,
        .pg=config->gain,
        .badc=config->bus_adc_mode,
        .sadc=config->shunt_adc_mode,
        .mode=config->mode
    };
    ESP_GOTO_ON_ERROR(ina219_write_reg(ina219_dev,INA219_REG_CONFIG,reg_val.val),err,TAG,"fail to setup");

    uint16_t cal= (uint16_t) roundf(0.04096f/config->r_shunt_ohm/ina219_dev->lsb_cur);
    cal<<=1;
    ESP_GOTO_ON_ERROR(ina219_write_reg(ina219_dev,INA219_REG_CALIB,cal),err,TAG,"fail to setup calib");
err:
    if(dev)
        free(dev);
    return ret;
}

esp_err_t ina219_deinit(ina219_dev_t dev){
    esp_err_t ret=ESP_OK;
    ESP_GOTO_ON_FALSE(dev,ESP_ERR_INVALID_ARG,err,TAG,"device is invalid");

    ina219_dev_s *ina219_dev=dev;
    ESP_GOTO_ON_ERROR(i2c_master_bus_rm_device(ina219_dev->dev_handle),err,TAG,"fail to remove device");
err:
    if(dev)
        free(dev);
    return ret;
}

esp_err_t ina219_read(ina219_dev_t dev, ina219_value_t type, float *val){
    esp_err_t ret=ESP_OK;
    ina219_dev_s *ina219_dev=dev;

    uint16_t tmp=0x0000;
    int16_t stmp=0x0000;
    switch(type){
        case INA219_VAL_BUS_VOL:
            ESP_GOTO_ON_ERROR(ina219_read_reg(ina219_dev,INA219_REG_BUS_VOLTAGE,&tmp),err,TAG,"fail to get BUS VOL");
            *val=tmp*ina219_dev->lsb_bus;
            break;

        case INA219_VAL_SHUNT_VOL:
            ESP_GOTO_ON_ERROR(ina219_read_reg(ina219_dev,INA219_REG_SHUNT_VOLTAGE,(uint16_t *)(&stmp)),err,TAG,"fail to get SHUNT VOL");
            stmp&=ina219_dev->shunt_mask;
            *val=stmp*ina219_dev->lsb_shunt;
            break;

        case INA219_VAL_CUR:
            ESP_GOTO_ON_ERROR(ina219_read_reg(ina219_dev,INA219_REG_CURRENT,(uint16_t *)(&stmp)),err,TAG,"fail to get CURRENT");
            *val=stmp*ina219_dev->lsb_cur;
            break;

        case INA219_VAL_POW:
            ESP_GOTO_ON_ERROR(ina219_read_reg(ina219_dev,INA219_REG_POWER,&tmp),err,TAG,"fail to get POWER");
            *val=tmp*ina219_dev->lsb_pow;
            break;

        default: 
            ret=ESP_ERR_INVALID_ARG;
            goto err;
            break;
    }
err:
    return ret;
}

esp_err_t ina219_trig_bus(ina219_dev_t dev, ina219_adc_mode_t mode){
    esp_err_t ret=ESP_OK;
    ina219_dev_s *ina219_dev=dev;
    ina219_config_reg_t reg_val;

    ESP_GOTO_ON_ERROR(ina219_read_reg(ina219_dev,INA219_REG_CONFIG,&(reg_val.val)),err,TAG,"fail to read CONFIG");
    reg_val.badc=mode;
    ESP_GOTO_ON_ERROR(ina219_write_reg(ina219_dev,INA219_REG_CONFIG,reg_val.val),err,TAG,"fail to write CONFIG");
err:
    return ret;
}

esp_err_t ina219_trig_shunt(ina219_dev_t dev, ina219_adc_mode_t mode){
    esp_err_t ret=ESP_OK;
    ina219_dev_s *ina219_dev=dev;
    ina219_config_reg_t reg_val;

    ESP_GOTO_ON_ERROR(ina219_read_reg(ina219_dev,INA219_REG_CONFIG,&(reg_val.val)),err,TAG,"fail to read CONFIG");
    reg_val.sadc=mode;
    ESP_GOTO_ON_ERROR(ina219_write_reg(ina219_dev,INA219_REG_CONFIG,reg_val.val),err,TAG,"fail to write CONFIG");
err:
    return ret;
}

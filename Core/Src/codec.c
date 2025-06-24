#include "codec.h"
#include "main.h"
#include "stm32f7xx_hal_gpio.h"
#include "stm32f7xx_hal_i2c.h"
#include <stdint.h>
#include <stdio.h>

extern I2C_HandleTypeDef hi2c1;

static uint8_t codec_page = 0xFF;

static void codec_write_register(uint8_t page, uint8_t *payload, size_t len) {
    int result;

    if (page != codec_page) {
        uint8_t page_cmd[2] = { 0x00, page };
        result = HAL_I2C_Master_Transmit(&hi2c1, 0x30, page_cmd, 2, HAL_MAX_DELAY);
        if (result != HAL_OK) {
            Error_Handler();
        }

        codec_page = page;
    }

    result = HAL_I2C_Master_Transmit(&hi2c1, 0x30, payload, len, HAL_MAX_DELAY);
    if (result != HAL_OK) {
        Error_Handler();
    }
}

#define CODEC_WRITE_REGISTER(REG, ...) \
    do { \
        uint8_t _tmp_data[] = { (REG & 0xFF), __VA_ARGS__ }; \
        codec_write_register(REG >> 8, _tmp_data, sizeof(_tmp_data)); \
    } while (0)


void codec_init(void) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 0);
    for (volatile int k = 0; k < 1000000; ++k) {}
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1);
    for (volatile int k = 0; k < 1000000; ++k) {}
    CODEC_WRITE_REGISTER(0x0001, 0x01); // software reset
    for (volatile int k = 0; k < 1000000; ++k) {}
    
    CODEC_WRITE_REGISTER(0x000B, 0x81, 0x82); // NDAC=1, MDAC=2, dividers powered on
    CODEC_WRITE_REGISTER(0x0012, 0x81, 0x82); // NADC=1, MADC=2, dividers powered on

    /* POWER CONFIGURATION */

    CODEC_WRITE_REGISTER(0x0102, 0x09); // power up AVDD LDO
    CODEC_WRITE_REGISTER(0x0101, 0x08); // disable weak AVDD
    CODEC_WRITE_REGISTER(0x0102, 0x01); // keep AVDD LDO enabled and power up analog blocks
    // set full chip CM to 0.9V,
    // select 1.65V CM for HPL and HPR, 
    // use 3.3V/2 for LOL/LOR,
    // power HPL/HPR with LDOIN
    CODEC_WRITE_REGISTER(0x010A, 0x3B);
    CODEC_WRITE_REGISTER(0x013D, 0x00); // select ADC PTM_R4 (highest performance)
    CODEC_WRITE_REGISTER(0x0103, 0x00, 0x00); // select DAC PTM_P3/4 (highest performance)
    CODEC_WRITE_REGISTER(0x0147, 0x32); // set input power-up time to 3.1ms (for ADC)
    CODEC_WRITE_REGISTER(0x017B, 0x01); // set the REF charging time to 40ms

    /* OTHER STUFF? */

    CODEC_WRITE_REGISTER(0x003D, 0x01); // select ADC PRB_R1 (?)
    CODEC_WRITE_REGISTER(0x0014, 0x80); // set ADC OSR to 128

    CODEC_WRITE_REGISTER(0x0147, 0x32); // set MicPGA startup delay to 3.1ms
    CODEC_WRITE_REGISTER(0x0134, 0x80); // Route IN1L to LEFT_P with 20K input impedance
    CODEC_WRITE_REGISTER(0x0136, 0x80); // Route Common Mode to LEFT_M with impedance of 20K
    //CODEC_WRITE_REGISTER(0x0137, 0x80); // Route IN1R to RIGHT_P with input impedance of 20K
    CODEC_WRITE_REGISTER(0x0139, 0x80); // Route Common Mode to RIGHT_M with impedance of 20K
    // Unmute Left MICPGA and set gain to +20dB
    CODEC_WRITE_REGISTER(0x013B, 40);
    // Unmute Right MICPGA, Gain selection of 6dB to make channel gain 0dB
    // Register of 6dB with input impedance of 20K => Channel Gain of 0dB
    CODEC_WRITE_REGISTER(0x013C, 0x0C);

    CODEC_WRITE_REGISTER(0x0051, 0xC0); // Power up Left and Right ADC Channels
    CODEC_WRITE_REGISTER(0x0052, 0x00); // Unmute Left and Right ADC Digital Volume Control.

    /* DAC Configuration */

    CODEC_WRITE_REGISTER(0x0114, 0x25); // de-pop: 5 time constants, 6k resistance
    CODEC_WRITE_REGISTER(0x010C, 0x08, 0x08); // route LDAC/RDAC to HPL/HPR
    CODEC_WRITE_REGISTER(0x010E, 0x08, 0x08); // route LDAC/RDAC to LOL/LOR
    CODEC_WRITE_REGISTER(0x0109, 0x3C); // power up HPL/HPR and LOL/LOR
    CODEC_WRITE_REGISTER(0x0110, 0x00, 0x00); // unmute HPL/HPR driver, 0dB gain
    CODEC_WRITE_REGISTER(0x0112, 10, 10); // unmute LOL/LOR driver, 10dB gain
    CODEC_WRITE_REGISTER(0x0041, 0x00, 0x00); // DAC => 0dB
    CODEC_WRITE_REGISTER(0x003F, 0xD6); // power up LDAC/RDAC
    CODEC_WRITE_REGISTER(0x0040, 0x00); // unmute LDAC/RDAC
}

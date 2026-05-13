/*
 * bme280.c — Bosch BME280 driver for the FreeRTOS Assignment build.
 *            Polled I2C using the shared writeI2C1 / readI2C; init
 *            uses vTaskDelay so it must be called once the scheduler
 *            is running.
 *
 * readI2C is hardwired to 2 bytes, so multi-byte transfers are done
 * as a sequence of 2-byte reads with the chip's auto-pointer
 * increment. Inefficient but functional.
 *
 * Compensation math: 32-/64-bit fixed-point from Bosch datasheet
 * Appendix A reference C code.
 */

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "drivers/i2cOptDriver.h"
#include "drivers/bme280.h"
#include "utils/uartstdio.h"

/*-----------------------------------------------------------*/

static uint8_t g_bme_addr = 0x77;

#define BME_CHIP_ID            0x60

#define REG_CHIP_ID            0xD0
#define REG_RESET              0xE0
#define REG_CTRL_HUM           0xF2
#define REG_STATUS             0xF3
#define REG_CTRL_MEAS          0xF4
#define REG_CONFIG             0xF5
#define REG_DATA_START         0xF7

#define REG_CALIB_TP_START     0x88   /* 0x88..0xA1 = 26 bytes */
#define REG_CALIB_H_START      0xE1   /* 0xE1..0xE7 = 7 bytes  */

#define CMD_SOFT_RESET         0xB6

/*-----------------------------------------------------------*/

static uint16_t dig_T1;
static int16_t  dig_T2, dig_T3;
static uint16_t dig_P1;
static int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
static uint8_t  dig_H1, dig_H3;
static int16_t  dig_H2, dig_H4, dig_H5;
static int8_t   dig_H6;

static int32_t  t_fine;

/*-----------------------------------------------------------*/

static bool prvReadN(uint8_t reg, uint8_t *buf, uint32_t n)
{
    uint32_t i = 0;
    while (i < n)
    {
        uint8_t tmp[2];
        if (!readI2C(g_bme_addr, (uint8_t)(reg + i), tmp)) return false;
        buf[i] = tmp[0];
        if (i + 1 < n) buf[i + 1] = tmp[1];
        i += 2;
    }
    return true;
}

/*-----------------------------------------------------------*/

bool bme280_init(void)
{
    uint8_t id_buf[2];
    bool found = false;

    g_bme_addr = 0x77;
    if (readI2C(g_bme_addr, REG_CHIP_ID, id_buf) && id_buf[0] == BME_CHIP_ID)
    {
        found = true;
    }
    else
    {
        g_bme_addr = 0x76;
        if (readI2C(g_bme_addr, REG_CHIP_ID, id_buf) && id_buf[0] == BME_CHIP_ID)
            found = true;
    }
    UARTprintf("  BME280 probe: %s at 0x%02x  id=0x%02x\n",
               found ? "found" : "NOT FOUND",
               g_bme_addr, found ? id_buf[0] : 0xFFu);
    if (!found) return false;

    writeI2C1(g_bme_addr, REG_RESET, CMD_SOFT_RESET);
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Wait for NVM image copy to finish. */
    for (uint32_t i = 0; i < 100; i++)
    {
        uint8_t sb[2];
        if (readI2C(g_bme_addr, REG_STATUS, sb) && !(sb[0] & 0x01)) break;
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    uint8_t cal[26];
    if (!prvReadN(REG_CALIB_TP_START, cal, 26)) return false;
    dig_T1 = (uint16_t)(cal[0]  | ((uint16_t)cal[1]  << 8));
    dig_T2 = (int16_t) (cal[2]  | ((uint16_t)cal[3]  << 8));
    dig_T3 = (int16_t) (cal[4]  | ((uint16_t)cal[5]  << 8));
    dig_P1 = (uint16_t)(cal[6]  | ((uint16_t)cal[7]  << 8));
    dig_P2 = (int16_t) (cal[8]  | ((uint16_t)cal[9]  << 8));
    dig_P3 = (int16_t) (cal[10] | ((uint16_t)cal[11] << 8));
    dig_P4 = (int16_t) (cal[12] | ((uint16_t)cal[13] << 8));
    dig_P5 = (int16_t) (cal[14] | ((uint16_t)cal[15] << 8));
    dig_P6 = (int16_t) (cal[16] | ((uint16_t)cal[17] << 8));
    dig_P7 = (int16_t) (cal[18] | ((uint16_t)cal[19] << 8));
    dig_P8 = (int16_t) (cal[20] | ((uint16_t)cal[21] << 8));
    dig_P9 = (int16_t) (cal[22] | ((uint16_t)cal[23] << 8));
    dig_H1 = cal[25];

    uint8_t hc[8] = {0};
    if (!prvReadN(REG_CALIB_H_START, hc, 7)) return false;
    dig_H2 = (int16_t)(hc[0] | ((uint16_t)hc[1] << 8));
    dig_H3 = hc[2];
    dig_H4 = (int16_t)(((int16_t)(int8_t)hc[3] << 4) | (hc[4] & 0x0F));
    dig_H5 = (int16_t)(((int16_t)(int8_t)hc[5] << 4) | (hc[4] >> 4));
    dig_H6 = (int8_t)hc[6];

    /* ctrl_hum must be written before ctrl_meas. */
    writeI2C1(g_bme_addr, REG_CTRL_HUM,  0x01);    /* osrs_h = 1x */
    writeI2C1(g_bme_addr, REG_CONFIG,    0xA0);    /* standby 1000 ms, no filter */
    writeI2C1(g_bme_addr, REG_CTRL_MEAS, 0x27);    /* osrs_t=1, osrs_p=1, mode=normal */
    vTaskDelay(pdMS_TO_TICKS(20));

    return true;
}

/*-----------------------------------------------------------*/

static int32_t prvCompT(int32_t adc_T)
{
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) *
            ((int32_t)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) *
              ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) *
            ((int32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}

static uint32_t prvCompP(int32_t adc_P)
{
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
    var2 = var2 + (((int64_t)dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) +
           ((var1 * (int64_t)dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;
    if (var1 == 0) return 0;
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
    return (uint32_t)p;
}

static uint32_t prvCompH(int32_t adc_H)
{
    int32_t v;
    v = (t_fine - ((int32_t)76800));
    v = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) -
            (((int32_t)dig_H5) * v)) + ((int32_t)16384)) >> 15) *
         (((((((v * ((int32_t)dig_H6)) >> 10) *
              (((v * ((int32_t)dig_H3)) >> 11) +
               ((int32_t)32768))) >> 10) +
            ((int32_t)2097152)) *
              ((int32_t)dig_H2) + 8192) >> 14));
    v = (v - (((((v >> 15) * (v >> 15)) >> 7) *
              ((int32_t)dig_H1)) >> 4));
    if (v < 0) v = 0;
    if (v > 419430400) v = 419430400;
    return (uint32_t)(v >> 12);
}

/*-----------------------------------------------------------*/

bool bme280_read(float *temp_c, float *hum_pct, float *press_hpa)
{
    uint8_t d[8];
    if (!prvReadN(REG_DATA_START, d, 8)) return false;

    int32_t adc_P = ((int32_t)d[0] << 12) | ((int32_t)d[1] << 4) | (d[2] >> 4);
    int32_t adc_T = ((int32_t)d[3] << 12) | ((int32_t)d[4] << 4) | (d[5] >> 4);
    int32_t adc_H = ((int32_t)d[6] <<  8) |  (int32_t)d[7];

    int32_t  T = prvCompT(adc_T);
    uint32_t P = prvCompP(adc_P);
    uint32_t H = prvCompH(adc_H);

    *temp_c    = (float)T / 100.0f;
    *press_hpa = (float)P / 25600.0f;
    *hum_pct   = (float)H / 1024.0f;
    return true;
}

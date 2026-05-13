/*
 * bmi160.c — Bosch BMI160 accelerometer driver for the FreeRTOS
 *            Assignment build. Polled, shares I2C0 with OPT3001 and
 *            BME280. Uses vTaskDelay (scheduler must be running when
 *            init is called).
 *
 * Init:   probe 0x69 then 0x68 by chip-ID -> accel normal mode
 *         -> 100 Hz ODR -> +-2 g (16384 LSB / g)
 * Read:   three 2-byte reads of ACC_X_L / ACC_Y_L / ACC_Z_L.
 *         BMI160 sends LOW byte first (little-endian on the wire).
 */

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "drivers/i2cOptDriver.h"
#include "drivers/bmi160.h"
#include "utils/uartstdio.h"

/*-----------------------------------------------------------*/

static uint8_t g_bmi_addr = 0x69;

#define REG_CHIP_ID        0x00
#define REG_PMU_STATUS     0x03
#define BMI160_CHIP_ID     0xD1
#define REG_CMD            0x7E
#define REG_ACC_CONF       0x40
#define REG_ACC_RANGE      0x41
#define REG_ACC_X_L        0x12
#define REG_ACC_Y_L        0x14
#define REG_ACC_Z_L        0x16

#define CMD_ACC_NORMAL     0x11

#define ACC_CONF_100HZ     0x28
#define ACC_RANGE_4G       0x05      /* +-4 g  -> 8192 LSB / g
                                      * (more headroom for impact detection
                                      * than the +-2 g range used in the
                                      * UI tester; gravity still reads as
                                      * 1 g on the resting axis). */

#define ACC_LSB_PER_G      8192.0f

/*-----------------------------------------------------------*/

static bool prvReadAxis(uint8_t low_reg, int16_t *out_raw);

static bool prvBmiWriteCmd(uint8_t cmd)
{
    return writeI2C1(g_bmi_addr, REG_CMD, cmd);
}

static bool prvBmiWriteReg(uint8_t reg, uint8_t val)
{
    return writeI2C1(g_bmi_addr, reg, val);
}

/*-----------------------------------------------------------*/

bool bmi160_init(void)
{
    /* Probe both addresses by reading CHIP_ID. */
    uint8_t id_bytes[2];
    bool found = false;

    g_bmi_addr = 0x69;
    if (readI2C(g_bmi_addr, REG_CHIP_ID, id_bytes) &&
        id_bytes[0] == BMI160_CHIP_ID)
    {
        found = true;
    }
    else
    {
        g_bmi_addr = 0x68;
        if (readI2C(g_bmi_addr, REG_CHIP_ID, id_bytes) &&
            id_bytes[0] == BMI160_CHIP_ID)
            found = true;
    }
    UARTprintf("  BMI160 probe: %s at 0x%02x  id=0x%02x\n",
               found ? "found" : "NOT FOUND",
               g_bmi_addr, found ? id_bytes[0] : 0xFFu);
    if (!found) return false;

    /* Bring accel to NORMAL power mode. ~4 ms transition; pad to be safe. */
    prvBmiWriteCmd(CMD_ACC_NORMAL);
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Range + ODR. */
    prvBmiWriteReg(REG_ACC_RANGE, ACC_RANGE_4G);
    prvBmiWriteReg(REG_ACC_CONF,  ACC_CONF_100HZ);
    vTaskDelay(pdMS_TO_TICKS(5));

    /* Quick PMU sanity check. */
    uint8_t pmu[2] = {0xFF, 0xFF};
    if (readI2C(g_bmi_addr, REG_PMU_STATUS, pmu))
    {
        uint8_t acc_pmu = (pmu[0] >> 4) & 0x3;
        UARTprintf("  BMI160 PMU=0x%02x acc_pmu=%u\n", pmu[0], acc_pmu);
        if (acc_pmu != 1) return false;
    }

    int16_t test;
    return prvReadAxis(REG_ACC_X_L, &test);
}

/*-----------------------------------------------------------*/

static bool prvReadAxis(uint8_t low_reg, int16_t *out_raw)
{
    uint8_t buf[2];
    if (!readI2C(g_bmi_addr, low_reg, buf)) return false;
    *out_raw = (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
    return true;
}

bool bmi160_read_accel_g(float *ax, float *ay, float *az)
{
    int16_t rx, ry, rz;
    if (!prvReadAxis(REG_ACC_X_L, &rx)) return false;
    if (!prvReadAxis(REG_ACC_Y_L, &ry)) return false;
    if (!prvReadAxis(REG_ACC_Z_L, &rz)) return false;

    *ax = (float)rx / ACC_LSB_PER_G;
    *ay = (float)ry / ACC_LSB_PER_G;
    *az = (float)rz / ACC_LSB_PER_G;
    return true;
}

bool bmi160_read_y_g(float *ay)
{
    int16_t ry;
    if (!prvReadAxis(REG_ACC_Y_L, &ry)) return false;
    *ay = (float)ry / ACC_LSB_PER_G;
    return true;
}

bool bmi160_read_accel_raw(int16_t *ax, int16_t *ay, int16_t *az)
{
    if (!prvReadAxis(REG_ACC_X_L, ax)) return false;
    if (!prvReadAxis(REG_ACC_Y_L, ay)) return false;
    if (!prvReadAxis(REG_ACC_Z_L, az)) return false;
    return true;
}

float bmi160_lsb_per_g(void) { return ACC_LSB_PER_G; }

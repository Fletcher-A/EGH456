/*
 * i2cOptDriver.c — interrupt-driven I2C driver for the SHT31.
 *
 * Same as the opt3001 driver, but using I2C2
 * 
 * 
 * 
 * 
 * 
 * 
 */

#include "drivers/i2cOptDriver.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driver_lib/gpio.h"
#include "driver_lib/i2c.h"
#include "driver_lib/interrupt.h"
#include "driver_lib/pin_map.h"
#include "driver_lib/sysctl.h"
#include "driver_lib/systick.h"

/*-----------------------------------------------------------*/

typedef enum {
    I2C_STATE_IDLE,
    I2C_STATE_WR_DATA0,
    I2C_STATE_WR_DATA1,
    I2C_STATE_WR1_DATA0,    /* single-byte data write (BMI160 / BME280) */
    I2C_STATE_WR_DONE,
    I2C_STATE_RD_SHT31,
    I2C_STATE_RD_SHT31_FIN
} I2CState_t;

static volatile I2CState_t g_eState = I2C_STATE_IDLE;
static volatile bool       g_bError = false;
static volatile bool       g_bDone  = false;
static uint8_t             g_ui8Addr;
static uint8_t            *g_pui8Data;

// used by the sht to count to 6
static uint8_t sht31Count = 0;

#define I2C_TIMEOUT_LOOPS  2000000   /* ~tens of ms at 120 MHz */

/*-----------------------------------------------------------*/

void initI2C2(uint32_t ui32SysClock)
{
    /* GPIO + I2C0 peripheral. */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C2)) {}
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION)) {}

    GPIOPinConfigure(GPIO_PN5_I2C2SCL);
    GPIOPinConfigure(GPIO_PN4_I2C2SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTN_BASE, GPIO_PIN_5);
    GPIOPinTypeI2C(GPIO_PORTN_BASE, GPIO_PIN_4);
    I2CMasterInitExpClk(I2C2_BASE, ui32SysClock, false);  /* 100 kHz */

    /* Enable the master interrupt — our ISR drives the state machine. */
    I2CMasterIntEnable(I2C2_BASE);
    IntEnable(INT_I2C2);
}

/*-----------------------------------------------------------*/

static bool prvWaitDone(void)
{
    /* Busy-wait on the done flag, bounded by a loop count. The ISR
     * is what advances the state machine, so this loop just needs to
     * yield long enough for it to fire several times. */
    uint32_t spins = I2C_TIMEOUT_LOOPS;
    while (!g_bDone && spins--) { }
    if (!g_bDone)
    {
        g_eState = I2C_STATE_IDLE;
        return false;
    }
    g_bDone = false;
    return !g_bError;
}

/*-----------------------------------------------------------*/

bool writeI2C2(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *data)
{
    g_ui8Addr  = ui8Addr;
    g_pui8Data = data;
    g_bError   = false;
    g_bDone    = false;
    g_eState   = I2C_STATE_WR_DATA0;

    I2CMasterSlaveAddrSet(I2C2_BASE, ui8Addr, false);
    I2CMasterDataPut(I2C2_BASE, ui8Reg);
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_START);

    return prvWaitDone();
}

/*-----------------------------------------------------------*/

/* Single-byte data write: { START, addr+W, reg, data, STOP }.
 * Used by the SHT to start conversion
  */
bool writeI2C2Single(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t data)
{
    static uint8_t s_one;
    s_one = data;
    g_ui8Addr  = ui8Addr;
    g_pui8Data = &s_one;
    g_bError   = false;
    g_bDone    = false;
    g_eState   = I2C_STATE_WR1_DATA0;

    I2CMasterSlaveAddrSet(I2C2_BASE, ui8Addr, false);
    I2CMasterDataPut(I2C2_BASE, ui8Reg);
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_START);

    return prvWaitDone();
}


/*-----------------------------------------------------------*/
/* Initiates SHT31 read
*/
bool readSHT31(uint8_t ui8Addr, uint8_t *data)
{
    // uiAddr will always be 0x44
    g_ui8Addr  = ui8Addr;
    g_pui8Data = data;
    g_bError   = false;
    g_bDone    = false;
    g_eState   = I2C_STATE_RD_SHT31;
    sht31Count = 0;

    I2CMasterSlaveAddrSet(I2C2_BASE, ui8Addr, true);
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);

    return prvWaitDone();
}


/*-----------------------------------------------------------*/

void I2C2IntHandler(void)
{
    I2CMasterIntClear(I2C2_BASE);

    if (I2CMasterErr(I2C2_BASE) != I2C_MASTER_ERR_NONE)
    {
        /* Address NACK during a BURST transaction leaves the master
         * holding the bus low. Issue an explicit ERROR_STOP so SDA/SCL
         * get released — otherwise probes for absent sensors (e.g.
         * sht31_init at 0x44 with no chip fitted) wedge the bus for
         * every subsequent transaction. */
        I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_ERROR_STOP);
        g_bError = true;
        g_eState = I2C_STATE_IDLE;
        g_bDone  = true;
        return;
    }

    switch (g_eState)
    {
        case I2C_STATE_WR_DATA0:
            I2CMasterDataPut(I2C2_BASE, g_pui8Data[0]);
            I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
            g_eState = I2C_STATE_WR_DATA1;
            break;

        case I2C_STATE_WR_DATA1:
            I2CMasterDataPut(I2C2_BASE, g_pui8Data[1]);
            I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
            g_eState = I2C_STATE_WR_DONE;
            break;

        case I2C_STATE_WR1_DATA0:
            I2CMasterDataPut(I2C2_BASE, g_pui8Data[0]);
            I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
            g_eState = I2C_STATE_WR_DONE;
            break;

        case I2C_STATE_WR_DONE:
            g_eState = I2C_STATE_IDLE;
            g_bDone  = true;
            break;

        case I2C_STATE_RD_SHT31:
            g_pui8Data[sht31Count++] = (uint8_t)I2CMasterDataGet(I2C2_BASE);
            if (sht31Count >  5)
            {
                I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
                g_eState = I2C_STATE_RD_SHT31_FIN;
            } else {
                I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
                g_eState = I2C_STATE_RD_SHT31; // 
            }
            break;       

        case I2C_STATE_RD_SHT31_FIN:
            g_pui8Data[5] = (uint8_t)I2CMasterDataGet(I2C2_BASE);
            g_eState = I2C_STATE_IDLE;
            g_bDone  = true;
            break;

        default:
            g_eState = I2C_STATE_IDLE;
            g_bDone  = true;
            break;
    }
}

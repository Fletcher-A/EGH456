/*
 * i2cOptDriver.c — interrupt-driven I2C driver for the OPT3001.
 *
 * Bare-metal port of the FreeRTOS lab 5 driver:
 *   - Same state machine in the I2C ISR.
 *   - Same write / read sequences.
 *   - Instead of blocking on a FreeRTOS semaphore, the caller spins
 *     on a `volatile bool g_done` flag set by the ISR.
 *   - SysTick is used to enforce a millisecond timeout so a stuck
 *     bus doesn't hang the system.
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
    I2C_STATE_RD_REG_SENT,
    I2C_STATE_RD_DATA0,
    I2C_STATE_RD_DATA1
} I2CState_t;

static volatile I2CState_t g_eState = I2C_STATE_IDLE;
static volatile bool       g_bError = false;
static volatile bool       g_bDone  = false;
static uint8_t             g_ui8Addr;
static uint8_t            *g_pui8Data;

#define I2C_TIMEOUT_LOOPS  2000000   /* ~tens of ms at 120 MHz */

/*-----------------------------------------------------------*/

void initI2C(uint32_t ui32SysClock)
{
    /* GPIO + I2C0 peripheral. */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0)) {}
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB)) {}

    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
    I2CMasterInitExpClk(I2C0_BASE, ui32SysClock, false);  /* 100 kHz */

    /* Enable the master interrupt — our ISR drives the state machine. */
    I2CMasterIntEnable(I2C0_BASE);
    IntEnable(INT_I2C0);
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

bool writeI2C(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *data)
{
    g_ui8Addr  = ui8Addr;
    g_pui8Data = data;
    g_bError   = false;
    g_bDone    = false;
    g_eState   = I2C_STATE_WR_DATA0;

    I2CMasterSlaveAddrSet(I2C0_BASE, ui8Addr, false);
    I2CMasterDataPut(I2C0_BASE, ui8Reg);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

    return prvWaitDone();
}

/*-----------------------------------------------------------*/

/* Single-byte data write: { START, addr+W, reg, data, STOP }.
 * Needed for sensors with 8-bit registers (BMI160, BME280). The
 * 2-byte writeI2C() above is hardwired for OPT3001's 16-bit
 * registers and spills a stray byte into reg+1. */
bool writeI2C1(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t data)
{
    static uint8_t s_one;
    s_one = data;
    g_ui8Addr  = ui8Addr;
    g_pui8Data = &s_one;
    g_bError   = false;
    g_bDone    = false;
    g_eState   = I2C_STATE_WR1_DATA0;

    I2CMasterSlaveAddrSet(I2C0_BASE, ui8Addr, false);
    I2CMasterDataPut(I2C0_BASE, ui8Reg);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

    return prvWaitDone();
}

/*-----------------------------------------------------------*/

bool readI2C(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *data)
{
    g_ui8Addr  = ui8Addr;
    g_pui8Data = data;
    g_bError   = false;
    g_bDone    = false;
    g_eState   = I2C_STATE_RD_REG_SENT;

    I2CMasterSlaveAddrSet(I2C0_BASE, ui8Addr, false);
    I2CMasterDataPut(I2C0_BASE, ui8Reg);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);

    return prvWaitDone();
}

/*-----------------------------------------------------------*/

void I2C0MasterIntHandler(void)
{
    I2CMasterIntClear(I2C0_BASE);

    if (I2CMasterErr(I2C0_BASE) != I2C_MASTER_ERR_NONE)
    {
        g_bError = true;
        g_eState = I2C_STATE_IDLE;
        g_bDone  = true;
        return;
    }

    switch (g_eState)
    {
        case I2C_STATE_WR_DATA0:
            I2CMasterDataPut(I2C0_BASE, g_pui8Data[0]);
            I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
            g_eState = I2C_STATE_WR_DATA1;
            break;

        case I2C_STATE_WR_DATA1:
            I2CMasterDataPut(I2C0_BASE, g_pui8Data[1]);
            I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
            g_eState = I2C_STATE_WR_DONE;
            break;

        case I2C_STATE_WR1_DATA0:
            I2CMasterDataPut(I2C0_BASE, g_pui8Data[0]);
            I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
            g_eState = I2C_STATE_WR_DONE;
            break;

        case I2C_STATE_WR_DONE:
            g_eState = I2C_STATE_IDLE;
            g_bDone  = true;
            break;

        case I2C_STATE_RD_REG_SENT:
            I2CMasterSlaveAddrSet(I2C0_BASE, g_ui8Addr, true);
            I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
            g_eState = I2C_STATE_RD_DATA0;
            break;

        case I2C_STATE_RD_DATA0:
            g_pui8Data[0] = (uint8_t)I2CMasterDataGet(I2C0_BASE);
            I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
            g_eState = I2C_STATE_RD_DATA1;
            break;

        case I2C_STATE_RD_DATA1:
            g_pui8Data[1] = (uint8_t)I2CMasterDataGet(I2C0_BASE);
            g_eState = I2C_STATE_IDLE;
            g_bDone  = true;
            break;

        default:
            g_eState = I2C_STATE_IDLE;
            g_bDone  = true;
            break;
    }
}

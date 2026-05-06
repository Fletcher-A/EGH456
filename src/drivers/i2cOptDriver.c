/*
 * i2cOptDriver.c — interrupt-driven I2C driver for the OPT3001.
 *
 * Replaces the busy-wait polling version. Each transfer (write or read)
 * is started from task context, then the calling task blocks on a binary
 * semaphore. The I2C0 master interrupt advances a state machine through
 * the multi-byte transfer and gives the semaphore when the transfer is
 * complete or when an error is detected.
 *
 * State flow — write (reg + 2 data bytes):
 *   Task: DataPut(reg), BURST_SEND_START  ->  ISR fires
 *   ISR (WR_DATA0):  DataPut(data[0]), BURST_SEND_CONT  ->  ISR fires
 *   ISR (WR_DATA1):  DataPut(data[1]), BURST_SEND_FINISH  ->  ISR fires
 *   ISR (WR_DONE):   give semaphore
 *
 * State flow — read (write reg pointer, then receive 2 bytes):
 *   Task: DataPut(reg), SINGLE_SEND  ->  ISR fires
 *   ISR (RD_REG_SENT):  SlaveAddrSet(READ), BURST_RECEIVE_START  ->  ISR fires
 *   ISR (RD_DATA0):     data[0]=DataGet(), BURST_RECEIVE_FINISH  ->  ISR fires
 *   ISR (RD_DATA1):     data[1]=DataGet(), give semaphore
 *
 * Synchronisation: binary semaphore (not mutex). A binary semaphore is
 * used because the give comes from an ISR, not the same task that takes.
 * FreeRTOS mutexes carry task-ownership semantics that prevent ISR-side
 * giving; a binary semaphore has no owner and is the correct primitive
 * for ISR-to-task signalling.
 *
 * Timeout: xSemaphoreTake() uses a 200 ms deadline. If the ISR never
 * fires (bus stuck or clock-stretching hung), the function returns false
 * rather than blocking the calling task forever.
 */

#include "drivers/i2cOptDriver.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/i2c.h"
#include "driverlib/interrupt.h"
#include "FreeRTOS.h"
#include "semphr.h"

/*-----------------------------------------------------------*/

typedef enum {
    I2C_STATE_IDLE,
    I2C_STATE_WR_DATA0,
    I2C_STATE_WR_DATA1,
    I2C_STATE_WR_DONE,
    I2C_STATE_RD_REG_SENT,
    I2C_STATE_RD_DATA0,
    I2C_STATE_RD_DATA1
} I2CState_t;

static volatile I2CState_t g_eState = I2C_STATE_IDLE;
static volatile bool       g_bError = false;
static uint8_t             g_ui8Addr;
static uint8_t            *g_pui8Data;
static SemaphoreHandle_t   xI2CDone  = NULL;
static SemaphoreHandle_t   xI2CMutex = NULL;

#define I2C_TIMEOUT_MS  200

/*-----------------------------------------------------------*/

void initI2CDriver(void)
{
    xI2CDone  = xSemaphoreCreateBinary();
    xI2CMutex = xSemaphoreCreateMutex();
    I2CMasterIntEnable(I2C0_BASE);
    IntEnable(INT_I2C0);
}

/*-----------------------------------------------------------*/

bool writeI2C(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *data)
{
    if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) != pdPASS)
        return false;

    g_ui8Addr  = ui8Addr;
    g_pui8Data = data;
    g_bError   = false;
    g_eState   = I2C_STATE_WR_DATA0;

    I2CMasterSlaveAddrSet(I2C0_BASE, ui8Addr, false);
    I2CMasterDataPut(I2C0_BASE, ui8Reg);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

    if (xSemaphoreTake(xI2CDone, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) != pdPASS)
    {
        g_eState = I2C_STATE_IDLE;
        xSemaphoreGive(xI2CMutex);
        return false;
    }
    xSemaphoreGive(xI2CMutex);
    return !g_bError;
}

/*-----------------------------------------------------------*/

bool readI2C(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *data)
{
    if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) != pdPASS)
        return false;

    g_ui8Addr  = ui8Addr;
    g_pui8Data = data;
    g_bError   = false;
    g_eState   = I2C_STATE_RD_REG_SENT;

    I2CMasterSlaveAddrSet(I2C0_BASE, ui8Addr, false);
    I2CMasterDataPut(I2C0_BASE, ui8Reg);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);

    if (xSemaphoreTake(xI2CDone, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) != pdPASS)
    {
        g_eState = I2C_STATE_IDLE;
        xSemaphoreGive(xI2CMutex);
        return false;
    }
    xSemaphoreGive(xI2CMutex);
    return !g_bError;
}

/*-----------------------------------------------------------*/

void I2C0MasterIntHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    I2CMasterIntClear(I2C0_BASE);

    if (I2CMasterErr(I2C0_BASE) != I2C_MASTER_ERR_NONE)
    {
        g_bError = true;
        g_eState = I2C_STATE_IDLE;
        xSemaphoreGiveFromISR(xI2CDone, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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

        case I2C_STATE_WR_DONE:
            g_eState = I2C_STATE_IDLE;
            xSemaphoreGiveFromISR(xI2CDone, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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
            xSemaphoreGiveFromISR(xI2CDone, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            break;

        default:
            g_eState = I2C_STATE_IDLE;
            break;
    }
}

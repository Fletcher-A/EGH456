/*
 * main.c — Assignment entry point.
 *
 * Responsibilities:
 *   - Configure system clock, UART, I2C, GPIO, ADC, PWM, SPI
 *   - Create RTOS objects (queues, mutexes, event group)
 *   - Spawn tasks: motor, sensors, GUI, fault
 *   - Start the scheduler
 */

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/i2c.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/fpu.h"
#include "drivers/rtos_hw_drivers.h"
#include "utils/uartstdio.h"
#include "shared.h"
/*-----------------------------------------------------------*/

uint32_t g_ui32SysClock;

/* Single-definition site for shared RTOS object handles. */
QueueHandle_t      xMotorQueue    = NULL;
QueueHandle_t      xSensorQueue   = NULL;
QueueHandle_t      xCommandQueue  = NULL;
EventGroupHandle_t xSystemEvents  = NULL;
SemaphoreHandle_t  xUARTMutex     = NULL;
SemaphoreHandle_t  xI2CMutex      = NULL;

/* Task creators (defined in their own .c files). */
extern void vCreateMotorTask(void);
extern void vCreateSensorTask(void);
extern void vCreateGuiTask(void);
extern void vCreateFaultTask(void);

static void prvSetupHardware(void);
static void prvConfigureUART(void);
static void prvConfigureI2C(void);
static void prvConfigureButtons(void);

/*-----------------------------------------------------------*/

int main(void)
{
    prvSetupHardware();

    /* Create RTOS objects. */
    xMotorQueue    = xQueueCreate(8, sizeof(MotorMsgObj));
    xSensorQueue   = xQueueCreate(8, sizeof(SensorMsgObj));
    xCommandQueue  = xQueueCreate(4, sizeof(int32_t));   /* desired RPM commands */
    xSystemEvents  = xEventGroupCreate();
    xUARTMutex     = xSemaphoreCreateMutex();
    xI2CMutex      = xSemaphoreCreateMutex();

    if (!xMotorQueue || !xSensorQueue || !xCommandQueue ||
        !xSystemEvents || !xUARTMutex  || !xI2CMutex)
    {
        for (;;) {}     /* RTOS object creation failed */
    }

    /* Spawn tasks. */
    vCreateSensorTask();
    vCreateMotorTask();
    vCreateGuiTask();
    vCreateFaultTask();

    IntMasterEnable();
    vTaskStartScheduler();

    for (;;) {}
}

/*-----------------------------------------------------------*/

static void prvSetupHardware(void)
{
    /* 120 MHz from 25 MHz crystal via PLL. */
    g_ui32SysClock = MAP_SysCtlClockFreqSet(
        SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |
        SYSCTL_CFG_VCO_480, 120000000);

    /* FPU on for grlib + control math. */
    FPUEnable();
    FPULazyStackingEnable();

    PinoutSet(false, false);

    prvConfigureUART();
    prvConfigureI2C();
    prvConfigureButtons();

    /* TODO: configure ADC channels for motor phase currents
     *       (DRV8323 SOA/SOB pins).
     * TODO: configure GPIO interrupts for hall sensors A/B/C.
     * TODO: configure SPI for the LCD (handled by Kentec driver
     *       inside the GUI task — verify pins don't clash). */
}

/*-----------------------------------------------------------*/

static void prvConfigureUART(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    UARTStdioConfig(0, 9600, 16000000);
}

/*-----------------------------------------------------------*/

static void prvConfigureI2C(void)
{
    /* I2C0: PB2 (SCL) / PB3 (SDA), 100 kHz, used by the OPT3001
       and the optional I2C sensors (SHT31, BMI160, VL53L0X). */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0)) {}
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB)) {}
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
    I2CMasterInitExpClk(I2C0_BASE, g_ui32SysClock, false);
}

/*-----------------------------------------------------------*/

static void prvConfigureButtons(void)
{
    ButtonsInit();
    GPIOIntTypeSet(BUTTONS_GPIO_BASE, ALL_BUTTONS, GPIO_FALLING_EDGE);
    GPIOIntEnable(BUTTONS_GPIO_BASE, ALL_BUTTONS);
    IntEnable(INT_GPIOJ);
}

/*-----------------------------------------------------------*/
/* Application-level RTOS hooks. */

void vApplicationMallocFailedHook(void)
{
    IntMasterDisable();
    for (;;) {}
}

void vApplicationIdleHook(void)  {}
void vApplicationTickHook(void)  {}

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    (void)pxTask; (void)pcTaskName;
    IntMasterDisable();
    for (;;) {}
}

void *malloc(size_t xSize)
{
    (void)xSize;
    IntMasterDisable();
    for (;;) {}
}

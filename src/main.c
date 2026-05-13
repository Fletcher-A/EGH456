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
QueueHandle_t      xPowerRawQueue = NULL;
QueueHandle_t      xAccelRawQueue = NULL;

/* Runtime-adjustable safety thresholds. Initialised to the defaults
 * from shared.h, mutated by the GUI Thresholds tab, read by the
 * sensor task on each cycle. */
volatile float g_thresh_power_w      = DEFAULT_POWER_LIMIT_W;
volatile float g_thresh_accel_g      = DEFAULT_ACCEL_LIMIT_G;
volatile float g_thresh_distance_mm  = DEFAULT_DISTANCE_LIMIT_MM;
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

/*-----------------------------------------------------------*/

int main(void)
{
    prvSetupHardware();

    /* Create RTOS objects. */
    xMotorQueue    = xQueueCreate(8, sizeof(MotorMsgObj));
    xSensorQueue   = xQueueCreate(8, sizeof(SensorMsgObj));
    xCommandQueue  = xQueueCreate(4, sizeof(int32_t));   /* desired RPM commands */
    xPowerRawQueue = xQueueCreate(64, sizeof(PowerSampleRaw_t)); /* ADC ISR -> sensor */
    xAccelRawQueue = xQueueCreate(32, sizeof(AccelSampleRaw_t)); /* Timer1A -> sensor */
    xSystemEvents  = xEventGroupCreate();
    xUARTMutex     = xSemaphoreCreateMutex();
    xI2CMutex      = xSemaphoreCreateMutex();

    if (!xMotorQueue || !xSensorQueue || !xCommandQueue ||
        !xPowerRawQueue || !xAccelRawQueue ||
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
    /* I2C0 is set up by the sensor task (via initI2C in i2cOptDriver).
     * LCD/touch SPI is set up by the GUI task (Kentec...Init,
     * TouchScreenInit). LaunchPad buttons aren't used — input is via
     * the touchscreen. */
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
    /* 115200 baud — high enough for a 50 Hz CSV plot stream. */
    UARTStdioConfig(0, 115200, 16000000);
}

/*-----------------------------------------------------------*/

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

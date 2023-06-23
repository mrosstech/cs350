/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART2.h>
#include <ti/drivers/Timer.h>


/* Driver configuration */
#include "ti_drivers_config.h"


// Driver Handles - Global variables
Timer_Handle timer0;


// Task struct definition
typedef struct task {
   int state;                                       // Task's current state
   unsigned long period;                            // Task period
   unsigned long elapsedTime;                       // Time elapsed since last task tick
   int (*TickFct)(int);                             // Task tick function
} task;

// State enums
enum ButtonStates {BS_Init, NothingPressed, RaisePressed, LowerPressed} Button_state;
enum TempStates {TS_Init, TempLower, TempHigher} Temp_state;
enum ReportStates {RS_Init, Reporting} Report_state;

// Globals for temperature
int16_t temperature = 0;                            // Used to track temp when thermostat is running
int16_t setpoint = 40;                              // Used to track the current setpoint while the thermostat is running
char heat = 0;                                      // Tracks whether the heat is on or off 0 = off; 1 = on
char takeTemp = 1;                                  // Tracks whether to take the temperature in the main loop.

// Globals for button presses
char tempUpPressed = 0;
char tempDnPressed = 0;


// Constants for tasks:
const unsigned char tasksNum = 3;                   // Three total tasks to track
const unsigned long tasksPeriodGCD = 100;           // 100ms is the common denominator for the tasks
const unsigned long periodCheckButtons = 200;       // 200ms between button press checks.
const unsigned long periodReadTemp = 500;           // 500ms between temp reads
const unsigned long periodUpdateSystem = 1000;      // 1000ms between reports sent to the UART
unsigned long runningms = 0;                        // How long in ms has the system been running?
char showReport = 0;                                // Tracks whether to show the report on the UART.


// UART Global Variables
char output[64];                                    // Max output size for the report is 64 characters
size_t global_bytesWritten = 0;                     // Bytes written global variable for the UART2_write method.
UART2_Handle uart;                                  // Handle for the uart interface


// I2C Global Variables
static const struct {                               // Struct to hold addresses and registers of
    uint8_t address;                                // the various temp devices which can be present
    uint8_t resultReg;                              // on the CC3220S boards.
    char *id;
} sensors[3] = {
    { 0x48, 0x0000, "11X" },
    { 0x49, 0x0000, "116" },
    { 0x41, 0x0001, "006" }
};
uint8_t txBuffer[1];                                // Transmit buffer for i2c transactions
uint8_t rxBuffer[2];                                // Receive buffer for i2c transactions
I2C_Transaction i2cTransaction;                     // Transaction object for i2c transactions

// Driver Handles - Global variables
I2C_Handle i2c;                                     // i2c handle for i2c operations.


// Initialize the tasks array
task tasks[tasksNum];                               // Tasks array holds all of the tasks.


// Display helper function to display text to the UART
#define DISPLAY(...)  do { char _buf[384]; snprintf(_buf, sizeof _buf, __VA_ARGS__); UART2_write(uart, (void *)_buf, strlen(_buf), &global_bytesWritten); } while (0);

// Callback function for the timer.  Tracks the runtime of the application
// and rolls through the tasks, executing their handler functions when the
// appropriate period is reached.
void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{
    // Increment the total timer
    runningms = runningms + tasksPeriodGCD;
    // Implement a task scheduler
    unsigned char i;
    // Loop through the various tasks
    for (i = 0; i < tasksNum; i++) {
        // If the elapsed time has crossed the threshold for the task period.
        if ( tasks[i].elapsedTime >= tasks[i].period) {
            tasks[i].state = tasks[i].TickFct(tasks[i].state);
            tasks[i].elapsedTime = 0;
        }
        tasks[i].elapsedTime += tasksPeriodGCD;
    }
}


// Function to initialize the timer.
void initTimer(void)
{
    Timer_Params params;
    // Init the driver
    Timer_init();
    // Configure the driver
    Timer_Params_init(&params);
    params.period = 100000;                             // Set period to 100ms to support all function needs. LCD
    params.periodUnits = Timer_PERIOD_US;               // Period is measured in microseconds
    params.timerMode = Timer_CONTINUOUS_CALLBACK;       // Timer will call the callback function whenever the specified period is reached.
    params.timerCallback = timerCallback;               // Name of the callback function.
    // Open the driver
    timer0 = Timer_open(CONFIG_TIMER_0, &params);
    if (timer0 == NULL) {
        /* Failed to initialized timer */
        while (1) {}
    }
    if (Timer_start(timer0) == Timer_STATUS_ERROR) {
        /* Failed to start timer */
        DISPLAY("Timer open failed!");
        while (1) {}
    }
    DISPLAY("Timer Initialized!\n\r");
}


// Function to initialize the UART2 interface
void initUART(void)
{
    UART2_Params uartParams;
    // Configure the driver
    UART2_Params_init(&uartParams);
    uartParams.baudRate = 115200;                           // Set the baud rate to 115200 (max)
    // Open the driver
    uart = UART2_open(CONFIG_UART2_0, &uartParams);
    if (uart == NULL) {
        /* UART_open() failed */
        DISPLAY("UART open failed!");
        while (1);
    }
}




// Function for initializing the i2c system.
void initI2C(void)
{
    int8_t i, found;
    I2C_Params i2cParams;
    DISPLAY("Initializing I2C Driver - ");
    // Init the driver
    I2C_init();
    // Configure the driver
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    // Open the driver
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2c == NULL)
    {
        DISPLAY("Failed\n\r");
        while (1) {};
    }
    DISPLAY("Passed\n\r");
    // Boards were shipped with different sensors.
    // Welcome to the world of embedded systems.
    // Try to determine which sensor we have.
    // Scan through the possible sensor addresses
    /* Common I2C transaction setup */
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;
    found = false;
    for (i=0; i<3; ++i)
    {
        i2cTransaction.targetAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;
        DISPLAY("Is this %s? ", sensors[i].id);
        if (I2C_transfer(i2c, &i2cTransaction))
        {
            DISPLAY("Found\n\r");
            found = true;
            break;
        }
        DISPLAY("No\n\r");
    }
    if(found)
    {
        DISPLAY("Detected TMP%s I2C address:%x\n\r", sensors[i].id, i2cTransaction.targetAddress);
    }
    else
    {
        DISPLAY("Temperature sensor not found, contact professor\n\r");
    }
}
int16_t readTemp(void)
{
    int j;
    int16_t temperature1 = 0;
    i2cTransaction.readCount = 2;
    if (I2C_transfer(i2c, &i2cTransaction))                         // Initiate transfer of data from I2C device (temp sensor)
    {
        /*
        * Extract degrees C from the received data;
        * see TMP sensor datasheet
        */
        temperature1 = (rxBuffer[0] << 8) | (rxBuffer[1]);          // Read the the temperature data from the temp sensor
        temperature1 *= 0.0078125;                                  // Convert the raw data to usable Celsius
        /*
        * If the MSB is set '1', then we have a 2's complement
        * negative value which needs to be sign extended
        */
        if (rxBuffer[0] & 0x80)
        {
            temperature1 |= 0xF000;
        }
    }
    else
    {
        DISPLAY("Error reading temperature sensor (%d)\n\r", i2cTransaction.status);
        DISPLAY("Please power cycle your board by unplugging USB and plugging back in.\n\r");
    }
    return temperature1;
}









/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn0(uint_least8_t index)
{
    tempDnPressed = 1;                                              // When the button is pressed, set the flag for decrementing the set point temp
}

/*
 *  ======== gpioButtonFxn1 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_1.
 *  This may not be used for all boards.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn1(uint_least8_t index)
{
    tempUpPressed = 1;                                              // When the button is pressed, set the flag for incrementing the set point temp

}





// Tick functions
// ## Tick function for the button state determines
// ## whether the button has been pressed or not.
// ## When it is pressed, it decrements or increments the setpoint
// ## global variable.
int TickFct_Buttons(int state) {
    switch (state) {
        case BS_Init:
            state = NothingPressed;
            break;
        case NothingPressed:
            if (tempUpPressed == 1) {
                state = RaisePressed;
            }
            if (tempDnPressed == 1) {
                state = LowerPressed;
            }
            break;
    }
    switch (state) {
        case RaisePressed:
            setpoint += 1;
            state = NothingPressed;
            tempUpPressed = 0;
            break;
        case LowerPressed:
            setpoint -= 1;
            state = NothingPressed;
            tempDnPressed = 0;
            break;
    }
    return state;
}

// ## Tick function for the temp state determines
// ## whether the temp is lower or higher than the
// ## setpoint.  If the temp is lower then the heat
// ## flag is set to on.  It is set to off if the temp
// ## is higher than the setpoint.  The heat LED is
// ## also set accordingly.
int TickFct_Temp(int state) {
    switch (state) {
        case TS_Init:
            if (temperature < setpoint) {
                state = TempLower;
            }
            if (temperature > setpoint) {
                state = TempHigher;
            }
            break;
        case TempLower:
            if (temperature > setpoint) {
                state = TempHigher;
            }
            if (temperature < setpoint) {
                state = TempLower;
            }
        case TempHigher:
            if (temperature > setpoint) {
                state = TempHigher;
            }
            if (temperature < setpoint) {
                state = TempLower;
            }
    }
    switch (state) {
        case TempLower:
            // turn on heating light
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
            heat = 1;
            break;
        case TempHigher:
            // Turn off heating light
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            heat = 0;
            break;
    }
    return state;
}

// ## Tick function for the reporting state determines
// ## whether the report should be shown.
// ## Currently it is called every time the timer
// ## threshhold is reached, but it could be easily
// ## modified to allow the reporting function to be
// ## turned off via a flag.

int TickFct_Report(int state) {
    switch(state) {
    case RS_Init:
        state = Reporting;
        break;
    case Reporting:
        break;
    }
    switch(state) {
        case Reporting:
            showReport = 1;
    }
    return state;
}




/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0) {
    /* Call driver init functions */
    GPIO_init();
    initUART();
    initI2C();


    /* Configure the LED pin */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

    /* Configure BUTTON0 pin */
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* Configure BUTTON1 pin */
    GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    /* Allow the sensor to power on */

    /* Install Button callbacks */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);
    GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);
    GPIO_enableInt(CONFIG_GPIO_BUTTON_1);

    // Configure our tasks:
    unsigned char i = 0;
    // Button Press Task
    tasks[i].state = BS_Init;
    tasks[i].period = periodCheckButtons;
    tasks[i].elapsedTime = tasks[i].period; // This allows the task to kick off immediately.
    tasks[i].TickFct = &TickFct_Buttons;
    i++;
    // Read Temp Task
    tasks[i].state = TS_Init;
    tasks[i].period = periodReadTemp;
    tasks[i].elapsedTime = tasks[i].period; // This allows the task to kick off immediately.
    tasks[i].TickFct = &TickFct_Temp;
    i++;
    // Reporting Task
    tasks[i].state = RS_Init;
    tasks[i].period = periodUpdateSystem;
    tasks[i].elapsedTime = tasks[i].period; // This allows the task to kick off immediately.
    tasks[i].TickFct = &TickFct_Report;

    // Finally, kick off the timer.  Kicking it off here ensures that all of our initialization
    // activity has been completed.
    initTimer();

    // Loop forever
    while (1) {
        // Every 200ms check button flags
        // every 500ms read the temperature and update the LED
        // Every second output the following to the UART
        // "<%02d,%02d,%d,%04d>, temperature, setpoint, heat, seconds"
        if (takeTemp == 1) {                                                                    // If the takeTemp flag has been set, take the temperature
            temperature = readTemp();                                                           // we want to take it in the main loop to avoid blocking issues
            takeTemp = 0;                                                                       // when dealing with interrupts.
        }
        if (showReport == 1) {                                                                  // If the showReport flag is set then display the temp data to the UART.
            DISPLAY("<%02d,%02d,%d,%04d>\n\r", temperature, setpoint, heat, runningms/1000);
            showReport = 0;                                                                     // Set the flag back to 0
        }

    }
    return (NULL);
}

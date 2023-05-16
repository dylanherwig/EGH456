//*****************************************************************************
//
// hello.c - Simple hello world example.
//
// Copyright (c) 2013-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement testing CCS git implementaiton
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.4.178 of the EK-TM4C1294XL Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "drivers/pinout.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "drivers/MotorLib/motorlib.h"

//*****************************************************************************
//
// Global Defines
//
//*****************************************************************************
#define CRASH_ACCELERATION //test this figure?
#define CRASH_DISTANCE //what is this figure?

//*****************************************************************************
//GUID branch
//*****************************************************************************
//
// System clock rate in Hz.
//
//*****************************************************************************
uint32_t g_ui32SysClock;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void
ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, g_ui32SysClock);
}
//*****************************************************************************
//
// Motor Control
//
//*****************************************************************************
#define MOTOR_ACCELERATION_RPMs = 750 //Note that this is measured in RPM per Second
#define MOTOR_ESTOP_RPMs = 1000 //Note that this is measured in RPM per Second
#define MOTOR_CURR_MAX //we need to decide what this is
#define MOTOR_TEMP_MAX //we need to decide what this is
//
// E-Stop Thread / Interupt
//
void motor_eStop()
{
    // This Needs to check the following:
    // 1. Check Current isn't above MOTOR_CURR_MAX
    // 2. Motor isn't above MOTOR_TEMP_MAX
    // 3. get message from acceleration sensor, and check that the acceleration isn't above CRASH_ACCELERATION
    // 4. get message from distance sensor, and check that there isn't an object within CRASH_DISTANCE
    //
    // If any of those are true, it needs to call void stopMotor(bool brakeType);


}

//
// Motor Set RPM - Will likely be called by an interupt from the UI
//
void motor_SetRPM(int rpm)
{
    //This needs to:
    // Set motor RPM using void setDuty(uint16_t duty);
    // Will likely need to be setup in a closed loop with motor_GetRPM()
    // Will likely need to call motor_accelerate() & motor_decelerate
}

//
// Motor Get RPM
//
int motor_GetRPM()
{
    //This needs to:
    // Get motor RPM using void updateMotor(bool Hall_a, bool Hall_b, bool Hall_c);
}

//
// Accelerate the Motor
//
void motor_accelerate(bool direction)
{
    // Accelerate the motor at MOTOR_ACCELERATION_RPMs
    //  use the direction to establish if its acceleration or deceleration?
}

//*****************************************************************************
//
// Main
//
//*****************************************************************************
int
main(void)
{
    //
    // Run from the PLL at 120 MHz.
    //
    g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |
                SYSCTL_CFG_VCO_480), 120000000);

    //
    // Configure the device pins.
    //
    PinoutSet(false, false);

    //
    // Enable the GPIO pins for the LED D1 (PN1).
    //
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_1);

    //
    // Initialize the UART.
    //
    ConfigureUART();

    //
    // Init Motor
    //
    Error_Block motorError;
    uint16_t pwm_period = getMotorPWMPeriod();
    initMotorLib(pwm_period, motorError);

    //
    // Hello!
    //
    UARTprintf("Hello, world!\n");

    //
    // We are finished.  Hang around flashing D1.
    //
    while(1)
    {
        //
        // Turn on D1.
        //
        LEDWrite(CLP_D1, 1);

        //
        // Delay for a bit.
        //
        SysCtlDelay(g_ui32SysClock / 10 / 3);

        //
        // Turn off D1.
        //
        LEDWrite(CLP_D1, 0);

        //
        // Delay for a bit.
        //
        SysCtlDelay(g_ui32SysClock / 10 / 3);
    }
}

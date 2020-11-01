/*
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
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

/* XDC module Headers */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Log.h>				//needed for any Log_info() call
#include <xdc/cfg/global.h> 				//header file for statically defined objects/handles
#include <xdc/runtime/Diags.h>

/* BIOS module Headers */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/drivers/GPIO.h>

/* Include header files for adc and GPIO functions */
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/debug.h"
#include "driverlib/pin_map.h"
#include "driverlib/adc.h"
#include "driverlib/rom.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include <time.h>
#include <inc/hw_gpio.h>
#include "driverlib/uart.h"
#include <ti/drivers/ADC.h>
#include <ti/display/Display.h>

volatile int16_t i16ToggleCount1 = 0;
volatile int16_t i16ToggleCount2 = 0;
#define TASKSTACKSIZE   512

Task_Struct task0Struct;
Char task0Stack[TASKSTACKSIZE];

/* ADC sample count */
#define ADC_SAMPLE_COUNT  (10)

#define THREADSTACKSIZE   (768)

/* ADC conversion result variables */
uint16_t adcValue0;
uint32_t adcValue0MicroVolt;
uint16_t adcValue1[ADC_SAMPLE_COUNT];
uint32_t adcValue1MicroVolt[ADC_SAMPLE_COUNT];

//---------------------------------------------------------------------------
void delay_simple(void)
{
     SysCtlDelay(6700000);      // creates ~500ms delay - TivaWare fxn

}

/*
 *  ======== heartBeatFxn ========
 *  Toggle the Board_LED0. The Task_sleep is determined by arg0 which
 *  is configured for the heartBeat Task instance.
 */
void heartBeatFxn(UArg arg0, UArg arg1){
    while(1){
        Task_sleep((UInt)arg0);
        GPIO_toggle(Board_LED0);
    }
}

void init_ADC()
{
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);
    SysCtlDelay(80u);

    // Use ADC0 sequence 0 to sample channel 0 once for each timer period
    ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PIOSC | ADC_CLOCK_RATE_HALF, 1);

    SysCtlDelay(10); // Time for the clock configuration to set

    IntDisable(INT_ADC0SS0);
    ADCIntDisable(ADC0_BASE, 0u);
    ADCSequenceDisable(ADC0_BASE,0u);
    // With sequence disabled, it is now safe to load the new configuration parameters

    ADCSequenceConfigure(ADC0_BASE, 0u, ADC_TRIGGER_TIMER, 0u);
    ADCSequenceStepConfigure(ADC0_BASE,0u,0u,ADC_CTL_CH0| ADC_CTL_END | ADC_CTL_IE);
    ADCSequenceEnable(ADC0_BASE,0u); //Once configuration is set, re-enable the sequencer
    ADCIntClear(ADC0_BASE,0u);
    ADCSequenceDMAEnable(ADC0_BASE,0);
    IntEnable(INT_ADC0SS0);

}
void taskFxn1(void)
{
    while(1)
    {
    Semaphore_pend(task1sem, BIOS_WAIT_FOREVER);
    // LED values - 2=RED, 4=BLUE, 8=GREEN
    if(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_2))
    {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);
    }
    else
    {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 4);
    }

  //  delay_simple();                                // create a delay of ~1/2sec

    i16ToggleCount1 += 1;                    // keep track of #toggles

    Log_info1("LED2 TOGGLED [%u] times", i16ToggleCount1);    // send #toggles to Log Display
    //System_printf("Count: %d\n", i16ToggleCount);
    //System_flush();
    }
}


void taskFxn2(void)
{
    while(1)
    {
    Semaphore_pend(task2sem, BIOS_WAIT_FOREVER);
    // LED values - 2=RED, 4=BLUE, 8=GREEN
    if(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_3))
    {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);
    }
    else
    {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 8);
    }

  //  delay_simple();                                // create a delay of ~1/2sec

    i16ToggleCount2 += 1;                    // keep track of #toggles

    Log_info1("LED3 TOGGLED [%u] times", i16ToggleCount2);    // send #toggles to Log Display
    //System_printf("Count: %d\n", i16ToggleCount);
    //System_flush();
    }
}



//Timer 2 setup
void timer2Init()
{
    uint32_t ui32Period;
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);			// enable Timer 2 periph clks
    TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);		// cfg Timer 2 mode - periodic

    ui32Period = (SysCtlClockGet()/500) / 2;
    TimerLoadSet(TIMER2_BASE, TIMER_A, ui32Period-1);			// set Timer 2 period

    TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);		// enables Timer 2 to interrupt CPU

    TimerEnable(TIMER2_BASE, TIMER_A);						// enable Timer 2

}


volatile uint32_t tickCount=0;

/*
 *  ======== main ========
 */
int main(){
    Task_Params taskParams;

    /* Set up the System Clock */
    SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);

    /* Enable all the peripherals */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    /* Construct heartBeat Task  thread */
       Task_Params_init(&taskParams);
       taskParams.arg0 = 1000;
       taskParams.stackSize = TASKSTACKSIZE;
       taskParams.stack = &task0Stack;
       Task_construct(&task0Struct, (Task_FuncPtr)heartBeatFxn, &taskParams, NULL);

    /* Unlock pin PF0 */
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK)= GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK)= 0;

    /* Configure Enable pin as output */
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);

    timer2Init();
    BIOS_start();    /* Does not return */

    if (adc == NULL) {
            Display_printf(hSerial, 1, 0, "Error initializing CONFIG_ADC_0\n");
            while (1);
        }
        /* Configure the LED pin */
        GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_HIGH);

        /* Turn on user LED */
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
        /* Blocking mode conversion */

    if (adc == NULL) {
         Display_printf(hSerial, 1, 0, "Error initializing CONFIG_ADC_0\n");
         while (1);
     }
     /* Configure the LED pin */
     GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_HIGH);

     /* Turn on user LED */
     GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
     /* Blocking mode conversion */
     while(1)
     {
     res = ADC_convert(adc, &adcValue0);
     if (res == ADC_STATUS_SUCCESS) {
         Display_printf(hSerial, 1, 0, "CONFIG_ADC_0 raw result: %d\n", adcValue0);
         if(adcValue0 > 1000)
         {
             GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
         }
         else{
             GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
         }
     }
     else {
         Display_printf(hSerial, 1, 0, "CONFIG_ADC_0 convert failed\n");
       }
     usleep(500000);
     }
     return (NULL);
}


//---------------------------------------------------------------------------
// Timer ISR  to be called by BIOS Hwi
//
// Posts Semaphore for releasing tasks
//---------------------------------------------------------------------------
void Timer_ISR(void)
{
    TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);			// must clear timer flag FROM timer
    tickCount++;  //tickCount is incremented every 2 ms.

    if(tickCount == 300)
    {
        Semaphore_post(task1sem);
    }
    else if(tickCount == 600)
    {
        Semaphore_post(task2sem);
        tickCount = 0;
    }
}




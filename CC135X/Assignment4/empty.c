/*
 * Copyright (c) 2015-2019, Texas Instruments Incorporated
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
 *  ======== empty.c ========
 */

/* XDC module Headers */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Diags.h>

/* Include header files for adc and GPIO functions */
#include <stdint.h>
#include <stdbool.h>

#include <time.h>

/* For usleep() */
#include <unistd.h>
#include <stdint.h>
#include <stddef.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/PWM.h>

/* Driver Header files */
#include <ti/drivers/ADC.h>
#include <ti/display/Display.h>

/* Driver configuration */
#include "ti_drivers_config.h"

/* ADC conversion result variables */
uint16_t adcValue0;
static Display_Handle display;
ADC_Handle   adc;

//heartbeat task
void heartBeatFxn(UArg arg0, UArg arg1){
    while(1){
        Task_sleep((UInt)arg0);
        GPIO_toggle(CONFIG_GPIO_LED_0);
    }
}

//adc task
void adc_task(){
    PWM_Params params;
    adc = ADC_open(CONFIG_ADC_0, &params);

    if (adc == NULL) {
        Display_printf(display, 0, 0, "Error initializing CONFIG_ADC_0\n");
        while (1);
       }
}

//display task
void display_task(){
    uint32_t time = 1000000;
    uint32_t count = 0;
    int_fast16_t res;

    display = Display_open(Display_Type_UART, NULL);
       if (display == NULL) {
           /* Failed to open display driver */
           while (1);
       }
    while (1) {
        GPIO_toggle(CONFIG_GPIO_LED_0);
        Display_printf(display, 1, 0, "LED Toggle %d", count++);
        res = ADC_convert(adc, &adcValue0);
        if (res == ADC_STATUS_SUCCESS) {
                if(adcValue0 >= 1500){
                    GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
                }
                else{
                    GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
                }
             }
        Display_printf(display, 1, 0, "ADC Value %d", adcValue0);
        usleep(time);
    }
}

//PMW task
void PMW_task(){
    /* Period and duty in microseconds */
    uint32_t   pwmPeriod = 3000;
    uint32_t   duty = 0;
    uint32_t   dutyInc = 100;
    PWM_Params params0;
    PWM_Handle pwm1 = NULL;

    PWM_init();
    PWM_Params_init(&params0);
    params0.dutyUnits = PWM_DUTY_US;
    params0.dutyValue = 0;
    params0.periodUnits = PWM_PERIOD_US;
    params0.periodValue = pwmPeriod;
    pwm1 = PWM_open(CONFIG_PWM_0, &params0);

    if (pwm1 == NULL) {
        /* CONFIG_PWM_0 did not open */
        while (1);
    }
    PWM_start(pwm1);
    /* Loop forever incrementing the PWM duty */
    while (1) {
        PWM_setDuty(pwm1, duty);
        duty = (duty + dutyInc);
        if (duty == pwmPeriod || (!duty)) {
            dutyInc = - dutyInc;
        }
        usleep(time);
    }
}

void *mainThread(void *arg0){
    /* Call driver init functions */
    GPIO_init();
    ADC_init();
    adc_task();
    Display_init();

    /* Configure the LED pin */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

    /* Turn on user LED */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);

    display_task();

    PMW_task();
}

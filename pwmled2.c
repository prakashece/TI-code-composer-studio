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

#include <unistd.h>
#include <stdint.h>
#include <stddef.h>
#include <ti/drivers/GPIO.h>
#include<ti/drivers/Timer/GPTimerCC26XX.h>
#include <ti/drivers/PWM.h>
#include <xdc/runtime/System.h>
/* Board Header file */
#include "Board.h"
/*
PIN_Config pwmPinConfig[] =
{
    PIN_ID(CC2640R2_LAUNCHXL_PWM0)| PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_ID(GPT_PIN_1A) | PIN_GPIO_OUTPUT_DIS | PIN_INPUT_EN | PIN_HYSTERESIS,
  //  PIN_ID(TIMERPIN2) | PIN_GPIO_OUTPUT_DIS | PIN_INPUT_EN | PIN_HYSTERESIS,
    PIN_TERMINATE
};
#define PULSE_DELAY_EVT                       0x0080
*/

PWM_Handle hPWM=NULL;
PWM_Handle PWM2=NULL;
PIN_State gpTimerPinOutState;
PIN_Handle gpTimerPinOutHandle;
PIN_State gpTimerPinInState;
PIN_Handle gpTimerPinInHandle;
PWM_Params pwmParams1;
PWM_Params pwmParams2;
int count=0;

uint16_t   duty = 0;
uint16_t   dutyInc = 1000; //uint16_t   dutyInc = 1000;

void gpioButtonFxn0(uint_least8_t index)
{
    // Clear the GPIO interrupt and toggle an LED
    GPIO_toggle(Board_GPIO_LED0);
    duty=duty+dutyInc;
    if(duty>5000)
    {
        duty=0;
    }
    return duty;
}

static void PWMOutCallback(GPTimerCC26XX_Handle handle,GPTimerCC26XX_IntMask interruptMask)
{
if(count==0)
{
PWM2= PWM_open(Board_PWM1, &pwmParams2);
if (PWM2 == NULL)
{
     while (1);
}
}
PWM_start(PWM2);
PWM_setDuty(PWM2, duty);
count+=1;
}
/*
 *  ======== mainThread ========
 */

void *mainThread(void *arg0)
{
    System_printf("hello world\n");
    System_flush();

    GPIO_init();
    GPIO_setConfig(Board_GPIO_BUTTON0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
GPIO_setCallback(Board_GPIO_BUTTON0, gpioButtonFxn0);
GPIO_enableInt(Board_GPIO_BUTTON0);

 //PIN_init(pwmPinConfig);

    PWM_init();
        PWM_Params_init(&pwmParams1);
        pwmParams1.periodUnits =PWM_PERIOD_US;
        pwmParams1.periodValue = 10000;//100Hz;
        pwmParams1.dutyUnits = PWM_DUTY_US; // Fractional duty cycle
        pwmParams1.dutyValue = 2500;
        hPWM = PWM_open(Board_PWM0, &pwmParams1);
               if (hPWM == NULL)
               {
                   while (1);
               }


        PWM_Params_init(&pwmParams2);
        pwmParams2.periodUnits = PWM_PERIOD_US;
        pwmParams2.periodValue = 10000;//100Hz;
        pwmParams2.dutyUnits = PWM_DUTY_US; // Fractional duty cycle
        pwmParams2.dutyValue = 2500;



        GPTimerCC26XX_Params timerOutParams;
            GPTimerCC26XX_Handle hOutTimer;
            GPTimerCC26XX_Params_init(&timerOutParams);
            timerOutParams.mode = GPT_MODE_PERIODIC;
            //timerOutParams.mode=GPT_MODE_PWM;
            timerOutParams.width = GPT_CONFIG_16BIT;
            hOutTimer = GPTimerCC26XX_open(CC2640R2_LAUNCHXL_GPTIMER1A, &timerOutParams);
            if(hOutTimer==NULL)
            {
                while(1);
            }
            GPTimerCC26XX_setLoadValue(hOutTimer, 239999);

            // Register interrupt when capture happens
            GPTimerCC26XX_registerInterrupt(hOutTimer, PWMOutCallback, GPT_INT_TIMEOUT);

            // Open pin handle and route pin to timer
           /* gpTimerPinOutHandle = PIN_open(&gpTimerPinOutState, pwmPinConfig);
            GPTimerCC26XX_PinMux pinOutMux = GPTimerCC26XX_getPinMux(hOutTimer);
            PINCC26XX_setMux(gpTimerPinOutHandle, PIN_ID(GPT_PIN_1A), pinOutMux);

            GPTimerCC26XX_setCaptureEdge(hOutTimer, GPTimerCC26XX_POS_EDGE);
*/
            GPTimerCC26XX_start(hOutTimer);
            PWM_start(hPWM);



    while (1) {
       /* if(GPIO_read(Board_GPIO_BUTTON0)==1)
        {
            GPIO_toggle(Board_GPIO_LED0);
                duty=duty+dutyInc;
                if(duty>10000)
                {
                    duty=0;
                }
        }*/

       PWM_setDuty(hPWM, duty);

    }
}

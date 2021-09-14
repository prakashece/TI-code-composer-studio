
#include <stdint.h>
#include <stddef.h>
#include <unistd.h>
/* POSIX Header files */
#include <pthread.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include<ti/drivers/Timer/GPTimerCC26XX.h>
#include <ti/drivers/ADC.h>
#include <ti/drivers/PWM.h>
#include <ti/display/Display.h>


/* Example/Board Header files */
#include "Board.h"
// PWM configurations
PWM_Handle hPWM=NULL;
PWM_Handle PWM2=NULL;
PIN_State gpTimerPinOutState;
PIN_Handle gpTimerPinOutHandle;
PIN_State gpTimerPinInState;
PIN_Handle gpTimerPinInHandle;
PWM_Params pwmParams1;
PWM_Params pwmParams2;
int count=0;
uint32_t currVal=0;
uint16_t   duty = 0;
uint16_t   dutyInc = 1000;
//void PWM_parameters_init(void);


/* ADC conversion result variables */
int sample;
uint64_t adcValue;
uint32_t volt;
//uint32_t adcValue1MicroVolt[ADC_SAMPLE_COUNT];

static Display_Handle display;

/*
 *  ======== mainThread ========
 */
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

void *mainThread(void *arg0)
{
    /* Call driver init functions */
    GPIO_init();
    ADC_init();
    Display_init();
    /* Open the display for output */
    display = Display_open(Display_Type_UART, NULL);
    if (display == NULL) {
        /* Failed to open display driver */
        while (1);
    }
   // GPIO_write(Board_DIO26_ANALOG,0);
    GPIO_setConfig(Board_DIO26_ANALOG, GPIO_CFG_INPUT);
    GPIO_write(Board_DIO26_ANALOG,0);
    //PWM_parameters_init();
    Display_printf(display, 0, 0, "Starting the adc single channel example\n");
      uint16_t     i;
      ADC_Handle   adc1;
      ADC_Handle   adc2;
      ADC_Params   params;
      int_fast16_t res;

      ADC_Params_init(&params);
      adc1 = ADC_open(Board_ADC0, &params);
    //  adc2 = ADC_open(Board_ADC1, &params);

      if (adc1 == NULL) {
          Display_printf(display, 0, 0, "Error initializing ADC1 and ADC2\n");
          while (1);
      }

      PWM_init();
    Display_printf(display, 0, 0,"inside init\n");

              PWM_Params_init(&pwmParams1);
              pwmParams1.periodUnits =PWM_PERIOD_US;
              pwmParams1.periodValue = 10000;//100Hz;
              pwmParams1.dutyUnits = PWM_DUTY_US; // Fractional duty cycle
              pwmParams1.dutyValue = 0;
              hPWM = PWM_open(Board_PWM0, &pwmParams1);
                     if (hPWM == NULL)
                     {
                         while (1);
                     }


              PWM_Params_init(&pwmParams2);
              pwmParams2.periodUnits = PWM_PERIOD_US;
              pwmParams2.periodValue = 10000;//100Hz;
              pwmParams2.dutyUnits = PWM_DUTY_US; // Fractional duty cycle
              pwmParams2.dutyValue = 0;



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
                  GPTimerCC26XX_start(hOutTimer);
                  PWM_start(hPWM);


    while(1)
    {
        // currVal = GPIO_read(Board_DIO26_ANALOG);
       //  Display_printf(display, 0, 0, "pin-value: %d\n",currVal);
          res = ADC_convert(adc1, &adcValue);
        //  res = ADC_convert(adc2, &volt);
           if (res == ADC_STATUS_SUCCESS)
           // adcValue1MicroVolt[i] = ADC_convertRawToMicroVolts(adc, adcValue1[i]);

           {
            Display_printf(display, 0, 0, "ADC1 raw result: %d\n",adcValue);
        //   Display_printf(display, 0, 0, "ADC2 raw result: %d\n",volt);
           }
          //  Display_printf(display, 0, 0, "ADC1 convert result (%d): %d uV\n", i,adcValue1MicroVolt[i]);
           else
            Display_printf(display, 0, 0, "ADC1 convert failed (%d)\n", i);

           if(adcValue>=0 && adcValue<=20)
           {
               duty= 0;
               Display_printf(display, 0, 0,"duty 0\n");
           }
           if(adcValue>20 && adcValue<=100)
           {
               Display_printf(display, 0, 0,"duty 10\n");
                duty= 1000;
           }
           if(adcValue>100 && adcValue<=800)
           {
                duty= 2000;
                Display_printf(display, 0, 0,"duty 20\n");
           }
           if(adcValue>800 && adcValue<4000)
           {
               duty= 5000;
               Display_printf(display, 0, 0,"duty 20\n");
           }


         PWM_setDuty(hPWM, duty);
      sleep(1);
        }

   // return (NULL);
}

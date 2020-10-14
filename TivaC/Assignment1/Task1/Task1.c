#include <stdint.h>
#include <stdbool.h>
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/adc.h"
#include "utils/uartstdio.c"
#include <string.h>

volatile uint32_t value=0;
volatile uint32_t ui8LED = 0;

void PortFIntHandler(){
  uint32_t status=0;

  status = GPIOIntStatus(GPIO_PORTF_BASE,true);
  GPIOIntClear(GPIO_PORTF_BASE,status);

  if(status & GPIO_INT_PIN_4 == GPIO_INT_PIN_4){
    //Then there was a Button pin interrupt
    uint8_t value=0;

    value= GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_4);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
    if( value==0)
        ui8LED ^= GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;

    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, ui8LED);
  }
}

int main(void){
    //Set the clock to 80Mhz
    SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
    //Enable GPIO for UART0 pins.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //Enable UART0 to configure the clock.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //Use the internal 16MHz oscillator as the UART clock source.
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //Initialize the UART
    UARTStdioConfig(0, 115200, 16000000);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlDelay(3);


    //Configure GPIO_PIN_4
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);
    GPIOPadConfigSet(GPIO_PORTF_BASE,GPIO_PIN_4 ,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
    GPIOIntTypeSet(GPIO_PORTF_BASE,GPIO_PIN_4,GPIO_FALLING_EDGE);
    GPIOIntRegister(GPIO_PORTF_BASE,PortFIntHandler);
    GPIOIntEnable(GPIO_PORTF_BASE, GPIO_INT_PIN_4);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);

    uint32_t ADCValues[1];

    //These variables are used to store the temperature conversions for Celsius and Fahrenheit.
    uint32_t TempValueC ;
    uint32_t TempValueF ;

    //The ADC0 peripheral enabled
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlDelay(3);
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_TS | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 3);

    //Clear the interrupt
    ADCIntClear(ADC0_BASE, 3);

    //Display the temperature Continuously to the terminal
    while(1){
         ADCProcessorTrigger(ADC0_BASE, 3);

         //Wait for conversion to be completed
         while(!ADCIntStatus(ADC0_BASE, 3, false)){}

         ADCIntClear(ADC0_BASE, 3);

         //Read ADC Value
         ADCSequenceDataGet(ADC0_BASE, 3, ADCValues);
         TempValueC = (uint32_t)(147.5 - ((75.0*3.3 *(float)ADCValues[0])) / 4096.0);

         //Get Fahrenheit value
         TempValueF = ((TempValueC * 9) + 160) / 5;
         // Display the temperature to terminal
         UARTprintf("Temperature = %3d*C or %3d*F\r", TempValueC, TempValueF);

         //5 seconds delay
         SysCtlDelay(80000000 / 6);

         value= GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_4);
              if( value==0){
                  ui8LED^=GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
                  GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, ui8LED);
              }
    }
}

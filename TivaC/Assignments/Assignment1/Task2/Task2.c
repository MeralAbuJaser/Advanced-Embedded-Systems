#include <stdint.h>
#include <stdbool.h>
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"
#include "driverlib/adc.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_types.h"

void UARTPrintf(const char *str)
{
    while(*str)
        UARTCharPut(UART0_BASE ,*str++);
}

char* atoi(int value, char *buffer)
{
    char buff[10];
    int i = 0, j;
    do {
        buff[i++] = value % 10;
        value /= 10;
    }while(value);

    for(j = 0; j < i; j++)
        buffer[j] = buff[i-j-1]|0x30;
    buffer[j] = 0;
    return buffer;
}

int main(void)
{
    char cmd, buffer[16];
    uint32_t ui32ADC0Value[4];
    volatile uint32_t ui32TempAvg;
    volatile uint32_t ui32TempValueC;
    volatile uint32_t ui32TempValueF;

    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_TS);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_TS);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_TS);
    ADCSequenceStepConfigure(ADC0_BASE,1,3,ADC_CTL_TS|ADC_CTL_IE|ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 1);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0|GPIO_PIN_1);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
    (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    while(true)
    {
        UARTPrintf("Enter the Cmd : ");
        while(!UARTCharsAvail(UART0_BASE));
        {
            cmd = UARTCharGet(UART0_BASE);
            UARTCharPut(UART0_BASE,cmd); UARTCharPut(UART0_BASE,'\n'); UARTCharPut(UART0_BASE,'\r');
            switch(cmd)
            {
            case 'R' : { GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1); break; }
            case 'G' : { GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3); break; }
            case 'B' : { GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2); break; }
            case 'r' : { GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0); break; }
            case 'g' : { GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0); break; }
            case 'b' : { GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_2 , 0); break; }
            case 'T' : { ADCIntClear(ADC0_BASE, 1);
                         ADCProcessorTrigger(ADC0_BASE, 1);
                         while(!ADCIntStatus(ADC0_BASE, 1, false));
                         ADCSequenceDataGet(ADC0_BASE, 1, ui32ADC0Value);
                         ui32TempAvg = (ui32ADC0Value[0] + ui32ADC0Value[1] + ui32ADC0Value[2] + ui32ADC0Value[3] + 2)/4;
                         ui32TempValueC = (1475 - ((2475 * ui32TempAvg)) / 4096)/10;
                         ui32TempValueF = ((ui32TempValueC * 9) + 160) / 5;
                         atoi(ui32TempValueC, buffer);
                         UARTPrintf("Temperature:");
                         UARTPrintf(buffer);UARTCharPut(UART0_BASE,'\n'); UARTCharPut(UART0_BASE,'\r');
                         break;
                       }
            case 't' : { ADCIntClear(ADC0_BASE, 1);
                         ADCProcessorTrigger(ADC0_BASE, 1);
                         while(!ADCIntStatus(ADC0_BASE, 1, false));
                         ADCSequenceDataGet(ADC0_BASE, 1, ui32ADC0Value);
                         ui32TempAvg = (ui32ADC0Value[0] + ui32ADC0Value[1] + ui32ADC0Value[2] + ui32ADC0Value[3] + 2)/4;
                         ui32TempValueC = (1475 - ((2475 * ui32TempAvg)) / 4096)/10;
                         ui32TempValueF = ((ui32TempValueC * 9) + 160) / 5;
                         atoi(ui32TempValueF, buffer);
                         UARTPrintf("Temperature:");
                         UARTPrintf(buffer);UARTCharPut(UART0_BASE,'\n'); UARTCharPut(UART0_BASE,'\r');
                         break;
                       }
            case 'S' : { if(!(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1)) && !(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_2)) && !(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_3))){
                             UARTPrintf("RGB is turned off");UARTCharPut(UART0_BASE,'\n'); UARTCharPut(UART0_BASE,'\r');
                         }
                         if((GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1))){
                             UARTPrintf("Red LED is turned on");
                             UARTCharPut(UART0_BASE,'\n'); UARTCharPut(UART0_BASE,'\r');
                         }
                         if((GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_2))){
                             UARTPrintf("Blue LED is turned on");
                             UARTCharPut(UART0_BASE,'\n'); UARTCharPut(UART0_BASE,'\r');
                         }
                         if((GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_3))){
                             UARTPrintf("Green LED is turned on");
                             UARTCharPut(UART0_BASE,'\n'); UARTCharPut(UART0_BASE,'\r');
                         }
                        break;
                        }
                   }
            }
        }
    return 0;
}

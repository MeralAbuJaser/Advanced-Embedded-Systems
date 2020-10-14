#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_adc.h"
#include "inc/hw_types.h"
#include "inc/hw_udma.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/adc.h"
#include "driverlib/udma.h"
#include "driverlib/timer.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/systick.h"
#include "utils/uartstdio.h"

#define ADC_SAMPLE_BUF_SIZE     64

enum BUFFERSTATUS
                  { EMPTY,
                    FILLING,
                    FULL
                  };

#pragma DATA_ALIGN(ucControlTable, 1024)
uint8_t ucControlTable[1024];

static uint16_t ADC_Out1[ADC_SAMPLE_BUF_SIZE];
static uint16_t ADC_Out2[ADC_SAMPLE_BUF_SIZE];
static enum BUFFERSTATUS BufferStatus[2];

void ConfigureUART(void);
void init_ADC(void);
void init_TIMER(void);
void init_DMA(void);
static uint32_t g_ui32DMAErrCount = 0u;
static uint32_t g_ui32SysTickCount;

void UARTPrintf(const char *str){
    while(*str)
        UARTCharPut(UART0_BASE ,*str++);
}

char* atoi(int value, char *buffer){
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

void ADCseq0Handler(){
    ADCIntClear(ADC0_BASE, 0);

    if ((uDMAChannelModeGet(UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT) == UDMA_MODE_STOP)
            && (BufferStatus[0] == FILLING)){
        BufferStatus[0] = FULL;
        BufferStatus[1] = FILLING;
    }
    else if ((uDMAChannelModeGet(UDMA_CHANNEL_ADC0 | UDMA_ALT_SELECT) == UDMA_MODE_STOP)
            && (BufferStatus[1] == FILLING)){
        BufferStatus[0] = FILLING;
        BufferStatus[1] = FULL;
    }
}

int main(void){
    // Set the system clock to run at 80MHz from the PLL.
    SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
    SysCtlDelay(20u);

    char cmd, buffer[16];
    uint32_t ui32ADC0Value[4];
    volatile uint32_t ui32TempAvg;
    volatile uint32_t ui32TempValueC;
    volatile uint32_t ui32TempValueF;;

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

    while(true){
        UARTPrintf("Enter the Cmd : ");
        while(!UARTCharsAvail(UART0_BASE));{
            cmd = UARTCharGet(UART0_BASE);
            UARTCharPut(UART0_BASE,cmd); UARTCharPut(UART0_BASE,'\n'); UARTCharPut(UART0_BASE,'\r');
            switch(cmd){
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


void init_TIMER(){
    MAP_TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC);
    // Set sample frequency to 16KHz (every 62.5uS)
    MAP_TimerLoadSet(TIMER0_BASE, TIMER_A, MAP_SysCtlClockGet()/16000 -1);   //TODO: Timer Load Value is set here
    MAP_TimerControlTrigger(TIMER0_BASE, TIMER_A, true);
    MAP_TimerControlStall(TIMER0_BASE, TIMER_A, true); //Assist in debug by stalling timer at breakpoints
}

void init_ADC(){
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

void init_DMA(){
    uDMAEnable(); // Enables uDMA
    uDMAControlBaseSet(ucControlTable);

    uDMAChannelAttributeDisable(UDMA_CHANNEL_ADC0, UDMA_ATTR_ALTSELECT | UDMA_ATTR_HIGH_PRIORITY | UDMA_ATTR_REQMASK);

    uDMAChannelAttributeEnable(UDMA_CHANNEL_ADC0, UDMA_ATTR_USEBURST);
    // Only allow burst transfers

    uDMAChannelControlSet(UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT, UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_1);
    uDMAChannelControlSet(UDMA_CHANNEL_ADC0 | UDMA_ALT_SELECT, UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_1);

    uDMAChannelTransferSet(UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT, UDMA_MODE_PINGPONG, (void *)(ADC0_BASE + ADC_O_SSFIFO0), &ADC_Out1, ADC_SAMPLE_BUF_SIZE);
    uDMAChannelTransferSet(UDMA_CHANNEL_ADC0 | UDMA_ALT_SELECT, UDMA_MODE_PINGPONG, (void *)(ADC0_BASE + ADC_O_SSFIFO0), &ADC_Out2, ADC_SAMPLE_BUF_SIZE);

    uDMAChannelEnable(UDMA_CHANNEL_ADC0); // Enables DMA channel so it can perform transfers
}

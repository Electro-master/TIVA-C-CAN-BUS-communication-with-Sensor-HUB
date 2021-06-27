/*
 * master_final.c
 *
 *  Created on: 21-Jun-2021
 *      Author: Pranjal
 */
#include "master_final.h"

int i=0;
int string_ready = 0;
char ch,ch_prev='a';
char str[10];
volatile bool new_can_recv = false;
//Readings
int curr_temp_int = 0,curr_temp_frac=0;
int curr_pres_int = 0,curr_pres_frac=0;
int curr_lux_int=0,curr_lux_frac=0;

volatile bool g_bErrFlag = 0;    //error flag
volatile uint32_t g_ui32ErrFlag = 0; //error flag

volatile bool g_bRXFlag = 0; //reception flag
volatile uint32_t g_ui32RXMsgCount = 0; //Store count of recieved frames
volatile uint32_t g_ui32TXMsgCount = 0;  //Store count of sent frames

tCANMsgObject sCANRxMessage;  //Receive CAN object
tCANMsgObject sCANTxMessage; //Transmit CAN object
uint32_t ui32MsgDataTx[2];      //Transmit data
uint32_t ui32MsgDataRx;      //Receive Data
uint8_t ui8RxMsgData[8];     //Receive data 8-bit
uint8_t *pui8MsgData;        //Tansmit data 8-bit
//*****************************************************************************
//
// This function sets up UART0 to be used for a console to display information
// as the example is running.
//
//*****************************************************************************
void
InitConsole(void)
{
    //
    // Enable GPIO port A which is used for UART0 pins.
    // TODO: change this to whichever GPIO port you are using.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Configure the pin muxing for UART0 functions on port A0 and A1.
    // This step is not necessary if your part does not support pin muxing.
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    //
    // Enable UART0 so that we can configure the clock.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    //UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    //
    // Select the alternate (UART) function for these pins.
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
    //Enable UArt Interrupt as well
    IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE,(UART_INT_RX | UART_INT_RI));
    UARTFIFODisable(UART0_BASE);
    UARTIntClear(UART0_BASE,(UART_INT_RX | UART_INT_RI));
}


void Uart_Handler(void)
{
   // extern volatile int i;
    ch = UART0_DR_R;
    if(ch!=255 )
        UARTCharPut(UART0_BASE,ch);

     if ( (ch >= 97) & (ch <= 122) ) //to enable both upper and lower case
         ch = ch-32 ;
     if(!((ch_prev==' ' && ch==' ')||(ch_prev=='_' && ch=='_')))
     {str[i]=ch;
      ch_prev=ch;
       ++i;
      }
     if(ch==8) // backspace
        {--i;--i;}
     if(ch=='\r')
     {  string_ready=1;
         str[i] = '\0';
     }
    UARTIntClear(UART0_BASE,(UART_INT_RX | UART_INT_RI));
}

void
SimpleDelay(void)
{
    //
    // Delay cycles for 1 second
    //
    SysCtlDelay(16000000 / 3);
}

void
CANIntHandler(void)
{
    uint32_t ui32Status;

    //
    // Read the CAN interrupt status to find the cause of the interrupt
    //
    ui32Status = CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE);

    //
    // If the cause is a controller status interrupt, then get the status
    //
    if(ui32Status == CAN_INT_INTID_STATUS)
    {
        //
        // Read the controller status.  This will return a field of status
        // error bits that can indicate various errors.  Error processing
        // is not done in this example for simplicity.  Refer to the
        // API documentation for details about the error status bits.
        // The act of reading this status will clear the interrupt.  If the
        // CAN peripheral is not connected to a CAN bus with other CAN devices
        // present, then errors will occur and will be indicated in the
        // controller status.
        //
        ui32Status = CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);

        //
        // Set a flag to indicate some errors may have occurred.
        //
        g_bErrFlag = 1;
    }

    //
    // Check if the cause is message object 1, which what we are using for
    // sending messages.
    //
    else if(ui32Status == TXOBJECT)
        {
            //
            // Getting to this point means that the TX interrupt occurred on
            // message object TXOBJECT, and the message reception is complete.
            // Clear the message object interrupt.
            //
            CANIntClear(CAN0_BASE, TXOBJECT);

            //
            // Increment a counter to keep track of how many messages have been
            // transmitted. In a real application this could be used to set
            // flags to indicate when a message is transmitted.
            //
            g_ui32TXMsgCount++;

            //
            // Since a message was transmitted, clear any error flags.
            // This is done because before the message is transmitted it triggers
            // a Status Interrupt for TX complete. by clearing the flag here we
            // prevent unnecessary error handling from happening
            //
            g_ui32ErrFlag = 0;
            g_bErrFlag = 0;
        }


    else if(ui32Status == RXOBJECT)
        {
            //
            // Getting to this point means that the RX interrupt occurred on
            // message object RXOBJECT, and the message reception is complete.
            // Clear the message object interrupt.
            //
            CANIntClear(CAN0_BASE, RXOBJECT);

            //
            // Increment a counter to keep track of how many messages have been
            // received.  In a real application this could be used to set flags to
            // indicate when a message is received.
            //
            g_ui32RXMsgCount++;

            //
            // Set flag to indicate received message is pending.
            //
            g_bRXFlag = true;

            //
            // Since a message was received, clear any error flags.
            // This is done because before the message is received it triggers
            // a Status Interrupt for RX complete. by clearing the flag here we
            // prevent unnecessary error handling from happening
            //
            g_ui32ErrFlag = 0;
            g_bErrFlag = 0;
        }
    //
    // Otherwise, something unexpected caused the interrupt.  This should
    // never happen.
    //
    else
    {
        //
        // Spurious interrupt handling can go here.
        //
    }
}

void InitCanbus(void)
{
    #if defined(TARGET_IS_TM4C129_RA0) ||                                         \
    defined(TARGET_IS_TM4C129_RA1) ||                                         \
    defined(TARGET_IS_TM4C129_RA2)
    uint32_t ui32SysClock;
#endif



    //
    // Set the clocking to run directly from the external crystal/oscillator.
    // TODO: The SYSCTL_XTAL_ value must be changed to match the value of the
    // crystal on your board.
    //
#if defined(TARGET_IS_TM4C129_RA0) ||                                         \
    defined(TARGET_IS_TM4C129_RA1) ||                                         \
    defined(TARGET_IS_TM4C129_RA2)
    ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                       SYSCTL_OSC_MAIN |
                                       SYSCTL_USE_OSC)
                                       25000000);
#else
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ);
#endif
    // For this example CAN0 is used with RX and TX pins on port B4 and B5.
    // The actual port and pins used may be different on your part, consult
    // the data sheet for more information.
    // GPIO port B needs to be enabled so these pins can be used.
    // TODO: change this to whichever GPIO port you are using
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    //
    // Configure the GPIO pin muxing to select CAN0 functions for these pins.
    // This step selects which alternate function is available for these pins.
    // This is necessary if your part supports GPIO pin function muxing.
    // Consult the data sheet to see which functions are allocated per pin.
    // TODO: change this to select the port/pin you are using
    //
    GPIOPinConfigure(GPIO_PE4_CAN0RX);
    GPIOPinConfigure(GPIO_PE5_CAN0TX);

    //
    // Enable the alternate function on the GPIO pins.  The above step selects
    // which alternate function is available.  This step actually enables the
    // alternate function instead of GPIO for these pins.
    // TODO: change this to match the port/pin you are using
    //
    GPIOPinTypeCAN(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    //
    // The GPIO port and pins have been set up for CAN.  The CAN peripheral
    // must be enabled.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);

    //
    // Initialize the CAN controller
    //
    CANInit(CAN0_BASE);

    //
    // Set up the bit rate for the CAN bus.  This function sets up the CAN
    // bus timing for a nominal configuration.  You can achieve more control
    // over the CAN bus timing by using the function CANBitTimingSet() instead
    // of this one, if needed.
    // In this example, the CAN bus is set to 500 kHz.  In the function below,
    // the call to SysCtlClockGet() or ui32SysClock is used to determine the
    // clock rate that is used for clocking the CAN peripheral.  This can be
    // replaced with a  fixed value if you know the value of the system clock,
    // saving the extra function call.  For some parts, the CAN peripheral is
    // clocked by a fixed 8 MHz regardless of the system clock in which case
    // the call to SysCtlClockGet() or ui32SysClock should be replaced with
    // 8000000.  Consult the data sheet for more information about CAN
    // peripheral clocking.
    //
#if defined(TARGET_IS_TM4C129_RA0) ||                                         \
    defined(TARGET_IS_TM4C129_RA1) ||                                         \
    defined(TARGET_IS_TM4C129_RA2)
    CANBitRateSet(CAN0_BASE, ui32SysClock, 100000);
#else
    CANBitRateSet(CAN0_BASE, SysCtlClockGet(), 100000);
#endif

    //
    // Enable interrupts on the CAN peripheral.  This example uses static
    // allocation of interrupt handlers which means the name of the handler
    // is in the vector table of startup code.  If you want to use dynamic
    // allocation of the vector table, then you must also call CANIntRegister()
    // here.
    //
    // CANIntRegister(CAN0_BASE, CANIntHandler); // if using dynamic vectors
    //
    CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);

    //
    // Enable the CAN interrupt on the processor (NVIC).
    //
    IntEnable(INT_CAN0);

    //
    // Enable the CAN for operation.
    //
    CANEnable(CAN0_BASE);
    sCANRxMessage.ui32MsgID = CAN0RXID;
    sCANRxMessage.ui32MsgIDMask = 0;
    sCANRxMessage.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;
    sCANRxMessage.ui32MsgLen = sizeof(ui8RxMsgData);
    sCANRxMessage.pui8MsgData = (uint8_t *)&ui8RxMsgData;
    // Now load the message object into the CAN peripheral.  Once loaded the
        // CAN will receive any message on the bus, and an interrupt will occur.
        // Use message object RXOBJECT for receiving messages (this is not the
        //same as the CAN ID which can be any value in this example).
        //
    CANMessageSet(CAN0_BASE, RXOBJECT, &sCANRxMessage, MSG_OBJ_TYPE_RX);
    //
    pui8MsgData = (uint8_t *)&ui32MsgDataTx[0];
    //
    // Initialize the message object that will be used for sending CAN
    // messages.  The message will be 4 bytes that will contain an incrementing
    // value.  Initially it will be set to 0.
    //
   // ui32MsgDataTx = 0; //Change this and msg changes

    sCANTxMessage.ui32MsgID = CAN0TXID;
    sCANTxMessage.ui32MsgIDMask = 0;
    sCANTxMessage.ui32Flags = MSG_OBJ_TX_INT_ENABLE;
    sCANTxMessage.ui32MsgLen = 2*sizeof(pui8MsgData);
    sCANTxMessage.pui8MsgData = pui8MsgData;
}

void send_can(char str[10])
{
    ui32MsgDataTx[0] = 0;ui32MsgDataTx[1] = 0;
        switch(str[0])
        { case 'T' : ui32MsgDataTx[0] += ((uint8_t)curr_temp_int)<<24;ui32MsgDataTx[0] += ((uint8_t)(curr_temp_frac>>8))<<16;ui32MsgDataTx[0] += ((uint8_t)curr_temp_frac)<<8;ui32MsgDataTx[0] += ((uint8_t)'1');
                    break;
          case 'P' : ui32MsgDataTx[0] += ((uint8_t)curr_pres_int)<<24;ui32MsgDataTx[0] += ((uint8_t)(curr_pres_frac>>8))<<16;ui32MsgDataTx[0] += ((uint8_t)curr_pres_frac)<<8;ui32MsgDataTx[0] += ((uint8_t)'2');
                        break;
          //case 'L' : ui32MsgDataTx += ((uint8_t)(curr_lux_int>>8))<<24;ui32MsgDataTx += ((uint8_t)(curr_lux_int))<<16;ui32MsgDataTx += ((uint8_t)curr_lux_frac/4)<<8;ui32MsgDataTx += ((uint8_t)'3');
          case 'L' : ui32MsgDataTx[1] += ((uint8_t)(curr_lux_int>>8));ui32MsgDataTx[0] += ((uint8_t)(curr_lux_int))<<24;ui32MsgDataTx[0] += ((uint8_t)(curr_lux_frac>>8))<<16;ui32MsgDataTx[0] += ((uint8_t)(curr_lux_frac))<<8;ui32MsgDataTx[0] += ((uint8_t)'3');
                        break;
          default : ui32MsgDataTx[0] += ((uint8_t)str[3])<<24;ui32MsgDataTx[0] += ((uint8_t)str[2])<<16;ui32MsgDataTx[0] += ((uint8_t)str[1])<<8;ui32MsgDataTx[0] += ((uint8_t)str[0]);
                          break;
        }

        //
        // Send the CAN message using object number 1 (not the same thing as
        // CAN ID, which is also 1 in this example).  This function will cause
        // the message to be transmitted right away.
        //
        CANMessageSet(CAN0_BASE, TXOBJECT, &sCANTxMessage, MSG_OBJ_TYPE_TX); //Inplace of TxObject I mistakenly wrote 1 which is Rxobject

        //
        // Now wait 1 second before continuing
        //
        SimpleDelay();

        //
        // Check the error flag to see if errors occurred
        //
        if(g_bErrFlag)
        {
            UARTprintf(" error - cable connected?\n");
        }
        else
        {
            //
            // If no errors then print the count of message sent
            //

        }

}

void receive_can(void)
{
    if(g_bRXFlag)
            {
                //
                // Reuse the same message object that was used earlier to configure
                // the CAN for receiving messages.  A buffer for storing the
                // received data must also be provided, so set the buffer pointer
                // within the message object.
                //
                sCANRxMessage.pui8MsgData = ui8RxMsgData;

                //
                // Read the message from the CAN.  Message object RXOBJECT is used
                // (which is not the same thing as CAN ID).  The interrupt clearing
                // flag is not set because this interrupt was already cleared in
                // the interrupt handler.
                //
                CANMessageGet(CAN0_BASE, RXOBJECT, &sCANRxMessage, 0);

                //
                // Clear the pending message flag so that the interrupt handler can
                // set it again when the next message arrives.
                //
                g_bRXFlag = 0;

                //
                // Check to see if there is an indication that some messages were
                // lost.
                //
                if(sCANRxMessage.ui32Flags & MSG_OBJ_DATA_LOST)
                {
                    UARTprintf("\nCAN message loss detected\n");
                }
                new_can_recv = true;

            }
}
void print_reading(void)
{
    new_can_recv = false;
    switch(ui8RxMsgData[0])
    {
    case '1': UARTprintf("Current temperature is %3d.%03d Celsius\n",ui8RxMsgData[3],(int)(ui8RxMsgData[2]<<8)+ui8RxMsgData[1]);
            break;
    case '2': UARTprintf("Current pressure is %3d.%3d kPa\n",ui8RxMsgData[3],(int)(ui8RxMsgData[2]<<8)+ui8RxMsgData[1]);
            break;
    //case '3': UARTprintf("Current lux is %3d.%03d lux\n",ui8RxMsgData[3],(int)(ui8RxMsgData[2]<<8)+ui8RxMsgData[1]);
    case '3': UARTprintf("Current lux is %3d.%03d lux\n",(int)(ui8RxMsgData[4]<<8)+ui8RxMsgData[3],(int)(ui8RxMsgData[2]<<8)+ui8RxMsgData[1]);
            break;
    default: UARTprintf("Data is %3d.%03d\n",ui8RxMsgData[2],ui8RxMsgData[3]);
    }
}

void send_reading(void)
{
    new_can_recv = false;
    get_temp_pres();
    get_lux();
    switch(ui8RxMsgData[1])
    {
    case '1': UARTprintf("Sending temperature is %3d.%03d Celsius\n",curr_temp_int,curr_temp_frac);send_can("T");
            break;
    case '2': UARTprintf("Sending pressure is %3d.%03d kPa\n",curr_pres_int,curr_pres_frac);send_can("P");
            break;
    case '3': UARTprintf("Sending lux is %3d.%03d lux\n",curr_lux_int,curr_lux_frac);send_can("L");
            break;
    default: UARTprintf("Sending Data is %3d.%03d\n",curr_temp_int,curr_temp_frac);send_can("X");
    }
}
void menu(void)
{
    UARTprintf("Commands Available - \n");
    UARTprintf("'Temp' - to get temperature\n");
    UARTprintf("'Pres' - to get pressure\n");
    UARTprintf("'Lux' - to get lux\n");
}

void do_branch(char str[10])
{

    switch(str[0])
    {
    case 'T': UARTprintf("Asking for Temperature\n");send_can("01");
            break;
    case 'P': UARTprintf("Asking for Pressure\n");send_can("02");
            break;
    case 'L': UARTprintf("Asking for Lux\n");send_can("03");
            break;
    default :  UARTprintf("Command not supported\n"); menu();
    }

}

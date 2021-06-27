#ifndef MASTER_FINAL_H
#define MASTER_FINAL_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "inc/hw_can.h"
#include <inc/tm4c123gh6pm.h>
#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/can.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

//Defining IDs for Rx and Tx, If CAN0RXID = 0 it will receive all the frames on bus
#define CAN0RXID                0
#define RXOBJECT                1
#define CAN0TXID                2
#define TXOBJECT                2

// Can related ***********************
extern tCANMsgObject sCANRxMessage;  //Receive CAN object
extern tCANMsgObject sCANTxMessage; //Transmit CAN object
extern uint32_t ui32MsgDataTx[2];      //Transmit data
extern uint32_t ui32MsgDataRx;      //Receive Data
extern uint8_t ui8RxMsgData[8];     //Receive data 8-bit
extern uint8_t *pui8MsgData;        //Tansmit data 8-bit

// Uart related
extern int string_ready;
extern char ch,ch_prev;
extern int i;
extern char str[10];
extern volatile bool new_can_recv;
//Readings
extern int curr_temp_int,curr_temp_frac;
extern int curr_pres_int,curr_pres_frac;
extern int curr_lux_int,curr_lux_frac;

//*****************************************************************************
//
// A flag to indicate that some transmission error occurred.
//
//*****************************************************************************
extern volatile bool g_bErrFlag;    //error flag
extern volatile uint32_t g_ui32ErrFlag; //error flag

extern volatile bool g_bRXFlag; //reception flag
extern volatile uint32_t g_ui32RXMsgCount; //Store count of recieved frames
extern volatile uint32_t g_ui32TXMsgCount;  //Store count of sent frames

//Function Declarations
void InitConsole(void); //Initialises UART
void SimpleDelay(void);  //Delay of 1 sec
void InitCanbus(void);      //Initialises CAN bus communication
void send_can(char str[10]);    //Function to make and send CAN frames
void receive_can(void);         //Function o reeive can frame
void do_branch(char str[10]);   //Funcion to handle uart input
void print_reading(void);       //Print the received reading
void send_reading(void);        //Send the reading
void menu(void);                //TO print menu
extern void CANIntHandler(void);    //Can interrupt handler
extern void Uart_Handler(void);     //Uart interrupt handler

#endif


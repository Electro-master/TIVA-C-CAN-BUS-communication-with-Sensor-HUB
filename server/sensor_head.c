/*
 * sensor_head.c
 *
 *  Created on: 21-Jun-2021
 *      Author: Pranjal
 */
#include "sensor_head.h"

tI2CMInstance g_sI2CInst;

//*****************************************************************************
//
// Global instance structure for the BMP180 sensor driver.
//
//*****************************************************************************
tBMP180 g_sBMP180Inst;
tISL29023 g_sISL29023Inst;
//*****************************************************************************
//
// Global new data flag to alert main that BMP180 data is ready.
//
//*****************************************************************************
volatile uint_fast8_t g_vui8DataFlag;
volatile unsigned long g_vui8ErrorFlag;
volatile unsigned long g_vui8IntensityFlag;

void BMP180AppCallback(void* pvCallbackData, uint_fast8_t ui8Status)
{
    if(ui8Status == I2CM_STATUS_SUCCESS)
    {
        g_vui8DataFlag = 1;
    }
}

void
ISL29023AppCallback(void *pvCallbackData, uint_fast8_t ui8Status)
{
    //
    // If the transaction succeeded set the data flag to indicate to
    // application that this transaction is complete and data may be ready.
    //
    if(ui8Status == I2CM_STATUS_SUCCESS)
    {
        g_vui8DataFlag = 1;
    }

    //
    // Store the most recent status in case it was an error condition
    //
    g_vui8ErrorFlag = ui8Status;
}

void
IntGPIOd(void)
{
    unsigned long ulStatus;

    ulStatus = GPIOIntStatus(GPIO_PORTD_BASE, true);

    //
    // Clear all the pin interrupts that are set
    //
    GPIOIntClear(GPIO_PORTD_BASE, ulStatus);

    if(ulStatus & GPIO_PIN_2)
    {
        //
        // ISL29023 has indicated that the light level has crossed outside of
        // the intensity threshold levels set in INT_LT and INT_HT registers.
        //
        g_vui8IntensityFlag = 1;
    }
}

//*****************************************************************************
//
// Called by the NVIC as a result of I2C3 Interrupt. I2C3 is the I2C connection
// to the BMP180.
//
//*****************************************************************************
void
BMP180I2CIntHandler(void)
{
    //
    // Pass through to the I2CM interrupt handler provided by sensor library.
    // This is required to be at application level so that I2CMIntHandler can
    // receive the instance structure pointer as an argument.
    //
    I2CMIntHandler(&g_sI2CInst);
}

//*****************************************************************************
//
// Called by the NVIC as a SysTick interrupt, which is used to generate the
// sample interval
//
//*****************************************************************************
void
SysTickIntHandler()
{
    BMP180DataRead(&g_sBMP180Inst, BMP180AppCallback, &g_sBMP180Inst);
    ISL29023DataRead(&g_sISL29023Inst, ISL29023AppCallback, &g_sISL29023Inst);
}

void
ISL29023AppI2CWait(char *pcFilename, uint_fast32_t ui32Line)
{
    //
    // Put the processor to sleep while we wait for the I2C driver to
    // indicate that the transaction is complete.
    //
    while((g_vui8DataFlag == 0) && (g_vui8ErrorFlag == 0))
    {
        //
        // Do Nothing
        //
    }

    //
    // clear the data flag for next use.
    //
    g_vui8DataFlag = 0;
}

void
ISL29023AppAdjustRange(tISL29023 *pInst)
{
    float fAmbient;
    uint8_t ui8NewRange;

    ui8NewRange = g_sISL29023Inst.ui8Range;

    //
    // Get a local floating point copy of the latest light data
    //
    ISL29023DataLightVisibleGetFloat(&g_sISL29023Inst, &fAmbient);

    //
    // Check if we crossed the upper threshold.
    //
    if(fAmbient > g_fThresholdHigh[g_sISL29023Inst.ui8Range])
    {
        //
        // The current intensity is over our threshold so adjsut the range
        // accordingly
        //
        if(g_sISL29023Inst.ui8Range < ISL29023_CMD_II_RANGE_64K)
        {
            ui8NewRange = g_sISL29023Inst.ui8Range + 1;
        }
    }

    //
    // Check if we crossed the lower threshold
    //
    if(fAmbient < g_fThresholdLow[g_sISL29023Inst.ui8Range])
    {
        //
        // If possible go to the next lower range setting and reconfig the
        // thresholds.
        //
        if(g_sISL29023Inst.ui8Range > ISL29023_CMD_II_RANGE_1K)
        {
            ui8NewRange = g_sISL29023Inst.ui8Range - 1;
        }
    }

    //
    // If the desired range value changed then send the new range to the sensor
    //
    if(ui8NewRange != g_sISL29023Inst.ui8Range)
    {
        ISL29023ReadModifyWrite(&g_sISL29023Inst, ISL29023_O_CMD_II,
                                ~ISL29023_CMD_II_RANGE_M, ui8NewRange,
                                ISL29023AppCallback, &g_sISL29023Inst);

        //
        // Wait for transaction to complete
        //
        ISL29023AppI2CWait(__FILE__, __LINE__);
    }
}


void InitI2C()
{
    uint8_t ui8Mask; //Mask for ISL
    //
    // The I2C3 peripheral must be enabled before use.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C3);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    //
    // Configure the pin muxing for I2C3 functions on port D0 and D1.
    // This step is not necessary if your part does not support pin muxing.
    //
    GPIOPinConfigure(GPIO_PD0_I2C3SCL);
    GPIOPinConfigure(GPIO_PD1_I2C3SDA);

    //
    // Select the I2C function for these pins.  This function will also
    // configure the GPIO pins pins for I2C operation, setting them to
    // open-drain operation with weak pull-ups.  Consult the data sheet
    // to see which functions are allocated per pin.
    //
    GPIOPinTypeI2CSCL(GPIO_PORTD_BASE, GPIO_PIN_0);
    GPIOPinTypeI2C(GPIO_PORTD_BASE, GPIO_PIN_1);
    //Pin d2 interrupt
    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_2);
    GPIOIntEnable(GPIO_PORTD_BASE, GPIO_PIN_2);
    GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_FALLING_EDGE);
    IntEnable(INT_GPIOD);

    IntPrioritySet(INT_I2C3, 0x00);
    IntPrioritySet(FAULT_SYSTICK, 0x40);
    IntPrioritySet(INT_CAN0, 0x80);
    IntPrioritySet(INT_GPIOD, 0x80);
    IntPrioritySet(INT_UART0, 0x80);

    IntMasterEnable();

    //
    // Initialize the I2C3 peripheral.
    //
    I2CMInit(&g_sI2CInst, I2C3_BASE, INT_I2C3, 0xff, 0xff,
                 SysCtlClockGet());
    //
    // Initialize the BMP180.
    //
    BMP180Init(&g_sBMP180Inst, &g_sI2CInst, BMP180_I2C_ADDRESS,
               BMP180AppCallback, &g_sBMP180Inst);

    //
    // Wait for initialization callback to indicate reset request is complete.
    //
    while(g_vui8DataFlag == 0)
    {
        //
        // Wait for I2C Transactions to complete.
        //
    }

    //
    // Reset the data ready flag
    //
    g_vui8DataFlag = 0;

    //Initialize ISL29023
    ISL29023Init(&g_sISL29023Inst, &g_sI2CInst, ISL29023_I2C_ADDRESS,
                 ISL29023AppCallback, &g_sISL29023Inst);

    //
    // Wait for transaction to complete
    //
    ISL29023AppI2CWait(__FILE__, __LINE__);

    //
    // Configure the ISL29023 to measure ambient light continuously. Set a 8
    // sample persistence before the INT pin is asserted. Clears the INT flag.
    // Persistence setting of 8 is sufficient to ignore camera flashes.
    //
    ui8Mask = (ISL29023_CMD_I_OP_MODE_M | ISL29023_CMD_I_INT_PERSIST_M |
               ISL29023_CMD_I_INT_FLAG_M);
    ISL29023ReadModifyWrite(&g_sISL29023Inst, ISL29023_O_CMD_I, ~ui8Mask,
                            (ISL29023_CMD_I_OP_MODE_ALS_CONT |
                             ISL29023_CMD_I_INT_PERSIST_8),
                            ISL29023AppCallback, &g_sISL29023Inst);

    //
    // Wait for transaction to complete
    //
    ISL29023AppI2CWait(__FILE__, __LINE__);

    //
    // Configure the upper threshold to 80% of maximum value
    //
    g_sISL29023Inst.pui8Data[1] = 0xCC;
    g_sISL29023Inst.pui8Data[2] = 0xCC;
    ISL29023Write(&g_sISL29023Inst, ISL29023_O_INT_HT_LSB,
                  g_sISL29023Inst.pui8Data, 2, ISL29023AppCallback,
                  &g_sISL29023Inst);

    //
    // Wait for transaction to complete
    //
    ISL29023AppI2CWait(__FILE__, __LINE__);

    //
    // Configure the lower threshold to 20% of maximum value
    //
    g_sISL29023Inst.pui8Data[1] = 0x33;
    g_sISL29023Inst.pui8Data[2] = 0x33;
    ISL29023Write(&g_sISL29023Inst, ISL29023_O_INT_LT_LSB,
                  g_sISL29023Inst.pui8Data, 2, ISL29023AppCallback,
                  &g_sISL29023Inst);
    //
    // Wait for transaction to complete
    //
    ISL29023AppI2CWait(__FILE__, __LINE__);


    //
    // Enable the system ticks at 10 Hz.
    //
    SysTickPeriodSet(SysCtlClockGet() / (10 * 3));
    SysTickIntEnable();
    SysTickEnable();
}

void get_temp_pres()
{
    float fTemperature, fPressure;
    int32_t i32IntegerPart;
    int32_t i32FractionPart;

    BMP180DataRead(&g_sBMP180Inst, BMP180AppCallback, &g_sBMP180Inst);
    while(g_vui8DataFlag == 0)
    {
        //
        // Wait for the new data set to be available.
        //
    }

    //
    // Reset the data ready flag.
    //
    g_vui8DataFlag = 0;

    //
    // Get a local copy of the latest temperature data in float format.
    //
    BMP180DataTemperatureGetFloat(&g_sBMP180Inst, &fTemperature);

    //
    // Convert the floats to an integer part and fraction part for easy
    // print.
    //
    i32IntegerPart = (int32_t) fTemperature;
    i32FractionPart =(int32_t) (fTemperature * 1000.0f);
    i32FractionPart = i32FractionPart - (i32IntegerPart * 1000);
    if(i32FractionPart < 0)
    {
        i32FractionPart *= -1;
    }

    //
    // Print temperature with three digits of decimal precision.
    //
    //UARTprintf("Temperature %3d.%03d\t\t", i32IntegerPart, i32FractionPart);
    curr_temp_int = i32IntegerPart;
    curr_temp_frac = i32FractionPart;

    //
    // Get a local copy of the latest air pressure data in float format.
    //
    BMP180DataPressureGetFloat(&g_sBMP180Inst, &fPressure);

    //
    // Convert the floats to an integer part and fraction part for easy
    // print.
    //
    i32IntegerPart = (int32_t) fPressure/1000;
    i32FractionPart = (int32_t)fPressure - (i32IntegerPart * 1000);
    if(i32FractionPart < 0)
    {
        i32FractionPart *= -1;
    }

    //
    // Print Pressure with three digits of decimal precision.
    //
    curr_pres_int = i32IntegerPart;
    curr_pres_frac = i32FractionPart;
}

void get_lux()
{
    float fAmbient;
    int32_t i32IntegerPart, i32FractionPart;


    while(g_vui8DataFlag == 0)
        {
            //
            // Wait for the new data set to be available.
            //
        }
    g_vui8DataFlag = 0;

                //
                // Get a local floating point copy of the latest light data
                //
                ISL29023DataLightVisibleGetFloat(&g_sISL29023Inst, &fAmbient);

                //
                // Perform the conversion from float to a printable set of integers
                //
                i32IntegerPart = (int32_t)fAmbient;
                i32FractionPart = (int32_t)(fAmbient * 1000.0f);
                i32FractionPart = i32FractionPart - (i32IntegerPart * 1000);
                if(i32FractionPart < 0)
                {
                    i32FractionPart *= -1;
                }

                //
                // Print the temperature as integer and fraction parts.
                //
                curr_lux_int = i32IntegerPart;
                curr_lux_frac = i32FractionPart;

                //
                // Check if the intensity of light has crossed a threshold. If so
                // then adjust range of sensor readings to track intensity.
                //
                if(g_vui8IntensityFlag)
                {
                    //
                    // Disable the low priority interrupts leaving only the I2C
                    // interrupt enabled.
                    //
                    IntPriorityMaskSet(0x40);

                    //
                    // Reset the intensity trigger flag.
                    //
                    g_vui8IntensityFlag = 0;

                    //
                    // Adjust the lux range.
                    //
                    ISL29023AppAdjustRange(&g_sISL29023Inst);

                    //
                    // Now we must manually clear the flag in the ISL29023
                    // register.
                    //
                    ISL29023Read(&g_sISL29023Inst, ISL29023_O_CMD_I,
                                 g_sISL29023Inst.pui8Data, 1, ISL29023AppCallback,
                                 &g_sISL29023Inst);

                    //
                    // Wait for transaction to complete
                    //
                    ISL29023AppI2CWait(__FILE__, __LINE__);

                    //
                    // Disable priority masking so all interrupts are enabled.
                    //
                    IntPriorityMaskSet(0);
                }
}

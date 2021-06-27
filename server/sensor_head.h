#ifndef SENSOR_H
#define SENSOR_H

#include "master_final.h"
#include <math.h>
#include "sensorlib/i2cm_drv.h"
#include "sensorlib/hw_bmp180.h"
#include "sensorlib/bmp180.h"
#include "sensorlib/hw_isl29023.h"
#include "sensorlib/isl29023.h"

//*****************************************************************************
//
// Define BMP180 & ISL29023 I2C Address.
//
//*****************************************************************************
#define BMP180_I2C_ADDRESS      0x77
#define ISL29023_I2C_ADDRESS    0x44

//*****************************************************************************
//
// Global instance structure for the I2C master driver.
//
//*****************************************************************************
extern tI2CMInstance g_sI2CInst;

//*****************************************************************************
//
// Global instance structure for the BMP180 sensor driver.
//
//*****************************************************************************
extern tBMP180 g_sBMP180Inst;
extern tISL29023 g_sISL29023Inst;
//*****************************************************************************
//
// Global new data flag to alert main that BMP180 data is ready.
//
//*****************************************************************************
extern volatile uint_fast8_t g_vui8DataFlag;
extern volatile unsigned long g_vui8ErrorFlag;
extern volatile unsigned long g_vui8IntensityFlag;
//*****************************************************************************
//
// Constants to hold the floating point version of the thresholds for each
// range setting. Numbers represent an 81% and 19 % threshold levels. This
// creates a +/- 1% hysteresis band between range adjustments.
//
//*****************************************************************************
static const float g_fThresholdHigh[4] =
{
    810.0f, 3240.0f, 12960.0f, 64000.0f
};
static const float g_fThresholdLow[4] =
{
    0.0f, 760.0f, 3040.0f, 12160.0f
};

//*****************************************************************************
//
// BMP180 Sensor callback function.  Called at the end of BMP180 sensor driver
// transactions. This is called from I2C interrupt context. Therefore, we just
// set a flag and let main do the bulk of the computations and display.
//
//*****************************************************************************
//Functions
void BMP180AppCallback(void* pvCallbackData, uint_fast8_t ui8Status); //Callback function BMP180
void InitI2C(void);                                 //Initialise I2C communication and sensors
void get_temp_pres(void);                           //To measure temp and pressure
void ISL29023AppCallback(void *pvCallbackData, uint_fast8_t ui8Status); //Callback function ISL29023
void ISL29023AppI2CWait(char *pcFilename, uint_fast32_t ui32Line); //Wait till I2C transaction is complete
void ISL29023AppAdjustRange(tISL29023 *pInst);  //Adjust range of LUX
void get_lux(void);                             //Function to meaure latest LUX
extern void BMP180I2CIntHandler(void);//I2C interrupt handler
extern void SysTickIntHandler(void); //Systick handler
extern void IntGPIOd(void);         //Port D interrupt handler

#endif



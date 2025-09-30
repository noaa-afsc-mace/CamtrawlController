/*
 *  PAxLD - simple driver for the Keller-Druck PAxLD line of 
 *  i2c pressure transducers.
 *  
 *  
 *  Rick Towler
 *  Midwater Assesment and Conservation Engineering Group
 *  NOAA Alaska Fisheries Science Center, Seattle, WA
 *  rick.towler@noaa.gov
 */

#ifndef __PAxLD_H__
#define __PAxLD_H__

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
  #include "pins_arduino.h"
#endif
#include <Wire.h>

#define  PAXLD_DEF_I2C_ADDR (0x40)
#define  PAXLD_REQUEST_DATA (0xAC)
#define  PAXLD_DATA_ADDR    (0x40)
#define  PAXLD_SCALING1_CMD (0x13)
#define  PAXLD_SCALING2_CMD (0x14)
#define  PAXLD_SCALING3_CMD (0x15)
#define  PAXLD_SCALING4_CMD (0x16)
#define  PAXLD_READ_INT_MS  8

#define  PAXLD_STATUS_BIT   (0x40) // Read OK when set
#define  PAXLD_BUSY_BIT     (0x20) // Busy when set
#define  PAXLD_CHKSUM_BIT   (0x04) // Error when set


typedef union {
  uint32_t i;
  float f;
 } intToFloat_t;


class PAxLD
{

  public:

    float     pressure;
    float     temperature;
    uint32_t  rawPressure;
    float     pMin;
    float     pMax;

    PAxLD();
    PAxLD(uint8_t address);
    bool  init();
    bool  pollSensor();
    bool  convertData();

  protected:

    uint32_t  pollTime;
    uint8_t   i2cAddress;
    uint8_t   data[5];
    
    void I2CwriteByte(uint8_t subAddress, uint8_t data);
    uint8_t I2CreadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count);
    
};

#endif

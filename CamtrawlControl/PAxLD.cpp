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


#include "PAxLD.h"


PAxLD::PAxLD() 
{
  i2cAddress = PAXLD_DEF_I2C_ADDR;
}


PAxLD::PAxLD(uint8_t address) 
{
  i2cAddress = address;
}


bool PAxLD::init() 
{
  uint32_t temp0 = 0;
  uint32_t temp1 = 0;
  uint32_t temp2 = 0;
  intToFloat_t tempX;

  Wire.begin();

  //  get the Pmin scaling value from the sensor
  I2CwriteByte(PAXLD_SCALING1_CMD, 1);
  delay(10);
  I2CreadBytes(PAXLD_DATA_ADDR, data, 3);
  
  //  check if we got a response
  if (((data[0] & PAXLD_STATUS_BIT) >> 6) != 1)
    //  no response - no sensor attached
    return false;
    
  //  got data - shift the MSB and add the LSB
  temp0 = (((uint32_t) data[1]) << 8) + data[2];

  //  read the next 2 bytes
  delay(10);
  I2CwriteByte(PAXLD_SCALING2_CMD, 1);
  delay(10);
  I2CreadBytes(PAXLD_DATA_ADDR, data, 3);  
  temp1 = (((uint32_t) data[1]) << 8) + data[2];

  //  combine the two 16 bit values and convert to float
  tempX.i = (temp0 << 16) + temp1;
  pMax = (float) tempX.f;

  //  now get the Pmax scaling value
  delay(10);
  I2CwriteByte(PAXLD_SCALING3_CMD, 1);
  delay(10);
  I2CreadBytes(PAXLD_DATA_ADDR, data, 3);
  temp0 = (((uint32_t) data[1]) << 8) + data[2];

  delay(10);
  I2CwriteByte(PAXLD_SCALING4_CMD, 1);
  delay(10);
  I2CreadBytes(PAXLD_DATA_ADDR, data, 3);
  temp1 = (((uint32_t) data[1]) << 8) + data[2];

  //  combine the two 16 bit values and convert to float
  tempX.i = (temp0 << 16) + temp1;
  pMax = (float) tempX.f;
  
  pollTime = millis();

}


bool PAxLD::pollSensor() 
{
  // throttle the polling speed
  if (millis() - pollTime < PAXLD_READ_INT_MS)
    return false;
    
  //  send a read reuqest
  I2CwriteByte(PAXLD_REQUEST_DATA, 1);

  //  note the time
  pollTime = millis();

  return true;
}


bool PAxLD::convertData()
{
  uint8_t ok;

  //  check that we've given the sensor enough time
  if (millis() - pollTime < PAXLD_READ_INT_MS)
    //  not enough time
    return false;
  
  // read the data
  I2CreadBytes(PAXLD_DATA_ADDR, data, 5);

  //  check if the read was successful
  ok = (data[0] & 0x40) >> 6;
  if (ok != 1)
    //  problem reading data
    return false;
  
  //  combine and convert raw pressure to pressure in bar
  rawPressure = (float)((((uint32_t) data[1]) << 8) + (((uint32_t) data[2])));
  pressure = (rawPressure - 16384) * (pMax - pMin) / 32768.0 + pMin;
  //  convert from bar to decibar
  pressure = pressure * 10.0;
  //  combine and convert raw temperature to temperature in deg c
  temperature = (float)(((((uint32_t) data[3]) << 8) + (((uint32_t) data[4]))) >> 4);
  temperature = ((temperature - 24) * 0.05) - 50;

  return true;
}


void PAxLD::I2CwriteByte(uint8_t subAddress, uint8_t data)
{
  Wire.beginTransmission(i2cAddress);
  Wire.write(subAddress);
  Wire.write(data);
  Wire.endTransmission();
}

uint8_t PAxLD::I2CreadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count)
{
  byte retVal;
  Wire.beginTransmission(i2cAddress);
  Wire.write(subAddress | 0x80);
  retVal = Wire.endTransmission(false);
  if (retVal != 0)
    return 0;
  
  retVal = Wire.requestFrom(i2cAddress, count);
  if (retVal != count)
    return 0;
  
  for (int i=0; i<count;)
    dest[i++] = Wire.read();
  
  return count;
}

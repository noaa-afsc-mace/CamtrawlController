/*
 *  CamtrawlControl is the firmware for the version of the CamTrawl power
 *  and control board that provides power management and control of the 
 *  3rd generation CamTrawl underwater stereo camera platform. These systems
 *  are based on the ARM based ODroid XU4 SBC and the Udoo X86 SBC and
 *  power and control logic is provided by either a ARM Cortex M0 based
 *  microcontroller or the Intel Curie microcontroller.
 *  
 *  
 *  Rick Towler
 *  Midwater Assesment and Conservation Engineering Group
 *  NOAA Alaska Fisheries Science Center, Seattle, WA
 *  rick.towler@noaa.gov
 */

#ifndef __CTCONTROL_H__
#define __CTCONTROL_H__


//  -----------            Platform options             --------------
//  ---      Uncomment for specific control board versions         ---

//  ODroid XU4 board uses A1 for external On/Off control
//#define XU4

//  UDOO-X86 control board uses A1 for the 5v DC/DC control
#define UDOO



//  -----------                 Version                 --------------
//  --specify the PCB version - ignore the decimal - 2.3 becomes 23---
//             UDOO VERSIONS SHOULD BE SET TO 23!!!!
#define PCBVERSION  23


//  -----------           Debugging options             --------------
//  -----------   !comment these for production use!    --------------

//  uncomment to output debugging information to SAMD21 USB Serial console
//#define DEBUGPRINT

//  uncomment to disable PC power control. When uncommented, the PC DC
//  supply will be switched on when the MCU starts and will remain on
//  regardless of controller state. Note that the power will turn off
//  momentarily if the MCU is reset.
//#define DEBUGPOWER

//  ------------------------------------------------------------------


//  includes
#include <Wire.h>
#include <Adafruit_Sensor.h> 
#include <Adafruit_BNO055.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_INA260.h>
#include <utility/imumaths.h>
#include <Serial1Command.h>
#include <RunningMedian.h>
#include <FlashStorage.h>
//#include "ArduinoLowPower.h"
#include "RTClib.h"
#include "wiring_private.h"
#include "PAxLD.h"
#include "DebugUtils.h"


#define SERIALBAUD        921600  //  baud rate for primary MCU<->SBC serial interface
#define AUXSERIALBAUD     115200  //  baud rate for aux MCU serial interface
#define PARMSPROG         123     //  value used to signify that the FLASH has been programmed with operational parameters
#define TRIGDURUS         25      //  duration of the camera trigger pulse in microseconds
#define NSENSORSAMPS      5       //  number of samples to hold in median buffer for sensor outputs
#define NVOLTAGESAMPS     7       //  number of samples to hold in median buffer for the system voltages
#define NPOWERONFLASHES   4       //  number of times the strobes will flash at power up (if enabled)
#define PCOFFTIMEOUT      30000   //  number of milliseconds the controller should wait for PC to signal it is turning off before timing out
#define PCPOWWAITINT      30000   //  number of milliseconds after PC acks shutdown before turning off power 
#define SENSORSAMPINT     50000   //  HALF interval in MICROseconds of the sensor sampling timer - 50000 = 10Hz - cannot be less than 5000us!
#define VOLTSAMPINT       1000    //  interval in milliseconds of the voltage sampling period
#define PCBOOTTIMEOUT     300000  //  interval in milliseconds that the controller waits for the PC to boot before giving up
#define ANALOGDEPTHMIN    150     //  minimum raw A/D value requried to "detect" analog pressure transducer
#define RADEG             180/PI  //  constant
#define SENSORDEBUGINT    2000    //  interval in milliseconds between sensor console messages when debugging 
#define DEPTHCHKDELAY     10000   //  interval in milliseconds the controller will delay checking the system depth
#define INA260_I2CADDR    0x45    //  the INA260 installed on the controller has both addr pins pulled high

//  define SAMD21 Mini pin connections
const uint8_t powerEnable       = A2;         //  output pin - set high to enable system power (switched input power for Jetson Hadron)
const uint8_t strobeEnable      = A3;         //  output pin - set high to enable strobe power
const uint8_t externalTrigger   = 8;          //  input pin pulled high - set low to externally trigger camera system
const uint8_t switchedPower     = A1;         //  output pin - set high to enable switched 5v power (0.5A max)
const uint8_t presssureSwitch   = 13;         //  input pin pulled high - goes low if pressure switch closes
const uint8_t strobeTrigOne     = 10;         //  output pin - set high to trigger strobe channel 1
const uint8_t mcu_gpio_1        = 9;          //  input/output pin - 
const uint8_t mcu_gpio_2        = 11;          // input/output pin - 
const uint8_t Cam2_GPIO1        = 4;          //  output pin - set high to trigger left camera
const uint8_t Cam1_GPIO1        = 6;          //  output pin - set high to trigger right camera
const uint8_t Cam2_GPIO2        = 5;          //  input/output pin - 
const uint8_t Cam1_GPIO2        = 7;          //  input/output pin - 
const uint8_t forceOn           = 12;         //  input pin pulled high - goes low if force-on plug is installed
const uint8_t statusLED         = A0;         //  output pin - connected to status LED black wire
const uint8_t intOrLED          = PIN_LED_RXL;// RX LED on pin 25
const uint8_t intGrLED          = PIN_LED_TXL;// TX LED on pin 26



//  define default parameter values
bool checkPCTimeout       = true;    //  true if you want the system to park when the PC can't boot
bool forcedOn             = false;   //  true if the system was forced on
bool presSwClosed         = false;   //  true if the pressure switch is closed
bool flashOnPowerup       = true;    //  if true, flash the strobes a few times on power up
bool useExtAD             = false;   //  if true, output 2nd A/D input as a general value (cannot use with analog pressure sensor)
bool acquiring            = false;   //  true if the system is in the acquisition loop (PC on and at depth)
bool lowVoltage           = false;   //  true if the system battery voltage is below the minimum required for operation
bool monitorVoltage       = true;    //  false=No input voltage monitoring, true=voltage monitoring enabled
bool depthTurnsOn         = true;    //  true if the system uses depth to turn on/off, false if not
bool pollSensors          = true;    //  state var that stores if we're in a poll or read mode wrt the sensors
bool canCheckDepth        = false;   //  state used to delay checking the depth when system starts
uint8_t imuCalibrated     = 0;       //  holds the minimum IMU calibration state used to track if we should save updated parms
uint8_t strobeMode        = 1;       //  0=Off, 1=Strobed, 2=Constant On
uint8_t RTCInstalled      = 0;       //  0=No RTC, 1=DS3231 installed
uint8_t IMUInstalled      = 0;       //  0=No IMU, 1=Adafruit BNO055 IMU installed
uint8_t INAInstalled      = 0;       //  0=No INA260, 1=INA260 voltage/current sensor installed
uint8_t pXDCRInstalled    = 0;       //  0=No pressture transducer, 1=i2c pressure transducer installed, 2=Analog pressure transducer installed
uint8_t PCState           = 0;       //  0=Not ready, 1=Ready, 254=error, 255=Shutting down
uint32_t TurnOnDelay      = 0;       //  delay, in minutes, between when the system signals it is at depth and it turns on
float turnOnDepth         = 25;      //  system turn on depth in m
float turnOffDepth        = 15;      //  system turn off depth in m
float P2DSlope            = 1.0;     //  default analog pressure to depth slope
float P2DIntercept        = 0.0;     //  default analog pressure to depth offset
float A2VSlope            = 1.0;     //  default aux A/D to voltage conversion slope
float A2VIntercept        = 0.0;     //  default aux A/D to voltage conversion offset
float startupVoltage      = 23.5;    //  input voltage must exceed this value for the system to turn on
float voltageCutoff       = 22.0;    //  value of the input power threshold in volts (should not be below 1.0v * nCells)
float P2DLatitude         = 50.0;    //  Latitude used to convert pressure to depth
float P2DX                = 0.0;     //  stores the abs latitude in radians used when converting pressure to depth
const float sysVConv      = 0.00139; //  A/D conversion factor for system voltage (Vin / rawADC value)

//  sensor data variables
float systemVoltage       = 0.0;     //  current system input voltage
float auxADRaw            = 0.0;     //  current aux A/D input value
float auxADVal            = 0.0;     //  current aux A/D input value
float sysVRaw             = 0.0;     //  current raw system input voltage
float depth               = 0.0;     //  current system depth
float pitch               = 0.0;     //  current system pitch
float yaw                 = 0.0;     //  current system yaw
float roll                = 0.0;     //  current system roll
float accelX              = 0.0;     //  current system liner accelleration in X m/s^2
float accelY              = 0.0;     //  current system liner accelleration in Y m/s^2
float accelZ              = 0.0;     //  current system liner accelleration in Z m/s^2
float internalTemp        = 0.0;     //  current system temperature in C
float externalTemp        = 0.0;     //  current water temperature in C

//  define some utility variables
bool grLEDState           = false;
bool rdLEDState           = false;
bool pSwitchState         = false;
bool forceOnState         = false;
bool extShutdownState     = false;
uint8_t imuCalSys         = 0;       //  stores overall IMU cal status 0- uncalibrated to 3-fully calibrated
uint8_t imuCalGyr         = 0;       //  stores IMU gyro cal status 0- uncalibrated to 3-fully calibrated
uint8_t imuCalAcc         = 0;       //  stores IMU accelerometer cal status 0- uncalibrated to 3-fully calibrated
uint8_t imuCalMag         = 0;       //  stores IMU magnetometer cal status 0- uncalibrated to 3-fully calibrated
uint32_t n                = 0;       //  general use unsigned long
uint32_t m                = 0;       //  general use unsigned long
uint32_t o                = 0;       //  general use unsigned long
uint32_t sSampCounter     = 0;       //  tracks elapsed time in sensor timer event for sensor sampling
uint32_t vSampCounter     = 0;       //  tracks elapsed time in sensor timer event for voltage sampling
uint32_t sleepCounter     = 0;


//  define turn on delay variables
bool delayAtStart     = false;
DateTime turnOnTime   = DateTime();
TimeSpan delaySpan    = TimeSpan();

//  set up the flash for parameter storage
FlashStorage(fsConfigured, uint8_t);
FlashStorage(fsIMUCalState, uint8_t);
FlashStorage(fsTurnOnDepth, float);
FlashStorage(fsTurnOffDepth, float);
FlashStorage(fsP2DSlope, float);
FlashStorage(fsP2DIntercept, float);
FlashStorage(fsTurnOnDelay, uint32_t);
FlashStorage(fsA2VSlope, float);
FlashStorage(fsA2VIntercept, float);
FlashStorage(fsvoltageCutoff, float);
FlashStorage(fsvoltageStartup, float);
FlashStorage(fsP2DLatitude, float);
FlashStorage(fsmonitorVoltage, uint8_t);
FlashStorage(fsFlashOnPowerup, uint8_t);
FlashStorage(fsdepthTurnsOn, uint8_t);
FlashStorage(fsIMUAxisRemap, uint8_t);
FlashStorage(fsIMUAxisSign, uint8_t);
FlashStorage(fsIMUCalParms, adafruit_bno055_offsets_t);
FlashStorage(fshasAnalogPrSensor, uint8_t);
FlashStorage(fsuseExtAD, uint8_t);


//  define serial2 - D2-TX, D3-RX
Uart Serial2 (&sercom2, 3, 2, SERCOM_RX_PAD_1, UART_TX_PAD_2);
void SERCOM2_Handler()
{
  Serial2.IrqHandler();
}

//  define the current/voltage monitor object
Adafruit_INA260       ina260;

//  define the A/D converter object
Adafruit_ADS1115      ads1115;

//  define the pressure sensor object
PAxLD                 pSensor;

// define the serial command processor object
Serial1Command          sCmd;

//  define the IMU object and set the default axis properties
Adafruit_BNO055 imuSensor = Adafruit_BNO055(55);
Adafruit_BNO055::adafruit_bno055_axis_remap_config_t imuRemapConf = Adafruit_BNO055::REMAP_CONFIG_P1;
Adafruit_BNO055::adafruit_bno055_axis_remap_sign_t imuRemapSign = Adafruit_BNO055::REMAP_SIGN_P1;

//  define the RTC object
RTC_DS3231              rtc;

//  define running median objects for sensor data
RunningMedian rawDepth = RunningMedian(NSENSORSAMPS);
RunningMedian rawTemp = RunningMedian(NSENSORSAMPS);
RunningMedian rawSystemVoltage = RunningMedian(NVOLTAGESAMPS);
RunningMedian rawAuxADValue = RunningMedian(NSENSORSAMPS);

//  define the struct to hold the IMU calibration data
adafruit_bno055_offsets_t imuCalParms;

//  define the struct to hold the imu raw data
imu::Quaternion         imuQuat;
imu::Vector<3>          imuVect;


//  define a faster version of DigitalWrite that manipulates the registers directly
//  this is specific to the SAMD21 MCU and alternatives will need to be used if
//  this firmware is used on other Atmel/Arduino platforms
inline void digitalWriteDirect(uint8_t PIN, boolean val){
  if(val)  PORT->Group[g_APinDescription[PIN].ulPort].OUTSET.reg = (1ul << g_APinDescription[PIN].ulPin);
  else     PORT->Group[g_APinDescription[PIN].ulPort].OUTCLR.reg = (1ul << g_APinDescription[PIN].ulPin);
}

//  create an enum for the system shutdown condition
enum controllerStateEnum {
  SLEEP,
  FORCED_ON,
  AT_DEPTH,
  PRESSURE_SW_CLOSED,
  FORCE_ON_REMOVED,
  SHALLOW,
  PRESSURE_SW_OPENED,
  LOW_BATT,
  PC_ERROR,
  EXT_SHUTDOWN
} controllerState;



#endif

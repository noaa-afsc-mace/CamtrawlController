/*
 *  CamtrawlControl is the firmware for the 4th generation of the CamTrawl
 *  power and control board that provides power management and control of the 
 *  CamTrawl underwater stereo camera platform. The 4th gen systems are based
 *  on the NVidia Jetson + ConnectTech Hadron carrier board and power and
 *  control logic is provided by an ARM Cortex M0 based microcontroller.
 *  
 * Dependencies:
 *   Adafruit_Sensor at version ?
 *   Adafruit_BNO055 at version ?
 *   Adafruit_ADS1X15 at version ?
 *   Adafruit INA260
 *   Serial1Command (modified SerialCommand to output to Serial1)
 *   RunningMedian
 *   FlashStorage at version ?
 *   RTClib at version ?
 *
 *   
 *  Rick Towler
 *  Midwater Assesment and Conservation Engineering Group
 *  NOAA Alaska Fisheries Science Center, Seattle, WA
 *  rick.towler@noaa.gov
 */

#include "CamtrawlControl.h"

void setup()
{
  //  set up the IO pins
  pinMode(powerEnable, OUTPUT);
  pinMode(strobeEnable, OUTPUT);
  pinMode(switchedPower, OUTPUT);
  pinMode(externalTrigger, INPUT);
  pinMode(mcu_gpio_1, INPUT_PULLUP);
  pinMode(mcu_gpio_2, OUTPUT);
  pinMode(forceOn, INPUT);
  pinMode(presssureSwitch, INPUT);
  pinMode(strobeTrigOne, OUTPUT);
  pinMode(statusLED, OUTPUT);
  pinMode(Cam2_GPIO1, OUTPUT);
  pinMode(Cam1_GPIO1, OUTPUT);
  pinMode(Cam2_GPIO2, INPUT);
  pinMode(Cam1_GPIO2, INPUT);
  digitalWrite(powerEnable, LOW);
  digitalWrite(strobeEnable, LOW);
  digitalWrite(strobeTrigOne, LOW);
  digitalWrite(switchedPower, LOW);
  digitalWrite(mcu_gpio_2, LOW);
  digitalWrite(statusLED, LOW);
  digitalWrite(Cam2_GPIO1, LOW);
  digitalWrite(Cam1_GPIO1, LOW);
  //  these pins have inverted logic
  digitalWrite(intOrLED, HIGH);
  digitalWrite(intGrLED, HIGH);

  //  set the initial controller state
  controllerState = SLEEP;

  //  start the SBC serial UART
  Serial1.begin(SERIALBAUD);

  //  start the secondary serial UART
  Serial2.begin(AUXSERIALBAUD);

  //  if we're in debug mode, wait 5 seconds to allow time to connect with
  //  a serial console to see the setup debug messages
  #ifdef DEBUGPRINT
  //  start the DEBUG UART
  SerialUSB.begin(SERIALBAUD);
  //  delay a bit to allow time to connect debug console
  delay(5000);
  DEBUG_PRINT("Controller starting up");
  #endif

  //  set up the pin mux for Serial2
  pinPeripheral(2, PIO_SERCOM);
  pinPeripheral(3, PIO_SERCOM_ALT);

  // Set up the callbacks for SerialCommand
  DEBUG_PRINT("Setting up SerialCommand callbacks");
  sCmd.addCommand("trigger", trigger);
  sCmd.addCommand("getState", getSystemState);
  sCmd.addCommand("setState", setSystemState);
  sCmd.addCommand("setPCState", setPCState);
  sCmd.addCommand("setP2DParms", setPressureXDRParms);
  sCmd.addCommand("getP2DParms", getPressureXDRParms);
  sCmd.addCommand("setAuxADParms", setAuxADParms);
  sCmd.addCommand("getAuxADParms", getAuxADParms);
  sCmd.addCommand("setRTC", setRTC);
  sCmd.addCommand("getRTC", getRTC);  
  sCmd.addCommand("setStartDelay", setStartDelay);
  sCmd.addCommand("getStartDelay", getStartDelay);
  sCmd.addCommand("calIMU", calibrateIMU);
  sCmd.addCommand("getIMUCal", getIMUCalParms);
  sCmd.addCommand("setShutdownVoltage", setShutdownVoltage);
  sCmd.addCommand("getShutdownVoltage", getShutdownVoltage);
  sCmd.addCommand("setStartupVoltage", setStartupVoltage);
  sCmd.addCommand("getStartupVoltage", getStartupVoltage);
  sCmd.addCommand("setStrobeMode", setStrobeMode);
  sCmd.addCommand("getStrobeMode", getStrobeMode);
  sCmd.addCommand("setIMUAxConf", setIMUAxisConf);
  sCmd.setDefaultHandler(unknownSerialCommand);

  //  set the serial command delimeter to a comma
  sCmd.setDelimiter(',');
  
  //  load persistent parameters from FLASH
  DEBUG_PRINT("Checking flash for persistent parameters");
  if (fsConfigured.read() == PARMSPROG)
  {
    //  FLASH has been programmed - read the values
    DEBUG_PRINT("Reading initial parameters from flash");

    imuCalibrated = fsIMUCalState.read();
    turnOnDepth = fsTurnOnDepth.read();
    turnOffDepth = fsTurnOffDepth.read();
    P2DSlope = fsP2DSlope.read();
    P2DIntercept = fsP2DIntercept.read();
    TurnOnDelay = fsTurnOnDelay.read();
    A2VSlope = fsA2VSlope.read();
    A2VIntercept = fsA2VIntercept.read();
    P2DLatitude = fsP2DLatitude.read();
    voltageCutoff = fsvoltageCutoff.read();
    startupVoltage = fsvoltageStartup.read();
    imuRemapConf = (Adafruit_BNO055::adafruit_bno055_axis_remap_config_t)fsIMUAxisRemap.read();
    imuRemapSign = (Adafruit_BNO055::adafruit_bno055_axis_remap_sign_t)fsIMUAxisSign.read();
    useExtAD = fsuseExtAD.read();
    
    n = fsmonitorVoltage.read();
    if (n == 1)
      monitorVoltage = true;
    else
      monitorVoltage = false;
    n = fsdepthTurnsOn.read();
    if (n == 1)
      depthTurnsOn = true;
    else
      depthTurnsOn = false;
    n = fsFlashOnPowerup.read();
    if (n == 1)
      flashOnPowerup = true;
    else
      flashOnPowerup = false;
    imuCalParms = fsIMUCalParms.read();
  }
  else
  {
    //  FLASH has not been programmed yet - write defaults to FLASH
    DEBUG_PRINT("Defaults not found, writing defaults to flash");
    fsTurnOnDepth.write(turnOnDepth);
    fsIMUCalState.write(imuCalibrated);
    fsTurnOffDepth.write(turnOffDepth);
    fsP2DSlope.write(P2DSlope);
    fsP2DIntercept.write(P2DIntercept);
    fsTurnOnDelay.write(TurnOnDelay);
    fsA2VSlope.write(A2VSlope);
    fsA2VIntercept.write(A2VIntercept);
    fsvoltageCutoff.write(voltageCutoff);
    fsvoltageStartup.write(startupVoltage);
    fsIMUAxisRemap.write(imuRemapConf);
    fsIMUAxisSign.write((int)imuRemapSign);
    fsP2DLatitude.write((int)P2DLatitude);
    if (flashOnPowerup)
      fsFlashOnPowerup.write(1);
    else
      fsFlashOnPowerup.write(0);
    if (monitorVoltage)
      fsmonitorVoltage.write(1);
    else
      fsmonitorVoltage.write(0);
    if (depthTurnsOn)
      fsdepthTurnsOn.write(1);
    else
      fsdepthTurnsOn.write(0);
    fsIMUCalParms.write(imuCalParms);
    fsuseExtAD.write(useExtAD);
    fsConfigured.write(PARMSPROG);
  }

  //  start the IMU, let it settle, and set it to use the external crystal
  DEBUG_PRINT("Check for IMU...");
  if (imuSensor.begin())
  {
    IMUInstalled = 1;
    DEBUG_PRINT("IMU found");
    
    //  set the axis parameters
    imuSensor.setAxisRemap(imuRemapConf);
    imuSensor.setAxisSign(imuRemapSign);
    delay(15);
    
    //  check if we have any IMU cal data
    if (imuCalibrated > 0)
    {
      //  we have at least some cal data - apply it
      imuSensor.setSensorOffsets(imuCalParms);
      DEBUG_PRINT("IMU Calibration parameters loaded.");
      DEBUG_PRINT(String("IMU calibration state:") + String(imuCalibrated));
      delay(15);
    }

    imuSensor.setExtCrystalUse(true);
    delay(15);
  }
  else
  {
    //  couldn't start the IMU - don't use it
    DEBUG_PRINT("NO IMU found!");
    IMUInstalled = 0;
  }

  DEBUG_PRINT("Check for INA260...");
  if (ina260.begin(i2c_addr=INA260_I2CADDR))
  {
    INAInstalled = 1;
    DEBUG_PRINT("INA260 found.");

    //  set a fast conversion time, but average over 256 samples (~36 ms update interval)
    ina260.setAveragingCount(INA260_COUNT_256);
    ina260.setVoltageConversionTime(INA260_TIME_140_us);
    ina260.setCurrentConversionTime(INA260_TIME_140_us);

    //  wait for the sensor to take first reading
    delay(50);

    //  report first reading
    n = ina260.readCurrent() / 1000.;
    m = ina260.readBusVoltage() / 1000.;
    o = ina260.readPower() / 1000.;
    DEBUG_PRINT(String("  Voltage:") + String(m,2) + String("V  Current:") + String(n,2) + String("A  Power:") + String(o,2) + String("W"));

  }
  else
  {
    INAInstalled = 0;
    DEBUG_PRINT("INA260 NOT found!");
  }


  //  THE CURRENT VERSION OF THE CONTROL BOARD DOES NOT HAVE AN RTC - WE'LL
  //  KEEP THIS CODE FOR NOW IN CASE ONE IS ADDED.

  //  start the RTC
  if (rtc.begin())
  {
    RTCInstalled = 1;
    DEBUG_PRINT("RTC found");
    
    //  check if it lost power
    if (rtc.lostPower())
    {
      //  RTC lost power - set a default date and time - Jan 1 00:00:00 2018
      DEBUG_PRINT("RTC power loss - setting default date 1-1-2018 00:00:00");
      rtc.adjust(DateTime(2018, 1, 1, 0, 0, 0));
    }
  }
  else
  {
    //  couldn't start the RTC - don't use it
    RTCInstalled = 0;
    DEBUG_PRINT("NO RTC installed!");
    
    //  zero out TurnOnDelay since we don't have an RTC
    TurnOnDelay = 0;
  }

  //  start the A/D converter
  ads1115.begin();
  
  //  and set the gain
  ads1115.setGain(GAIN_ONE);

  //  set up the pressure sensor
  //  compute X value used in computing gravity variation with latitude
  P2DX = sin(abs(P2DLatitude) * (PI / 180.0));
  P2DX = P2DX * P2DX;

  //  try to initialize the PAxLD 12c sensor
  DEBUG_PRINT("Looking for a PA4LD sensor...");
  bool ok = pSensor.init();
  if (ok)
  {
    pXDCRInstalled = 1;
    DEBUG_PRINT("Found PA4LD. Getting initial readings...");
    DEBUG_PRINT(String("  Pmin:") + String(pSensor.pMin,4) + String("  pMax:") + String(pSensor.pMax,4));
    
    //  get the initial readings
    n = 1;
    while (n <= NSENSORSAMPS)
    {
      pSensor.pollSensor();
      delay(15);
      pSensor.convertData();
      rawDepth.add(pSensor.pressure);
      rawTemp.add(pSensor.temperature);
      ++n;
      delay(15);
    }
    //  update the external temp value
    externalTemp = rawTemp.getAverage(3);

    //  convert pressure to depth
    depth = pressureToDepth(rawDepth.getAverage(3));
    //  and apply corrections
    depth = depth * P2DSlope + P2DIntercept;
  }
  else
  {
    if (useExtAD)
    {
      //  no i2c pressure sensor and we're using the 2nd A/D channel for logging an external voltage
      DEBUG_PRINT("Unable to initialize PA4LD. Analog input is configured for external use. No pressure sensor available.");
      
      //  no pressure transducer installed - set the depth and ext termp values to -999.9
      pXDCRInstalled = 0;      
    }
    else
    {
      DEBUG_PRINT("Unable to initialize PA4LD. Looking for analog depth sensor");
      n = 1;
      while (n <= NSENSORSAMPS)
      {
        rawDepth.add(ads1115.readADC_SingleEnded(3));
        ++n;
        delay(15);
      }
      //  analog sensor does not provide temp
      externalTemp = -999.9;
  
      //  get the raw depth value
      depth = rawDepth.getAverage(3);
  
      //  check if it exceeds our minimum threshold
      if (depth < ANALOGDEPTHMIN)
      {
        DEBUG_PRINT("Unable to find analog depth sensor.");
        pXDCRInstalled = 0;
      }
      else
      {
        //  found an analog input, assume analog pressure transducer
        pXDCRInstalled = 2;
        DEBUG_PRINT("Analog depth sensor found.");
        
        //  get the mean of the 3 median values and convert to depth
        depth = (rawDepth.getAverage(3) * P2DSlope) + P2DIntercept;
      }
    }
  }

  if (pXDCRInstalled == 0)
  {
    //  no pressure transducer installed - set the depth and ext termp values to -999.9
    depth = -999.9;
    externalTemp = -999.9;
    
    //  if we don't have a pressure transducer we set the turn on/off
    //  depths to values that will cause the depth check to never change
    //  the system state.
    turnOnDepth = 255;
    turnOffDepth = -255;
  }

  //  check if we should initialize the external analog value
  if (useExtAD)
  {
    //  initialize external analog value
    n = 1;
    while (n <= NSENSORSAMPS)
    {
      rawAuxADValue.add(ads1115.readADC_SingleEnded(3));
      ++n;
      delay(15);
    }

    //  get the mean of the 3 median values and convert to the output value
    auxADRaw = rawAuxADValue.getAverage(3);
    auxADVal = (auxADRaw * A2VSlope) + A2VIntercept;
  }

  //  get the initial voltage values
  DEBUG_PRINT("Getting initial system voltage readings...");
  n = 1;
  while (n <= NVOLTAGESAMPS)
  {
    //  read the system and external voltages
    rawSystemVoltage.add(ads1115.readADC_SingleEnded(2));
    ++n;
    delay(50);
  }
  //  get the mean of the 3 median values and convert to a voltage
  systemVoltage = rawSystemVoltage.getAverage(3) * sysVConv;

  //  initialize the voltage samping interval counter
  vSampCounter = millis();

  //  and the sensor samping interval counter
  sSampCounter = micros();
  
  //  and the sleep interval counter
  sleepCounter = millis();

  //  set up the depth checking delay
  o = millis();

#ifdef DEBUGPOWER
  DEBUG_PRINT(String("Debug power enabled. Turning on SBC power"));
  digitalWrite(powerEnable, HIGH);
  //  we also force checkPCTimeout to false since we don't want the
  //  controller to bail if we're debugging PC boot and software
  //  startup issues
  checkPCTimeout = false;
#endif

  DEBUG_PRINT("Setup complete. Entering main application loop");  
}


void loop()
{

  //  report the controller state
  DEBUG_PRINT(String("Controller state:") + String(controllerState));

  //  sample the system sensors
  sampleSensors();

  //  check for any serial commands
  sCmd.readSerial();

  //  report the system depth
  if (pXDCRInstalled)
    DEBUG_PRINT(String("depth: ") + String(depth, 2));

  //  check the system voltage - we will not start if the system
  //  voltage is below the cutoff voltage
  DEBUG_PRINT(String("Current system voltage value:") + String(systemVoltage));
  if (monitorVoltage)
  {
    //  check if the systemVoltage is below the cutoff
    if (systemVoltage < startupVoltage)
    {
      //  it is - park the system
      DEBUG_PRINT(String("System input voltage below startup threshold of ") + String(startupVoltage));
      DEBUG_PRINT(String("Parking system..."));
      parkSystem();
    }
  }

  DEBUG_PRINT("Ext Shut:" + String(extShutdownState));

  //  check if we should remain in an offline state (ext shutdown is high)
  while (extShutdownState)
  {
    greenLEDOn();
    DEBUG_PRINT("Controller in standby due to external sutdown signal");
    delay(1000);
    greenLEDOff();
    extShutdownState = digitalRead(externalShutdown);
    if (!extShutdownState)
    {
      //  debounce
      delay(500);
      extShutdownState = digitalRead(externalShutdown);
      if (!extShutdownState)
        //  we can now turn back on
        controllerState = SLEEP;
    }
    else
    {
      delay(1000);
      redLEDOn();
      delay(250);
      redLEDOff();
      delay(1000);
    }
  }

  //  check if we should check if we're at depth - we delay this
  //  check for a short time to allow systems with incorrectly set or
  //  failing pressure sensors to be started in download/service mode.
  //  Some camera systems do not have wired ethernet connections and
  //  will shut down their WiFi when in deployed mode so if you don't
  //  give the user some time to set the mode, they can get locked out
  //  and would have to open up the system and access it directly.
  if ((canCheckDepth == false) && ((millis() - o) > DEPTHCHKDELAY))
    canCheckDepth = true;
  
  //  check if we've been forced on
  if (forceOnState)
  {
    //  wait here and check again (crude debounce)
    n = millis();
    while ((millis() - n) < 1000);
    if (forceOnState)
    {
      //  system forced on
      controllerState = FORCED_ON;
      DEBUG_PRINT("System forced on");
    }
  }
  //  check if we should turn on because we're at depth
  else if (canCheckDepth && (pSwitchState || (depthTurnsOn && (depth >= turnOnDepth))))
  {
    //  wait here and check again (crude debounce)
    n = millis();
    while ((millis() - n) < 1000);
    if ((pSwitchState) || (depth >= turnOnDepth))
    {
      //  system at depth
      if (pSwitchState)
      {
        controllerState = PRESSURE_SW_CLOSED;
        DEBUG_PRINT("Pressure switch closed.");
      }
      else
      {
        controllerState = AT_DEPTH;
        DEBUG_PRINT("Depth >= turn on depth");
      }
    }
  }
  
  //  check if we should power up
  if (controllerState > SLEEP)
  {
    //  check if we should delay power up. We only do this if we're
    //  at depth. We don't delay if we've been forced on.
    DEBUG_PRINT(String("Turn on delay:") + String(TurnOnDelay));
    if ((TurnOnDelay > 0) && (controllerState == 2))
    {
      //  set up the red and green LEDs to alternate while waiting to turn on
      greenLEDOff();
      redLEDOn();
      turnOnTime = rtc.now() + TimeSpan(TurnOnDelay);
      delaySpan = turnOnTime - rtc.now();
      DEBUG_PRINT(String("Delaying power up by ") + String(delaySpan.totalseconds()) + String(" seconds"));
      while (delaySpan.totalseconds() > 0)
      {
        delay(1000);
        delaySpan = turnOnTime - rtc.now();
        toggleRedLED();
        toggleGreenLED();
      }  
    }
    
    //  we're in an operational state
    DEBUG_PRINT("Entering operational state");
    
    //  set the status LED solid green
    redLEDOff();
    greenLEDOn();

    //  turn on the strobe power supply if required
    if (strobeMode > 0)
    {
      DEBUG_PRINT("Turning on Strobe power");
      digitalWrite(strobeEnable, HIGH);
      //  pause to let the power settle
      delay(1000);

      //  trigger the strobes to signal that we're ready to go
      if (flashOnPowerup)
      {
        n = 0;
        DEBUG_PRINT("Flashing strobes to indicate system is ready to operate");
        while (n < NPOWERONFLASHES)
        {
          //  trigger strobe channel 1
          digitalWriteDirect(strobeTrigOne, HIGH);
          delay(2);
          digitalWriteDirect(strobeTrigOne, LOW);
  
          //  pause
          delay(500);
  
          //  increment counter
          ++n;
        }
      }
    }

#ifdef DEBUGPOWER
    //  nothing to do since SBC power is already on
#else
    //  turn on the PC power supply
    DEBUG_PRINT("Turning on SBC power");
    digitalWrite(powerEnable, HIGH);
#endif

    //  turn on AUX 5v (Udoo x86 systems only)
    DEBUG_PRINT("Turning on Aux 5v power");
    digitalWrite(switchedPower, HIGH);

    
    //  wait here until the PC responds that it is ready. We wait either
    //  until the PC sends the ready signal (PCState variable changes) or
    //  until we timeout.
    PCState = 0;
    n = millis();
    m = millis();
    o = millis();
    DEBUG_PRINT("Waiting for the SBC to signal it is ready to acquire");
    
    sCmd.clearBuffer();
    
    //  note that we only timeout when the system is "at depth".
    while ((PCState == 0) && (((millis() - m) < PCBOOTTIMEOUT) || (controllerState == FORCED_ON)))
    {
      //  check for serial commands
      sCmd.readSerial();

      //  poll/read-out system sensors
      sampleSensors();

      //  toggle the state LED
      if (millis() - n > 120)
      {
        toggleGreenLED();
        n = millis();
      }

      //  check the system voltage
      if (millis() - o > VOLTSAMPINT)
      {
        //  check the system voltage - shutdown if the system voltage is below the cutoff voltage
        DEBUG_PRINT(String("Current system voltage value:") + String(systemVoltage));
        if (monitorVoltage)
          //  check if the systemVoltage is below the cutoff
          if (systemVoltage < voltageCutoff)
            //  it is - park the system
            parkSystem();
        o = millis();
      }

      //  check if we timed out waiting
      if (checkPCTimeout && ((millis() - m) >= PCBOOTTIMEOUT) && (controllerState != FORCED_ON))
      {
        //  we timed out - bail and park
        DEBUG_PRINT("Timed out waiting for SBC - shutting down");
        
        //  turn off strobes, SBC, and aux power
#ifdef DEBUGPOWER
        //  nothing to do since we aren't shutting down SBC power
        DEBUG_PRINT("Debug power enabled. Turning off strobes and AUX power only");
#else
        //  turn off the SBC power supply
        DEBUG_PRINT("Turning off SBC, strobes and AUX power.");
        digitalWrite(powerEnable, LOW);
#endif

        digitalWrite(switchedPower, LOW);
        turnOffStrobes();
  
        //  flash RED LED to indicate boot failure
        greenLEDOff();
        n = 0;
        while (n < 30)
        {
          toggleRedLED();
          delay(250);
          ++n;
        }
  
        //  and park the system
        parkSystem();
      }
    }

    //  set the status LED solid green to indicate that we're ready
    greenLEDOn();

    //  at this point the system will be up and running. We now enter a loop 
    //  processing serial commands until our operational state changes because
    //  we're shallower than the turn-off depth, or the pressure switch opened,
    //  or the force on circuit is open.
    DEBUG_PRINT("System is operational");
    n = millis();
    acquiring = true;
    
    while (acquiring)
    {

      //  poll/read-out system sensors
      sampleSensors();
      
      //  check for serial commands
      sCmd.readSerial();

      //  check if we're supposed to stop acquiring
      checkControllerState();
      if (controllerState >= FORCE_ON_REMOVED)
      {
        //  we're suppose to stop acquiring
        acquiring = false;

        //  transmit the new state to the PC so it knows it has to shutdown
        getSystemState();

        //  report the shutdown reason and set flags if required
        switch (controllerState)
        {
          case FORCE_ON_REMOVED:
            // force on plug removed
            DEBUG_PRINT("System is shutting down: force-on plug being removed");
            break;
          case SHALLOW:
            // system shallower than turn off depth
            DEBUG_PRINT("System is shutting down: system above turn-off depth");
            break;
          case PRESSURE_SW_OPENED:
            // pressure switch opened
            DEBUG_PRINT("System is shutting down: pressure switch opened");
            break;
          case LOW_BATT:
            // low battery
            DEBUG_PRINT("System is shutting down due to low battery voltage");
            //  set the low voltage flag so the system doesn't try to restart
            //  at depth.
            lowVoltage = true;
            break;
          case PC_ERROR:
            // shutdown due to software/PC issue
            DEBUG_PRINT("System is shutting down do to PC/Acquisition software error");
            //  set the low voltage flag so the system doesn't start back up until
            //  power is cycled. No need to restart if there is a hardware/PC problem
            //  (e.g. cameras not responding, storage full, etc.)
            lowVoltage = true;
            break;
          case EXT_SHUTDOWN:
            // shutdown due to external 
            DEBUG_PRINT("System is shutting down as external shutdown signal is high");
            break;
        }
      }

#ifdef DEBUGPRINT
      //  output some sensor data when debugging
      if (millis() - n > SENSORDEBUGINT)
      {
        DEBUG_PRINT(String("Current system state:") + String(controllerState));
        DEBUG_PRINT(String("Current system voltage value:") + String(systemVoltage));
        n = millis();
      }
#endif
    }

    //  turn off the green LED
    greenLEDOff();

    //  turn off the strobe power supply
    DEBUG_PRINT("Turning off Strobe power");
    turnOffStrobes();

    //  wait for the PC to signal shutdown is imminent
    DEBUG_PRINT("Waiting for the PC to signal it is ready to shut down");
    redLEDOn();
    n = millis();
    m = n;
    while ((PCState != 0) && (millis() - m < PCOFFTIMEOUT))
    {
      //  check for serial commands
      sCmd.readSerial();

      //  toggle the state LED
      if (millis() - n > 150)
      {
        toggleRedLED();
        n = millis();
      }
    }

    //  turn on the red LED solid
    DEBUG_PRINT("PC signaled (or timed out) waiting 20 seconds then turning off PC power");
    redLEDOn();

    //  now wait for a bit before turning off power to PC
    delay(PCPOWWAITINT);

    //  turn off the SBC power supply
#ifdef DEBUGPOWER
      //  nothing to do since we aren't shutting down SBC power
#else
      //  turn off the SBC power supply
      DEBUG_PRINT("Turning off SBC power");
      digitalWrite(powerEnable, LOW);
#endif

    //  turn off AUX 5v (Udoo x86 systems only)
    DEBUG_PRINT("Turning off Aux 5v power");
    digitalWrite(switchedPower, LOW);


    //  check if the IMU has improved it's calibration
    DEBUG_PRINT("Checking if we need to update the IMU cal params");
    DEBUG_PRINT(String("Current calibration state:") + String(imuCalibrated));
    if (imuCalSys >= imuCalibrated && imuCalGyr >= imuCalibrated && 
        imuCalAcc >= imuCalibrated && imuCalMag >= imuCalibrated &&
        imuCalibrated < 3)
    {
      //  at least one sensor has improved and none are worse
      //  get the IMU calibration parameters
      imuSensor.getSensorOffsets(imuCalParms);
      //  write to flash
      fsIMUCalParms.write(imuCalParms);

      //  update the imuCalibrated state - first get the lowest cal value
      imuCalibrated = imuCalSys;
      if (imuCalibrated < imuCalGyr)
        imuCalibrated = imuCalGyr;
      if (imuCalibrated < imuCalAcc)
        imuCalibrated = imuCalAcc;
      if (imuCalibrated < imuCalMag)
        imuCalibrated = imuCalMag;
      //  now write it to flash
      fsIMUCalState.write(imuCalibrated);

      DEBUG_PRINT("IMU Calibration parameters updated.");
      DEBUG_PRINT(String("New calibration state:") + String(imuCalibrated));
    }

    redLEDOff();

    //  check if we're parking...
    if ((controllerState == LOW_BATT) || (controllerState == PC_ERROR))
    {
      DEBUG_PRINT("Parking system...");
      parkSystem();
    }

    //  check if we hold due to external shutdown
    if((controllerState == EXT_SHUTDOWN))
    {
     //  the controller will hold here until the ext shutdown signal goes low
      while (controllerState == EXT_SHUTDOWN)
      {
        greenLEDOn();
        DEBUG_PRINT("Controller in standby due to external sutdown signal-1");
        delay(1000);
        greenLEDOff();
        extShutdownState = digitalRead(externalShutdown);
        if (!extShutdownState)
        {
          //  debounce
          delay(500);
          extShutdownState = digitalRead(externalShutdown);
          if (!extShutdownState)
            //  we can now turn back on
            controllerState = SLEEP;
        }
        delay(1000);
        redLEDOn();
        delay(1000);
        redLEDOff();
        delay(1000);
      }
    }
    else
    {
      controllerState = SLEEP;
      DEBUG_PRINT("Controller is back in sleep mode");
    }
  }

  //  toggle the green status LED while we're in standby
  greenLEDOn();
  delay(250);
  greenLEDOff();
  delay(2000);

}


/*
 * checkControllerState determines if the system should stop collecting
 * and shut down. It alters the controllerState value, setting it to one
 * of the following values based on the reason the system is shutting down.
 * 
 *   4 - force on plug removed
 *   5 - shallower than turn-off depth
 *   6 - pressure switch opened
 *   7 - low battery
 *   8 - PC told us to shut down (usually PC error)
 *   9 - External shutdown signal
 *   
 */
void checkControllerState()
{
  
  //  if we're forced on, check if the force-on plug has been removed
  if ((controllerState == FORCED_ON) && (!forceOnState))
  {
    // force on plug removed - debounce...
    n = millis();
    while ((millis() - n) < 1000);
    //  and check again
    if ((controllerState == FORCED_ON) && (!forceOnState))
      //  force-on plug removed - shutdown
      controllerState = FORCE_ON_REMOVED;
  }
  
  //  if we're "at-depth" - check if we've come to the surface
  if (controllerState == AT_DEPTH)
  {
    //  check if we're using depth to turn on/off and if we're above turn off depth
    if (depthTurnsOn && (depth <= turnOffDepth))
      //  we're above turn-off depth - shutdown
      controllerState = SHALLOW;
  }

  //  same as above, but with the pressure switch
  if (controllerState == PRESSURE_SW_CLOSED)
  {
    //  check if the pressure switch has opened
    if (!pSwitchState)
    {
      // pressure switch opened - debounce...
      n = millis();
      while ((millis() - n) < 1000);
      //  and check again
      if (!pSwitchState)
        //  pressure switch still open - shutdown
        controllerState = PRESSURE_SW_OPENED;
    }
  }
  //  check if the acquisition PC has told us to shutdown
  if (PCState == 254)
  {
    //  the acquisition PC has told us to shutdown
    controllerState = PC_ERROR;
  }
    
  //  if we're monitoring voltage check if we need to shut down
  if (monitorVoltage && (systemVoltage < voltageCutoff))
  {
    //  low system voltage detected - shutdown
    controllerState = LOW_BATT;
  }

  if (extShutdownState)
  {
    // external shutdown signal received - debounce...
    n = millis();
    while ((millis() - n) < 1000);
    //  and check again
    if (extShutdownState)
      //  an external signal is telling us to shut down
      controllerState = EXT_SHUTDOWN;
  }
}


/*
 * parkSystem is called after a low battery or PC error shutdown
 * or when the system is started with a low battery. The system
 * will stay parked until power is cycled.
 */
void parkSystem()
{
  while (true)
  {
    DEBUG_PRINT("Controller is in low battery/error mode.");
    greenLEDOff();
    redLEDOff();
    delay(5000);
    redLEDOn();
    delay(60);
    redLEDOff();
  } 
}


/*
 * sampleSensors polls or reads and transmits data from the attitude and
 * pressure sensors. It also tracks the state of the force-on and pressure
 * switch pins.
 * 
 * The i2c based PA4LD pressure sensor must be commanded to take a sample
 * before you can read out the data. The conversion takes up to 8ms so this
 * function is broken up into a polling block and a reading and sending
 * block and it toggles between the two. 
 * 
 * This function tracks the microsecond timer and returns immediately until
 * the counter exceeds our half-sampling interval. It then either polls 
 * or reads based on the last action taken. This results in an output rate
 * that is 2x the specified half-sampling interval. Note that since the
 * PA4LD takes up to 8ms to generate a reading, the half-sampling interval
 * should not be less than 4000 us. This is excessive anyways. A more
 * reasonable sampling rate is 10 Hz = 100 ms interval = 100000 us / 2
 * yielding a 50000 us half-sampling interval.
 */
void sampleSensors()
{
  //  check if it's time to poll or read and report
  if ((micros() - sSampCounter) > SENSORSAMPINT)
  {    
    //  check if we're polling or reading this round
    if (pollSensors)
    {
      //  we're polling - poll sensors that require polling
      if (pXDCRInstalled == 1)
        //  poll the i2c pressure sensor
        pSensor.pollSensor();
  
      //  update the state of the force-on, pressure switch, and external shutdown pins
      forceOnState = !digitalRead(forceOn);
      pSwitchState = !digitalRead(presssureSwitch);
      extShutdownState = digitalRead(externalShutdown);
      
      //  toggle the polling state
      pollSensors = false;

      //  update the counter value
      sSampCounter = micros();
    }
    else
    {
      //  we're reading - read out the values and update
      if (IMUInstalled)
      {
        //  get the orientation data
        
        // https://forums.adafruit.com/viewtopic.php?f=25&t=108290
        //imuQuat = imuSensor.getQuat();
        //imuQuat.normalize();
        //float temp = imuQuat.x();  imuQuat.x() = -imuQuat.y();  imuQuat.y() = temp;
        //imuQuat.z() = -imuQuat.z();
        //imuVect = imuQuat.toEuler();
        //yaw = -RADEG * imuVect.x();
        //pitch = -RADEG * imuVect.z();
        //roll = -RADEG * imuVect.y();

        imuVect = imuSensor.getVector(Adafruit_BNO055::VECTOR_EULER);
        yaw = imuVect.x();
        pitch = -imuVect.z();
        roll = imuVect.y();
    
        //  and the internal temp
        internalTemp = imuSensor.getTemp();
      
        //  get the linear accelleration data
        imuVect = imuSensor.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
        accelX = imuVect.x();
        accelY = imuVect.y();
        accelZ = imuVect.z();      
      }
      if (pXDCRInstalled == 1)
      {
        // i2c sensor installed - readout and convert 
        pSensor.convertData();
        rawDepth.add(pSensor.pressure);
        rawTemp.add(pSensor.temperature);
  
        //  update the external temp value
        externalTemp = rawTemp.getAverage(3);
  
        //  convert pressure to depth
        depth = pressureToDepth(rawDepth.getAverage(3));
        
        //  and apply corrections
        depth = depth * P2DSlope + P2DIntercept;
        
      }
      else if (pXDCRInstalled == 2)
      {
        //  analog pressure transducer installed - read the A/D
        rawDepth.add(ads1115.readADC_SingleEnded(3));
  
        //  get the mean of the 3 median values
        depth = rawDepth.getAverage(3);
  
        //  and convert to depth
        depth = (depth * P2DSlope) + P2DIntercept;
      }

      if (acquiring)
      {
        //  send the attitude sensor data string
        //  $OHPR, <yaw>, <pitch>, <roll>, <external temp c>, <depth>, <linear acc X>, <linear acc Y>, <linear acc Z>/n
        Serial1.print("$OHPR,");
        Serial1.print(yaw, 1); Serial1.print(",");
        Serial1.print(pitch, 1); Serial1.print(",");
        Serial1.print(roll, 1); Serial1.print(",");
        Serial1.print(externalTemp, 2); Serial1.print(",");
        Serial1.print(depth, 2); Serial1.print(",");
        Serial1.print(accelX, 4); Serial1.print(",");
        Serial1.print(accelY, 4); Serial1.print(",");
        Serial1.println(accelZ, 4);
      }
      
      //  check if we should update the system voltage and IMU status
      if ((millis() - vSampCounter) > VOLTSAMPINT)
      {
        //  read the system and external voltages
        rawSystemVoltage.add(ads1115.readADC_SingleEnded(2));
      
        //  get the mean of the 3 median values and convert to a voltage
        sysVRaw = rawSystemVoltage.getAverage(3);
        systemVoltage = sysVRaw * sysVConv;
  
        if (IMUInstalled)
          //  read the IMU calibration state
          imuSensor.getCalibration(&imuCalSys, &imuCalGyr, &imuCalAcc, &imuCalMag);

        //  if we aren't configured with an analog pressure sensor, sample the 2nd A/D channel
        if (useExtAD)
        {
          //  read A/D
          rawAuxADValue.add(ads1115.readADC_SingleEnded(3));
    
          //  get the mean of the 3 median values and convert to the output value
          auxADRaw = rawAuxADValue.getAverage(3);
          auxADVal = (auxADRaw * A2VSlope) + A2VIntercept;
        }

        if (acquiring)
        {
          //  send the system voltage and internal temp message
          //$CTSV, <raw system voltage>, <system voltage>, <internal temp>/n
          Serial1.print("$CTSV,");
          Serial1.print(sysVRaw, 4); Serial1.print(",");
          Serial1.print(systemVoltage, 4); Serial1.print(",");
          Serial1.println(internalTemp, 2);
        
          //  send the IMU calibration status
          /*
           * $IMUC, <System Cal>, <Gyro Cal>, <Accelerometer Cal>, <Magnetomoeter Cal>/n
           *The values range from 0 to 3 where 0 is uncalibrated and 3 is fully calibrated.
           * You should not use data from a sensor if the cal value is 0
           */
          Serial1.print("$IMUC,");
          Serial1.print(imuCalSys); Serial1.print(",");
          Serial1.print(imuCalGyr); Serial1.print(",");
          Serial1.print(imuCalAcc); Serial1.print(",");
          Serial1.println(imuCalMag);

          if (useExtAD)
          {
            //  send the Aux A/D data string
            //  $AUXV,<raw A/D value>, <converted value>/n
            Serial1.print("$AUXV,");
            Serial1.print(auxADRaw, 4); Serial1.print(",");
            Serial1.println(auxADVal, 4);
          }
        }
 
        //  reset iTime
        vSampCounter = millis();
      }

      //  toggle the polling state
      pollSensors = true;
    }
    
    //  update the counter value
    sSampCounter = micros();
  }  
}


/*
 * trigger is called when the trigger command is received. This command triggers
 * the cameras and strobes using the provided parameters. All times are in microseconds.
 * The command is in the form:
 * 
 * trigger,<strobe pre fire as int>,<strobe 1 exposure as int>,<strobe 2 exposure as int>,<left cam trig as int>,<right cam trig as int>\n
 * 
 * positive strobe pre-fire values delay the camera triggering. Set to 0 to trigger the cameras first
 * set left/right trigger values to 0 to not trigger that camera
 * set strobe duration to 0 to not trigger that channel
 * 
 */
void trigger()
{
  char          *arg;
  bool          leftTrig;
  bool          rightTrig;
  int           preFire;
  int           strobe1Exp;
  int           strobe2Exp;
  int           delay1us;
  int           delay2us;
  int           strobeOrder;

  //  turn the status LEDs off
  greenLEDOff();
  redLEDOff();

  //  read in the strobe pre fire value and convert to int
  arg = sCmd.next();
  preFire = atoi(arg);
  
  //  read in the strobe channel 1 exposure value and convert to int
  arg = sCmd.next();
  strobe1Exp = atoi(arg);

  //  read in the strobe channel 2 exposure value and convert to int
  arg = sCmd.next();
  strobe2Exp = atoi(arg);

  //  read in the left camera trigger value and convert to bool
  arg = sCmd.next();
  leftTrig = (atoi(arg) > 0);

  //  read in the right camera trigger value and convert to bool
  arg = sCmd.next();
  rightTrig = (atoi(arg) > 0);

  DEBUG_PRINT(String("TRIGGERED: ") + String(preFire) + String(",") + \
    String(strobe1Exp) + String(",") + String(strobe2Exp) + String(",") + \
    String(leftTrig) + String(",") + String(rightTrig) + String(","));

  //  determine if we need to trigger the strobe or not
  if (strobeMode == 1)
  {
    //  we are triggering the strobes
    
    //  set up the delays - The constants below account for the
    //  execution speed and adjust the delays such that the total
    //  strobe pulse width is correct.
    delay1us = strobe1Exp - 26;

    //  ensure that we have delays >= 0
    if (delay1us < 0)
      delay1us = 0;
  
    //  triggering order depends on if we have a positive or negative strobe pre-fire
    if (preFire > 0)
    {
      //  if we're triggering strobes before the cameras, we need to subtract the camera
      //  trigger signal length from the strobe duration to ensure the strobe trigger
      //  signal is the commanded length.
      delay1us -= TRIGDURUS;
      
      //  with a positive pre-fire we trigger the strobes first
      if (strobe1Exp > 0)
        digitalWriteDirect(strobeTrigOne, true);

      //  delay for the pre-fire period
      delayMicroseconds(preFire);

      //  trigger the cameras
      if (leftTrig)
        digitalWriteDirect(Cam2_GPIO1, true);
      if (rightTrig)
        digitalWriteDirect(Cam1_GPIO1, true);
      delayMicroseconds(TRIGDURUS);
      digitalWriteDirect(Cam2_GPIO1, false);
      digitalWriteDirect(Cam1_GPIO1, false);
    }
    else
    {
      //  with a 0 pre-fire we trigger the cameras first - this would be used with xenon strobes

      //  trigger the cameras
      if (leftTrig)
        digitalWriteDirect(Cam2_GPIO1, true);
      if (rightTrig)
        digitalWriteDirect(Cam1_GPIO1, true);
      delayMicroseconds(TRIGDURUS);
      digitalWriteDirect(Cam2_GPIO1, false);
      digitalWriteDirect(Cam1_GPIO1, false);

      if (strobe1Exp > 0)
        digitalWriteDirect(strobeTrigOne, true);
    }

    //  delay for the exposure length and bring the strobe trigger low
    delayMicroseconds(delay1us);
    digitalWriteDirect(strobeTrigOne, false);
  }
  else
  {
    //  strobes are either in constant on mode or are off so we don't
    //  need to handle strobe triggering here.

    //  trigger the cameras
    if (leftTrig)
      digitalWriteDirect(Cam2_GPIO1, true);
    if (rightTrig)
      digitalWriteDirect(Cam1_GPIO1, true);
    delayMicroseconds(TRIGDURUS);
    digitalWriteDirect(Cam2_GPIO1, false);
    digitalWriteDirect(Cam1_GPIO1, false);
  }

  //  turn the green LED back on
  greenLEDOn();

}


/*
 * setAuxADParms is called when the setAuxADParms command is received.
 * The command is in the form:
 * 
 * setAuxADParms,<enable as Int>,<slope as float>,<intercept as float>\n
 * 
 * Set enable=1 to use the auxilary A/D input for something other than
 * an analog pressure transducer that provides the system depth. Do not
 * enable this if you are using an analog pressure transducer to provide
 * the system depth.
 * 
 */
void setAuxADParms()
{

  char    *arg;

  DEBUG_PRINT("setAuxADParms command received");

  //  determine if we're enabled or not
  arg = sCmd.next();
  useExtAD = (atoi(arg) == 1) ? true : false;

  //  store in flash
  fsuseExtAD.write(useExtAD);
  
  //  read in the slope value and convert to float
  arg = sCmd.next();
  A2VSlope = atof(arg);
  
  //  store in flash
  fsA2VSlope.write(A2VSlope);

  //  read in the intercept value and convert to float
  arg = sCmd.next();
  A2VIntercept = atof(arg);

  //  store in flash
  fsA2VIntercept.write(A2VIntercept);

  Serial1.println("setAtoDParms,OK");

}


/*
 * getAuxADParms is called when the getAuxADParms command is received. This command
 * returns the current auxillary A/D conversion parameters.
 * The command is in the form:
 * 
 * getAuxADParms\n
 * 
 * The return string is in the form:
 * 
 * getAuxADParms,<slope as float>,<intercept as float>\n
 * 
 */
void getAuxADParms()
{

  DEBUG_PRINT("getAuxADParms command received");

  //  send the response string 
  Serial1.print("getAuxADParms,");
  if (useExtAD)
    Serial1.print("1,");
  else
    Serial1.print("0,");
  Serial1.print(A2VSlope,6);
  Serial1.print(",");
  Serial1.println(A2VIntercept,6);

}


/*
 * setPressureXDRParms is called when the setP2D command is received.
 * The command is in the form:
 * 
 * setP2DParms,<slope as float>,<intercept as float>,<turn on depth as int>,<turn off depth as int>,<P2D Lat as float>\n
 * 
 */
void setPressureXDRParms()
{

  char    *arg;

  DEBUG_PRINT("setP2DParms command received");

  //  read in the slope value and convert to float
  arg = sCmd.next();
  P2DSlope = atof(arg);
  
  //  store in flash
  fsP2DSlope.write(P2DSlope);

  //  read in the intercept value and convert to float
  arg = sCmd.next();
  P2DIntercept = atof(arg);

  //  store in flash
  fsP2DIntercept.write(P2DIntercept);

  //  read in the slope value and convert to int
  arg = sCmd.next();
  turnOnDepth = atoi(arg);
  
  //  store in flash
  fsTurnOnDepth.write(turnOnDepth);

  //  read in the intercept value and convert to int
  arg = sCmd.next();
  turnOffDepth = atoi(arg);

  //  store in flash
  fsTurnOffDepth.write(turnOffDepth);

//  read in the pressure to depth latitude value and convert to float
  arg = sCmd.next();
  P2DLatitude = atof(arg);
  
  //  store in flash
  fsP2DLatitude.write(P2DLatitude);

  Serial1.println("setP2DParms,OK");

}


/*
 * getPressureXDRParms is called when the getP2D command is received. This command
 * returns the current pressure transducer related parameters
 * The command is in the form:
 * 
 * getP2DParms\n
 * 
 * The return string is in the form:
 * 
 * getP2DParms,<mode as int>,<slope as float>,<intercept as float>,<turn on depth as int>,<turn off depth as int>,<P2D Lat as float>\n
 * 
 */
void getPressureXDRParms()
{

  DEBUG_PRINT("getP2DParms command received");

  //  send the response string 
  Serial1.print("getP2DParms,");
  Serial1.print(pXDCRInstalled);
  Serial1.print(",");
  Serial1.print(P2DSlope,6);
  Serial1.print(",");
  Serial1.print(P2DIntercept,6);
  Serial1.print(",");
  Serial1.print(turnOnDepth);
  Serial1.print(",");
  Serial1.print(turnOffDepth);
  Serial1.print(",");
  Serial1.print(P2DLatitude);
  Serial1.print("\n");

}


/*
 * setIMUAxisConf is called when the setIMUAxConf command is received.
 * The first and only parameter is an integer representing the 8 possible
 * mounting orientations of the IMU as shown on pg. 25 of the BOSCH BNO055
 * datasheet v 1.2. The integer matches the integer after the "P" in the
 * axis configuration parameters table.
 * 
 * The command is in the form:
 * 
 * setIMUAxConf,<config>\n
 * 
 */
void setIMUAxisConf()
{
  char    *arg;
  int     remapVal;

  DEBUG_PRINT("setIMUAxConf command received");
  
  //  read in the slope value and convert to int
  arg = sCmd.next();
  remapVal = atoi(arg);

  switch (remapVal)
  {
    case 0:
      imuRemapConf = Adafruit_BNO055::REMAP_CONFIG_P0;
      imuRemapSign = Adafruit_BNO055::REMAP_SIGN_P0;
      break;
    case 1:
      imuRemapConf = Adafruit_BNO055::REMAP_CONFIG_P1;
      imuRemapSign = Adafruit_BNO055::REMAP_SIGN_P1;
      break;
    case 2:
      imuRemapConf = Adafruit_BNO055::REMAP_CONFIG_P2;
      imuRemapSign = Adafruit_BNO055::REMAP_SIGN_P2;
      break;
    case 3:
      imuRemapConf = Adafruit_BNO055::REMAP_CONFIG_P3;
      imuRemapSign = Adafruit_BNO055::REMAP_SIGN_P3;
      break;
    case 4:
      imuRemapConf = Adafruit_BNO055::REMAP_CONFIG_P4;
      imuRemapSign = Adafruit_BNO055::REMAP_SIGN_P4;
      break;
    case 5:
      imuRemapConf = Adafruit_BNO055::REMAP_CONFIG_P5;
      imuRemapSign = Adafruit_BNO055::REMAP_SIGN_P5;
      break;
    case 6:
      imuRemapConf = Adafruit_BNO055::REMAP_CONFIG_P6;
      imuRemapSign = Adafruit_BNO055::REMAP_SIGN_P6;
      break;
    case 7:
      imuRemapConf = Adafruit_BNO055::REMAP_CONFIG_P7;
      imuRemapSign = Adafruit_BNO055::REMAP_SIGN_P7;
      break;
  }

    //  set the axis parameters
    imuSensor.setAxisRemap(imuRemapConf);
    delay(15);
    imuSensor.setAxisSign(imuRemapSign);

    //  update the settings in flash
    fsIMUAxisRemap.write((int)imuRemapConf);
    fsIMUAxisSign.write((int)imuRemapSign);

    //  reset the imuCalibrated state to force saving new cal parameters at shutdown
    imuCalibrated = 0;
}

/*
 * setShutdownVoltage is called when the setShutdownVoltage command is received.
 * The command is in the form:
 * 
 * setShutdownVoltage,<enable monitoring as int>,<shutdown voltage as float>\n
 * 
 */
void setShutdownVoltage()
{

  char    *arg;

  DEBUG_PRINT("setShutdownVoltage command received");
  
  //  read in the slope value and convert to int
  arg = sCmd.next();
  monitorVoltage = (atoi(arg) > 0);

  //  store in flash
  if (monitorVoltage)
    fsmonitorVoltage.write(1);
  else
    fsmonitorVoltage.write(0);

  //  read in the voltage cutoff value
  arg = sCmd.next();
  voltageCutoff = atof(arg);
  
  //  store in flash
  fsvoltageCutoff.write(voltageCutoff);

  Serial1.println("setShutdownVoltage,OK");
  
}


/*
 * getShutdownVoltage is called when the getShutdownVoltage command
 * is received. This command returns the current input voltage monitoring
 * parameters. The command is in the form:
 * 
 * getShutdownVoltage\n
 * 
 * The return string is in the form:
 * 
 * getShutdownVoltage,<enabled as int>,<shutdown threshold as float>\n
 * 
 */
void getShutdownVoltage()
{

  DEBUG_PRINT("getShutdownVoltage command received");
  
  //  send the response string 
  Serial1.print("getShutdownVoltage,");
  if (monitorVoltage)
    Serial1.print("1,");
  else
    Serial1.print("0,");
  Serial1.print(voltageCutoff);
  Serial1.print("\n");

}


/*
 * setStartupVoltage is called when the setStartupVoltage command is received.
 * The command is in the form:
 * 
 * setStartupVoltage,<startup voltage as float>\n
 * 
 */
void setStartupVoltage()
{

  char    *arg;

  DEBUG_PRINT("setStartupVoltage command received");
  
  //  read in the startup voltage value
  arg = sCmd.next();
  startupVoltage = atof(arg);
  
  //  store in flash
  fsvoltageStartup.write(startupVoltage);

  Serial1.println("setStartupVoltage,OK");
  
}


/*
 * getStartupVoltage is called when the getStartupVoltage command
 * is received. This command returns the startup voltage threshold. 
 * The system will go into sleep if when at any time in standby the 
 * voltage drops below this threshold. The command is in the form:
 * 
 * getStartupVoltage\n
 * 
 * The return string is in the form:
 * 
 * getStartupVoltage,<startup voltage threshold as float>\n
 * 
 */
void getStartupVoltage()
{

  DEBUG_PRINT("getStartupVoltage command received");
  
  //  send the response string 
  Serial1.print("getStartupVoltage,");
    if (monitorVoltage)
    Serial1.print("1,");
  else
    Serial1.print("0,");
  Serial1.print(startupVoltage);
  Serial1.print("\n");

}


/*
 * setRTC is called when the setRTC command is received.
 * The command is in the form:
 * 
 * setRTC,<year as int>,<month as int>,<day as int>,<hour as int>,<minute as int>,<second as int>\n
 * 
 */
void setRTC()
{
  char    *arg;
  int     rtcYear;
  int     rtcMonth;
  int     rtcDay;
  int     rtcHour;
  int     rtcMinute;
  int     rtcSecond;

  DEBUG_PRINT("setRTC command received");
  
  //  read in the year value and convert to int
  arg = sCmd.next();
  rtcYear = atoi(arg);
  
  //  read in the month value and convert to int
  arg = sCmd.next();
  rtcMonth = atoi(arg);

  //  read in the day value and convert to int
  arg = sCmd.next();
  rtcDay = atoi(arg);
  
  //  read in the hour value and convert to int
  arg = sCmd.next();
  rtcHour = atoi(arg);

    //  read in the minute value and convert to int
  arg = sCmd.next();
  rtcMinute = atoi(arg);
  
  //  read in the second value and convert to int
  arg = sCmd.next();
  rtcSecond = atoi(arg);

  if (RTCInstalled == 1)
  {
    // and set the clock
    rtc.adjust(DateTime(rtcYear, rtcMonth, rtcDay, rtcHour, rtcMinute, rtcSecond));
  
    Serial1.println("setRTC,OK");
  }
  else
  {
    Serial1.println("setRTC,ERROR");
  }
}


/*
 * getRTC is called when the getRTC command is received. This command
 * returns the current date and time of the controllers RTC
 * The command is in the form:
 * 
 * getRTC\n
 * 
 * The return string is in the form:
 * 
 * getRTC,<year as int>,<month as int>,<day as int>,<hour as int>,<minute as int>,<second as int>\n
 * 
 */
void getRTC()
{
  DEBUG_PRINT("getRTC command received");
  
  if (RTCInstalled == 1)
  {
    DateTime timeNow = rtc.now();
  
    //  send the response string 
    Serial1.print("getRTC,");
    Serial1.print(timeNow.year()); Serial1.print(",");
    Serial1.print(timeNow.month()); Serial1.print(",");
    Serial1.print(timeNow.day()); Serial1.print(",");
    Serial1.print(timeNow.hour()); Serial1.print(",");
    Serial1.print(timeNow.minute()); Serial1.print(",");
    Serial1.print(timeNow.second()); Serial1.print("\n");

  }
  else
  {
    //  no RTC installed
    Serial1.println("getRTC,0,0,0,0,0,0");
  }
}


/*
 * getStartDelay is called when the getStartDelay command is received. 
 * This command returns the current startup delay from the controller.
 * The command is in the form:
 * 
 * getStartDelay\n
 * 
 * The return string is in the form:
 * 
 * getStartDelay,<Startup Delay in Secs as int>\n
 * 
 */
void getStartDelay()
{
  DEBUG_PRINT("getStartDelay command received");
  
  //  send the response string 
  Serial1.print("getStartDelay,");
  Serial1.print(TurnOnDelay); Serial1.print("\n");

}


/*
 * setStartDelay is called when the setStartDelay command is received. This command
 * sets the RTC operating parameters
 * The command is in the form:
 * 
 * setStartDelay,<Startup Delay in Secs as int>\n
 * 
 */
void setStartDelay()
{
  char    *arg;

  DEBUG_PRINT("setStartDelay command received");
  
  //  read in the TurnOnDelay time
  arg = sCmd.next();
  TurnOnDelay = atoi(arg);
  fsTurnOnDelay.write(TurnOnDelay);

  Serial1.println("setStartDelay,OK");

}


/*
 * calibrateIMU is called when the calIMU command is received.
 * The command is in the form:
 * 
 * calIMU\n
 * 
 * This will start the process of calibrating the IMU. Calibration feedback
 * will be reported via the serial port
 */
void calibrateIMU()
{

  uint8_t         sys;
  uint8_t         gyro;
  uint8_t         accel;
  uint8_t         mag;
  sensors_event_t event;

  if (IMUInstalled == 1)
  {

    sys = gyro = accel = mag = 0;
    imuSensor.getCalibration(&sys, &gyro, &accel, &mag);
  
    //  calibrate the gyro
    Serial1.println("");
    Serial1.println("");
    Serial1.println("");
    Serial1.println("-------- - Starting IMU Calibration - --------");
    Serial1.println("");
    Serial1.println("Calibrating Gyro...");
    Serial1.println("");
    Serial1.println("  --> You must keep the camera still during this process. <--");
    Serial1.println("");
    n = 5;
    Serial1.print("  Starting Gyro cal in "); Serial1.print(n);
    n = 0;
    while (n > 1)
    {
      delay(1200);
      n = n - 1;
      Serial1.print(",");Serial1.print(n);
    }
    Serial1.println("  Begin");
    while (gyro < 3)
    {
      imuSensor.getEvent(&event);
      imuSensor.getCalibration(&sys, &gyro, &accel, &mag);
      delay(50);
    }
    Serial1.println("Gyro calibrated.");
  
    //  calibrate the Magnetometer
    Serial1.println("");
    Serial1.println("");
    Serial1.println("Calibrating Magnetometer...");
    Serial1.println("");
    Serial1.println("  You should move the camera around in a figure 8 pattern.");
    Serial1.println("  Begin");
    while (mag < 3)
    {
      imuSensor.getEvent(&event);
      imuSensor.getCalibration(&sys, &gyro, &accel, &mag);
      delay(50);
    }
    Serial1.println("Magnetometer calibrated.");
  
    //  calibrate the Accelerometer
    Serial1.println("");
    Serial1.println("");
    Serial1.println("Calibrating Accelerometer. This is the hardest one.");
    Serial1.println("");
    Serial1.println("  Place the camera in 6 standing positions: back, front, top, bottom,");
    Serial1.println("  left side, and right side. Order doesn't matter. Move it slowly between");
    Serial1.println("  positions. It must be stable for a few seconds in each position.");
    Serial1.println("");
    while (mag < 3)
    {
      imuSensor.getEvent(&event);
      imuSensor.getCalibration(&sys, &gyro, &accel, &mag);
      delay(50);
    }
    Serial1.println("Accelerometer calibrated.");
  
    Serial1.println("");
    Serial1.println("Writing calibration parameters to flash...");
    imuSensor.getSensorOffsets(imuCalParms);
    fsIMUCalParms.write(imuCalParms);
    imuCalibrated = 3;
    fsIMUCalState.write(imuCalibrated);
    Serial1.println("Done.");
 
    Serial1.println("");
    Serial1.println("IMU Calibration complete.");
    Serial1.println("calibrateIMU,OK");

  }
}


/*
 * getIMUCalParms is called when the getIMUCal command is received. This command
 * returns the current date and time of the controllers RTC
 * The command is in the form:
 * 
 * getIMUCal\n
 * 
 * The return string is in the form:
 * 
 * getIMUCal,<accel_offset_x as int>,<accel_offset_y as int>,<accel_offset_z as int>,
 *           <gyro_offset_x as int>,<gyro_offset_y as int>,<gyro_offset_z as int>,
 *           <mag_offset_x as int>,<mag_offset_y as int>,<mag_offset_z as int>,
 *           <accel_radius as int>,<mag_radius as int>\n
 * 
 */
void getIMUCalParms()
{

  DEBUG_PRINT("getIMUCal command received");
  
  imuSensor.getSensorOffsets(imuCalParms);

  Serial1.print("getIMUCal,");
  Serial1.print(imuCalParms.accel_offset_x); Serial1.print(",");
  Serial1.print(imuCalParms.accel_offset_y); Serial1.print(",");
  Serial1.print(imuCalParms.accel_offset_z); Serial1.print(",");
  Serial1.print(imuCalParms.gyro_offset_x); Serial1.print(",");
  Serial1.print(imuCalParms.gyro_offset_y); Serial1.print(",");
  Serial1.print(imuCalParms.gyro_offset_z); Serial1.print(",");
  Serial1.print(imuCalParms.mag_offset_x); Serial1.print(",");
  Serial1.print(imuCalParms.mag_offset_y); Serial1.print(",");
  Serial1.print(imuCalParms.mag_offset_z); Serial1.print(",");
  Serial1.print(imuCalParms.accel_radius); Serial1.print(",");
  Serial1.print(imuCalParms.mag_radius); Serial1.print("\n");

}


/*
 * setSystemState is called when the setState command is received. This command
 * forces the controller into a different operational state. This is primarily
 * used for testing.
 * 
 * The command is in the form:
 * 
 * setState,<System state as int>\n
 * 
 * 
 */
void setSystemState()
{
  char    *arg;

  DEBUG_PRINT("setState command received");

  //  read in the state value and convert to int
  arg = sCmd.next();
  controllerState = (controllerStateEnum) atoi(arg);

  Serial1.println("setSystemState,OK");

}


/*
 * getSystemState is called when the getState command is received. This command
 * returns the current system state so the acquisition software knows if the
 * system has been forced on while on deck, or is at depth.
 * 
 * The command is in the form:
 * 
 * getState\n
 * 
 * The return string is in the form:
 * 
 * getState,<state as int>\n
 * 
 */
void getSystemState()
{

  //  send the response string 
  Serial1.print("getState,");
  Serial1.print((int)controllerState); Serial1.print("\n");
  DEBUG_PRINT(String("getState sent. State:") + String(controllerState));

}


/*
 * setPCState is called when the setPCState command is received. This command
 * informs the controller that the PC operational state is changing and that it should
 * switch to a different operational state. It is primarily used to tell the controller
 * that the PC received a state change message and it is ready to shut down.
 * 
 * It can also indicate that the PC encountered an error and the system should shutdown.
 * 
 * The command is in the form:
 * 
 * setPCState, <PC state as int>\n
 * 
 * 254 = PC error - initiates a shutdown from the PC side
 * 255 = PC acknowledges shutdown command
 * 
 */
void setPCState()
{
  char    *arg;

  DEBUG_PRINT("setPCState command received");

  //  read in the state value and convert to int
  arg = sCmd.next();
  PCState = atoi(arg);

  Serial1.println("setPCState,OK");
}


/*
 * setStrobeMode is called when the setStrobeMode command is received.
 * The command is in the form:
 * 
 * setStrobeMode,<mode as int>,<flash at start as int>\n
 * 
 */
void setStrobeMode()
{
  char    *arg;
  int     mode;
  int     flash;

  DEBUG_PRINT("setStrobeMode command received");

  //  read in the strobe mode and convert to int
  arg = sCmd.next();
  mode = atoi(arg);

  //  read in the flash at start flag and convert to int
  arg = sCmd.next();
  flash = atoi(arg);
  
  //  update and store
  if (flash > 0)
  {
    flashOnPowerup = true;
    fsFlashOnPowerup.write(1);
  }
  else
  {
    flashOnPowerup = false;
    fsFlashOnPowerup.write(0);
  }

  //  update the strobe mode
  if (mode >= 0 && mode <=3)
  {
    strobeMode = mode;
    DEBUG_PRINT(String("Strobe mode set to:") + String(strobeMode));

    //  check if we need to turn the strobe power on or off
    if (strobeMode == 0)
    {
      //  turn off the strobe power supply
      DEBUG_PRINT("Turning off Strobe power");
      turnOffStrobes();
    }
    else if (strobeMode > 0 && (controllerState == 1 || controllerState == 2))
    {
      //  turn on the strobe power supply
      DEBUG_PRINT("Turning on Strobe power");
      digitalWrite(strobeEnable, HIGH);
    }
  }
}


/*
 * getStrobeMode is called when the getStrobeMode command is received. This command
 * returns the current strobe mode of the system
 * The command is in the form:
 * 
 * getStrobeMode\n
 * 
 * The return string is in the form:
 * 
 * getStrobeMode,<mode as int>, <flash on start as int>\n
 * 
 */
void getStrobeMode()
{
  //  send the response string 
  Serial1.print("getStrobeMode,");
  Serial1.print(strobeMode);
  if (flashOnPowerup)
    Serial1.print(",1\n");
  else
    Serial1.print(",0\n");
  DEBUG_PRINT(String("getStrobeMode sent. Mode:") + String(strobeMode));
}


// This is the handler for unrecognized serial commands
void unknownSerialCommand(const char *command)
{
  //Serial1.println("NAK");
  DEBUG_PRINT(String("Unknown command received:") + String(*command));
}


void turnOffStrobes()
{
  //  disable strobe power
  digitalWrite(strobeEnable, LOW);

  //  repeatedly trigger strobes to drain storage caps
  for (int i=0; i <= 6; i++)
  {
      //  trigger strobe channel 1
      digitalWriteDirect(strobeTrigOne, HIGH);
      delay(2);
      digitalWriteDirect(strobeTrigOne, LOW);

      //  pause
      delay(500);
  }
}

void greenLEDOn()
{
  grLEDState = true;
  digitalWriteDirect(statusLEDRed, grLEDState);
  digitalWriteDirect(statusLEDBlk, false);
  digitalWriteDirect(intGrLED, !grLEDState);
}


void greenLEDOff()
{
  grLEDState = false;
  digitalWriteDirect(statusLEDRed, grLEDState);
  digitalWriteDirect(intGrLED, !grLEDState);
  
}


void toggleGreenLED()
{
  grLEDState = !grLEDState;
  digitalWriteDirect(statusLEDRed, grLEDState);
  digitalWriteDirect(intGrLED, !grLEDState);
}


void redLEDOn()
{
  rdLEDState = true;
  digitalWriteDirect(statusLEDBlk, rdLEDState);
  digitalWriteDirect(statusLEDRed, false);
  digitalWriteDirect(intOrLED, !rdLEDState);
  
}


void redLEDOff()
{
  rdLEDState = false;
  digitalWriteDirect(statusLEDBlk, rdLEDState);
  digitalWriteDirect(intOrLED, !rdLEDState);
}


void toggleRedLED()
{
  rdLEDState = !rdLEDState;
  digitalWriteDirect(statusLEDBlk, rdLEDState);
  digitalWriteDirect(intOrLED, !rdLEDState);
}


//  function to calculate depth based on latitude and pressure. From:
//  Unesco 1983. Algorithms for computation of fundamental properties of
//  seawater, 1983. _Unesco Tech. Pap. in Mar. Sci._, No. 44, 53 pp.
//
//  input is pressure in decibar
float pressureToDepth(float p)
{
  float g;
  float d;

  // Adapted from example funtion, p28.  UNESCO 1983.
  g = (9.780318 * (1.0 + (5.2788e-3 + 2.36e-5 * P2DX) * P2DX) + 1.092e-6 * p);
  d = (((-1.82e-15 * p + 2.279e-10) * p + -2.2512e-5) * p + 9.72659) * p;

  return d / g;
}

/*
 *  CamtrawlRTO is the firmware for the ministereo remote turn-on
 *  (RTO) board used in the TriggerCams and related units housed
 *  in the Sexton GoDeep Stereo housing.
 *   
 *  The RTO has two operating modes. op_mode 0 always starts with the
 *  camera system off. op_mode 1 retains the camera system mode and
 *  will start the camera in the last commanded mode. op_mode 1 is
 *  useful when the power to the camera system may be interrupted while
 *  deployed and you want the camera system to start back up when
 *  power is restored.
 *  
 *  To change the mode:
 *  
 *    1 - hold the magnet to the hall effect sensor and then turn the power on
 *    2 - hold the magnet there for ~5 seconds
 *    3 - remove magnet when status LED flashes
 *        The RTO LED will flash 4 blue flashes and 4 pink flashes when changing to mode 1
 *        The RTO LED will flash 4 alternating blue and yellow flashes when changing to mode 0
 *  
 *   
 *  Rick Towler
 *  Midwater Assesment and Conservation Engineering Group
 *  NOAA Alaska Fisheries Science Center, Seattle, WA
 *  rick.towler@noaa.gov
 */

#include <EEPROM.h>

#define MODECHANGE_TIME     5000
#define OP_MODE_ADDR        3
#define SYS_MODE_ADDR       4

//  pin definitions
const uint8_t   HES = 0;
const uint8_t   forceOn = 1;
const uint8_t   pressureSwitch = 2;
const uint8_t   yellowLED = 3;
const uint8_t   blueLED = 4;


bool            ok;
uint8_t         i = 0;
uint8_t         sys_mode = 0;             //  the camera system mode: 0 = no plug installed, 1 = force-on installed, 2 = pressure switch closed)
uint8_t         op_mode = 0;              //  the RTO operating mode: 0 = does not retain sys_mode between power cycles. 1 = DOES retain sys_mode


void setup()
{
  //  set up pins
  pinMode(HES, INPUT);
  pinMode(forceOn, OUTPUT);
  pinMode(pressureSwitch, OUTPUT);
  pinMode(yellowLED, OUTPUT);
  pinMode(blueLED, OUTPUT);
  digitalWrite(forceOn, HIGH);
  digitalWrite(pressureSwitch, HIGH);
  digitalWrite(yellowLED, LOW);
  digitalWrite(blueLED, LOW);
  
  //  check if we have just been programmed
  ok = true;
  for (i = 0; i < OP_MODE_ADDR; i++)
  {
    op_mode = EEPROM.read(i);
    ok &= (op_mode == i);
  }
  if (ok)
    //  mode has been set, read it
    op_mode = EEPROM.read(OP_MODE_ADDR);
  else
  {
    //  we have just been programmed and the mode is not set
    
    //  write the signature bytes
    for (i = 0; i < OP_MODE_ADDR; i++)
      EEPROM.write(i, i);

    //  and write the default op and sys modes
    EEPROM.write(OP_MODE_ADDR, 0);
    op_mode = 0;
    EEPROM.write(SYS_MODE_ADDR, 0);
    sys_mode = 0;
  }

  //  check if a user is changing the operating mode
  if(!digitalRead(HES))
    {
      delay(MODECHANGE_TIME);
      if(!digitalRead(HES))
      {
        //  reset the system mode when changing the operating mode
        EEPROM.write(SYS_MODE_ADDR, 0);
        sys_mode = 0;
        
        //  user is changing the operating mode
        if (op_mode == 0)
        {
          //  update the mode and write to eeprom
          op_mode = 1;
          EEPROM.write(OP_MODE_ADDR, op_mode);
          
          //  signal switching to mode 1 by 4 blue flashes and 4 pink flashes
          for (i = 0; i < 4; i++)
          {
            digitalWrite(blueLED, HIGH);
            delay(333);
            digitalWrite(blueLED, LOW);
            delay(333);
          }
          delay(333);
          for (i = 0; i < 4; i++)
          {
            digitalWrite(blueLED, HIGH);
            digitalWrite(yellowLED, HIGH);
            delay(333);
            digitalWrite(blueLED, LOW);
            digitalWrite(yellowLED, LOW);
            delay(333);
          }
        }
        else
        {
          //  update the mode and write to eeprom
          op_mode = 0;
          EEPROM.write(OP_MODE_ADDR, op_mode);
          
          //  signal switching to mode 0 by 4 alternating blue and yellow flashes
          for (i = 0; i < 4; i++)
          {
            digitalWrite(blueLED, HIGH);
            delay(333);
            digitalWrite(blueLED, LOW);
            digitalWrite(yellowLED, HIGH);
            delay(333);
            digitalWrite(yellowLED, LOW);
          }
        }
      }
    }

    if (op_mode == 1)
    {
      sys_mode = EEPROM.read(SYS_MODE_ADDR);
    }
    else
    {
      //  in mode 0 we always start in sys_mode 0 = no plug installed
      sys_mode = 0;
    }

    
    if (sys_mode == 1)
      digitalWrite(forceOn, LOW);
    else if (sys_mode == 2)
      digitalWrite(pressureSwitch, LOW);

}


void loop()
{

  if (sys_mode == 0)
  {
    if(!digitalRead(HES))
    {
      delay(2000);
      if(!digitalRead(HES))
      {
          digitalWrite(yellowLED, HIGH);
          //  
          sys_mode = 1;
          delay(3000);
          if(!digitalRead(HES))
          {
            digitalWrite(blueLED, HIGH);
            sys_mode = 2;
          }
          
          if (sys_mode == 1)
            digitalWrite(forceOn, LOW);
          else if (sys_mode == 2)
            digitalWrite(pressureSwitch, LOW);

          if (op_mode == 1)
            EEPROM.write(SYS_MODE_ADDR, sys_mode);

          delay(8000);
          digitalWrite(yellowLED, LOW);
          digitalWrite(blueLED, LOW);
      }
    }
  }

  //  if're running, check if we should pull the plugs
  if (sys_mode != 0)
  {
    if(!digitalRead(HES))
    {
      digitalWrite(yellowLED, HIGH);
      if (sys_mode == 2)
        digitalWrite(blueLED, HIGH);
      delay(3000);
      if(!digitalRead(HES))
      {
        //  indicate we're pulling all the plugs (shutting down)
        digitalWrite(yellowLED, LOW);
        digitalWrite(blueLED, HIGH);
        digitalWrite(pressureSwitch, HIGH);
        digitalWrite(forceOn, HIGH);
        delay(8000);

        sys_mode = 0;
        if (op_mode == 1)
          EEPROM.write(SYS_MODE_ADDR, sys_mode);
      }
      digitalWrite(blueLED, LOW);
      digitalWrite(yellowLED, LOW);
      
    }
  }
}

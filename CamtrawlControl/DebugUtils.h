/*
 * Simple debug macro obtained from http://forum.arduino.cc/index.php?topic=46900.0
 * Modified to output debugging info on the SAMD21 USB serial monitor port
 */

#ifndef DEBUGUTILS_H
#define DEBUGUTILS_H

#ifdef DEBUGPRINT
#define DEBUG_PRINT(str)    \
   SerialUSB.print("DEBUG: ");    \
   SerialUSB.println(str);
#else
#define DEBUG_PRINT(str)
#endif

#endif

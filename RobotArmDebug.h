#ifndef ROBOT_ARM_DEBUG_H
#define ROBOT_ARM_DEBUG_H

#include "HardwareSerial.h"

#define ROBOT_ARM_DEBUG

#ifdef ROBOT_ARM_DEBUG
#define R_DPRINT(x) Serial.print(x)
#define R_DPRINTLN(x) Serial.println(x)
#define R_DPRINTLN_ARR(arr, n) for (int _iterator_ = 0; _iterator_ < n; _iterator_++) Serial.println(arr[_iterator_])
#else
#define R_DPRINT(x)
#define R_DPRINTLN(x)
#define R_DPRINTLN_ARR(arr, n)
#endif


#endif
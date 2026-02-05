#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#include "common/time.h"
#include "io/serial.h"

/*
  * Periodic task with low priority and 33 Hertz frequency.
  *
  * Note : Higher priorities cause error on runtime. Remain low priority.
 */

extern int FUZE_STATUS;
extern int LIDAR_STATUS;
extern int ALTITUDE_STATUS;
extern bool CHARGE_DISPLAYING;

extern serialPortIdentifier_e FUZE_PORT_IDENTIFIER;

void donmezogluUpdate(timeUs_t currentTimeUs);

void donmezogluSerialPrintS(const char* str);
 
void donmezogluSerialPrintC(uint8_t ch);
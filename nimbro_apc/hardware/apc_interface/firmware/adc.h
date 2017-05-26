// ADC IO for pressure sensor
// Author: Sebastian Schüller<schuell1@cs.uni-bonn.de>

#ifndef ADC_H
#define ADC_H

#include <stdint.h>

namespace adc
{
void pressure_init();
uint16_t pressure_sync_read();
}

#endif

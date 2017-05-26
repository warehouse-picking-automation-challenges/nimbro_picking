// ADC IO for pressure sensor
// Author: Sebastian Schüller<schuell1@cs.uni-bonn.de>

#include "adc.h"
#include <avr/io.h>

namespace adc
{

void pressure_init()
{
	// Set reference voltage to AVCC
	ADMUX = (1 << REFS0);

	// Set clock prescaling factor to 128
	ADCSRA = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	// Enable ADC
	ADCSRA |= (1 << ADEN);

	// Wait for the initial conversion
	ADCSRA |= (1 << ADSC);
	while(ADCSRA & (1 << ADSC));
}

uint16_t pressure_sync_read()
{
	uint16_t pressure;
	ADCSRA |= (1 << ADSC);
	while(ADCSRA & (1 << ADSC));
	pressure = ADCL;
	pressure |= (ADCH << 8);
	return pressure;
}

}

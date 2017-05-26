// I/O methods
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "io.h"

#include <avr/io.h>

void io_init()
{
	// PB1: TTL transmit enable
	// PB7: LED
	PORTB = (1 << 7);
	DDRB = (1 << 1) | (1 << 7);

	// PD4: RS485 output enable
	DDRD = (1 << 4);
	PORTD = 0;

	// PA1: TTL receive enable
	DDRA = (1 << 1);
	PORTA = (1 << 1);

// 	DDRC = 0xff;
// 	PORTC = 0;

// 	DDRF = 0xff;
// 	PORTF = 0;

	// Pullups for UART RX pins
	PORTE |= (1 << 0); // RXD0
	PORTD |= (1 << 2); // RXD1

	// ADC: external capacitor at AVCC
	ADMUX = (1 << REFS0) | 0;
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0) | (1 << ADFR);

	ADCSRA |= (1 << ADSC);
}

uint16_t io_analog()
{
	return ADC;
}

// I/O methods
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef HAND_CONTROLLER_IO_H
#define HAND_CONTROLLER_IO_H

#include <stdint.h>
#include <avr/io.h>

void io_init(void);

static inline void io_setRS485Output(uint8_t output)
{
	if(output)
	{
// 		PORTB &= ~(1 << 7);
		PORTD |= (1 << 4);
	}
	else
	{
// 		PORTB |= (1 << 7);
		PORTD &= ~(1 << 4);
	}
}

static inline void io_setTTLOutput(uint8_t output)
{
	if(output)
	{
		PORTA &= ~(1 << 1);
		PORTB |= (1 << 1);
	}
	else
	{
		PORTB &= ~(1 << 1);
		PORTA |= (1 << 1);
	}
}

static inline void io_setLED(uint8_t led)
{
	if(led)
		PORTB &= ~(1 << 7);
	else
		PORTB |= (1 << 7);
}

static inline void io_toggleLED()
{
	PORTB ^= (1 << 7);
}

uint16_t io_analog();

#endif

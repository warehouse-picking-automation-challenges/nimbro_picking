// APC IO control
// Author: Sebastian Schüller<schuell1@cs.uni-bonn.de>

#include "io.h"
#include <avr/io.h>

namespace io
{
void init()
{
	// Set PB4 to ouput (for vacuum)
	PORTB = 0;
	DDRB = (1 << PB4);
	// Use PB5 for vacuum power switch
	DDRB |= (1 << PB6);

	// PJ7: LED
	PORTJ = 0;
	DDRJ = (1 << PJ7);

	// PWM for SR300 LED dimming
	// Set PB5 to ouput for PWM
	DDRB |= (1 << PB5);
	// Enable 8-Bit Fasl PWM (WGM13,WGM12,WGM11) on OC1A (PB5)
	// without Prescaling (CS10)
	TCCR1A = (1 << COM1A1) | (1 << WGM11);
	TCCR1B = (1 << CS10) | (1 << WGM13) | (1 << WGM12);
	ICR1 = 255;
	// Set initial PWM duty to 5 to be able to see if LEDs are running
	OCR1A = 5;
}

void init_RS485()
{
	// Use onboard RS485 on PORTJ
	PORTJ = 0;
	DDRJ |= (1 << PJ2);
}

void set_RS485_output(bool out)
{
	if(out)
		PORTJ |= (1 << PJ2);
	else
		PORTJ &= ~(1 << PJ2);
}

void reset_RS485_txComplete()
{
	UCSR3A |= (1 << TXC3);
}

bool is_RS485_txComplete_set()
{
	return (UCSR3A & (1 << TXC3));
}

void set_vacuum(bool on)
{
	if(on)
	{
		PORTB |= (1 << PB4);
	}
	else
	{
		PORTB &= ~(1 << PB4);
	}
}

void set_vacuum_power(bool on)
{
	if(on)
	{
		PORTB |= (1 << PB6);
	}
	else
	{
		PORTB &= ~(1 << PB6);
	}
}

bool is_vacuum_on()
{
	return !!(PORTB & (1 << PB4));
}

bool is_vacuum_power_on()
{
	return !!(PORTB & (1 << PB6));
}

void dim_light(uint8_t duty)
{
	OCR1A = duty;
}

uint8_t light_duty()
{
	return OCR1A;
}

namespace debug
{
void toggle_board_led()
{
	PORTJ ^= (1 << PJ7);
}

}

}

// I/O methods
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "io.h"

#include <avr/io.h>

void io_init()
{
	// PJ2: RS485 output enable
	// PJ7: LED
	PORTJ = 0;
	DDRJ = (1 << 2) | (1 << 7);

	// PG0, PG1: "Hard" emergency switch
	// PC1: "Soft" emergency switch
	PORTG = (1 << 1) | (1 << 2);
	DDRG = (1 << 0);
	PORTC = (1 << 1);
	DDRC = 0;

	// PE3-5: RGB LED
	// blue: 3
	DDRE = (1 << 3) | (1 << 4) | (1 << 5);
	PORTE = (1 << 3) | (1 << 4) | (1 << 5);
}

void io_setRS485Output(bool output)
{
	if(output)
		PORTJ |= (1 << 2);
	else
		PORTJ &= ~(1 << 2);
}

// We only have one emergency switch connected right now (PG1), wire that
// internally to the "soft" E-stop logic.

bool io_emergencySwitch_hard()
{
// 	return PING & (1 << 1);
	return false;
}

bool io_emergencySwitch_soft()
{
// 	return !(PINC & (1 << 1));
// 	return !(PING & (1 << 2));
	return (PING & (1 << 1)) || !(PINC & (1 << 1));
}

void io_redLED(bool on)
{
	if(!on)
		PORTE |= (1 << 5);
	else
		PORTE &= ~(1 << 5);
}

void io_greenLED(bool on)
{
	if(!on)
		PORTE |= (1 << 4);
	else
		PORTE &= ~(1 << 4);
}

void io_blueLED(bool on)
{
	if(!on)
		PORTE |= (1 << 3);
	else
		PORTE &= ~(1 << 3);
}

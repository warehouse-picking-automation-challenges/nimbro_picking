// I/O methods
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef HAND_CONTROLLER_IO_H
#define HAND_CONTROLLER_IO_H

#include <stdint.h>
#include <avr/io.h>

void io_init();

void io_setRS485Output(bool output);

// The hard emergency switch is the official DARPA E-Stop. It will stop the
// motors immediately and disable power gradually.
bool io_emergencySwitch_hard();

// The soft emergency switch is our own emergency stop. It will stop the
// motors immediately and take no further action.
bool io_emergencySwitch_soft();

void io_redLED(bool on);
void io_greenLED(bool on);
void io_blueLED(bool on);

inline void io_rgb(bool r, bool g, bool b)
{
	io_redLED(r);
	io_greenLED(g);
	io_blueLED(b);
}

#endif

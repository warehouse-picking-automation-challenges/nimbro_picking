// Controller for Momaro's actuators
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <avr/interrupt.h>
#include <avr/io.h>

#include <util/delay.h>

#include "uart_wrapper.h"

#include "comm_pc.h"
#include "comm_dxl.h"
#include "servo.h"
#include "io.h"
#include "imu.h"

int main()
{
	servo_init();

	ftdi_pc_init(UART_BAUD_SELECT_DOUBLE_SPEED(115200ULL, F_CPU));
	rs485_init(UART_BAUD_SELECT_DOUBLE_SPEED(115200ULL, F_CPU));

	io_init();
	imu_init();

	sei();

	while(1)
	{
		if(io_emergencySwitch_hard())
			io_rgb(false, false, true);
		else if(io_emergencySwitch_soft() || g_softwareStop)
			io_rgb(false, true, false);
		else
			io_rgb(true, false, false);

		if(io_emergencySwitch_hard() && g_cycles_since_estop < 1000)
		{
			g_cycles_since_estop++;
		}

		if(!io_emergencySwitch_hard() && !io_emergencySwitch_soft())
			g_cycles_since_estop = 0;

		if(ftdi_pc_bytesWaiting())
		{
			uint8_t c = ftdi_pc_getc();
			comm_pc_processByte(c);
		}
		else if(io_emergencySwitch_hard() || io_emergencySwitch_soft())
		{
			if(g_cycles_since_last_write++ > 50)
				dxl_writeActuators();

			_delay_ms(1);
		}
	}
}

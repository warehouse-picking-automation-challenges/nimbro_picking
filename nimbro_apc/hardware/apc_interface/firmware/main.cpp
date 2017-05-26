// Hardware controller for Amazon Picking Challengs
// Author: Sebastian Schüller<schuell1@cs.uni-bonn.de>

#include <avr/interrupt.h>

#include "uart_wrapper.h"
#include "comm.h"
#include "adc.h"
#include "io.h"
#include "dxl.h"


int main()
{
	ftdi_pc_init(UART_BAUD_SELECT_DOUBLE_SPEED(115200ULL, F_CPU));
	rs485_init(UART_BAUD_SELECT_DOUBLE_SPEED(1000000ULL, F_CPU));
	adc::pressure_init();
	io::init();
	io::init_RS485();
	dxl::init();

	sei();

	while(1)
	{
		if(ftdi_pc_bytesWaiting())
		{
			uint8_t c = ftdi_pc_getc();
			comm::processByte(c);
		}
	}
}

// Communication control
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <dxlpro_slave/dxlpro_slave.h>
#include <dxl_master/dxl_master.h>

#include <avr/io.h>
#include <avr/interrupt.h>

#include "config.h"
#include "io.h"

#include <util/delay.h>

static volatile uint8_t g_ttl_txBuf[256];
static volatile uint8_t g_ttl_txHead = 0;
static volatile uint8_t g_ttl_txTail = 0;

static volatile uint8_t g_rs485_txBuf[256];
static volatile uint8_t g_rs485_txHead = 0;
static volatile uint8_t g_rs485_txTail = 0;

uint8_t dxl_recv_count;

void comm_init(void)
{
	dxlpro_slave_init(BOARD_ID, 0xFBFA, 0x01);
	dxl_master_init();

	// Init TTL bus
// 	UCSR0A = (1 << U2X0);

	UBRR0L = 0; // 1MBit at 16 MHz
	UBRR0H = 0;

	UCSR0B = (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0);

	// frame format: 8 data bits, no parity, 1 stop bit
	UCSR0C = (3 << UCSZ00);

	// Init RS485 bus
	UCSR1A = (1 << U2X1);

	UBRR1L = 16; // 115200 baud at 16 MHz with U2X = 1
	UBRR1H = 0;

	UCSR1B = (1 << TXEN1) | (1 << RXEN1) | (1 << RXCIE1);

	// frame format: 8 data bits, no parity, 1 stop bit
	UCSR1C = (3 << UCSZ10);
}

// RS485 bus methods
//@{

void dxlpro_slave_send_begin(void)
{
	_delay_us(100);
}

void dxlpro_slave_send(uint8_t c)
{
	uint8_t tmphead = g_rs485_txHead + 1;

	if(tmphead == g_rs485_txTail)
	{
		// dropping byte (buffer is full)
		io_toggleLED();
		return;
	}

	io_setRS485Output(1);

	g_rs485_txBuf[tmphead] = c;
	g_rs485_txHead = tmphead;

	UCSR1B |= (1 << UDRIE1);
}

ISR(USART1_UDRE_vect)
{
	PORTC |= (1 << 3);

	if(g_rs485_txHead == g_rs485_txTail)
	{
		UCSR1B &= ~(1 << UDRIE1);
		UCSR1B |= (1 << TXCIE1);

		PORTC &= ~(1 << 3);
		return;
	}

	uint8_t tmptail = g_rs485_txTail + 1;
	g_rs485_txTail = tmptail;
	UDR1 = g_rs485_txBuf[tmptail];

	PORTC &= ~(1 << 3);
}

ISR(USART1_TX_vect)
{
	PORTC |= (1 << 4);

	UCSR1A |= (1 << TXC1);
	UCSR1B &= ~(1 << TXCIE1);

	if(g_rs485_txHead == g_rs485_txTail)
	{
		_delay_us(200);
		io_setRS485Output(0);
	}

	PORTC &= ~(1 << 4);
}

ISR(USART1_RX_vect)
{
	PORTC |= (1 << 5);
	dxlpro_slave_putc(UDR1);
	PORTC &= ~(1 << 5);
}
//@}

// TTL bus functions
//@{

void dxl_master_sendByte(uint8_t c)
{
// 	uint8_t tmphead = g_ttl_txHead + 1;
//
// 	io_setTTLOutput(1);
//
// 	while(tmphead == g_ttl_txTail)
// 		; // wait for free space
//
// 	g_ttl_txBuf[tmphead] = c;
// 	g_ttl_txHead = tmphead;
//
// 	UCSR0B |= (1 << UDRIE0);

	while(!(UCSR0A & (1 << UDRE0)));

	UCSR0A |= (1 << TXC0);
	UDR0 = c;
}

void dxl_master_send(const uint8_t* data, uint8_t len)
{
	io_setTTLOutput(1);
	while(len-- != 0)
		dxl_master_sendByte(*(data++));

	while(!(UCSR0A & (1 << TXC0)));

	io_setTTLOutput(0);
}

// ISR(USART0_UDRE_vect)
// {
// 	PORTC |= (1 << 0);
//
// 	if(g_ttl_txHead == g_ttl_txTail)
// 	{
// 		UCSR0B &= ~(1 << UDRIE0);
// 		UCSR0B |= (1 << TXCIE0);
//
// 		PORTC &= ~(1 << 0);
// 		return;
// 	}
//
// 	uint8_t tmptail = g_ttl_txTail + 1;
// 	io_setTTLOutput(1);
// 	UDR0 = g_ttl_txBuf[tmptail];
// 	UCSR0A |= (1 << TXC0);
// 	g_ttl_txTail = tmptail;
//
// 	PORTC &= ~(1 << 0);
// }
//
// ISR(USART0_TX_vect)
// {
// 	PORTC |= (1 << 1);
//
// 	UCSR0A |= (1 << TXC0);
// 	UCSR0B &= ~(1 << TXCIE0);
//
// // 	if(g_ttl_txHead == g_ttl_txTail)
// // 	{
// 		_delay_us(5);
// 		io_setTTLOutput(0);
// // 	}
//
// 	PORTC &= ~(1 << 1);
// }

ISR(USART0_RX_vect)
{
	PORTC |= (1 << 2);
	dxl_master_putc(UDR0);
	PORTC &= ~(1 << 2);
// 	dxlpro_slave_send(UDR0);
}

//@}

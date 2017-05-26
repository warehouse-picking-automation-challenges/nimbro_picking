// Simple bootloader speaking the DXL 2.0 protocol
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <dxlpro_slave/dxlpro_slave.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>

#include <util/delay.h>

#include "interface.h"

// avr-libc 1.8.0 has a check in boot.h which uses the identifier SPMCR, which
// is poisoned on ATmega128. Patched in our own version of boot.h.

#if __AVR_LIBC_VERSION__ == 10800UL
#  include "contrib/boot.h"
#else
#  include <avr/boot.h>
#endif

static uint8_t g_page_buffer[SPM_PAGESIZE];
static uint8_t g_command = BOOTLOADER_COMMAND_IDLE;
static uint32_t g_page_addr;
static uint8_t g_exit = 0;

static void boot_program_page(uint32_t page, uint8_t *buf)
{
	uint16_t i;

	eeprom_busy_wait ();
	boot_page_erase (page);
	boot_spm_busy_wait (); // Wait until the memory is erased.

	for (i=0; i<SPM_PAGESIZE; i+=2)
	{
		// Set up little-endian word.
		uint16_t w = *buf++;
		w += (*buf++) << 8;
		boot_page_fill (page + i, w);
	}
	boot_page_write (page); // Store buffer in flash page.
	boot_spm_busy_wait(); // Wait until the memory is written.

	// Reenable RWW-section again. We need this if we want to jump back
	// to the application after bootloading.
	boot_rww_enable ();
}

void dxlpro_slave_ctrl_set(uint16_t addr, uint8_t value)
{
	if(addr == BOOTLOADER_REG_COMMAND)
	{
		g_command = value;
	}
	else if(addr >= BOOTLOADER_REG_ADDRESS && addr < BOOTLOADER_REG_ADDRESS+4)
	{
		((uint8_t*)&g_page_addr)[addr - BOOTLOADER_REG_ADDRESS] = value;
	}
	else if(addr >= BOOTLOADER_REG_PAGEBUFFER && addr < BOOTLOADER_REG_PAGEBUFFER + SPM_PAGESIZE)
	{
		g_page_buffer[addr - BOOTLOADER_REG_PAGEBUFFER] = value;
	}
}

void dxlpro_slave_ctrl_apply(void)
{
	uint16_t i;

	switch(g_command)
	{
		case BOOTLOADER_COMMAND_WRITE_PAGE:
			if(g_page_addr >= FLASHEND)
			{
				g_command = BOOTLOADER_COMMAND_FAILURE;
				break;
			}

			boot_program_page(g_page_addr, g_page_buffer);
			g_command = BOOTLOADER_COMMAND_IDLE;
			break;
		case BOOTLOADER_COMMAND_READ_PAGE:
			if(g_page_addr >= FLASHEND)
			{
				g_command = BOOTLOADER_COMMAND_FAILURE;
				break;
			}

			for(i = 0; i < SPM_PAGESIZE; ++i)
			{
				g_page_buffer[i] = pgm_read_byte_far(g_page_addr + i);
			}

			g_command = BOOTLOADER_COMMAND_IDLE;
			break;
		case BOOTLOADER_COMMAND_EXIT:
			// Jump to application code
			g_exit = 1;
			break;
	}
}

uint8_t dxlpro_slave_ctrl_get(uint16_t addr)
{
	switch(addr)
	{
		case DXLPRO_SLAVE_REG_MODEL_NUMBER_L:
			return BOOTLOADER_MODEL & 0xFF;
		case DXLPRO_SLAVE_REG_MODEL_NUMBER_H:
			return BOOTLOADER_MODEL >> 8;
		case DXLPRO_SLAVE_REG_VERSION:
			return 0x01;
		case BOOTLOADER_REG_COMMAND:
			return g_command;
	}

	if(addr >= BOOTLOADER_REG_PAGEBUFFER && addr < BOOTLOADER_REG_PAGEBUFFER + SPM_PAGESIZE)
	{
		return g_page_buffer[addr - BOOTLOADER_REG_PAGEBUFFER];
	}

	return 0xFF;
}

void dxlpro_slave_send_begin(void)
{
	// Stop timeout buffer
	TCCR1B = 0;

	_delay_us(250);

	// Enable RS485 transmitter
	PORTD |= (1 << 4);

	_delay_us(250);
}

void dxlpro_slave_send(uint8_t c)
{
	while(!(UCSR1A & (1 << UDRE1)));
	UCSR1A |= (1 << TXC1);
	UDR1 = c;
}

void dxlpro_slave_send_end(void)
{
	// Wait for last byte to complete transmission
	while(!(UCSR1A & (1 << UDRE1)));
	while(!(UCSR1A & (1 << TXC1)));
	_delay_us(20);

	// Disable transmitter
	PORTD &= ~(1 << 4);
}

uint8_t mcucsr_mirror __attribute__ ((section (".noinit")));
void get_mcusr(void) __attribute__((naked)) __attribute__((section(".init3")));
void get_mcusr(void)
{
	mcucsr_mirror = MCUCSR;
	MCUCSR = 0;
	wdt_disable();
}

extern const uint16_t crc_table[256];

int main()
{
	cli();

	// Prevent us from writing to the bootloader section
	boot_lock_bits_set_safe((1 << BLB11));

	// Setup UART1 for 115200 baud communication
	UBRR1L = 16;
	UBRR1H = 0;
	UCSR1A = (1 << U2X1);
	UCSR1B = (1 << RXEN1) | (1 << TXEN1);
	UCSR1C = (3 << UCSZ10);

	// Setup timer 1 to count the elapsed time
	TCCR1A = 0;
	TCCR1B = 0;
	TCCR1C = 0;
	TIFR = (1 << TOV1); // clear overflow flag
	TCNT1 = 0;
	TCCR1B = (1 << CS12) | (1 << CS10); // prescaler 1024 => 15625 kHz

	// Setup the dxlpro_slave library
	dxlpro_slave_init(BOOTLOADER_DXL_ID, BOOTLOADER_MODEL, BOOTLOADER_VERSION);

	// LED
	DDRB |= (1 << 7);

	// RS485 transmit enable
	PORTD &= ~(1 << 4);
	DDRD |= (1 << 4);

	// Pullups for UART RX pins
	PORTE |= (1 << 0); // RXD0
	PORTD |= (1 << 2); // RXD1

	g_exit = 0;
	while(!g_exit)
	{
		if(UCSR1A & (1 << RXC1))
		{
			uint8_t c = UDR1;
			dxlpro_slave_putc(c);
		}

		if(TIFR & (1 << TOV1))
		{
			// approx. 2s have elapsed without valid DXL 2.0 communication.
			break;
		}

		if(TCNT1 & (1 << 10))
			PORTB |= (1 << 7);
		else
			PORTB &= ~(1 << 7);
	}

	// jump to main code
	boot_rww_enable_safe();
	asm volatile ( "jmp 0x0" );

	return 0;
}

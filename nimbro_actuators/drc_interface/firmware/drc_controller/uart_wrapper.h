/*
 * uart_wrapper.h
 *
 *  Created on: 28.05.2009
 *      Author: waldukat
 *
 *  Used to define a general interface for the uart library
 *  on the crumb2560, the uarts are asigned as follows:
 *  uart0 = ftdi_pc
 *  uart1 = ftdi
 *  uart2 = max232
 *  uart3 = rs485 transceiver
 */

#ifndef UART_WRAPPER_H_
#define UART_WRAPPER_H_

extern "C"
{
#include "uart.h"
}

/*

extern void ftdi_pc_init(unsigned int baudrate);
extern unsigned int ftdi_pc_getc(void);
extern void ftdi_pc_putc(unsigned char data);
extern uint8_t ftdi_pc_bytesWaiting(void);
extern void ftdi_pc_puts(const char *s );
extern void ftdi_pc_puts_p(const char *s );
#define ftdi_pc_puts_P(__s)       ftdi_pc_puts_p(PSTR(__s))

extern void ftdi_init(unsigned int baudrate);
extern unsigned int ftdi_getc(void);
extern void ftdi_putc(unsigned char data);
extern uint8_t ftdi_bytesWaiting(void);
extern void ftdi_puts(const char *s );
extern void ftdi_puts_p(const char *s );
#define ftdi_puts_P(__s)       ftdi_puts_p(PSTR(__s))

extern void max232_init(unsigned int baudrate);
extern unsigned int max232_getc(void);
extern void max232_putc(unsigned char data);
extern uint8_t max232_bytesWaiting(void);
extern void max232_puts(const char *s );
extern void max232_puts_p(const char *s );
#define max232_puts_P(__s)       max232_puts_p(PSTR(__s))

extern void rs485_init(unsigned int baudrate);
extern unsigned int rs485_getc(void);
extern void rs485_putc(unsigned char data);
extern uint8_t rs485_bytesWaiting(void);
extern void rs485_puts(const char *s );
extern void rs485_puts_p(const char *s );
#define rs485_puts_P(__s)       rs485_puts_p(PSTR(__s))

*/

#if defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega640__)
#define ftdi_pc_init uart0_init
#define ftdi_pc_getc uart0_getc
#define ftdi_pc_putc uart0_putc
#define ftdi_pc_bytesWaiting uart0_bytesWaiting
#define ftdi_pc_txWaiting uart0_txWaiting
#define ftdi_pc_puts uart0_puts
#define ftdi_pc_puts_p uart0_puts_p
#define ftdi_pc_puts_P(__s)       ftdi_pc_puts_p(PSTR(__s))
#define ftdi_pc_rx_clear uart0_rx_clear

#define ftdi_init uart1_init
#define ftdi_getc uart1_getc
#define ftdi_putc uart1_putc
#define ftdi_bytesWaiting uart1_bytesWaiting
#define ftdi_txWaiting uart1_txWaiting
#define ftdi_puts uart1_puts
#define ftdi_puts_p uart1_puts_p
#define ftdi_puts_P(__s)       ftdi_puts_p(PSTR(__s))

#define max232_init uart2_init
#define max232_getc uart2_getc
#define max232_putc uart2_putc
#define max232_bytesWaiting uart2_bytesWaiting
#define max232_txWaiting uart2_txWaiting
#define max232_puts uart2_puts
#define max232_puts_p uart2_puts_p
#define max232_puts_P(__s)       max232_puts_p(PSTR(__s))

#define rs485_init uart3_init
#define rs485_getc uart3_getc
#define rs485_putc uart3_putc
#define rs485_bytesWaiting uart3_bytesWaiting
#define rs485_txWaiting uart3_txWaiting
#define rs485_puts uart3_puts
#define rs485_puts_p uart3_puts_p
#define rs485_puts_P(__s)       rs485_puts_p(PSTR(__s))
#define rs485_rx_clear uart3_rx_clear


#define CHECK_CP2101_FINISH bit_is_set(UCSR0A,TXC0)		// TXD Shift register empty
#define CHECK_FTDI_FINISH bit_is_set(UCSR1A,TXC1)
#define CHECK_MAX232_FINISH bit_is_set(UCSR2A,TXC2)
#define CHECK_RS485_FINISH bit_is_set(UCSR3A,TXC3)


#elif defined(__AVR_ATmega64__) || defined(__AVR_ATmega128__)

#define rs485_init uart0_init
#define rs485_getc uart0_getc
#define rs485_putc uart0_putc
#define rs485_bytesWaiting uart0_bytesWaiting
#define rs485_puts uart0_puts
#define rs485_puts_p uart0_puts_p
#define rs485_puts_P(__s)       rs485_puts_p(PSTR(__s))

#define ftdi_pc_init uart1_init
#define ftdi_pc_getc uart1_getc
#define ftdi_pc_putc uart1_putc
#define ftdi_pc_bytesWaiting uart1_bytesWaiting
#define ftdi_pc_puts uart1_puts
#define ftdi_pc_puts_p uart1_puts_p
#define ftdi_pc_puts_P(__s)       ftdi_pc_puts_p(PSTR(__s))

#define ftdi_init uart1_init
#define ftdi_getc uart1_getc
#define ftdi_putc uart1_putc
#define ftdi_bytesWaiting uart1_bytesWaiting
#define ftdi_puts uart1_puts
#define ftdi_puts_p uart1_puts_p
#define ftdi_puts_P(__s)       ftdi_puts_p(PSTR(__s))


#define CHECK_RS485_FINISH bit_is_set(UCSR0A,TXC0)		// TXD Shift register empty
#define CHECK_CP2101_FINISH bit_is_set(UCSR1A,TXC1)
#define CHECK_FTDI_FINISH bit_is_set(UCSR1A,TXC1)

#endif /* defined(__AVR_ATmega64__) || defined(__AVR_ATmega128__) */

#endif /* UART_WRAPPER_H_ */

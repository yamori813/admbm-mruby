/*
 *
 */

#include "cfe/adm5120.h"

#define KSEG1	(volatile char *)0xa0000000

void setbaud(int baud, int port)
{
	unsigned int clkdiv;
	volatile char *portbase;

	if (port == 0)
		portbase = KSEG1 + UART0_BASE;
	else
		portbase = KSEG1 + UART1_BASE;

	/* disable the uart */
	*(volatile char*)(portbase + UART_CR_REG) = 0;

	/* UART_LCR_H_REG must be set after UART_LCR_M_REG, UART_LCR_L_REG */
	clkdiv = UART_BAUDDIV(baud);
	*(volatile char*)(portbase + UART_LCR_M_REG) = (clkdiv >> 8) & 0xff;
	*(volatile char*)(portbase + UART_LCR_L_REG) = clkdiv & 0xff;
	/* 8bit, fifo enable */
	*(volatile char*)(portbase + UART_LCR_H_REG) = UART_WLEN_8BITS | UART_ENABLE_FIFO;

	/* enable */
	*(volatile char*)(portbase + UART_CR_REG) = UART_PORT_EN;
}

int havech(int port)
{
	volatile char* lsr;

	if (port == 0)
		lsr = KSEG1 + UART0_BASE + UART_FR_REG; // Line status register.
	else
		lsr = KSEG1 + UART1_BASE + UART_FR_REG; // Line status register.

	return (*lsr & UART_RX_FIFO_EMPTY) == 0;
}

int getch(int port)
{
	volatile char* lsr;
	volatile char* thr;

	if (port == 0) {
		lsr = KSEG1 + UART0_BASE + UART_FR_REG; // Line status register.
		thr = KSEG1 + UART0_BASE + UART_DR_REG; // Transmitter holding register.
	} else {
		lsr = KSEG1 + UART1_BASE + UART_FR_REG; // Line status register.
		thr = KSEG1 + UART1_BASE + UART_DR_REG; // Transmitter holding register.
	}
	while ((*lsr & UART_RX_FIFO_EMPTY) != 0) ;

	return *thr;
}

void put2(char c)
{
	volatile char* lsr = KSEG1 + UART1_BASE + UART_FR_REG; // Line status register.
	volatile char* thr = KSEG1 + UART1_BASE + UART_DR_REG; // Transmitter holding register.

	while(((*lsr) & UART_TX_FIFO_FULL) != 0) ; // Wait until THR is empty.

	*thr = c;
}

void print2(char *ptr)
{
	while(*ptr) {
		put2(*ptr);
		++ptr;
	}
}

void put(char c)
{
	volatile char* lsr = KSEG1 + UART0_BASE + UART_FR_REG; // Line status register.
	volatile char* thr = KSEG1 + UART0_BASE + UART_DR_REG; // Transmitter holding register.
 
	while(((*lsr) & UART_TX_FIFO_FULL) != 0) ; // Wait until THR is empty.
 
	*thr = c;
}

void print(char *ptr)
{
	while(*ptr) {
		put(*ptr);
		++ptr;
	}
}

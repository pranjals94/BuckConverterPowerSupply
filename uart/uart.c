#include "uart.h"

void uart_init(uint16_t ubrr) {
	// Set baud rate
	UBRRH = (ubrr >> 8);
	UBRRL = ubrr;

	// Enable transmitter and receiver
	UCSRB = (1 << TXEN) | (1 << RXEN);

	// 8-bit data, 1 stop bit, no parity
	UCSRC = (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0);
}

void uart_tx(char data) {
	while (!(UCSRA & (1 << UDRE)));  // Wait for buffer to be empty
	UDR = data;                      // Send data
}

char uart_rx(void) {
	while (!(UCSRA & (1 << RXC)));   // Wait for data
	return UDR;                      // Return received data
}

void uart_print(const char* str) {
	while (*str) {// this terminates if null character is received
		uart_tx(*str++);
	}
}
void uart_println(const char* str) {
	while (*str) {
		uart_tx(*str++);
	}
	uart_print("\n") ; // send new line and carriage return
}
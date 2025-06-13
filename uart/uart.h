#ifndef UART_H
#define UART_H

#include <avr/io.h>

void uart_init(uint16_t ubrr);
void uart_tx(char data);
char uart_rx(void);
void uart_print(const char* str);
void uart_println(const char* str);

#endif

/* sample usage
#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include "uart/uart.h"

below baud with acceptable 0.2% error @16MHz Crystal  (refer Data sheet)
Baud	UBRR	%error
9600	103		0.2%
19.2k	51		0.2%
38.4k	25		0.2%
76.8k	12		0.2%
250k	3		0.0%

#define BAUD 9600
#define UBRR_VALUE ((F_CPU / (16UL * BAUD)) - 1)

int main(void) {
	uart_init(UBRR_VALUE);

	uart_print("Hello from ATmega8!\r\n");

	while (1) {
		char c = uart_rx();      // Wait and receive a character
		uart_tx(c);              // Echo it back
	}
}

//uart_init(3); //250k baud @16mhz

// send data to serial plotter of arduino

char buffer[10];

	// value 1
	uart_print("Temp:");
	itoa(value1, buffer, 10); // convert to string and store in the buffer, base 10 is decimal
	uart_print(buffer);
	uart_print("\t");// use tab to separate between two values
	//value 2
	uart_print("cosine:");
	itoa(value2, buffer, 10); // convert to string, base 10 is decimal
	uart_print(buffer);
	uart_print("\t");// use tab to separate between two values
	// value 3
	uart_print("Light:");
	itoa(value3, buffer, 10);   // convert to string, base 10 is decimal
	uart_print(buffer);
	uart_print("\n");         // end the line , this is necessary
	
	// Convert float to string with 2 decimal places
	dtostrf(filtered_value, 0, 2, buffer);// 0 means no padding(space is filled) only the float number is stored





*/
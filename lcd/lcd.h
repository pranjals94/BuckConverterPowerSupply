#ifndef LCD_H
#define LCD_H

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>

// LCD pin connections
#define LCD_DATA_PORT PORTC
#define LCD_DATA_DDR  DDRC
#define LCD_CONTROL_PORT PORTD
#define LCD_CONTROL_DDR  DDRD
#define RS PD1
#define EN PD0

#define D4 PC2
#define D5 PC3
#define D6 PC4
#define D7 PC5


void lcd_init(void);
void lcd_command(unsigned char cmd);
void lcd_data(unsigned char data);
void lcd_string(const char *str);
void lcd_set_cursor(uint8_t row, uint8_t col);
void lcd_clear(void);
void lcd_create_char(uint8_t location, uint8_t *charmap);
void lcd_create_char_P(uint8_t location, const uint8_t *charmap_P);

#endif

/*
16 by 2 LCD
*/
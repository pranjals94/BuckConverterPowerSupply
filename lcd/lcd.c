#include "lcd.h"

void lcd_command(unsigned char cmd) {
	// only last 4 (pins) bits used, command masking high and low nibble
	//LCD_DATA_PORT = (LCD_DATA_PORT & 0x0F) | (cmd & 0xF0);
	
	// send high nibble
	LCD_DATA_PORT = (LCD_DATA_PORT & 0x3) | ((cmd & 0xF0)>>2);// masking the custom pins
	LCD_CONTROL_PORT &= ~(1 << RS);
	LCD_CONTROL_PORT |= (1 << EN);
	_delay_us(1);
	LCD_CONTROL_PORT &= ~(1 << EN);
	_delay_us(200);

	//send the low nibble
	LCD_DATA_PORT = (LCD_DATA_PORT & 0x03) | (cmd << 2);
	LCD_CONTROL_PORT |= (1 << EN);
	_delay_us(1);
	LCD_CONTROL_PORT &= ~(1 << EN);
	_delay_ms(2);
}

void lcd_data(unsigned char data) {
	//LCD_DATA_PORT = (LCD_DATA_PORT & 0x0F) | (data & 0xF0);
	LCD_DATA_PORT = (LCD_DATA_PORT & 0x3) | ((data & 0xF0)>>2);
	LCD_CONTROL_PORT |= (1 << RS);
	LCD_CONTROL_PORT |= (1 << EN);
	_delay_us(1);
	LCD_CONTROL_PORT &= ~(1 << EN);
	_delay_us(200);

	//LCD_DATA_PORT = (LCD_DATA_PORT & 0x0F) | (data << 4);
	LCD_DATA_PORT = (LCD_DATA_PORT & 0x03) | (data << 2);
	LCD_CONTROL_PORT |= (1 << EN);
	_delay_us(1);
	LCD_CONTROL_PORT &= ~(1 << EN);
	_delay_ms(2);
}

void lcd_create_char(uint8_t location, uint8_t *charmap) {
	if (location < 8) {
		lcd_command(0x40 + (location << 3)); // Set CGRAM address
		for (uint8_t i = 0; i < 8; i++) {
			lcd_data(charmap[i]);
		}
	}
}

// create character from flash memory
void lcd_create_char_P(uint8_t location, const uint8_t *charmap_P) {
	if (location < 8) {
		lcd_command(0x40 + (location << 3));
		for (uint8_t i = 0; i < 8; i++) {
			lcd_data(pgm_read_byte(&charmap_P[i]));
		}
	}
}

void lcd_init(void) {
	LCD_DATA_DDR |=  (1 << D4) | (1 << D5) | (1 << D6) | (1 << D7);
	LCD_CONTROL_DDR |=(1 << RS) | (1 << EN) ;
	//Make sure your LCD RW pin is grounded (write-only).
	_delay_ms(40);         // Wait after power-up
	lcd_command(0x03);      // First 8-bit mode init
	_delay_ms(5);
	lcd_command(0x03);      // Repeat
	_delay_us(150);
	lcd_command(0x03);      // Repeat
	_delay_us(150);

	lcd_command(0x02);  // 4-bit mode
	_delay_ms(2); 
	lcd_command(0x28);  // 2 lines, 5x8 dots
	_delay_ms(2); 
	lcd_command(0x0C);  // Display ON, cursor OFF
	_delay_ms(2); 
	lcd_command(0x06);  // Increment cursor
	_delay_ms(2); 
	lcd_command(0x01);  // Clear display
	_delay_ms(2);
}

void lcd_string(const char *str) {
	while (*str) {
		lcd_data(*str++);
	}
}

void lcd_set_cursor(uint8_t row, uint8_t col) {
	uint8_t row_offsets[] = {0x00, 0x40};
	lcd_command(0x80 | (col + row_offsets[row]));
}

void lcd_clear(void) {
	lcd_command(0x01);
	_delay_ms(2);
}
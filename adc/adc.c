#define F_CPU 16000000UL // always define at the top
#include "adc.h"
#include <util/delay.h>

void adc_init(uint8_t ref, uint8_t prescaler) {
	// Set reference voltage (REFS1:0 in ADMUX)
	ADMUX = (ADMUX & 0x3F) | (ref << 6);  // ref: 0 = AREF, 1 = AVCC, 3 = Internal 2.56V

	// Set prescaler (ADPS2:0 in ADCSRA)
	ADCSRA = (1 << ADEN)| (prescaler & 0x07);  // Enable ADC, set prescaler
}

uint16_t adc_read(uint8_t channel) {
	// Select ADC channel (0–7)
	channel &= 0x07;
	ADMUX = (ADMUX & 0xF8) | channel;
	//_delay_us(10); // Give a tiny delay for MUX to settle

	// Start conversion
	ADCSRA |= (1 << ADSC);

	// Wait for conversion to complete
	while (ADCSRA & (1 << ADSC));

	return ADC;  // ADC = ADCL + (ADCH << 8)
}

// ==================== ADC for dummy read =====================

void adc_select_channel(uint8_t ch) {
	ADMUX = (ADMUX & 0xF0) | (ch & 0x0F);
	//_delay_us(20); // Let mux stabilize
}

uint16_t adc_read_with_dummy(uint8_t ch) {
	adc_select_channel(ch);

	// Dummy read
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));
	//_delay_us(20); // Give time for capacitor to settle
	// Real read
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));

	return ADC;
}

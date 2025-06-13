#ifndef ADC_H
#define ADC_H

#include <avr/io.h>

void adc_init(uint8_t ref, uint8_t prescaler);
uint16_t adc_read(uint8_t channel);
void adc_select_channel(uint8_t ch);
uint16_t adc_read_with_dummy(uint8_t ch);

#endif
/*
ref		Voltage Reference		Description
0		AREF (External)		External voltage on AREF pin
1		AVCC				AVCC with external capacitor on AREF pin
2		(Reserved)			Not used on ATmega8
3		Internal 2.56V		Internal 2.56V reference with capacitor on AREF

?? A capacitor (e.g., 100nF) must be connected to AREF pin when using internal references.


prescaler Parameter – ADC Clock Prescaler
prescaler	ADPS2:0 Bits	Division Factor		ADC Clock (at 16 MHz)		Use
0			000				2					8 MHz						? Too fast (unstable)
1			001				2					8 MHz						? Too fast
2			010				4					4 MHz						? Too fast
3			011				8					2 MHz						? Too fast
4			100				16					1 MHz						? Still too fast
5			101				32					500 kHz						?? Borderline
6			110				64					250 kHz						? Acceptable
7			111				128					125 kHz						? Recommended

? ADC works best at 50 – 200 kHz clock. For F_CPU = 16 MHz, prescaler 7 (128) is best.

Best Choice (for 16 MHz crystal):

adc_init(3, 7);  // internal(2.56V) reference, prescaler 128 (125 kHz ADC clock)

Use twisted pair wires, short wires, or shielded cables for ADC inputs, to reduce noises especially the 50/60Hz

good stability (low noise) while using battery

*/
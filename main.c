/*max 17 volt attained with 1450 OCR1A (dutycycle), ICR1A ICR1 = 1500 (10.5khz freq approx)
tune the ki,kp and the voltage divider constant only if calibration is needed,
Measured at about 26 degree environmental tempr

add maximum allow able current to 2.5 amps due to 3 amps diode restriction.

*/


#define F_CPU 16000000UL // always define at the top
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "uart/uart.h"
#include "lcd/lcd.h"
#include "digitalFilter/digitalFilter.h" // dont' forget to add this. The compiler does not show error any error and filter works unexpected way
#include "adc/adc.h"

const uint8_t smiley[8] PROGMEM = {
	0b00000,
	0b01010,
	0b01010,
	0b00000,
	0b00000,
	0b10001,
	0b01110,
	0b00000
};

const uint8_t Heart[8] PROGMEM  = {
	0b00000,
	0b01010,
	0b11111,
	0b11111,
	0b01110,
	0b00100,
	0b00000,
	0b00000
};

const uint8_t Bell[8] PROGMEM  = {
	0b00100,
	0b01110,
	0b01110,
	0b01110,
	0b11111,
	0b00000,
	0b00100,
	0b00000
};

const uint8_t Plug[8] PROGMEM = {
	0b01010,
	0b11111,
	0b11111,
	0b11111,
	0b01110,
	0b00100,
	0b00100,
	0b00100
};

const uint8_t Battery[8] PROGMEM = {
	0b01110,
	0b11011,
	0b10001,
	0b10001,
	0b11111,
	0b11111,
	0b11111,
	0b11111
};

const uint8_t Lock[8] PROGMEM = {
	0b01110,
	0b10001,
	0b10001,
	0b11111,
	0b11011,
	0b11011,
	0b11111,
	0b00000
};

// pre calculate to reduce time in running code especially float values that take more time.
#define VOLTAGE_SCALE (2.56 * 7.60 / 1023.0) //(vref * voltage divider constant/ adc resolution)7.67, if voltage shown is less than actual voltage decrease it and vice versa
#define CURRENT_SCALE ((2.56 * 1000/ 1023.0) / 0.21) // milli amp scale, (vref/adc resolution)/sense resistance


#define INC_BUTTON_PIN		PB2 // Increment button
#define SET_BUTTON_PIN		PB3  // Set button
#define BUTTON_PORT			PORTB
#define BUTTON_DDR			DDRB
#define BUTTON_PINREG		PINB

// === Control Gains ===
#define KP_volt 0.85f    // Proportional gain
#define KI_volt 0.08f // Integral Gain

#define KP_current 0.02f    // Proportional gain
#define KI_current 0.08f // Integral Gain

// Global or static variable to hold integral term
static float integral_voltage = 0.0, integral_current=0.0;

bool currentMode = false;
volatile uint16_t ADC_value;
char buffer[10];
volatile float filtered_value_current, filtered_value_voltage , setVoltage = 1.20, setCurrent=0.150;// milli amps
// Declare Kalman filters for two channels
KalmanFilter kf_voltage, kf_current;

// timer 0 for multitasking between main and ISR timer0
ISR(TIMER0_OVF_vect) {

ADC_value = adc_read(1);
filtered_value_voltage = kalman_update(&kf_voltage, ADC_value);
float voltage = filtered_value_voltage * VOLTAGE_SCALE;

// === Current Control ===
ADC_value = adc_read(0);
filtered_value_current = kalman_update(&kf_current, ADC_value);
float current = filtered_value_current * CURRENT_SCALE, setCurrent_mA = setCurrent*1000;// in milli amp

if(current>(setCurrent_mA)||currentMode)	{
	// === P Controller ===
	currentMode = true;
	if(current<(setCurrent_mA-50)){currentMode= false;} // hysteresis
	
	float error =(setCurrent_mA - current);
	
	// Integrate the error (accumulate over time)
	integral_current += error;

	// Optional: Anti-windup (clamp the integral to prevent overflow)
	if (integral_current > 10) integral_current = 10;
	if (integral_current < -10) integral_current = -10;

	// PI Control output
	int16_t control =OCR1A + (int16_t)(KP_current * error);// + KI_current * integral_current);

	if (control < 1) control = 0;
	if (control > 1450) control = 1450;

	OCR1A = control;

	} else { // VOLtage mode
	
	//pi control
	float error = setVoltage - voltage;
	
	// Integrate the error (accumulate over time)
	integral_voltage += error;

	// Optional: Anti-windup (clamp the integral to prevent overflow)
	if (integral_voltage > 10) integral_voltage = 10;
	if (integral_voltage < -10) integral_voltage = -10;

	// PI Control output
	int16_t control =OCR1A + (int16_t)(KP_volt * error + KI_volt * integral_voltage);
	
	if (control < 1) control = 0;
	if (control > 1450) control = 1450;

	OCR1A = control;
	}
}


// initialize timer0
/*
CS02 CS01 CS00		Description
0	 0	  0			No clock source (Timer/Counter stopped)
0	 0	  1			clkI/O/(No prescaling)
0	 1	  0			clkI/O/8 (From prescaler)
0	 1	  1			clkI/O/64 (From prescaler)
1	 0	  0			clkI/O/256 (From prescaler)
1	 0	  1			clkI/O/1024 (From prescaler)
*/

void timer0_init(){
	TCCR0 |=(1<<CS00)|(1<<CS01); // timer start Set prescaler, sampling/signal processing rate depends on it
	TIMSK |=(1<<TOIE0); // enable ISR
}

// initialize timer 1
void timer1_pwm_init(void) {
	// Set PB1 (OC1A) as output
	DDRB |= (1<<PB1); // debugging leds
	PORTB |=(1<<PB1); // initially pull high as the switch in active low mode

	// Set Fast PWM mode 14: WGM13=1, WGM12=1, WGM11=1, WGM10=0
	TCCR1A |= (1 << WGM11)|(1 << COM1A0);
	TCCR1B |= (1 << WGM13) | (1 << WGM12);
	
	TCCR1A |= (1 << COM1A0); // Inverting mode OC1A

	// Toggle OC1A (PB1) on Compare Match,
	TCCR1A |= (1 << COM1A1);

	//no prescaler
	TCCR1B |= (1 << CS10);

	// Set TOP value for frequency 1500 around 10.65 khz
	ICR1 = 1500;

	// Set duty cycle, max value (100% duty cycle) will be equal to ICR1 value

	//OCR1A =1450;//max usable pwm duty (active low) 9.2 volt with blue led strip load
	//OCR1A =200;//pwm duty (active low) 1.7 volt with load
	OCR1A =300;
}

void button_init() {
	BUTTON_DDR &= ~((1 << INC_BUTTON_PIN)|(1 << SET_BUTTON_PIN));
	BUTTON_PORT |= (1 << INC_BUTTON_PIN)|(1 << SET_BUTTON_PIN);// enable internal pull up
}

uint8_t is_pressed(uint8_t pinreg, uint8_t pinbit) {
	return !(pinreg & (1 << pinbit));  // Active-low logic
}

void wait_for_release(volatile uint8_t *pinreg, uint8_t pinbit) {
	while (!(*pinreg & (1 << pinbit)));  // Wait for release
	_delay_ms(50);
}

float get_value( float param){
	char digit =0;
	uint8_t bufferIndx = 0;
	lcd_set_cursor(1, 0);
	lcd_string("00.00");
	lcd_set_cursor(1, 0);
	//lcd_command(0x0F);  // 00001111: Display ON, Cursor On, Blink ON
	lcd_command(0x0D); //Display ON, Cursor OFF, Blink ON
	while(1){
		if (is_pressed(BUTTON_PINREG, INC_BUTTON_PIN)) {
			_delay_ms(15);
			wait_for_release(&BUTTON_PINREG, INC_BUTTON_PIN);
			digit++;
			if(digit>9){digit=0;}
			lcd_data(digit+48);
			lcd_set_cursor(1, bufferIndx);
		}
		
		if (is_pressed(BUTTON_PINREG, SET_BUTTON_PIN)) {
			_delay_ms(25); // debounce delay
			//--detect button long press ---
			if (is_pressed(BUTTON_PINREG, SET_BUTTON_PIN)) {
				uint16_t hold_time = 0;

				// Count how long button is held
				while (is_pressed(BUTTON_PINREG, SET_BUTTON_PIN)) {
					_delay_ms(10);
					hold_time += 10;

					if (hold_time >= 800) {  // Long press threshold
						lcd_command(0x0C);  // 00001100: Display ON, Cursor OFF, Blink OFF
						return param ;  // Long press
					}
				}
			}
			//---long press end-----

			buffer[bufferIndx++]=digit+48;
			digit = 0;
			if(bufferIndx==2){
				buffer[bufferIndx++]='.';
			}
			lcd_set_cursor(1, bufferIndx);
			if (bufferIndx>4){
				buffer[bufferIndx] = '\0';  // Add null terminator at the end
				break;
			}
		}
	}
	lcd_command(0x0C);  // 00001100: Display ON, Cursor OFF, Blink OFF
	return (float)atof(buffer) ;
}

void setUp(){
	lcd_clear();
	lcd_set_cursor(0, 0);
	lcd_string("S Volt <");
	lcd_set_cursor(1, 0);
	lcd_string("S Amps");
	uint8_t selection = 1;
	float temp;
	wait_for_release(&BUTTON_PINREG, SET_BUTTON_PIN);
	
	while (1) {
		if (is_pressed(BUTTON_PINREG, INC_BUTTON_PIN)) {
			_delay_ms(15);
			selection++;
			if (selection==2){
				lcd_set_cursor(0, 7);
				lcd_string(" ");
				lcd_set_cursor(1, 7);
				lcd_string("<");
			}
			
			if (selection>2) {selection=1;
				lcd_set_cursor(0, 7);
				lcd_string("<");
				lcd_set_cursor(1, 7);
				lcd_string(" ");
			}
			wait_for_release(&BUTTON_PINREG, INC_BUTTON_PIN);
		}

		if (is_pressed(BUTTON_PINREG, SET_BUTTON_PIN)) {
			_delay_ms(15);
			wait_for_release(&BUTTON_PINREG, SET_BUTTON_PIN);
			lcd_clear();
			if(selection==1){
				lcd_set_cursor(0, 0);
				//lcd_string("set (V)");
				dtostrf(setVoltage, 0, 2, buffer);
				//lcd_set_cursor(0,9);
				lcd_string(buffer);
				lcd_string(" Vs ");
				temp = get_value(setVoltage);
				if (temp>18){
					lcd_set_cursor(1,0);
					lcd_string("Denied ");
					lcd_data(2); // bell icon
					}else{
					setVoltage=temp;
					lcd_set_cursor(1,0);
					lcd_string("  Ok.  ");
					lcd_data(2); // lock icon
				}
			}
			if(selection==2){
				lcd_set_cursor(0, 0);
				//lcd_string("set (A)");
				dtostrf(setCurrent, 0, 2, buffer);
				//lcd_set_cursor(0,9);
				lcd_string(buffer);
				lcd_string(" As");
				temp = get_value(setCurrent);
				if (temp>5){
					lcd_set_cursor(1,0);
					lcd_string("Denied ");
					lcd_data(2); // bell icon
					}else{
					setCurrent=temp;
					lcd_set_cursor(1,0);
					lcd_string("  Ok.  ");
					lcd_data(2); // lock icon
				}
			}
			_delay_ms(500);
			break;
		}
	}
}


int main(void) {
	
	DDRB |=(1<<PB0);
	PORTB |= (1<<PB0);
	_delay_ms(500);
	PORTB &=~(1<<PB0);
	
	button_init();
	
	lcd_init();
	
	lcd_create_char_P(0, smiley);     // Store smiley at index 0
	lcd_create_char_P(1, Heart);     // Store smiley at index 1
	lcd_create_char_P(2, Bell);     // Store smiley at index 2
	lcd_create_char_P(3, Plug);     // Store smiley at index 3
	lcd_create_char_P(4, Battery);  // Store smiley at index 4
	lcd_create_char_P(5, Lock);     // Store smiley at index 5
	
	lcd_set_cursor(0,0);
	lcd_data(0);// print smiley
	lcd_string(" Lab  ");
	lcd_data(1);// print heart
	lcd_set_cursor(1,0);
	lcd_data(3);// print plug
	lcd_string(" Power");
	lcd_data(4);// print battery
	_delay_ms(2000);
	
	adc_init(3, 7);  // internal(2.56V)(measure with multimeter in AREF pin for real time voltage) reference, prescaler 128 (125 kHz ADC clock)
	
	// Initialize filters
	// kalman_init(KalmanFilter* kf, float estimate, float err_estimate, float err_measurement, float alpha_lowpass)
	kalman_init(&kf_voltage, 0.0, 5, 50, 0.5);
	kalman_init(&kf_current, 0.0, 5, 50, 0.5);

	timer1_pwm_init();    // Initialize Timer1 for pwm output
	timer0_init(); // initialize timer0 for multitasking
	sei();                // Enable global interrupts
	
	while (1) {
	
		PORTB ^= (1<<PB0);// debuging led
		
		if (is_pressed(BUTTON_PINREG, SET_BUTTON_PIN)) {
			//wait_for_release(&BUTTON_PINREG, SET_BUTTON_PIN);
			setUp();
			lcd_clear();
		}
		dtostrf(filtered_value_voltage * VOLTAGE_SCALE, 0, 2, buffer); // calculate (47k and 10k)voltage and scale up using multiplier and a voltage divider
		lcd_set_cursor(0,0);
		lcd_string(buffer);
		lcd_string(" Vr ");

// 		dtostrf(OCR1A , 0, 0, buffer); // calculate (47k and 10k)voltage and scale up using multiplier and a voltage divider
// 		lcd_set_cursor(0,0);
// 		lcd_string(buffer);
// 		lcd_string(" ");

		
		dtostrf((filtered_value_current * CURRENT_SCALE)/1000, 0, 3, buffer); //
		lcd_set_cursor(1,0);
		lcd_string(buffer);
		lcd_string(" Ar ");
		
		if(currentMode){
			lcd_set_cursor(1,5);
			lcd_data(2);// print bell
			}else{
			lcd_set_cursor(1,5);
			lcd_data(32);// print space
		}
	}
}
/*
The actual values of the resistors may vary so check it with multimeter
while prototying in bread board make sure the connections are rigid else readings jitters and varies.
better to make the circuit in zero pcb for prototyping

+15V
|
[ R1 ] 47k
|
[ R2 ] 10k
|
GND|
ADC Pin (reads voltage across R2)

//--------------
0.038 ohm enamelled coper wire wound in inductance cancellation mode.
has a rc filter in the analog signal, calibrated with a 20k pot and 1 uf electrolytic capacitor
also using a simple proportional controller

//------------------------------
// use non polarized capacitor for bootstrap circuit for mosfet gate driving if possible
*/
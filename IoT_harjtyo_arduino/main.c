/*
 * IoT_harjtyo_arduino.c
 *
 * Created: 28.11.2019 13:46:59
 * Author : s08865
 */ 
/*
This c-program measures temperature from one LM36 sensor and sends it via UART to raspberry pi.
It also controls a LED with PWM depending on commands sent from raspberry pi via UART, LED brightness can be adjusted by
sending "+" and "-" from raspberry. In this case the LED is just a component to show different PWM levels sent from the
raspberry, it could be replaced for example with a heating/cooling machine and which could be similarly controlled from raspberry.
*/

#define F_CPU 16000000UL
#define BAUD 9600
#include <avr/io.h>
#include <avr/portpins.h>
#include <avr/interrupt.h>
#include <util/setbaud.h>
#include <util/delay.h>
#include <stdio.h>
#include <math.h>

// Defines for PWM values
#define PWM_0 0
#define PWM_1 51
#define PWM_2 102
#define PWM_3 153
#define PWM_4 204
#define PWM_5 255

// LCD Defines
#define LCD_RS_DDR DDRD
#define LCD_RS_PIN PD2		// LCD Register Select
#define LCD_RS_BIT PORTD2
#define LCD_RS_PORT PORTD

#define LCD_EN_DDR DDRD
#define LCD_EN_PIN PD3		// LCD Enable
#define LCD_EN_BIT PORTD3
#define LCD_EN_PORT PORTD

#define LCD_DDR DDRD		// LCD DDR

#define LCD_D4_BIT PORTD4	// LCD data port pins
#define LCD_D5_BIT PORTD5
#define LCD_D6_BIT PORTD6
#define LCD_D7_BIT PORTD7

#define LCD_D4_PORT PORTD	// LCD data port
#define LCD_D5_PORT PORTD
#define LCD_D6_PORT PORTD
#define LCD_D7_PORT PORTD

#define LCD_D4_DDR DDRD		// LCD data register
#define LCD_D5_DDR DDRD	
#define LCD_D6_DDR DDRD
#define LCD_D7_DDR DDRD

#define B_SET(byte, bit) byte |= (1<<bit)		// macro to set a bit
#define B_CLR(byte, bit) byte &= (~(1<<bit))	// macro to clear a bit

#define LCD_LineOne 0x00	// LCD module lines
#define LCD_LineTwo 0x40

#define LCD_Clear 0b00000001	// replace all characters with ASCII space
#define LCD_Home 0b00000010		// return cursor to the first position on first line
#define LCD_EntryMode 0b00000110	// shift cursor from left to right on read/write
#define LCD_DisplayOff 0b00001000	// turn display off
#define LCD_DisplayOn 0b00001100	// turn display on, cursor off
#define LCD_FunctionReset 0b00110000	// reset the LCD
#define LCD_FunctionSet4Bit 0b00101000	// 8-bit data, 2-line display, 5x7 font
#define LCD_SetCursor 0b10000000	// set cursor position

// Funtions for LCD
void LCD_write(uint8_t);
void LCD_write_instruction(uint8_t);
void LCD_write_char(uint8_t);
void LCD_write_string(uint8_t *);
void LCD_init(void);

volatile uint8_t gcPWM = 0;

// Functions for UART
void UART0_init (void)
{
	UBRR0H = UBRRH_VALUE;	// baud rate high value (8 bits set in setbaud.h)
	UBRR0L = UBRRL_VALUE;	// low value

	UCSR0B |= (1<<TXEN0) |(1<<RXEN0) |(1<<RXCIE0);	// enable receiver and transmitter
	UCSR0C |= (1<<UCSZ00)|(1<<UCSZ01);	// 8 bit data format, (default: 1 stop bit, no parity)
}

void UART0_transmit (uint8_t *pData)
{
	while(*pData) {	// parameter is string data, MUST contain null at the end!
		while (!( UCSR0A & (1<<UDRE0)));	// wait while register is free i.e. until bit UDRE2 in register UCSR2A becomes 1

		UDR0 = *pData;	// load data in the register
		pData++;
	}
}

uint8_t UART0_receive (void)
{
	while (!((UCSR0A) & (1<<RXC0)));	// wait while data is being received i.e. until
										//bit RXC1 in register UCSR2A becomes 1
	return UDR0;	// return 8-bit data
}

// Function for ADC reading
uint16_t getTempIn()
{
	uint16_t value;		// variable for ADC
	ADCSRA |= (1<<ADEN);	// ADC on
	_delay_ms(10);		// wait for sensor
	ADCSRA |= (1<<ADSC);	// start conversion
	while(ADCSRA&(1<<ADSC));	// wait for conversion to be ready
	value = ADC;	// read the value

	ADCSRA &= ~(1<<ADEN);	// ADC off
	
	return(value);	// return ADC value
}

// Functions for float to string conversion
void reverse(char* str, int len)
{
	int i = 0, j = len - 1, temp;
	while (i < j) {
		temp = str[i];
		str[i] = str[j];
		str[j] = temp;
		i++;
		j--;
	}
}

int intToStr(int x, char str[], int d)
{
	int i = 0;
	while (x) {
		str[i++] = (x % 10) + '0';
		x = x / 10;
	}

	while (i < d)	// If number of digits required is more, then
	str[i++] = '0';	// add 0s at the beginning

	reverse(str, i);
	str[i] = '\0';
	return i;
}

void ftoa(float n, char* res, int afterpoint)
{
	int ipart = (int)n;		// Extract integer part
	
	float fpart = n - (float)ipart;		// Extract floating part

	int i = intToStr(ipart, res, 0);	// convert integer part to string

	if (afterpoint != 0) {	// check for display option after point
		res[i] = '.';		// add dot
		fpart = fpart * pow(10, afterpoint);	// Get the value of fraction part up to given no.
		intToStr((int)fpart, res + i + 1, afterpoint);	// of points after dot. The third parameter
														// is needed to handle cases like 233.007
	}
}

// Main starts
int main(void)
{
	int tempInsideC;
	float voltsin;
	float tempin;
	
	char str1[100];

	// Timer1 setup
	TCCR1A = (1<<COM1A1) | (1<<WGM10);	// toggle OC1A on compare match, 8-bit phase corrected PWM
	OCR1A = PWM_0;		// start with PWM duty cycle 0% (no output)
	TCCR1B = (1<<CS10) | (1<<CS11);		// Timer1 prescaler 8 (PWM 16MHz/8*(2*255)=3,9kHz), start = PWM

	// pins for data lines
	LCD_D7_DDR |= (1<<LCD_D7_BIT);	// 4 data lines as output
	LCD_D6_DDR |= (1<<LCD_D6_BIT);
	LCD_D5_DDR |= (1<<LCD_D5_BIT);
	LCD_D4_DDR |= (1<<LCD_D4_BIT);

	// pins for control lines
	LCD_EN_DDR |= (1<<LCD_EN_BIT);	// control lines as output
	LCD_RS_DDR |= (1<<LCD_RS_BIT);

	UART0_init();
	LCD_init();

	// port inputs and outputs
	DDRB = 0xFF;	// DDRB register as outputs
	DDRC &= ~(PC5);	// DDRC pin #A5 input
	DDRC |= (1<<PC4);	// DDRC pin #A4 output

    // Setup ADC
	ADMUX |= (1<<MUX1) | (1<<MUX0);		// input on ADC0 (Uno #A3)
	ADMUX |= (1<<REFS0);		// reference voltage 5V
	ADCSRA |= ((1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0));	// prescaler 128: 16MHz/128= 125kHz
	ADCSRA &= ~(1<<ADATE);		// Autotrigger off, not needed
	ADCSRB &= ~((1<<ADTS2) | (1<<ADTS1) | (1<<ADTS0));	// freerunning mode, no interrupts, read all the time

	sei();

    while (1)
    {
		tempInsideC = getTempIn();	// variable to catch ADC value (10 bit value)

		voltsin = tempInsideC * 5.0;	// 10 bit value to degrees in celsius
		voltsin /= 1024.0;
		tempin = (voltsin - 0.5) * 100;

		ftoa(tempin, str1, 1);	// change celsius value to string

		LCD_write_instruction(LCD_SetCursor | LCD_LineOne);	// LCD to beginning of line 1
		_delay_us(80);
		
		// write string to 1st line
		LCD_write_string("Temperature");

		LCD_write_instruction(LCD_SetCursor | LCD_LineTwo);	// LCD to beginning of 2nd line
		_delay_us(80);

		LCD_write_string("In: ");	// write string to 2nd line
		LCD_write_string(str1);		// print the temperature to 2nd line as string

		_delay_ms(1000);

		UART0_transmit("I");	// send "I" to raspberry pi before next temperature value
		UART0_transmit(str1);	// send temperature to raspberry pi as string

		switch(gcPWM) {		// switch case for LED PWM different brightness levels
			case 0:
				OCR1A = PWM_0;
				OCR1B = PWM_0;
			break;

			case 1:
				OCR1A = PWM_1;
			break;

			case 2:
				OCR1A = PWM_2;
			break;

			case 3:
				OCR1A = PWM_3;
			break;
			
			case 4:
				OCR1A = PWM_4;
			break;

			case 5:
				OCR1A = PWM_5;
			break;

			default:
			break;
		}
    }
}
// Main ends

// Interrupts
ISR(USART_RX_vect)
{
	uint8_t RxChar = UDR0;
	UDR0 = RxChar;

	PORTB = (1<<PB5);

	if(RxChar == '+') {
		if (gcPWM == 5) {
			gcPWM = 5;
		}
		else {
			gcPWM++;
		}
	}
	else if (RxChar == '-') {
		if (gcPWM == 0) {
			gcPWM = 0;
		}
		else {
			gcPWM--;
		}
	}
}

// Functions needed for LCD
// Init
void LCD_init(void)
{
	_delay_ms(100);

	LCD_RS_PORT &= ~(1<<LCD_RS_BIT);
	LCD_EN_PORT &= ~(1<<LCD_EN_BIT);

	LCD_write(LCD_FunctionReset);
	_delay_ms(10);

	LCD_write(LCD_FunctionReset);
	_delay_us(200);

	LCD_write(LCD_FunctionReset);
	_delay_us(200);

	LCD_write(LCD_FunctionSet4Bit);
	_delay_us(80);

	LCD_write_instruction(LCD_FunctionSet4Bit);
	_delay_us(80);

	LCD_write_instruction(LCD_DisplayOff);
	_delay_us(80);

	LCD_write_instruction(LCD_Clear);
	_delay_ms(4);

	LCD_write_instruction(LCD_EntryMode);
	_delay_us(80);

	LCD_write_instruction(LCD_DisplayOn);
	_delay_us(80);
}

void LCD_write_string(uint8_t str[])
{
	volatile int i = 0;
	while (str[i] != 0)
	{
		LCD_write_char(str[i]);
		i++;
		_delay_us(80);
	}
}

void LCD_write_char(uint8_t data)
{
	LCD_RS_PORT |= (1<<LCD_RS_BIT);
	LCD_EN_PORT |= (1<<LCD_EN_BIT);
	LCD_write(data);
	LCD_write(data << 4);
}

void LCD_write_instruction(uint8_t cmnd)
{
	LCD_RS_PORT &= ~(1<<LCD_RS_BIT);
	LCD_EN_PORT &= ~(1<<LCD_EN_BIT);
	LCD_write(cmnd);
	LCD_write(cmnd << 4);
}

void LCD_write(uint8_t byte)
{
	LCD_D7_PORT &= ~(1<<LCD_D7_BIT);
	if (byte & 1<<7) LCD_D7_PORT |= (1<<LCD_D7_BIT);

	LCD_D6_PORT &= ~(1<<LCD_D6_BIT);
	if (byte & 1<<6) LCD_D6_PORT |= (1<<LCD_D6_BIT);

	LCD_D5_PORT &= ~(1<<LCD_D5_BIT);
	if (byte & 1<<5) LCD_D5_PORT |= (1<<LCD_D5_BIT);

	LCD_D4_PORT &= ~(1<<LCD_D4_BIT);
	if (byte & 1<<4) LCD_D4_PORT |= (1<<LCD_D4_BIT);

	// write data
	LCD_EN_PORT |= (1<<LCD_EN_BIT);
	_delay_us(1);
	LCD_EN_PORT &= ~(1<<LCD_EN_BIT);
	_delay_us(1);
}

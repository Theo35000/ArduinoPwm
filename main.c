#include <avr/io.h>  // Pour les définitions des registres
#include<stdio.h>
#include<stdlib.h>
#include <string.h>
#include<avr/delay.h>
#include <avr/interrupt.h>  /* Pour les interuptions*/
#include<stdarg.h>// POur fonction variadique
#include <inttypes.h> // make easier using of the type for variable because they are different on eclipse and arduino
#include <avr/sleep.h>

#define DDR_SPI DDRB	/* SPI PORTB */
#define DD_MOSI 3	/* MISO : PB4*/
#define DD_SCK 5	/* SCK : PB5 */
// Definition of constant for the PID
#define Kp 20
#define Ki 10
#define Kd 0
#define UBRR_VALUE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)
#define FOSC 16000000                       // Clock Speed
#define BAUD 9600
#define MYUBRR FOSC/16/BAUD -1
// Prototype
void
initPort(void);
void
init_pwm_timer(void);
void usartTransmit(unsigned char data);
void usartInitialise();
void TWI_init();
void TWIStart(void);
void TWIWrite(uint8_t u8data);
void TWIStop(void);
//asservisement function every 20ms
void Timer1_asserv_init(void);
void my_print(char *f);
//Timer un counter mode for count the motor tick

/* Volatile Variable */
//volatile char ReceivedChar;
volatile char etat = '0';
volatile int inc, Vitesse, Dist_tot = 0;
volatile int Consigne_V = 2000; // 25 cm/s
volatile int nb_tick = 0;
volatile char receivechar;
volatile char a = 0;
volatile int data;
volatile char buffer[50];
/* Don't forget to chek if internal pull up don't nessesit more resistor for I2C
 *
 *
 *
 *
 *  */

int main() {
	int i = 0;
	initPort();
	PRR1 |= 1 << PRUSB; // When we allow interupt with sei() The the AVR gets an USB interrupt that reset the AVR, it's a bif problem

	// motor stop

	//MotorSens 1=advance 0= back    in PD4
	//PORTD |= (1 << PD4); // Sens advance E1 (PD4) = 1 */
	PORTE |= (1 << PORT6); /*sens advance E2 (PE6) = 1 */

	usartInitialise();
	TWI_init();
	Timer1_asserv_init();
	init_pwm_timer(); // 3.7khz 8 bit timer
	OCR3A = 0;
	OCR4D = 0;
	sei();
	// vectors off to __bad_interupt(), which ... restarts the application.
	// enable interrupts
//	OCR4D=150;

//	do {
//	for (i = 0; i < 50; i++) {
//		//OCR3A = i;
//		OCR4D = i;
//		_delay_ms(30);
//	}
//	_delay_ms(2000);
//	for (i = 50; i > 0; i--) {
//		//OCR3A = i;
//		OCR4D = i;
//		_delay_ms(30);
//	}
//	OCR3A = 0;
//	OCR4D = 0;
	while (1){

	}


	//_delay_ms(2000); // to not busy the CPU to much
//	} while (1);
}

void TWI_init() {
	TWSR = 0x00;    // no prescaler
	TWBR = 72; // 100kHz @16Mhz
	//TWBR = 0x0C;    // 400 KHz @16MHz
	TWCR = (1 << TWEA) | (1 << TWEN) | (1 << TWIE) | (1 << TWINT);

}

void initPort(void) {
	DDRC |= (1 << PORTC7); //LED

	// Enable output for choose motor 1 sens//
	DDRD |= (1 << PORTD4);

	DDRD &= ~(1 << PORTD2); // initialize pin PD2 input pin -> Rx for reception
	DDRD |= (1 << PORTD3); // initialize pin PD3 output pin -> Tx for transmission
	PORTD = (1 << PD0) | (1 << PD1); // activate internal pull_ups for twi
	/* enable pin of Timer 3A out ( Digit 5) */
	DDRC |= (1 << PORTC6);
	/* enable pin of Timer 4b out ( Digit 6) */
	DDRD |= (1 << PORTD7);

	PORTB|=(1<<PB0)|(1<<PB1);
}
/// of the timer for the pwm, We need to set a top for make it a
void init_pwm_timer(void) {

	/*~~~~~~~~~~~~~~| Timer 3 |~~~~~~~~~~~~~~*/
	/* set timer3 prescale factor to 64 */
	TCCR3B = (1 << CS31) | (1 << CS30);
	/* put timer3 in phase correct PWM mode */
	TCCR3A = (1 << WGM30);

	/* enable OC3A pin as PWM pin */
	TCCR3A |= (1 << COM3A1);
	/*~~~~~~~~~~~~~~| Timer 4 |~~~~~~~~~~~~~~*/
	TC4H = 0;
	/* set timer4 prescale factor to 64 */
	TCCR4B = (1 << CS42) | (1 << CS41) | (1 << CS40);
	/* put timer 4 in phase and frequency correct PWM mode */
	TCCR4D = (1 << WGM40);
	/* not used */
	TCCR4E = 0;
	/* Set top to 0xFF */
	OCR4C = 0xFF;

	/* enable OC4D pin as PWM pin */TCCR4C |= (1 << PWM4D);
	TCCR4C |= (1 << COM4D1);

}
void TWIStart(void) {
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)))
		;
//while (TWSR!=0x08);

}
//send stop signal
void TWIStop(void) {
	TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}
//F=100hz, every 10ms more or less
void TWIWrite(uint8_t u8data) {
	TWDR = u8data;
	TWCR = (1 << TWINT) | (1 << TWEN); // Send data or adress

	while ((TWCR & (1 << TWINT)) == 0)
		// Wait ack
		;


}
void usartInitialise() {
	/*Set baud rate */

	UBRR1H = (MYUBRR >> 8);
	UBRR1L = MYUBRR;

	UCSR1B |= (1 << RXEN1) | (1 << TXEN1);    // Enable receiver and transmitter
	UCSR1B |= (1 << RXCIE1);                    // Enable receiver interrupt
	UCSR1C |= (1 << UCSZ11) | (1 << UCSZ10);    // Set frame: 8data, 1 stp

}
void init_USART(void) {

	UBRR1 = 103; /* 9600 Baud at 16MHz */

	UCSR1A = 0;

	//8 bits no parity, 1 stop bit
	UCSR1C = (1 << UCSZ11) | (1 << UCSZ10);
	UCSR1B |= (1 << RXEN1); // receive enable
	/* enable interrupt on the RXC1 flag */
	UCSR1B |= (1 << RXCIE1);
	SREG |= (1 << SREG_I);

}
ISR(USART1_RX_vect) {

	receivechar = UDR1;

}
ISR(TWI_vect) {
	 switch(TWSR)
	 {
	 case 0x38 :
		 TWDR = data;
		 break;
	 case 0x40 :
		 TWDR = data;
		 break;
	 case 0x48 :
		 TWDR = data;
		 break;
	 }
	  TWCR |= (1 << TWINT); // TWINT flag bit is cleared


}
ISR(TIMER1_COMPA_vect) {

	float static S_error = 0, error = 0, D_error = 0, PWM = 0, last_error = 0;
	double static tick;
	float static Dist;

	TWIStart(); // envois start et adresse

	TWIWrite(0b00000000); // Address brodcast
	nb_tick = data;
TWIStop();

	my_print("Nombre de tick   : ");
	sprintf(buffer, "%d", nb_tick);
	my_print(buffer);
	my_print("\n \n \r");
	tick = nb_tick; // Storage number of tick
	nb_tick = 0; // clean count of tick
	Dist = nb_tick * 0.005852; // To calculate
	Vitesse = Dist * 10; // vitesse en cm/s ?
	Dist_tot += Dist;

	error = Consigne_V - Vitesse;
	my_print("error   : ");
	sprintf(buffer, "%d", error);
	my_print(buffer);
	my_print("\n \n \r");
	S_error += error;
	D_error = error - last_error;
	last_error = error;

	PWM = error * Kp + S_error * Ki + D_error * Kd;
	if (PWM > 255)
		PWM = 255;
	if (PWM < 0)
		PWM = 0;
	//OCR3A = (char) (PWM);
	my_print("commande  : ");
	sprintf(buffer, "%d", OCR3A);
	my_print(buffer);
	my_print("\n \n \r");

}
/// Interupt every 20ms more or less
void Timer1_asserv_init() {
	DDRC |= 0x80;
	cli();
	// disable global interrupts
	TCCR1A = 0;     // set entire TCCR1A register to 0
	TCCR1B = 0;     // same for TCCR1B
	// set compare match register to desired timer count
	OCR1A = 3125;	//15624; //312; //15624;
	// turn on CTC mode:
	TCCR1B |= (1 << WGM12);
	// Set CS10 and CS12 bits for 1024 prescaler
	TCCR1B |= (1 << CS10);
	TCCR1B |= (1 << CS12);
	// enable timer compare interrupt
	TIMSK1 |= (1 << OCIE1A);

}

void usartTransmit(unsigned char data) {
	while (!(UCSR1A & (1 << UDRE1)))
		;
	UDR1 = data;
}
void my_print(char *f) {
	int i = 0;
	while (f[i]) {
		usartTransmit(f[i++]);
//      _delay_ms(50);
	}
}

/* Tranmision et reception */

//void
//usartTransmit (unsigned char data)
//{
//  // wait for empty transmit buffer
//  while (!(UCSR0A & (1 << UDRE0)))
//    ;
//  // put data info buffer, sends the data
//  UDR0 = data;
//}

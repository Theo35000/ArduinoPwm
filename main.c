#include <avr/io.h>  // Pour les définitions des registres
#include<stdio.h>
#include<stdlib.h>
#include <string.h>
#include<avr/delay.h>
#include <avr/interrupt.h>  /* Pour les interuptions*/
#include<stdarg.h>// POur fonction variadique
#include <inttypes.h> // make easier using of the type for variable because they are different on eclipse and arduino
#include <avr/sleep.h>
// Definition of constant for the PID
#define Kp 10
#define Ki 0
#define Kd 0

// Prototype
void
initPort(void);
void
init_pwm_timer(void);
void
init_USART(void);
void TWI_init();
//asservisement function every 20ms
void Timer1_asserv_init(void);
//Timer un counter mode for count the motor tick

/* Volatile Variable */
//volatile char ReceivedChar;
volatile char etat = '0';
volatile int inc, Vitesse, Dist_tot = 0;
volatile int Consigne_V = 25; // 25 cm/s
volatile double nb_tick = 0;
volatile char receivechar;
volatile char a = 0;
volatile int data;
int main() {
	initPort();

	Timer1_asserv_init();
	init_pwm_timer(); // 3.7khz 8 bit timer
	// motor stop
	OCR3A = 150;
	//MotorSens 1=advance 0= back    in PD4
	PORTD |= (1 << PD4); // Sens advance
//salut

	init_USART();
	TWI_init();

	PRR1 |= 1<<PRUSB; // When we allow interupt with sei() The the AVR gets a USB interrupt and
	sei();           // vectors off to __bad_interupt(), which ... restarts the application.
	// enable interrupts
//	OCR4D=150;








	while(1){

		_delay_ms(20); // to not busy the CPU to much
	}

}


void TWI_init()
{
	  TWSR = 0x00;    // no prescaler
	  TWBR = 72; // 100kHz @16Mhz
	  //TWBR = 0x0C;    // 400 KHz @16MHz
	  TWCR = (1 << TWEA) | (1 << TWEN) | (1 << TWIE) | (1 << TWINT);
}

void initPort(void) {
	DDRC |= (1 << PORTC7); //LED

	/* enable pin of Timer 3A out ( Digit 5) */
	DDRC |= (1 << PORTC6);

	// Enable output for choose motor 1 sens//
	DDRD |= (1 << PORTD4);

	DDRD &= ~(1 << PORTD2); // initialize pin PD2 input pin -> Rx for reception
	DDRD |= (1 << PORTD3); // initialize pin PD3 output pin -> Tx for transmission
	PORTD = (1 << PD0) | (1 << PD1); // activate internal pull_ups for twi
}
void init_pwm_timer(void) {
	/* set timer3 prescale factor to 64 */
	TCCR3B = (1 << CS31); //| (1 << CS30);
	/* put timer3 in phase correct PWM mode */
	TCCR3A = (1 << WGM30);
	TC4H = 0;
	/* enable OC3A pin as PWM pin */
	TCCR3A |= (1 << COM3A1);

}
void TWIStart(void)
{
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));
}
//send stop signal
void TWIStop(void)
{
    TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
}
//F=100hz, every 10ms more or less
void TWIWrite(uint8_t u8data)
{
    TWDR = u8data;
    TWCR = (1<<TWINT)|(1<<TWEN);
    while ((TWCR & (1<<TWINT)) == 0);
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
ISR(TWI_vect)
{
	  TWDR = data;
}
ISR(TIMER1_COMPA_vect) {



		float S_error = 0, error = 0, D_error = 0, PWM = 0, last_error = 0;
		double tick;
		float Dist;

		TWIStart(); // envois start et adresse

		TWIWrite(0b00000000); // Address brodcast
		TWIWrite(1); // Adresse memoire

		_delay_ms(10);
		nb_tick=data;

		TWIStop();
if (nb_tick>3)
{
	PORTC|=(1<<PORTC7);
}
		tick=nb_tick;// Storage number of tick
		nb_tick=0; // clean count of tick
		Dist = nb_tick* 0.005852; // To calculate
		Vitesse = Dist * 10; // vitesse en cm/s ?
		Dist_tot += Dist;

		error = Consigne_V - Vitesse;
		S_error += error;
		D_error = error - last_error;
		last_error = error;

		PWM = error * Kp + S_error * Ki + D_error * Kd;
		if (PWM > 255)
			PWM = 255;
		if (PWM < 0)
			PWM = 0;
		OCR3A = (int) (PWM);


}
void Timer1_asserv_init() {
    DDRC |= 0x80;
    cli();          // disable global interrupts
    TCCR1A = 0;     // set entire TCCR1A register to 0
    TCCR1B = 0;     // same for TCCR1B
    // set compare match register to desired timer count
    OCR1A =312; //15624;
    // turn on CTC mode:
    TCCR1B |= (1 << WGM12);
    // Set CS10 and CS12 bits for 1024 prescaler
    TCCR1B |= (1 << CS10);
    TCCR1B |= (1 << CS12);
    // enable timer compare interrupt
    TIMSK1 |= (1 << OCIE1A);

}
//void
//my_print (char *f)
//{
//  int i = 0;
//  while (f[i])
//    {
//      usartTransmit (f[i++]);
////      _delay_ms(50);
//    }
//}

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


#include <avr/io.h>  // Pour les définitions des registres
#include<stdio.h>
#include<stdlib.h>
#include <string.h>
#include<avr/delay.h>
#include <avr/interrupt.h>  /* Pour les interuptions*/
#include<stdarg.h>// POur fonction variadique
#include <inttypes.h>

// Definition of constant for the PID
#define Kp 10
#define Ki 0
#define Kd 0

// Prototype
//void
//my_print(char *f);
//void
//usartInitialise();
//void
//interupt_extern();
//void
//usartTransmit(unsigned char data);
void
initPort(void);
void
init_pwm_timer(void);
void
init_USART(void);
void Timer1_asserv_init();
/* Volatile Variable */
//volatile char ReceivedChar;
volatile char etat = '0';
volatile int inc, Vitesse, Dist_tot = 0;
volatile int Consigne_V = 25; // 25 cm/s
volatile double nb_tick = 0;
volatile char receivechar;
void main() {

	initPort();
	//Timer1_asserv_init();
	//init_pwm_timer(); // 3.7khz 8 bit timer
	// motor stop
	//OCR3A = 0;
	//MotorSens 1=advance 0= back    in PD4
	//PORTD |= (1 << PD4); // Sens advance
//salut

	  //	init_USART();

	 TIMSK1|=(1<<TOIE1); // enabled global and timer overflow interrupt;
	TCCR1A = 0x00; // normal operation page 148 (mode0);
	TCNT1=0x0BDC; // set initial value to remove time error (16bit counter register)
	TCCR1B  |=(1<<CS12); // start timer/ set clock
	sei();

	// enable interrupts
//	OCR4D=150;
	do {

//
//		_delay_ms(20); // to not busy the CPU to much
	} while (1);

}

ISR(TIMER1_OVF_vect) {
PORTC=0xFF;
TCNT1=0x0BDC; // set initial value to remove time error (16bit counter register)

}
void initPort(void) {
	DDRC |= (1 << PORTC7); //LED

	/* enable pin of Timer 3A out ( Digit 5) */
	DDRC |= (1 << PORTC6);

	// Enable output for choose motor 1 sens//
	DDRD |= (1 << PORTD4);

	DDRD &= ~(1 << PORTD2); /* initialize pin PD2 input pin -> Rx for reception */
	DDRD |= (1 << PORTD3); /* initialize pin PD3 output pin -> Tx for transmission*/
	//	/* enable pin of Timer 4D out ( Digit 6) */
	//	DDRD |= (1 << PORTD7);



}
void init_pwm_timer(void) {
	/* set timer3 prescale factor to 64 */
	TCCR3B = (1 << CS31);	//| (1 << CS30);
	/* put timer3 in phase correct PWM mode */
	TCCR3A = (1 << WGM30);
	TC4H = 0;
	/* enable OC3A pin as PWM pin */
	TCCR3A |= (1 << COM3A1);

//	/***************************************************/
//	/* set timer4 prescale factor to 64 */
//	TCCR4B = (1 << CS42) | (1 << CS41) | (1 << CS40);
//	/* put timer 4 in phase and frequency correct PWM mode */
//	TCCR4D = (1 << WGM40);
//	/* not used */
//	TCCR4E = 0;
//	/* Set top to 0xFF  with that we have like timer 3 a 8 bit timer*/
//	OCR4C = 0xFF;
//	 /* TIMER 4D PWM MODE */
//	TCCR4C |= (1 << PWM4D);
//	  TCCR4C |= (1 << COM4D1);
}

//F=100hz, every 10ms more or less
//ISR(TIMER1_OVF_vect) {
//
//	float S_error = 0, error = 0, D_error = 0, PWM = 0, last_error = 0;
//	double tick;
//	float Dist;
//	tick = nb_tick; // Storage number of tick
//	nb_tick = 0; // clean
//	Dist = tick * 0.005852;
//	Vitesse = Dist * 10; // vitesse en cm/s ?
//	Dist_tot += Dist;
//
//	error = Consigne_V - Vitesse;
//	S_error += error;
//	D_error = error - last_error;
//	last_error = error;
//
//	PWM = error * Kp + S_error * Ki + D_error * Kd;
//	if (PWM > 255)
//		PWM = 255;
//	if (PWM < 0)
//		PWM = 0;
//	OCR3A = (int) (PWM);
//}
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
ISR( USART1_RX_vect) {

	receivechar = UDR1;
}



void Timer1_asserv_init() {

//	 TCCR1A = 0x00;
//	  TCCR1B = 0x00;
//	  TCNT1 = 0;
//
//	  OCR1A = 31250; // 16MHz/256/2Hz
//	  TCCR1B |= (1 << WGM12); // CTC mode
//	  TCCR1B |= (1 << CS12); // 256 prescaler
//	  TIMSK1 |= (1 << OCIE1A); // Activer le mode de comparaiso
	TIMSK1=0x01; // enabled global and timer overflow interrupt;
	TCCR1A = 0x00; // normal operation page 148 (mode0);
	TCNT1=0x0BDC; // set initial value to remove time error (16bit counter register)
	TCCR1B = 0x04; // start timer/ set clock
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
// receiver interrupt
//ISR(USART_RX_vect)
//{
////
////  ReceivedChar = UDR0;
////  usartTransmit (ReceivedChar);
//
//}
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

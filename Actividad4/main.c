#define F_CPU 16000000u
#include <avr/io.h>
#include <avr/interrupt.h>
#include "protocolo.h"
#include "clasificador.h"
#include "botones.h"
#include "hcsr04.h"
#include "tcrt5000.h"
#include "sg90.h"

static uint8_t time100ms;
static uint16_t aliveTimer = 0;

static void On2Ms(void);
static void initPort(void);
static void initTMR0(void);
static void initUSART0(void);

// Interrupción del Timer 0 en modo CTC. Se dispara cada 2ms (125 ticks a 16MHz/256).
// Solo activa una bandera en GPIOR0 para que el loop principal procese On2Ms().
ISR(TIMER0_COMPA_vect) {
	OCR0A += 125;
	GPIOR0 |= _BV(GPIOR00);
}

ISR(USART_RX_vect) {
	rx.rBuf.buf[rx.rBuf.iw++] = UDR0;
	rx.rBuf.iw &= (rx.rBuf.size-1);
}

// Se ejecuta cada 2ms cuando el loop principal detecta la bandera GPIOR00.
// Centraliza toda la lógica de tiempo real: parpadeo del LED, heartbeat alive,
// lectura de botones con debounce, y actualización de tiempos de brazos.
static void On2Ms(void) {
	GPIOR0 &= ~_BV(GPIOR00);
	
	switch (ledMode){
		case 0:
			time100ms--;
			if (!time100ms) {
				time100ms = 100;
				PINB = (1 << PINB5);
			}
		break;
		case 1:
			PORTB |= (1 << PINB5);
		break;
		case 2:
			// Sin logica implementada
		break;
	}
	
	aliveTimer++;
	if (aliveTimer >= 2500) {   
		aliveTimer = 0;
		uint8_t empty[1] = {0};
		Encode(0xF0, empty, 0);
	}
	
	botonesUpdate();
	checkBrazos();
	Clasificador_On2Ms();
}

static void initPort(void) {
	DDRB |= (1<<PORTB5)|(1<<PORTB3)|(1<<PORTB2)|(1<<PORTB1)|(1<<PORTB0);
	
	DDRD |= (1<<PORTD2)|(1<<PORTD3);      
	DDRD &= ~((1<<PORTD4)|(1<<PORTD5)|(1<<PORTD6)|(1<<PORTD7)); 
	PORTD |= (1<<PORTD4)|(1<<PORTD5)|(1<<PORTD6)|(1<<PORTD7);   
}

static void initTMR0(void) {
	TCCR0A = 0;
	TIFR0 = TIFR0;
	OCR0A = 124;
	TIMSK0 = (1<<OCIE0A);
	TCCR0B = (1<<CS02);
}

static void initUSART0(void) {
	UBRR0H = (uint8_t)(103 >> 8);
	UBRR0L = (uint8_t)103;
	UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);
	UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);
}

// Orden de inicialización:
// a) Protocolo y Clasificador primero 
// b) Seteo de callbacks entre módulos 
// c) Inicializa UART, puertos y timer
// d) Registro de botones 
// f) sei() habilita interrupciones solo cuando todo está listo
int main(void) {
	cli();

	Protocolo_Init();
	Clasificador_Init();

	Protocolo_SetCmdParser(CmdParser);
	Clasificador_SetEncode(Encode);

	initUSART0();
	initPort();
	initTMR0();

	botonesInit();
	botonesRegister(&PIND, &PORTD, &DDRD, (1<<PORTD4), onStartStop);
	botonesRegister(&PIND, &PORTD, &DDRD, (1<<PORTD5), onReset);
	botonesRegister(&PIND, &PORTD, &DDRD, (1<<PORTD6), onVelUp);
	botonesRegister(&PIND, &PORTD, &DDRD, (1<<PORTD7), onVelDown);
	
	sei();

	time100ms = 100;

	while (1) {
		if (GPIOR0 & _BV(GPIOR00))
		On2Ms();

		if (rx.rBuf.ir != rx.rBuf.iw)
		Decode();

		if (tx.rBuf.iw != tx.rBuf.ir) {
			if (UCSR0A & _BV(UDRE0)) {
				UDR0 = tx.rBuf.buf[tx.rBuf.ir++];
				tx.rBuf.ir &= (tx.rBuf.size-1);
			}
		}
	}
}
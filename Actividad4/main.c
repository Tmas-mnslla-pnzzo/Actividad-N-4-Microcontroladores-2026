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
static uint16_t testServoTimer = 0;
static uint8_t testServoAngle = 0;
SG90_t servo1, servo2, servo3;
HCSR04_t g_hcsr04;
IR_Sensor_t ir_s0, ir_s1, ir_s2, ir_s3;

void On2Ms(void);
void initPort(void);
void initTMR0(void);
void initUSART0(void);
void on_resultado(float dis);
void on_s0_released(void);
void on_s0_detected(void);
void on_s1_detected(void);
void on_s2_detected(void);
void on_s3_detected(void);
void hal_servo1(uint8_t state);
void hal_servo2(uint8_t state);
void hal_servo3(uint8_t state);
uint8_t hal_s0(void);
uint8_t hal_s1(void);
uint8_t hal_s2(void);
uint8_t hal_s3(void);
void hal_trig(uint8_t s);
uint8_t hal_echo(void);

// Interrupción del Timer 0 en modo CTC. Se dispara cada 2ms (125 ticks a 16MHz/256).
// Solo activa una bandera en GPIOR0 para que el loop principal procese On2Ms().
ISR(TIMER0_COMPA_vect) {
	static uint8_t contador = 0;
	
	SG90_Tick(&servo1, 200);
	SG90_Tick(&servo2, 200);
	SG90_Tick(&servo3, 200);
	IR_Tick(&ir_s0);
	IR_Tick(&ir_s1);
	IR_Tick(&ir_s2);
	IR_Tick(&ir_s3);
	
	contador++;
	if (contador >= 10) {  
		contador = 0;
		GPIOR0 |= _BV(GPIOR00);
	}
}

ISR(USART_RX_vect) {
	rx.rBuf.buf[rx.rBuf.iw++] = UDR0;
	rx.rBuf.iw &= (rx.rBuf.size-1);
}

// Se ejecuta cada 2ms cuando el loop principal detecta la bandera GPIOR00.
// Centraliza toda la lógica de tiempo real: parpadeo del LED, heartbeat alive,
// lectura de botones con debounce, y actualización de tiempos de brazos.
void On2Ms(void) {
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
	
	testServoTimer++;
	if (testServoTimer >= 500) {
		testServoTimer = 0;
		
		if (testServoAngle == 0) {
			SG90_SetAngle(&servo1, 0);
			testServoAngle = 1;
			} else if (testServoAngle == 1) {
			SG90_SetAngle(&servo1, 90);
			testServoAngle = 2;
			} else {
			SG90_SetAngle(&servo1, 180);
			testServoAngle = 0;
		}
	}
	
	//botonesUpdate();
	checkBrazos();
	Clasificador_On2Ms();
}

void initPort(void) {
	DDRB |= (1<<PORTB1) | (1<<PORTB3) | (1<<PORTB4) | (1<<PORTB5);  
	DDRB &= ~(1<<PORTB2);                                           
	DDRD |= (1<<PORTD7); 
	DDRD &= ~((1<<PORTD2)|(1<<PORTD3)|(1<<PORTD4)|(1<<PORTD5));
	PORTD |= (1<<PORTD2)|(1<<PORTD3)|(1<<PORTD4)|(1<<PORTD5);
}

void initTMR0(void) {
	TCCR0A = (1 << WGM01);    // CTC
	OCR0A = 49;               // 200µs
	TIMSK0 = (1 << OCIE0A);
	TCCR0B = (1 << CS01) | (1 << CS00);  // prescaler 64
}

void initUSART0(void) {
	UBRR0H = (uint8_t)(103 >> 8);
	UBRR0L = (uint8_t)103;
	UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);
	UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);
}

void on_resultado(float dis) {}
void on_s0_released(void) {}
void on_s0_detected(void) {}
void on_s1_detected(void) {}
void on_s2_detected(void) {}	
void on_s3_detected(void) {}
	
void hal_servo1(uint8_t state) {
	if (state) PORTD |= (1 << PORTD7);
	else       PORTD &= ~(1 << PORTD7);
}

void hal_servo2(uint8_t state) {
	if (state) PORTB |= (1 << PORTB4);
	else       PORTB &= ~(1 << PORTB4);
}

void hal_servo3(uint8_t state) {
	if (state) PORTB |= (1 << PORTB3);
	else       PORTB &= ~(1 << PORTB3);
}
uint8_t hal_s0(void) { 
	return (PIND&(1<<PORTD5))?0U:1U; 
}
uint8_t hal_s1(void) { 
	return (PIND&(1<<PORTD2))?0U:1U; 
}
uint8_t hal_s2(void){ 
	return (PIND&(1<<PORTD3))?0U:1U;
}
uint8_t hal_s3(void){ 
	return (PIND&(1<<PORTD4))?0U:1U; 
}

void hal_trig(uint8_t s) { 
	if(s) PORTB|=(1<<PORTB1); else PORTB&=~(1<<PORTB1); 
}

uint8_t hal_echo(void) { 
	return (PINB & (1<<PORTB2)) ? 1U : 0U; 
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
	
	HCSR04_Init(&g_hcsr04, hal_trig, hal_echo, on_resultado);
	IR_Init(&ir_s0, hal_s0, on_s0_detected, on_s0_released);
	IR_Init(&ir_s1, hal_s1, on_s1_detected, (void*)0);
	IR_Init(&ir_s2, hal_s2, on_s2_detected, (void*)0);
	IR_Init(&ir_s3, hal_s3, on_s3_detected, (void*)0);
	SG90_Init(&servo1, hal_servo1);
	SG90_Init(&servo2, hal_servo2);
	SG90_Init(&servo3, hal_servo3);

	//botonesInit();
	//botonesRegister(&PIND, &PORTD, &DDRD, (1<<PORTD4), onStartStop);
	//botonesRegister(&PIND, &PORTD, &DDRD, (1<<PORTD5), onReset);
	//botonesRegister(&PIND, &PORTD, &DDRD, (1<<PORTD6), onVelUp);
	//botonesRegister(&PIND, &PORTD, &DDRD, (1<<PORTD7), onVelDown);
	
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
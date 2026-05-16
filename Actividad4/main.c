#define F_CPU 16000000u
#include <avr/io.h>
#include <avr/interrupt.h>
#include "protocolo.h"
#include "clasificador.h"
#include "hcsr04.h"
#include "tcrt5000.h"
#include "sg90.h"

static uint8_t time100ms;
static uint16_t testServoTimer = 0;
static uint32_t t_detect = 0;
static uint8_t retractPending = 0;
uint8_t midiendo_vel = 0;  
uint16_t aliveTimer = 0;
uint8_t retractTimer = 0;
uint16_t hcsrTimer = 0;
uint8_t payload[2];
uint8_t tipo = 0;
uint32_t g_now_us;

//para el modo ciego
uint8_t modo_ciego = 1;     
uint8_t medir_auto   = 0;  
uint8_t vel_medida   = 1;    
float vel_cinta_cms = 30.0f;  
float dist_s0_a_salida[3] = {200.0f, 400.0f, 600.0f};
static uint16_t timer_ciego[3] = {0, 0, 0};
static uint8_t timer_ciego_activo[3] = {0, 0, 0};

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
void hal_output_a0(uint8_t state); 
void App_TriggerHCSR04(void);
void App_IniciarVelocidad(uint8_t ancho_cm);
void App_MoverBrazo(uint8_t cmd, uint8_t* payload, uint8_t n);

// Interrupción del Timer 0 en modo CTC. Se dispara cada 2ms (125 ticks a 16MHz/256).
// Solo activa una bandera en GPIOR0 para que el loop principal procese On2Ms().
ISR(TIMER0_COMPA_vect) {
	static uint8_t contador = 0;

	SG90_Tick(&servo1, 200);
	SG90_Tick(&servo2, 200);
	SG90_Tick(&servo3, 200);

	contador++;
	if (contador >= 10) {
		contador = 0;
		GPIOR0 |= _BV(GPIOR00);
	}
}
ISR(TIMER1_COMPA_vect) {
	g_now_us += 48;
	HCSR04_Tick(&g_hcsr04, g_now_us);
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
	
	IR_Tick(&ir_s0);
	IR_Tick(&ir_s1);
	IR_Tick(&ir_s2);
	IR_Tick(&ir_s3);
	
	aliveTimer++;
	if (aliveTimer >= ALIVE) {   
		aliveTimer = 0;
		uint8_t empty[1] = {0};
		Encode(0xF0, empty, 0);
	}
	
	testServoTimer++;
	
	if (modo_ciego) {
		for (uint8_t i = 0; i < 3; i++) {
			if (timer_ciego_activo[i] && timer_ciego[i] > 0) {
				timer_ciego[i]--;
				if (timer_ciego[i] == 0) {
					timer_ciego_activo[i] = 0;
					uint8_t p[2] = {(1 << i), (1 << i)};
					App_MoverBrazo(0x52, p, 2);
					
					brazos[i].activo = 1;
					brazos[i].timer  = time_arm_extend; 
					brazos[i].estado = 1;
				}
			}
		}
	}
	
	if (retractPending && retractTimer > 0) {
		retractTimer--;
		if (retractTimer == 0) {
			retractPending = 0;
			uint8_t ack[1] = {0xFF};
			CmdParser(0x52, ack, 1);
		}
	}
	
	checkBrazos();
	Clasificador_On2Ms();
}

void initPort(void) {
	DDRB |= (1<<PORTB1) | (1<<PORTB3) | (1<<PORTB4) | (1<<PORTB5);  
	DDRB &= ~(1<<PORTB2);                                           
	DDRD |= (1<<PORTD7); 
	DDRD |= (1<<PORTD6); //debug
	DDRB |= (1<<PORTB0); //debug
	DDRD &= ~((1<<PORTD2)|(1<<PORTD3)|(1<<PORTD4)|(1<<PORTD5));
	PORTD |= (1<<PORTD2)|(1<<PORTD3)|(1<<PORTD4)|(1<<PORTD5);
	
	DDRC  |=  (1 << DDC0);
	PORTC |=  (1 << PORTC0);
}

void initTMR0(void) {
	TCCR0A = (1 << WGM01);
	OCR0A  = 49;               
	TIMSK0 = (1 << OCIE0A);
	TCCR0B = (1 << CS01) | (1 << CS00);
}

void initTMR1(void) {
	TCCR1A = 0;
	TCCR1B = (1 << WGM12) | (1 << CS11); 
	OCR1A  = 95;                          
	TIMSK1 = (1 << OCIE1A);
}

void initUSART0(void) {
	UBRR0H = (uint8_t)(103 >> 8);
	UBRR0L = (uint8_t)103;
	UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);
	UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);
}

void initConfig(void) {
	uint8_t cfg[4] = {0x0D, calibracion[1], calibracion[2], calibracion[3]};
	CmdParser(0x50, cfg, 4);
}

void on_resultado(float dis) {
	tipo = 0;
	
	payload[0] =(uint8_t)dis;
	Encode(0x5F, payload, 1);
	Encode(0x61, payload, 1);

	if (dis >= calibracion[1]-tolerancia && dis <= calibracion[1]+tolerancia) { 
		tipo = calibracion[1];
	}else if (dis >= calibracion[2]-tolerancia && dis <= calibracion[2]+tolerancia) { 
		tipo = calibracion[2];
	}else if (dis >= calibracion[3]-tolerancia && dis <= calibracion[3]+tolerancia) {
		tipo = calibracion[3];
	}
	
	if (tipo > 0) {
		Clasificador_NuevaCaja(tipo);
	}
	
	if (modo_ciego && tipo > 0) {
		for (uint8_t i = 0; i < 3; i++) {
			if (tipo == configCajas[i]) {
				for (uint8_t j = 0; j < 3; j++) {
					if (j != i) timer_ciego_activo[j] = 0;
				}
				break;
			}
		}
	}
}

void on_s0_detected(void) {
	if (modo_ciego && vel_medida && vel_cinta_cms > 0) {
		for (uint8_t i = 0; i < 3; i++) {
			timer_ciego[i] = (uint16_t)((dist_s0_a_salida[i] / vel_cinta_cms) * 500.0f);
			timer_ciego_activo[i] = 1;
		}
	}
	if (midiendo_vel) {
		t_detect = g_now_us;
	}
	PORTD |= (1 << PORTD6);
	payload[0] = 0x03;
	payload[1] = 0x01;
	Encode(0x5E, payload, 2);
	App_TriggerHCSR04();
	//Encode(0x5F, payload, 1);
}
	
void on_s0_released(void) {
	if (midiendo_vel) {
		uint32_t dt_us = g_now_us - t_detect;
		uint16_t vel = (uint16_t)(((uint32_t)anchoCaja * 1000000UL) / dt_us);
		
		if (vel > 255) vel = 255;
		
		vel_cinta_cms = (float)vel;

		vel_medida = 1;
		midiendo_vel = 0;
	}
	PORTD &= ~(1 << PORTD6);
	payload[0] = 0x03;
	payload[1] = 0x00;
	Encode(0x5E, payload, 2);
	//Encode(0x5F, payload, 1);
}

void on_s1_detected(void) {
	payload[0] = 0x00;
	payload[1] = 0x01;
	Encode(0x5E, payload, 2);
	CmdParser(0x5E, payload, 2);
}

void on_s1_released(void) {
	payload[0] = 0x00;
	payload[1] = 0x00;
	Encode(0x5E, payload, 2);
}

void on_s2_detected(void) {
	payload[0] = 0x01; 
	payload[1] = 0x01;   
	Encode(0x5E, payload, 2);
	CmdParser(0x5E, payload, 2);
}	

void on_s2_released(void) {
	payload[0] = 0x01;
	payload[1] = 0x00;
	Encode(0x5E, payload, 2);
}

void on_s3_detected(void) {
	payload[0] = 0x02;
	payload[1] = 0x01;
	Encode(0x5E, payload, 2);
	CmdParser(0x5E, payload, 2);
}

void on_s3_released(void) {
	payload[0] = 0x02;
	payload[1] = 0x00;
	Encode(0x5E, payload, 2);
}	
	
void hal_servo1(uint8_t state) {
	if (state) PORTD |= (1 << PORTD7);
	else       PORTD &= ~(1 << PORTD7);
}

//cambie del servo2 a servo3
void hal_servo3(uint8_t state) {
	if (state) PORTB |= (1 << PORTB4);
	else       PORTB &= ~(1 << PORTB4);
}

//cambie del servo3 a servo2
void hal_servo2(uint8_t state) {
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

void hal_output_a0(uint8_t state) {
	if (state) PORTC &= ~(1 << PORTC0); 
	else       PORTC |=  (1 << PORTC0); 
}

void App_TriggerHCSR04(void) {
	HCSR04_Trigger(&g_hcsr04, g_now_us);
}

void App_IniciarVelocidad(uint8_t ancho_cm) {
	anchoCaja = ancho_cm;
	midiendo_vel = 1;
}

void App_MoverBrazo(uint8_t cmd, uint8_t* payload, uint8_t n) {
	if (cmd == 0x52) {
		uint8_t mascara = payload[0];
		uint8_t extender = (payload[1] != 0);

		if (extender) {
			if (mascara & (1<<0)) SG90_SetAngle(&servo1, SG90_ANGLE_DETECT);
			if (mascara & (1<<1)) SG90_SetAngle(&servo2, SG90_ANGLE_DETECT);
			if (mascara & (1<<2)) SG90_SetAngle(&servo3, SG90_ANGLE_DETECT);
			} else {
			if (mascara & (1<<0)) SG90_SetAngle(&servo1, SG90_ANGLE_REPOSE);
			if (mascara & (1<<1)) SG90_SetAngle(&servo2, SG90_ANGLE_REPOSE);
			if (mascara & (1<<2)) SG90_SetAngle(&servo3, SG90_ANGLE_REPOSE);
			
			retractTimer = SG90_RETRACT_TIME;   
			retractPending = 1;
		}
		Encode(cmd, payload, n);
	} else {
		Encode(cmd, payload, n);
	}
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

	sistemaListo = 1;
	
	Protocolo_SetCmdParser(CmdParser);
	Clasificador_SetTrigger(App_TriggerHCSR04);
	Clasificador_SetEncode(App_MoverBrazo);
	Clasificador_SetVelocidad(App_IniciarVelocidad);
	Clasificador_SetOutput(hal_output_a0);

	initUSART0();
	initPort();
	initTMR0();
	initTMR1();
	initConfig();
	
	HCSR04_Init(&g_hcsr04, hal_trig, hal_echo, on_resultado);
	IR_Init(&ir_s0, hal_s0, on_s0_detected, on_s0_released);
	IR_Init(&ir_s1, hal_s1, on_s1_detected, on_s1_released);
	IR_Init(&ir_s2, hal_s2, on_s2_detected, on_s2_released);
	IR_Init(&ir_s3, hal_s3, on_s3_detected, on_s3_released);
	SG90_Init(&servo1, hal_servo1);
	SG90_Init(&servo2, hal_servo2);
	SG90_Init(&servo3, hal_servo3);
	
	SG90_SetAngle(&servo1, SG90_ANGLE_REPOSE);
	SG90_SetAngle(&servo2, SG90_ANGLE_REPOSE);
	SG90_SetAngle(&servo3, SG90_ANGLE_REPOSE);
	
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
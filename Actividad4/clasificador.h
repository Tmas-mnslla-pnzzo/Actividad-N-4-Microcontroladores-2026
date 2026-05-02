#ifndef CLASIFICADOR_H
#define CLASIFICADOR_H

#include <avr/io.h>

#define MAX_CAJAS 20

// Estructura para manejar los buffers circulares FIFO de cada salida.
typedef struct {
	uint8_t destinos[MAX_CAJAS];
	uint8_t indiceLectura;
	uint8_t indiceEscritura;
	uint8_t cantidad;
} _sFifoCajas;

// Estructura para controlar el estado de cada brazo actuador.
typedef struct {
	uint8_t activo;
	uint8_t timer;
	uint8_t estado;    
	uint8_t pendiente;  
} _sBrazo;

// Estructura para registrar cajas que llegan al sensor mientras el brazo está ocupado.
typedef struct {
	uint8_t pendiente;
	uint8_t outNum;
} _sSensorPendiente;

extern _sBrazo brazos[3];
extern uint8_t sistemaListo;
extern uint8_t ledMode;
extern uint16_t cajasEntrada;
extern uint16_t cajasSalida;

typedef void (*EncodeCallback)(uint8_t cmd, uint8_t* payload, uint8_t n);

void Clasificador_Init(void);
void Clasificador_SetEncode(EncodeCallback cb);
void CmdParser(uint8_t cmd, uint8_t* params, uint8_t len);
void checkBrazos(void);
void Clasificador_On2Ms(void);
void onStartStop(void);
void onReset(void);
void onVelUp(void);
void onVelDown(void);
 
#endif /* CLASIFICADOR_H_ */
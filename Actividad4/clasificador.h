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

extern uint16_t ALIVE;
extern _sBrazo brazos[3];
extern uint8_t sistemaListo;
extern uint8_t ledMode;
extern uint16_t cajasEntrada;
extern uint16_t cajasSalida;
extern uint8_t anchoCaja;
extern uint8_t calibracion[4];
extern uint8_t tolerancia;
extern uint8_t time_arm_extend;
extern uint8_t time_arm_retract;
extern uint8_t retractTimer;
extern uint8_t SG90_ANGLE_DETECT;
extern uint8_t SG90_ANGLE_REPOSE;
extern uint8_t SG90_RETRACT_TIME;
extern uint16_t hcsrTimer;
extern uint16_t aliveTimer;
extern uint8_t configCajas[3];
extern uint8_t  modo_ciego;
extern float vel_cinta_cms;
extern uint8_t vel_medida;
extern uint8_t midiendo_vel;
extern uint8_t medir_auto;
extern float dist_s0_a_salida[3];

typedef void (*EncodeCallback)(uint8_t cmd, uint8_t* payload, uint8_t n);
typedef void (*TriggerCallback)(void);
typedef void (*VelocidadCallback)(uint8_t ancho_cm);
typedef void (*OutputCallback)(uint8_t state);

void Clasificador_SetOutput(OutputCallback cb);
void Clasificador_SetVelocidad(VelocidadCallback cb);
void Clasificador_Init(void);
void Clasificador_SetEncode(EncodeCallback cb);
void Clasificador_SetTrigger(TriggerCallback cb);
void Clasificador_NuevaCaja(uint8_t tipo);
void CmdParser(uint8_t cmd, uint8_t* params, uint8_t len);
void checkBrazos(void);
void Clasificador_On2Ms(void);
void onStartStop(void);
void onReset(void);
void onVelUp(void);
void onVelDown(void);
 
#endif /* CLASIFICADOR_H_ */
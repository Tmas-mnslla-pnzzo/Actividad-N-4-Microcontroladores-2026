#include "clasificador.h"

#define ALIVE_TIMEOUT_MS 5000  

static uint8_t configCajas[3];
static _sFifoCajas tramos[3];
static _sSensorPendiente sensorPendiente[3];
static uint16_t aliveTimeout = 0;
static uint8_t startFlag = 0;
static uint8_t velIdx = 1;
static EncodeCallback _encode = 0;
uint8_t sistemaListo = 0;
uint8_t ledMode;
uint16_t cajasEntrada = 0;
uint16_t cajasSalida = 0;
_sBrazo brazos[3];

void Clasificador_SetEncode(EncodeCallback cb) {
	_encode = cb;
}

// Agrega un destino al buffer circular FIFO de la salida indicada.
static void pushFifo(uint8_t tramoId, uint8_t destino) {
	if (tramos[tramoId].cantidad < MAX_CAJAS) {
		tramos[tramoId].destinos[tramos[tramoId].indiceEscritura] = destino;
		tramos[tramoId].indiceEscritura = (tramos[tramoId].indiceEscritura + 1) % MAX_CAJAS;
		tramos[tramoId].cantidad++;
	}
}

// Extrae el destino más antiguo del buffer circular FIFO de la salida indicada
// Retorna -1 si el buffer está vacío, indicando que no hay cajas encoladas
static int8_t popFifo(uint8_t tramoId) {
	if (tramos[tramoId].cantidad > 0) {
		uint8_t dest = tramos[tramoId].destinos[tramos[tramoId].indiceLectura];
		tramos[tramoId].indiceLectura = (tramos[tramoId].indiceLectura + 1) % MAX_CAJAS;
		tramos[tramoId].cantidad--;
		return dest;
	}
	return -1;
}

void Clasificador_Init(void) {
	for (uint8_t i = 0; i < 3; i++) {
		tramos[i].indiceLectura = 0;
		tramos[i].indiceEscritura = 0;
		tramos[i].cantidad = 0;
		brazos[i].activo = 0;
		brazos[i].timer = 0;
		configCajas[i] = 0;
		sensorPendiente[i].pendiente = 0;
		sensorPendiente[i].outNum = 0;
	}
}

void Clasificador_On2Ms(void) {
	for (uint8_t i = 0; i < 3; i++) {
		if (brazos[i].activo && brazos[i].timer > 0)
		brazos[i].timer--;
	}
}

// Evalúa el estado de los brazos y ejecuta las transiciones de la máquina de estados
// Estados: 0=retraído, 1=extendido, 2=esperando ACK de retracción
// Cuando el timer llega a 0 estando extendido, envía comando de retraer al simulador
void checkBrazos(void) {
	for (uint8_t i = 0; i < 3; i++) {
		if (brazos[i].activo && brazos[i].timer > 0) {
			} else if (brazos[i].activo && brazos[i].timer == 0 && brazos[i].estado == 1) {
			if (_encode) {
				uint8_t payload[2];
				payload[0] = (1 << i);
				payload[1] = 0x00; 
				_encode(0x52, payload, 2);
			}
			brazos[i].estado = 2;  
			brazos[i].timer = 125;
		}
	}
}

void CmdParser(uint8_t cmd, uint8_t* params, uint8_t len) {
	switch(cmd) {
		case 0xF0:
			if (params[0] == 0x0D) {
				sistemaListo = 1;
				aliveTimeout = 0;  
			}
		break;
        case 0x50:
			if (len >= 4) {
			    configCajas[0] = params[1];
			    configCajas[1] = params[2];
			    configCajas[2] = params[3];

			    for (uint8_t i = 0; i < 3; i++) {
			        tramos[i].indiceLectura = 0;
			        tramos[i].indiceEscritura = 0;
			        tramos[i].cantidad = 0;
			        brazos[i].activo = 0;
			        brazos[i].timer = 0;
			        brazos[i].estado = 0;
			        brazos[i].pendiente = 0;
			        sensorPendiente[i].pendiente = 0;
			    }
			    ledMode = 0;
			}
        break;
        case 0x51:  
			if (params[0] == 0x0D) {
				ledMode = 1;
				for (uint8_t i = 0; i < 3; i++) {
					brazos[i].activo = 0;
					brazos[i].timer = 0;
					brazos[i].estado = 0;
				}
				}
        break;
		// Busca el brazo que estaba esperando confirmación (estado = 2) y lo libera.
		// Si había una caja pendiente en esa salida, la procesa de inmediato:
		// Extrae del FIFO, si coincide activa el brazo, si no transfiere al siguiente.
        case 0x52:
			if (params[0] == 0xFF) {
			    for (uint8_t i = 0; i < 3; i++) {
			        if (brazos[i].estado == 2) {
				        brazos[i].estado = 0;     
				        brazos[i].activo = 0;
				        brazos[i].timer = 0;
				        if (sensorPendiente[i].pendiente) {
					        sensorPendiente[i].pendiente = 0;
					        int8_t dest = popFifo(i);
					        if (dest != -1) {
						        if (dest == i) {
							        if (_encode) {
								        uint8_t payload[2];
								        payload[0] = (1 << i);
								        payload[1] = (1 << i);
								        _encode(0x52, payload, 2);
							        }
							        brazos[i].activo = 1;
							        brazos[i].timer = 125;
							        brazos[i].estado = 1;
							        } else if (i < 2) {
							        pushFifo(i + 1, dest);
						        }
					        }
				        }
				        break;  
			        }
			    }
			}
        break;
        case 0x53:
		    if (params[0] == 0x0D) {
			    ledMode = 0;
			    } else if (params[0] == 0x0A) {
			    ledMode = 2;
		    }
		break;
		case 0x54:
		    if (params[1] == 0x0D) {
			    // Sin lógica adicional
		    }
		break;
		// Se procesan los pares: outNum, IRState. 
		// Lógica de encadenamiento de FIFOs:
		// a) popFifo(outNum) extrae el destino esperado para esta salida.
		// b) Si coincide (dest == outNum), se activa el brazo: es la salida correcta.
		// c) Si no coincide (outNum < 2), se transfiere al FIFO de la siguiente salida.
		// d) Si el FIFO está vacío, se guarda como pendiente hasta que llegue una caja.
        case 0x5E:  
			for (uint8_t i = 0; i < len; i += 2) {
				uint8_t outNum = params[i];
				uint8_t IRState = params[i+1];
				if (IRState == 1) {
					if (brazos[outNum].estado == 0) {
						int8_t dest = popFifo(outNum);
						if (dest == -1) {
							sensorPendiente[outNum].pendiente = 1;
							sensorPendiente[outNum].outNum = outNum;
							} else if (dest == outNum) {
							cajasSalida++;
							if (_encode) {
								uint8_t payload[2];
								payload[0] = (1 << outNum);
								payload[1] = (1 << outNum);
								_encode(0x52, payload, 2);
							}
							brazos[outNum].activo = 1;
							brazos[outNum].timer = 125;
							brazos[outNum].estado = 1;
							} else if (outNum < 2) {
							pushFifo(outNum + 1, dest);
						}
						} else {
						sensorPendiente[outNum].pendiente = 1;
						sensorPendiente[outNum].outNum = outNum;
					}
				}
				}
        break;
        case 0x5F: 
			cajasEntrada++;
			for (uint8_t i = 0; i < 3; i++) {
				if (params[0] == configCajas[i]) {
					pushFifo(0, i);
					if (sensorPendiente[0].pendiente && brazos[0].estado == 0) {
						sensorPendiente[0].pendiente = 0;
						int8_t dest = popFifo(0);
						if (dest == 0) {
							if (_encode) {
								uint8_t payload[2];
								payload[0] = (1 << 0);
								payload[1] = (1 << 0);
								_encode(0x52, payload, 2);
							}
							brazos[0].activo = 1;
							brazos[0].timer = 125;
							brazos[0].estado = 1;
							} else if (dest != -1) {
							pushFifo(1, dest);
						}
					}
					return;
				}
			}
        break;
	}
}

void onStartStop(void) {
	uint8_t empty[1] = {0};
	if (startFlag) {
		if (_encode) _encode(0x51, empty, 0);
			startFlag = 0;
	} else {
		if (_encode) _encode(0x50, empty, 0);
			startFlag = 1;
	}
}

void onReset(void) {
	uint8_t empty[1] = {0};
	if (_encode) _encode(0x53, empty, 0);
	startFlag = 0;
}

void onVelUp(void) {
	if (velIdx < 10) velIdx++;
	uint8_t payload[1] = {velIdx * 2};
	if (_encode) _encode(0x54, payload, 1);
}

void onVelDown(void) {
	if (velIdx > 1) velIdx--;
	uint8_t payload[1] = {velIdx * 2};
	if (_encode) _encode(0x54, payload, 1);
}
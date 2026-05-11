#ifndef TCRT5000_H_
#define TCRT5000_H_

#include <stdint.h>

typedef uint8_t (*ir_read_cb_t)(void);
typedef void (*ir_event_cb_t)(void);

typedef struct {
	ir_read_cb_t   read_pin;      /* lectura del pin digital  */
	ir_event_cb_t  on_detected;   /* flanco positivo          */
	ir_event_cb_t  on_released;   /* flanco negativo          */
	uint8_t        last_state;    /* último estado leído      */
} IR_Sensor_t;

void IR_Init (IR_Sensor_t *dev, ir_read_cb_t  read, ir_event_cb_t on_detected, ir_event_cb_t on_released);
void IR_Tick    (IR_Sensor_t *dev);
uint8_t IR_IsActive(const IR_Sensor_t *dev);

#endif /* TCRT5000_H_ */
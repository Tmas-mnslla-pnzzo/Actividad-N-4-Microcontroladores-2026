#ifndef TCRT5000_H_
#define TCRT5000_H_

#include <stdint.h>

extern uint8_t IR_DEBOUNCE_TICKS;

typedef uint8_t (*ir_read_cb_t)(void);
typedef void (*ir_event_cb_t)(void);

typedef struct {
	ir_read_cb_t   read_pin;      
	ir_event_cb_t  on_detected;  
	ir_event_cb_t  on_released;  
	uint8_t        last_state;   
	uint8_t        debounce_cnt;
} IR_Sensor_t;

void IR_Init (IR_Sensor_t *dev, ir_read_cb_t  read, ir_event_cb_t on_detected, ir_event_cb_t on_released);
void IR_Tick    (IR_Sensor_t *dev);
uint8_t IR_IsActive(const IR_Sensor_t *dev);

#endif /* TCRT5000_H_ */
#include "tcrt5000.h"

void IR_Init(IR_Sensor_t *dev, ir_read_cb_t  read, ir_event_cb_t on_detected, ir_event_cb_t on_released) {
	dev->read_pin    = read;
	dev->on_detected = on_detected;
	dev->on_released = on_released;
	dev->last_state  = (read != (void*)0) ? read() : 0U;
}

void IR_Tick(IR_Sensor_t *dev) {
	if (dev->read_pin == (void*)0) return;

	uint8_t current = dev->read_pin();

	/* Flanco positivo: libre ? detectado */
	if (current && !dev->last_state) {
		if (dev->on_detected) dev->on_detected();
	}
	/* Flanco negativo: detectado ? libre */
	else if (!current && dev->last_state) {
		if (dev->on_released) dev->on_released();
	}

	dev->last_state = current;
}

uint8_t IR_IsActive(const IR_Sensor_t *dev) {
	return dev->last_state;
}
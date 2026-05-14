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
	
	if (current == dev->last_state) {
		dev->debounce_cnt = 0U;
		return;
	}
	
	dev->debounce_cnt++;
	
	if (dev->debounce_cnt >= IR_DEBOUNCE_TICKS) {
		dev->debounce_cnt = 0U;
		dev->last_state   = current;
		
		if (current) {
			if (dev->on_detected) dev->on_detected();
		}
		else {
			if (dev->on_released) dev->on_released();
		}
	}
}


uint8_t IR_IsActive(const IR_Sensor_t *dev) {
	return dev->last_state;
}
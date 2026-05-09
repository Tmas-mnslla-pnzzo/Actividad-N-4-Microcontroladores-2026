#include "sg90.h"

static uint32_t _clamp(uint32_t v, uint32_t lo, uint32_t hi) {
	return (v < lo) ? lo : (v > hi) ? hi : v;
}

void SG90_Init(SG90_t *dev, sg90_pin_write_cb_t set_pin) {
	dev->set_pin    = set_pin;
	dev->pulse_us   = SG90_PULSE_NEUTRAL_US;
	dev->counter_us = 0;
	if (dev->set_pin) dev->set_pin(0);
}

void SG90_SetAngle(SG90_t *dev, uint8_t angle_deg) {
	if (angle_deg > 180U) angle_deg = 180U;
	uint32_t range = SG90_PULSE_MAX_US - SG90_PULSE_MIN_US;
	dev->pulse_us  = SG90_PULSE_MIN_US + ((uint32_t)angle_deg * range) / 180U;
}

void SG90_SetPulse(SG90_t *dev, uint32_t pulse_us) {
	dev->pulse_us = _clamp(pulse_us, SG90_PULSE_MIN_US, SG90_PULSE_MAX_US);
}

void SG90_Tick(SG90_t *dev, uint32_t elapsed_us) {
	if (dev->set_pin == (void*)0) return;

	uint32_t prev    = dev->counter_us;
	dev->counter_us += elapsed_us;

	if (dev->counter_us >= SG90_PERIOD_US) {
		/* ?? Inicio de nuevo periodo ?? */
		dev->counter_us -= SG90_PERIOD_US;
		dev->set_pin(1);  /* flanco de subida */

		/* Caso extremo: elapsed >= pulse (no ocurre con tick <= 200 µs) */
		if (dev->counter_us >= dev->pulse_us)
		dev->set_pin(0);
		} else {
		/* ?? Dentro del mismo periodo ?? */
		/* Flanco de bajada: fin de pulso activo */
		if (prev < dev->pulse_us && dev->counter_us >= dev->pulse_us)
		dev->set_pin(0);
	}
}

uint32_t SG90_GetPulse(const SG90_t *dev) { return dev->pulse_us; }
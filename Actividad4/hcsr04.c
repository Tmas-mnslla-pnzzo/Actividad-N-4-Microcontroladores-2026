#include "hcsr04.h"

void HCSR04_Init(HCSR04_t *dev, hcsr04_pin_write_cb_t set_trig, hcsr04_pin_read_cb_t get_echo, hcsr04_result_cb_t on_result) {
	dev->set_trigger   = set_trig;
	dev->get_echo      = get_echo;
	dev->on_result     = on_result;
	dev->state         = HCSR04_STATE_IDLE;
	dev->tick_ref_us   = 0;
	dev->echo_start_us = 0;

	if (dev->set_trigger) dev->set_trigger(0); /* TRIG en bajo */
}

void HCSR04_Trigger(HCSR04_t *dev, uint32_t now_us)
{
	if (dev->state != HCSR04_STATE_IDLE) return;

	dev->set_trigger(1);
	dev->tick_ref_us = now_us;
	dev->state = HCSR04_STATE_TRIGGER_HIGH;
}

void HCSR04_Tick(HCSR04_t *dev, uint32_t now_us)
{
	uint32_t elapsed = now_us - dev->tick_ref_us;

	switch (dev->state) {
		case HCSR04_STATE_IDLE:
		break;

		case HCSR04_STATE_TRIGGER_HIGH:
		if (elapsed >= HCSR04_TRIG_PULSE_US) {
			dev->set_trigger(0);
			dev->tick_ref_us = now_us;
			dev->state = HCSR04_STATE_WAIT_ECHO_HIGH;
		}
		break;

		case HCSR04_STATE_WAIT_ECHO_HIGH:
		if (dev->get_echo()) {
			dev->echo_start_us = now_us;
			dev->state = HCSR04_STATE_MEASURING;
			} else if (elapsed >= HCSR04_ECHO_TIMEOUT_US) {
			dev->state = HCSR04_STATE_IDLE;
			if (dev->on_result) dev->on_result(-1.0f);
		}
		break;

		case HCSR04_STATE_MEASURING:
		if (!dev->get_echo()) {
			uint32_t echo_us = now_us - dev->echo_start_us;
			float dist = (float)echo_us * HCSR04_US_TO_CM;
			dev->state = HCSR04_STATE_IDLE;
			if (dev->on_result) dev->on_result(dist);
			} else if ((now_us - dev->echo_start_us) >= HCSR04_ECHO_TIMEOUT_US) {
			dev->state = HCSR04_STATE_IDLE;
			if (dev->on_result) dev->on_result(-1.0f);
		}
		break;

		default:
		dev->set_trigger(0);
		dev->state = HCSR04_STATE_IDLE;
	}
}

uint8_t HCSR04_IsBusy(const HCSR04_t *dev)
{
	return (dev->state != HCSR04_STATE_IDLE) ? 1U : 0U;
}
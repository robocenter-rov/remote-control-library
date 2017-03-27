#include "SetFlashlightStateTask.h"

SetFlashlightStateTask_t::SetFlashlightStateTask_t(bool state) : Task_t(TI_SET_FLASH_LIGHT_STATE) {
	_flash_light_state = state;
}

void SetFlashlightStateTask_t::Promise_t::SetOk() {
	if (!_ended && _on_ok) {
		_on_ok();
	}
	_ended = true;
}

SetFlashlightStateTask_t::Promise_t* SetFlashlightStateTask_t::Promise_t::OnOk(empty_callback_t on_ok) {
	if (_ended) {
		on_ok();
	}
	_on_ok = on_ok;
	return this;
}

void SetFlashlightStateTask_t::SendCommand(Communicator_t* communicator, int worker_id, unsigned int tag) {
	communicator->SendFlashLightState(worker_id, tag, _flash_light_state);
}

SetFlashlightStateTask_t::PromisePtr_t SetFlashlightStateTask_t::GetPromise() {
	return std::static_pointer_cast<Promise_t>(Task_t::GetPromise());
}

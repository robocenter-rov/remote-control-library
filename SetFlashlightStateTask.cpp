#include "SetFlashlightStateTask.h"

void SetFlashlightStateTask_t::_UpdateState(AdditionalData_t* additional_data) {
	if (_promise) {
		if (_task_status == TS_OK) {
			GetPromise()->SetOk();
		}
	}
}

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
	return _GetPromise<Promise_t>();
}

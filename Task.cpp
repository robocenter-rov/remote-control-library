#include "Task.h"

bool DeliveryCommandState_t::operator!=(const DeliveryCommandState_t& b) const {
	return status != b.status || send_attempt != b.send_attempt || resend_attempt != b.resend_attempt;
}

Task_t::Task_t(TASK_ID task_id) {
	_task_id = task_id;
	_worker_id = -2;
	_task_tag = 0;
	_task_status = TS_NONE;
}

Task_t::Promise_t::Promise_t(): _cancelled(false) {}

void Task_t::Promise_t::SetDeliveryCommandState(DeliveryCommandState_t delivery_command_state) {
	if (_delivery_command_state != delivery_command_state && _on_dsc) {
		_on_dsc(delivery_command_state);
	}
	_delivery_command_state = delivery_command_state;
}

void Task_t::Promise_t::SetCancel() {
	_cancelled = true;
	if (_on_cancel) {
		_on_cancel();
	}
}

DeliveryCommandState_t Task_t::Promise_t::GetDeliveryCommandState() const {
	return _delivery_command_state;
}

Task_t::Promise_t* Task_t::Promise_t::OnDeliveryStatusChanged(on_dcs_callback_t on_dsc) {
	if (_delivery_command_state.status != DCS_NONE) {
		on_dsc(_delivery_command_state);
	}
	return this;
}

Task_t::Promise_t* Task_t::Promise_t::OnCancel(empty_callback_t on_cancel) {
	_on_cancel = on_cancel;
	if (_cancelled) {
		on_cancel();
	}
	return this;
}

void Task_t::SendStateRequest(Communicator_t* communicator) {
	communicator->GetWorkerState(_worker_id);
	_last_request_send_time = std::chrono::system_clock::now();
}

void Task_t::UpdateDeliveryState(DeliveryCommandState_t state) const {
	_promise->SetDeliveryCommandState(state);
}

DeliveryCommandState_t Task_t::GetDeliveryState() const {
	return _promise->GetDeliveryCommandState();
}

TASK_STATUS Task_t::GetStatus() const {
	return _task_status;
}

void Task_t::UpdateState(TASK_STATUS task_status, AdditionalData_t* additional_data) {
	_task_status = task_status;
	_last_state_update_time = std::chrono::system_clock::now();
	_UpdateState(additional_data);
}

void Task_t::SetWorkerId(int worker_id) {
	_worker_id = worker_id;
}

int Task_t::GetWorkerid() const {
	return _worker_id;
}

void Task_t::SetTag(unsigned int tag) {
	_task_tag = tag;
}

unsigned Task_t::GetTag() const {
	return _task_tag;
}

Task_t::PromisePtr_t Task_t::GetPromise() {
	return _promise ? _promise = std::make_shared<Promise_t>() : _promise;
}

std::chrono::time_point<std::chrono::system_clock> Task_t::GetLastStateUpdateTime() const {
	return _last_state_update_time;
}

std::chrono::time_point<std::chrono::system_clock> Task_t::GetLastRequestSendTime() const {
	return _last_request_send_time;
}

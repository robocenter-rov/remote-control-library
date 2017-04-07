#include "ReceiveBluetoothMessageTask.h"

void ReceiveBluetoothMessageTask_t::_UpdateState(AdditionalData_t* additional_data) {
	auto ad = static_cast<BluetoothDataReadedAdditionalData_t*>(additional_data);

	std::static_pointer_cast<Promise_t>(_promise)->SetOk(std::string(ad->message, 0, ad->message_size));
}

ReceiveBluetoothMessageTask_t::ReceiveBluetoothMessageTask_t() : Task_t(TI_RECEIVE_BLUETOOTH_MESSAGE){}

void ReceiveBluetoothMessageTask_t::Promise_t::SetWaiting() {
	if (!_waiting) {
		_on_waiting();
	}
	_waiting = true;
}

void ReceiveBluetoothMessageTask_t::Promise_t::SetOk(std::string message) {
	if (!_ended) {
		_message = message;
		_on_ok(message);
	}
	_ended = true;
}

ReceiveBluetoothMessageTask_t::Promise_t* ReceiveBluetoothMessageTask_t::Promise_t::OnWaiting(empty_callback_t on_waiting) {
	if (_waiting) {
		on_waiting();
	}
	_on_waiting = on_waiting;
	return this;
}

ReceiveBluetoothMessageTask_t::Promise_t* ReceiveBluetoothMessageTask_t::Promise_t::OnOk(on_receive_bluetooth_message_ok_t on_ok) {
	if (_ended) {
		on_ok(_message);
	}
	_on_ok = on_ok;
	return this;
}

void ReceiveBluetoothMessageTask_t::SendCommand(Communicator_t* communicator, int worker_id, unsigned tag) {
	communicator->StartBluetoothReading(worker_id, tag);
}

ReceiveBluetoothMessageTask_t::PromisePtr_t ReceiveBluetoothMessageTask_t::GetPromise() {
	return _GetPromise<Promise_t>();
}

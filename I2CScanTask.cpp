#include "I2CScanTask.h"

I2CScanTask_t::I2CScanTask_t() : Task_t(TI_I2C_SCAN) {}

void I2CScanTask_t::Promise_t::SetScanning() {
	if (!_scanning && _on_scanning) {
		_on_scanning();
	}
	_scanning = true;
}

void I2CScanTask_t::Promise_t::SetOk(std::vector<char> adresses) {
	if (!_ended && _on_ok) {
		_ended = true;
		_adresses = adresses;
		_on_ok(_adresses);
	}
	_ended = true;
}

I2CScanTask_t::Promise_t* I2CScanTask_t::Promise_t::OnScanning(empty_callback_t on_scanning) {
	if (_on_scanning) {
		on_scanning();
	}
	_on_scanning = on_scanning;
	return this;
}

I2CScanTask_t::Promise_t* I2CScanTask_t::Promise_t::OnOk(on_i2c_scan_ok_callback_t on_ok) {
	if (_on_ok) {
		on_ok(_adresses);
	}
	_on_ok = on_ok;
	return this;
}

void I2CScanTask_t::SendCommand(Communicator_t* communicator, int worker_id, unsigned tag) {
	communicator->ScanI2C(worker_id, tag);
}

I2CScanTask_t::PromisePtr_t I2CScanTask_t::GetPromise() {
	return std::static_pointer_cast<Promise_t>(Task_t::GetPromise());
}

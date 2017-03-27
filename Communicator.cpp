#include "Communicator.h"

const int BluetoothDataReadedAdditionalData_t::message_size;

BluetoothDataReadedAdditionalData_t::BluetoothDataReadedAdditionalData_t(const char* message) {
	memcpy(this->message, message, message_size);
}

I2CScanningDoneAdditionalData_t::I2CScanningDoneAdditionalData_t(const char* adresses, int count) {
	for (int i = 0; i < count; i++) {
		this->adresses.push_back(adresses[i]);
	}
}

void Communicator_t::Begin() {
	_updating_mutex.lock();
	_updating = true;
	_updating_mutex.unlock();
}

void Communicator_t::Stop() {
	_updating_mutex.lock();
	_updating = false;
	_updating_mutex.unlock();
}

void Communicator_t::SetOnWorkerStateReceive(std::function<void(WorkerState_t worker_state)> on_worker_state_receive) {
	_on_worker_state_receive = on_worker_state_receive;
}

void Communicator_t::SetOnLastUsedWorkerStateReceive(std::function<void(WorkerState_t worker_state)> on_last_used_worker_state_receive) {
	_on_last_used_worker_state_receive = on_last_used_worker_state_receive;
}

void Communicator_t::SetOnPongReceive(std::function<void()> on_pong_receive) {
	_on_pong_receive = on_pong_receive;
}

void Communicator_t::SetOnSensorDataReceive(std::function<void(float, float, float, float, float)> on_sensor_data_receive) {
	_on_sensor_data_receive = on_sensor_data_receive;
}

Communicator_t::~Communicator_t() {
	Stop();
}

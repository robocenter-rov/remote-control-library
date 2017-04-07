#include "Communicator.h"

const int BluetoothDataReadedAdditionalData_t::message_size;

BluetoothDataReadedAdditionalData_t::BluetoothDataReadedAdditionalData_t(const uint8_t* message) {
	memcpy(this->message, message, message_size);
}

I2CScanningDoneAdditionalData_t::I2CScanningDoneAdditionalData_t(const uint8_t* adresses, int count) {
	for (int i = 0; i < count; i++) {
		this->adresses.push_back(adresses[i]);
	}
}

WorkerState_t::WorkerState_t() {
	worker_id = -1;
	additional_data = nullptr;
	task_tag = -1; // MAXUINT
}

Communicator_t::Communicator_t(ConnectionProvider_t* connection_provider) : _updating(false), _connection_provider(connection_provider) {}

void Communicator_t::Begin() {
	_connection_provider->Begin();

	_updating_mutex.lock();
	if (!_updating) {
		_updating = true;
	}
	_updating_mutex.unlock();
}

void Communicator_t::Stop() {
	_updating_mutex.lock();
	bool updating = _updating;
	_updating_mutex.unlock();
	if (updating) {
		_updating_mutex.lock();
		_updating = false;
		_updating_mutex.unlock();

		_updater_thread.join();
	}

	_connection_provider->Stop();
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

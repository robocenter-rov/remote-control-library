#include "StdCommunicator.h"
#include <cstdint>
#include <algorithm>
#include "Utils.h"

enum RECEIVE_MESSAGE_ID {
	MSG_EXCEPTION = 100,
	MSG_WORKER_STATE = 1,
	MSG_LAST_USED_WORKER_STATE = 3,
	MSG_PONG = 20,
	MSG_SENSOR_DATA = 2,
};

StdCommunicator_t::StdCommunicator_t(ConnectionProvider_t* connection_provider) {
	_connection_provider = connection_provider;
}

ConnectionProvider_t* StdCommunicator_t::BeginMessage(SEND_MESSAGE_ID msg_id) const {
	return _connection_provider->BeginPacket()->Write(char(msg_id));
}

ConnectionProvider_t* StdCommunicator_t::BeginTaskMessage(SEND_MESSAGE_ID msg_id, int worker_id, unsigned int tag) const {
	return BeginMessage(msg_id)->Write(worker_id)->Write(tag);
}

void StdCommunicator_t::fill_task_msg(char* buffer, int workerId, int tag) {
	set_le_uint16(buffer, workerId);
	set_le_uint16(buffer + 2, tag);
}

void StdCommunicator_t::Update() {
	while (true) {
		_updating_mutex.lock();
		if (!_updating) {
			return;
		}
		_updating_mutex.unlock();

		const char* buffer;
		int received = _connection_provider->Receive(&buffer);
		ConnectionProvider_t::DataReader_t dr(buffer, received);

		RECEIVE_MESSAGE_ID msg_id = static_cast<RECEIVE_MESSAGE_ID>(dr.GetByte());

		AdditionalData_t* additional_data = nullptr;

		try {
		switch (msg_id) {
			case MSG_EXCEPTION: break;
			case MSG_WORKER_STATE: case MSG_LAST_USED_WORKER_STATE: {

				int worker_id = dr.GetInt();
				WORKER_STATUS worker_status = static_cast<WORKER_STATUS>(dr.GetInt());
				TASK_ID task_id = static_cast<TASK_ID>(dr.GetInt());
				int task_tag = dr.GetInt();
				TASK_STATUS task_status = static_cast<TASK_STATUS>(dr.GetInt());

				switch (task_status) {
					case TS_OK:
						switch (task_id) {
							case TI_BLINK_FLASHLIGHT: break;
							case TI_I2C_SCAN:
								uint16_t device_count;
								device_count = dr.GetUInt();
								additional_data = new I2CScanningDoneAdditionalData_t(dr.GetBytes(device_count), device_count);
								break;
							case TI_RECEIVE_BLUETOOTH_MESSAGE:
								additional_data = new BluetoothDataReadedAdditionalData_t(
									dr.GetBytes(BluetoothDataReadedAdditionalData_t::message_size)
								);
								break;
							case TI_SEND_SENSOR_DATA: break;
							case TI_SET_FLASH_LIGHT_STATE: break;
							default:;
						}
					case TS_CANCELLED: break;
					case TS_BLINKING:
						BlinkFlashlightAdditionalData_t* ad;
						ad = new BlinkFlashlightAdditionalData_t;
						ad->blinked_count = dr.GetUInt();
						additional_data = ad;
						break;
					case TS_BLUETOOTH_WAITING_FOR_CONNECTION: break;
					case TS_I2C_SCANNING: break;
					default: continue;
				}

				auto worker_state = WorkerState_t(worker_id, worker_status, task_id, task_tag, task_status, additional_data);

				if (msg_id == MSG_WORKER_STATE) {
					_on_worker_state_receive(worker_state);
				}
				else {
					_on_last_used_worker_state_receive(worker_state);
				}

				delete additional_data;
			}
								   break;
			case MSG_PONG: _on_pong_receive(); break;
			case MSG_SENSOR_DATA: {
				_on_sensor_data_receive(
					dr.GetFloat(),
					dr.GetFloat(),
					dr.GetFloat(),
					dr.GetFloat(),
					dr.GetFloat()
				);
			} break;
			default:;
		}
		} catch(ConnectionProvider_t::DataReader_t::too_short_buffer_exception_t) {}
	}
}

void StdCommunicator_t::Begin() {
	_connection_provider->Begin();

	_updater_thread = new std::thread(&StdCommunicator_t::Update, this);
}

void StdCommunicator_t::Stop() {
	_connection_provider->Stop();
}

void StdCommunicator_t::SendMotorsThrust(float motor1, float motor2, float motor3, float motor4, float motor5, float motor6) {
	BeginMessage(MSG_SET_MOTORS_THRUST)
		->Write(motor1)
		->Write(motor2)
		->Write(motor3)
		->Write(motor4)
		->Write(motor5)
		->Write(motor6)
	->EndPacket();
}

void StdCommunicator_t::SendLocalMotorsForce(float move_x, float move_y, float move_z, float rotate_y, float rotate_z) {
	BeginMessage(MSG_SET_LOCAL_FORCE)
		->Write(move_x)
		->Write(move_y)
		->Write(move_z)
		->Write(rotate_y)
		->Write(rotate_z)
	->EndPacket();
}

void StdCommunicator_t::SendFlashLightState(int workerId, unsigned tag, bool state) {
	BeginTaskMessage(MSG_SET_FLASHLIGHT_STATE, workerId, tag)
		->Write(state)
	->EndPacket();
}

void StdCommunicator_t::StartBluetoothReading(int workerId, unsigned tag) {
	BeginTaskMessage(MSG_START_BLUETOOTH_READING, workerId, tag)->EndPacket();
}

void StdCommunicator_t::ScanI2C(int workerId, unsigned tag) {
	BeginTaskMessage(MSG_SCAN_I2C, workerId, tag)->EndPacket();
}

void StdCommunicator_t::GetLastUsedWorkerState() {
	BeginMessage(MSG_GET_LAST_USED_WORKER_STATE)->EndPacket();
}

void StdCommunicator_t::GetWorkerState(int workerId) {
	BeginMessage(MSG_GET_WORKER_STATE)->Write(workerId)->EndPacket();
}

void StdCommunicator_t::SendPing() {
	BeginMessage(MSG_PING)->EndPacket();
}

void StdCommunicator_t::FreeWorker(int workerId) {
	BeginMessage(MSG_FREE_WORKER)->Write(workerId)->EndPacket();
}

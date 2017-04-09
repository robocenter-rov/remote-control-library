#include "StdCommunicator.h"
#include <cstdint>
#include <algorithm>
#include "Utils.h"
#include "DataReader.h"

enum RECEIVE_MESSAGE_ID {
	MSG_EXCEPTION = 100,
	MSG_WORKER_STATE = 1,
	MSG_LAST_USED_WORKER_STATE = 3,
	MSG_PONG = 20,
	MSG_SENSOR_DATA = 2,
};

StdCommunicator_t::StdCommunicator_t(ConnectionProvider_t* connection_provider) : Communicator_t(connection_provider) {}

ConnectionProvider_t* StdCommunicator_t::BeginMessage(SEND_MESSAGE_ID msg_id) const {
	return _connection_provider->BeginPacket()->WriteUInt8(msg_id);
}

ConnectionProvider_t* StdCommunicator_t::BeginTaskMessage(SEND_MESSAGE_ID msg_id, int worker_id, unsigned int tag) const {
	return BeginMessage(msg_id)->WriteInt16(worker_id)->WriteInt16(tag);
}

void StdCommunicator_t::fill_task_msg(char* buffer, int workerId, int tag) {
	set_le_uint16(buffer, workerId);
	set_le_uint16(buffer + 2, tag);
}

void StdCommunicator_t::Update() {
	while (true) {
		_updating_mutex.lock();
		bool updating = _updating;
		_updating_mutex.unlock();
		if (!updating) {
			return;
		}

		int received;

		if (!(received = _connection_provider->Receive())) {
			continue;
		}

		const void* buffer = _connection_provider->ReceiveBuffer();
		DataReader_t dr(buffer, received);

		RECEIVE_MESSAGE_ID msg_id = static_cast<RECEIVE_MESSAGE_ID>(dr.GetUInt8());

		AdditionalData_t* additional_data = nullptr;

		try {
		switch (msg_id) {
			case MSG_EXCEPTION: break;
			case MSG_WORKER_STATE: case MSG_LAST_USED_WORKER_STATE: {

				int worker_id = dr.GetInt16();

				auto worker_state = WorkerState_t();

				if (worker_id >= 0) {
					WORKER_STATUS worker_status = static_cast<WORKER_STATUS>(dr.GetInt16());
					TASK_ID task_id = static_cast<TASK_ID>(dr.GetInt16());
					int task_tag = dr.GetInt16();
					TASK_STATUS task_status = static_cast<TASK_STATUS>(dr.GetInt16());

					switch (task_status) {
					case TS_OK:
						switch (task_id) {
						case TI_BLINK_FLASHLIGHT: break;
						case TI_I2C_SCAN:
							uint16_t device_count;
							device_count = dr.GetUInt16();
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
						ad->blinked_count = dr.GetUInt16();
						additional_data = ad;
						break;
					case TS_BLUETOOTH_WAITING_FOR_CONNECTION: break;
					case TS_I2C_SCANNING: break;
					default: continue;
					}

					worker_state = WorkerState_t(worker_id, worker_status, task_id, task_tag, task_status, additional_data);
				}

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
		} catch(DataReader_t::too_short_buffer_exception_t) {}
	}
}

void StdCommunicator_t::Begin() {
	Communicator_t::Begin();

	_updater_thread = std::thread(&StdCommunicator_t::Update, this);
}

void StdCommunicator_t::SendMotorsThrust(float motor1, float motor2, float motor3, float motor4, float motor5, float motor6) {
	BeginMessage(MSG_SET_MOTORS_THRUST)
		->WriteFloat(motor1)
		->WriteFloat(motor2)
		->WriteFloat(motor3)
		->WriteFloat(motor4)
		->WriteFloat(motor5)
		->WriteFloat(motor6)
	->EndPacket();
}

void StdCommunicator_t::SendLocalMotorsForce(float move_x, float move_y, float move_z, float rotate_y, float rotate_z) {
	BeginMessage(MSG_SET_LOCAL_FORCE)
		->WriteFloat(move_x)
		->WriteFloat(move_y)
		->WriteFloat(move_z)
		->WriteFloat(rotate_y)
		->WriteFloat(rotate_z)
	->EndPacket();
}

void StdCommunicator_t::SendFlashLightState(int workerId, unsigned tag, bool state) {
	BeginTaskMessage(MSG_SET_FLASHLIGHT_STATE, workerId, tag)
		->WriteBool(state)
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
	BeginMessage(MSG_GET_WORKER_STATE)->WriteInt16(workerId)->EndPacket();
}

void StdCommunicator_t::SendPing() {
	BeginMessage(MSG_PING)->EndPacket();
}

void StdCommunicator_t::FreeWorker(int workerId) {
	BeginMessage(MSG_FREE_WORKER)->WriteInt16(workerId)->EndPacket();
}

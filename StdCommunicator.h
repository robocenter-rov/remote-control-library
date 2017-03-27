#pragma once
#include "Communicator.h"
#include "ConnectionProvider.h"

class StdCommunicator_t : public Communicator_t {
private:
	ConnectionProvider_t* _connection_provider;

	enum SEND_MESSAGE_ID {
		MSG_SET_MOTORS_THRUST = 1,
		MSG_SET_LOCAL_FORCE = 12,
		MSG_SET_FLASHLIGHT_STATE = 5,
		MSG_START_BLUETOOTH_READING = 3,
		MSG_SCAN_I2C = 17,
		MSG_GET_LAST_USED_WORKER_STATE = 9,
		MSG_GET_WORKER_STATE = 11,
		MSG_PING = 20,
		MSG_FREE_WORKER = 100,
	};

	ConnectionProvider_t* BeginMessage(SEND_MESSAGE_ID msg_id) const;
	ConnectionProvider_t* BeginTaskMessage(SEND_MESSAGE_ID msg_id, int worker_id, unsigned int tag) const;
	static inline void fill_task_msg(char* buffer, int workerId, int tag);

	void Update();
public:
	StdCommunicator_t(ConnectionProvider_t* connection_provider);

	void Begin() override;
	void Stop() override;
	void SendMotorsThrust(float motor1, float motor2, float motor3, float motor4, float motor5, float motor6) override;
	void SendLocalMotorsForce(float move_x, float move_y, float move_z, float rotate_y, float rotate_z) override;
	void SendFlashLightState(int workerId, unsigned tag, bool state) override;
	void StartBluetoothReading(int workerId, unsigned tag) override;
	void ScanI2C(int workerId, unsigned tag) override;
	void GetLastUsedWorkerState() override;
	void GetWorkerState(int workerId) override;
	void SendPing() override;
	void FreeWorker(int workerId) override;
};

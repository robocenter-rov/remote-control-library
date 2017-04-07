#pragma once
#include <functional>
#include <thread>
#include <mutex>
#include <vector>
#include "ConnectionProvider.h"

enum TASK_STATUS {
	TS_NONE,

	TS_OK,
	TS_CANCELLED,

	TS_BLINKING,

	TS_BLUETOOTH_WAITING_FOR_CONNECTION,

	TS_I2C_SCANNING,
};

enum TASK_ID {
	TI_BLINK_FLASHLIGHT = 0,
	TI_I2C_SCAN = 1,
	TI_RECEIVE_BLUETOOTH_MESSAGE = 2,
	TI_SEND_SENSOR_DATA = 3,
	TI_SET_FLASH_LIGHT_STATE = 4,
};

struct AdditionalData_t {
	virtual ~AdditionalData_t() = default;
};

struct BlinkFlashlightAdditionalData_t : AdditionalData_t {
	unsigned int blinked_count;
};

struct BluetoothDataReadedAdditionalData_t : AdditionalData_t {
	static const int message_size = 7;
	BluetoothDataReadedAdditionalData_t(const uint8_t* message);
	char message[message_size];
};

struct I2CScanningDoneAdditionalData_t : AdditionalData_t {
	I2CScanningDoneAdditionalData_t(const uint8_t* adresses, int count);
	std::vector<char> adresses;
};

enum WORKER_STATUS {
	WS_FREE,
	WS_BUSY,
	WS_AWAIT
};

struct WorkerState_t {
	int worker_id;
	WORKER_STATUS worker_status;
	TASK_ID task_id;
	unsigned int task_tag;
	TASK_STATUS task_status;
	AdditionalData_t* additional_data;

	WorkerState_t(int worker_id, WORKER_STATUS worker_status, TASK_ID task_id, unsigned int task_tag, TASK_STATUS task_status, AdditionalData_t* additional_data);
	WorkerState_t();
};

class Communicator_t {
protected:
	std::function<void(WorkerState_t worker_state)> _on_worker_state_receive;
	std::function<void(WorkerState_t worker_state)> _on_last_used_worker_state_receive;
	std::function<void()> _on_pong_receive;
	std::function<void(float, float, float, float, float)> _on_sensor_data_receive;

	std::thread _updater_thread;
	bool _updating;
	std::mutex _updating_mutex;

	ConnectionProvider_t* _connection_provider;
public:
	Communicator_t(ConnectionProvider_t* connection_provider);
	virtual void Begin();
	virtual void Stop();
	virtual void SendMotorsThrust(float motor1, float motor2, float motor3, float motor4, float motor5, float motor6) = 0;
	virtual void SendLocalMotorsForce(float move_x, float move_y, float move_z, float rotate_y, float rotate_z) = 0;
	virtual void SendFlashLightState(int workerId, unsigned int tag, bool state) = 0;
	virtual void StartBluetoothReading(int workerId, unsigned int tag) = 0;
	virtual void ScanI2C(int workerId, unsigned int tag) = 0;
	virtual void GetLastUsedWorkerState() = 0;
	virtual void GetWorkerState(int workerId) = 0;
	virtual void SendPing() = 0;
	virtual void FreeWorker(int workerId) = 0;

	void SetOnWorkerStateReceive(std::function<void(WorkerState_t worker_state)>);
	void SetOnLastUsedWorkerStateReceive(std::function<void(WorkerState_t worker_state)>);
	void SetOnPongReceive(std::function<void()>);
	void SetOnSensorDataReceive(std::function<void(float, float, float, float, float)>);

	virtual ~Communicator_t();
};

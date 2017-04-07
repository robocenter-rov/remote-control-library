#pragma once
#include "StdCommunicator.h"
#include "TaskManager.h"

class Controller_t {
private:
	Communicator_t* _communicator;
	TaskManager_t _task_manager;

	std::function<void ()> _on_pong;
public:
	Controller_t(Communicator_t* communicator);
	~Controller_t();

	void Begin();
	void Stop();
	void SetMotorsThrust(float m1, float m2, float m3, float m4, float m5, float m6) const;
	void SetLocalForce(float move_x, float move_y, float move_z, float rotate_y, float rotate_z) const;
	void Ping() const;
	void OnPong(std::function<void()> on_pong) const;
	ReceiveBluetoothMessageTask_t::PromisePtr_t ReadFromBluetooth();
	I2CScanTask_t::PromisePtr_t ScanI2C();
	SetFlashlightStateTask_t::PromisePtr_t SetFlashlightState(bool state);
};

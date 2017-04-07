#include "Controller.h"

Controller_t::Controller_t(Communicator_t* communicator) : _task_manager(communicator) {
	_communicator = communicator;
}

Controller_t::~Controller_t() {
	Stop();
}

void Controller_t::Begin() {
	_communicator->Begin();
	_task_manager.Begin();
}

void Controller_t::Stop() {
	_task_manager.Stop();
	_communicator->Stop();
}

void Controller_t::SetMotorsThrust(float m1, float m2, float m3, float m4, float m5, float m6) const {
	_communicator->SendMotorsThrust(m1, m2, m3, m4, m5, m6);
}

void Controller_t::SetLocalForce(float move_x, float move_y, float move_z, float rotate_y, float rotate_z) const {
	_communicator->SendLocalMotorsForce(move_x, move_y, move_z, rotate_y, rotate_z);
}

void Controller_t::Ping() const {
	_communicator->SendPing();
}

void Controller_t::OnPong(std::function<void()> on_pong) const {
	_communicator->SetOnPongReceive(on_pong);
}

ReceiveBluetoothMessageTask_t::PromisePtr_t Controller_t::ReadFromBluetooth() {
	return _task_manager.ReadFromBluetooth();
}

I2CScanTask_t::PromisePtr_t Controller_t::ScanI2C() {
	return _task_manager.ScanI2C();
}

SetFlashlightStateTask_t::PromisePtr_t Controller_t::SetFlashlightState(bool state) {
	return _task_manager.SetFlashlightState(state);
}

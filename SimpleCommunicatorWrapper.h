#pragma once
#include "SimpleCommunicator.h"

class SimpleCommunicatorWrapper_t {
private:
	SimpleCommunicator_t* _simple_communicator;
	ConnectionProvider_t* _connection_provider;

	bool _connection_state;
	struct { int receive; int send; } _packets_leaked;
	bool _robot_restarted;
	SimpleCommunicator_t::State_t _state;
	SimpleCommunicator_t::I2CDevices_t _scanned_devices;
	bool _i2c_rescanned;
	std::string _bluetooth_msg;
	SimpleCommunicator_t::Orientation_t _orientation;
	float _depth;
	SimpleCommunicator_t::RawSensorData_t _raw_sensor_data;
	SimpleCommunicator_t::CalibratedSensorData_t _calibrated_sensor_data;
	SimpleCommunicator_t::MotorsState_t _motors_state;
	std::string _error;
public:
	SimpleCommunicatorWrapper_t(SimpleCommunicator_t* simple_communicator, ConnectionProvider_t* connection_provider);

	void Begin();
	void Stop() const;
	void SetMotorsMultiplier(float m1, float m2, float m3, float m4, float m5, float m6) const;
	void SetMotorsPositions(int m1, int m2, int m3, int m4, int m5, int m6) const;
	void SetManipulatorState(float arm_pos, float hand_pos, float m1, float m2) const;
	void SetCamera1Pos(float camera1) const;
	void SetCamera2Pos(float camera2) const;
	void SetMotorsState(float m1, float m2, float m3, float m4, float m5, float m6, float m7, float m8) const;
	void SetMovementForce(float x, float y) const;
	void SetSinkingForce(float z) const;
	void SetDepth(float depth) const;
	void SetPitchForce(float pitch_force) const;
	void SetPitch(float pitch) const;
	void SetYawForce(float yaw_force) const;
	void SetYaw(float yaw) const;
	void ScanI2C() const;
	void SetFlashlightState(bool state) const;
	void SetReadBluetoothState(bool read) const;
	void SetDepthPid(float p, float i, float d) const;
	void SetPitcPid(float p, float i, float d) const;
	void SetYawPid(float p, float i, float d) const;
	void SetReceiveRawSensorData(bool receive) const;
	void SetReceiveCalibratedSensorData(bool receive) const;
	void SetRescanI2CDevices() const;

	void ConnectionState(bool& state) const;
	void PacketsLeak(int& receive, int& send);
	void RobotRestarted(int& restarted);
	void State(bool& flashlight_state) const;
	void I2CDevices(int& pca1, int& pca2, int& hmc58x3, int& itg3200, int& adxl345, int& bmp085, int& ms5803, int& rescanned);
	void BluetoothMessage(char* msg) const;
	void Orientation(float& q0, float& q1, float& q2, float& q3) const;
	void Depth(float& depth) const;
	void RawSensorData(int& ax, int& ay, int& az, int& gx, int& gy, int& gz, int& mx, int& my, int& mz, float& depth) const;
	void CalibratedSensorData(float& ax, float& ay, float& az, float& gx, float& gy, float& gz, float& mx, float& my, float& mz, float& depth) const;
	void MotorsState(float& m1, float& m2, float& m3, float& m4, float& m5, float& m6) const;
	int Error(char* error_msg, int buff_size);
};

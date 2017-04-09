#pragma once
#include "ConnectionProvider.h"
#include <thread>

enum I2C_DEVICES {
	I2C_PCA1 = 1,
	I2C_PCA2 = 2,
	I2C_HMC58X3 = 3,
	I2C_ITG3200 = 4,
	I2C_ADXL345 = 5,
	I2C_BMP085 = 6,
	I2C_MS5803 = 7,
};

class SimpleCommunicator_t {
public:
	struct State_t {
		bool FlashlightState : 1;
		bool ReadBluetooth : 1;
	};

	struct I2CDevices_t {
		bool PCA1 : 1;
		bool PCA2 : 1;
		bool HMC58X3 : 1;
		bool ITG3200 : 1;
		bool ADXL345 : 1;
		bool MS5803 : 1;
	};

	struct MotorsState_t {
		float M1Force;
		float M2Force;
		float M3Force;
		float M4Force;
		float M5Force;
		float M6Force;
	};

	struct ManipulatorState_t {
		float ArmPos;
		float HandPos;
		float M1;
		float M2;
	};

	struct RawSensorData_t {
		int Ax;
		int Ay;
		int Az;

		int Gx;
		int Gy;
		int Gz;

		int Mx;
		int My;
		int Mz;
	};

	struct CalibratedSensorData_t {
		float Ax;
		float Ay;
		float Az;

		float Gx;
		float Gy;
		float Gz;

		float Mx;
		float My;
		float Mz;
	};

	struct Orientation_t {
		float q1;
		float q2;
		float q3;
		float q4;
	};

	struct PidState_t {
		float In;
		float Target;
		float Out;
	};

	struct Pid_t {
		float P;
		float I;
		float D;
	};
private:
	ConnectionProvider_t* _connection_provider;

	struct {
		bool M1Dir : 1;
		bool M2Dir : 1;
		bool M3Dir : 1;
		bool M4Dir : 1;
		bool M5Dir : 1;
		bool M6Dir : 1;
	} _motors_directions;

	MotorsState_t _motors_state;

	ManipulatorState_t _manipulator_state;
	float _camera1_pos;
	float _camera2_pos;

	struct {
		float x_force;
		float y_force;
	} _movement_force;

	float _sinking_force;
	float _depth;

	float _pitch_force;
	float _pitch;

	float _yaw_force;
	float _yaw;

	enum MOVEMENT_CONTROL_TYPE {
		MCT_DIRECT,
		MCT_VECTOR,
	} _movement_control_type;

	enum CONTROL_TYPE {
		CT_DIRECT,
		CT_AUTO
	} _depth_control_type, _yaw_control_type, _pitch_control_type;

	State_t _current_remote_state;
	State_t _state;

	Pid_t _depth_pid;
	Pid_t _yaw_pid;
	Pid_t _pitch_pid;

	uint32_t _pid_hash;
	uint32_t _remote_pid_hash;

	uint32_t _last_received_msg_number;
	uint32_t _last_sended_msg_number;

	uint32_t _remote_packets_leak;

	std::chrono::system_clock::time_point _last_received_msg_time;
	std::chrono::system_clock::time_point _last_sended_msg_time;

	std::chrono::system_clock::duration _receive_time_out;
	std::chrono::system_clock::duration _send_frequency;

	bool _connected;

	int16_t _max_motor_force_val;

	uint8_t _last_i2c_scan;
	uint8_t _last_i2c_scan_remote;

	std::thread _updater;
	bool _updating;

	std::function<void(bool)> _on_connection_state_change;
	std::function<void(int, int)> _on_packets_leak;
	std::function<void()> _on_robot_restart;
	std::function<void(State_t)> _on_state_change;
	std::function<void(I2CDevices_t)> _on_i2c_devices_receive;
	std::function<void(std::string)> _on_bluetooth_msg_receive;
	std::function<void(Orientation_t)> _on_orientation_receive;
	std::function<void(float)> _on_depth_receive;
	std::function<void(RawSensorData_t)> _on_raw_sensor_data_receive;
	std::function<void(CalibratedSensorData_t)> _on_calibrated_sensor_data_receive;
	std::function<void(PidState_t, PidState_t, PidState_t)> _on_pid_state_receive;
	std::function<void(MotorsState_t)> _on_motors_state_receive;

	void _UpdatePidHash();

	void _Update();
public:
	SimpleCommunicator_t(ConnectionProvider_t* connection_provider);

	void Begin();
	void Stop();
	void SetMotorsDirection(bool m1, bool m2, bool m3, bool m4, bool m5, bool m6);
	void SetManipulatorState(float arm_pos, float hand_pos, float m1, float m2);
	void SetCamera1Pos(float camera1);
	void SetCamera2Pos(float camera2);
	void SetMotorsState(float m1, float m2, float m3, float m4, float m5, float m6);
	void SetMovementForce(float x, float y);
	void SetSinkingForce(float z);
	void SetDepth(float depth);
	void SetPitchForce(float pitch_force);
	void SetPitch(float pitch);
	void SetYawForce(float yaw_force);
	void SetYaw(float yaw);
	void ScanI2C();
	void SetFlashlightState(bool state);
	void SetReadBluetoothState(bool read);
	void SetDepthPid(float p, float i, float d);
	void SetPitcPid(float p, float i, float d);
	void SetYawPid(float p, float i, float d);

	void OnConnectionStateChange(std::function<void(bool)> on_connection_state_change);
	void OnPacketsLeak(std::function<void(int, int)> on_packets_leak);
	void OnRobotRestart(std::function<void()> on_robot_restart);
	void OnStateChange(std::function<void(State_t)> on_state_change);
	void OnI2CDevicesReceive(std::function<void(I2CDevices_t)> on_i2c_devices_receive);
	void OnBluetoothMsgReceive(std::function<void(std::string)> on_bluetooth_msg_receive);
	void OnOrientationReceive(std::function<void(Orientation_t)> on_orientation_receive);
	void OnDepthReceive(std::function<void(float)> on_depth_receive);
	void OnRawSensorDataReceive(std::function<void(RawSensorData_t)> on_raw_sensor_data_receive);
	void OnCalibratedSensorDataReceive(std::function<void(CalibratedSensorData_t)> on_calibrated_sensor_data_receive);
	void OnMotorsStateReceive(std::function<void(MotorsState_t)> on_motors_state_receive);
};

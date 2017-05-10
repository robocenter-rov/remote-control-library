#pragma once
#include "ConnectionProvider.h"
#include <thread>

class WrongMotorsPosition_t : public ControllerException_t {
public:
	WrongMotorsPosition_t() : ControllerException_t("Wrong motors position") {};
};

class WrongMotorId_t : public ControllerException_t {
public:
	WrongMotorId_t() : ControllerException_t("Wrong motor id") {};
};


class SimpleCommunicator_t {
public:
	struct State_t {
		bool FlashlightState : 1;
		bool ReadBluetooth : 1;
		bool SendRawSensorData : 1;
		bool SendCalibratedSensorData : 1;
		bool SendPidState : 1;
		bool SendMotorsState : 1;
	};

	struct I2CDevices_t {
		bool PCA1 : 1;
		bool PCA2 : 1;
		bool HMC58X3 : 1;
		bool ITG3200 : 1;
		bool ADXL345 : 1;
		bool BMP085 : 1;
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
		float Depth;
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
		float Depth;
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
#pragma pack(push, 1)
	struct {
		bool camera1_direction : 1;
		bool camera2_direction : 1;
	} Camera_directions;
#pragma pack(pop)

	struct
	{
		float camera1_offset;
		float camera2_offset;
	} Camera_offsets;
private:
	ConnectionProvider_t* _connection_provider;

#pragma pack(push, 1)
	struct {
		struct {
			unsigned char M1Pos;
			unsigned char M2Pos;
			unsigned char M3Pos;
			unsigned char M4Pos;
			unsigned char M5Pos;
			unsigned char M6Pos;
		} MPositions;

		struct {
			float M1mul;
			float M2mul;
			float M3mul;
			float M4mul;
			float M5mul;
			float M6mul;
		} MMultipliers;
	} _motors_config;
#pragma pack(pop)

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

	uint32_t _config_hash;
	uint32_t _remote_config_hash;

	uint32_t _last_received_msg_number;
	uint32_t _last_sended_msg_number;

	uint16_t _remote_packets_leak;

	unsigned long _remote_send_frequency;
	unsigned long _remote_receive_timeout;

	std::chrono::system_clock::time_point _last_received_msg_time;
	std::chrono::system_clock::time_point _last_sended_msg_time;

	std::chrono::system_clock::duration _receive_time_out;
	std::chrono::system_clock::duration _send_frequency;

	bool _connected;

	uint8_t _last_i2c_scan;
	uint8_t _last_i2c_scan_remote;

	std::thread _sender;
	std::thread _receiver;
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
	std::function<void(std::string)> _on_stop;
	std::function<void(unsigned long)> _on_msg_receive;
	std::function<void(unsigned long)> _on_remote_processor_load_receive;

	void _UpdateConfigHash();

	void _Receiver();
	void _Sender();
public:
	SimpleCommunicator_t(ConnectionProvider_t* connection_provider);

	void Begin();
	void Stop();

	void SetSendMessageFrequency(unsigned long millis);
	void SetReceiveTimeout(unsigned long millis);

	void SetRemoteSendMessageFrequency(unsigned long millis);
	void SetRemoteReceiveTimeout(unsigned long millis);

	void SetMotorsMultiplier(float m1, float m2, float m3, float m4, float m5, float m6);
	void SetMotorsPositions(int m1, int m2, int m3, int m4, int m5, int m6);
	void SetManipulatorState(float arm_pos, float hand_pos, float m1, float m2);
	void SetCamera1Pos(float camera1);
	void SetCamera2Pos(float camera2);
	void SetMotorsState(float m1, float m2, float m3, float m4, float m5, float m6);
	void SetMotorState(int motor_id, float force);
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
	void SetReceiveRawSensorData(bool receive);
	void SetReceiveCalibratedSensorData(bool receive);
	void SetReceivePidState(bool receive);
	void SetReceiveMotorsState(bool receive);
	void SetRescanI2CDevices();
    bool IsAutoDepthEnabled();
    bool IsAutoPitchEnabled();
    bool IsAutoYawEnabled();
	void SetCam1Offset(float offset);
	void SetCam2Offset(float offset);
	void SetCam1Direction(bool direction);
	void SetCam2Direction(bool direction);

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
	void OnPidStateReceive(std::function<void(PidState_t, PidState_t, PidState_t)> on_pid_state_receive);
	void OnMotorsStateReceive(std::function<void(MotorsState_t)> on_motors_state_receive);
	void OnStop(std::function<void(std::string)> on_stop);
	void OnMessageReceive(std::function<void(unsigned long)> on_msg_receive);
	void OnRemoteProcessorLoad(std::function<void(unsigned long)> on_remote_processor_load_receive);

};

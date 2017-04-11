#include "SimpleCommunicator.h"
#include "DataReader.h"
#include "Utils.h"
#include <complex>

enum RECEIVE_BLOCK_IDS {
	RBI_STATE = 0,
	RBI_SENSOR_DATA = 1,
	RBI_RAW_SENSOR_DATA = 2,
	RBI_CALIBRATED_SENSOR_DATA = 3,
	RBI_BLUETOOTH_MSG_RECEIVE = 4,
	RBI_MOTORS_STATE_RECEIVE = 5,
	RBI_PID_STATE_RECEIVE = 6,
};

enum SEND_BLOCK_IDS {
	SBI_STATE = 0,
	SBI_DEVICES_STATE = 1,
	SBI_MOTORS_STATE = 2,
	SBI_MOVEMENT = 3,
	SBI_PID = 4,
};

SimpleCommunicator_t::SimpleCommunicator_t(ConnectionProvider_t* connection_provider) {
	_connection_provider = connection_provider;
	_updating = false;
	_last_i2c_scan = 0;
	_last_i2c_scan_remote = 0;
	_last_received_msg_number = 0;
	_last_sended_msg_number = 0;
	_depth_control_type = CT_DIRECT;
	_pitch_control_type = CT_DIRECT;
	_yaw_control_type = CT_DIRECT;
	_yaw = 0;
	_yaw_force = 0;
	_pitch = 0;
	_pitch_force = 0;
	_depth = 0;
	_sinking_force = 0;
	_movement_control_type = MCT_DIRECT;
	_connected = false;
	_pid_hash = 0;
	_remote_pid_hash = 0;
	_remote_packets_leak = 0;
	_camera1_pos = 0;
	_camera2_pos = 0;

	_receive_time_out = std::chrono::milliseconds(500);
	_send_frequency = std::chrono::milliseconds(20);
}

void SimpleCommunicator_t::Begin() {
	_updating = true;
	_receiver = std::thread(&SimpleCommunicator_t::_Receiver, this);
	_sender = std::thread(&SimpleCommunicator_t::_Sender, this);
}

void SimpleCommunicator_t::Stop() {
	_updating = false;

	_receiver.join();
	_sender.join();
}

inline void SimpleCommunicator_t::SetMotorsDirection(bool m1, bool m2, bool m3, bool m4, bool m5, bool m6) {
	_motors_directions.M1Dir = m1;
	_motors_directions.M2Dir = m2;
	_motors_directions.M3Dir = m3;
	_motors_directions.M4Dir = m4;
	_motors_directions.M5Dir = m5;
	_motors_directions.M6Dir = m6;
}

void SimpleCommunicator_t::SetManipulatorState(float arm_pos, float hand_pos, float m1, float m2) {
	_manipulator_state.ArmPos = arm_pos;
	_manipulator_state.HandPos = hand_pos;
	_manipulator_state.M1 = m1;
	_manipulator_state.M2 = m2;
}

void SimpleCommunicator_t::SetCamera1Pos(float camera1) {
	_camera1_pos = camera1;
}

void SimpleCommunicator_t::SetCamera2Pos(float camera2) {
	_camera2_pos = camera2;
}

void SimpleCommunicator_t::SetMotorsState(float m1, float m2, float m3, float m4, float m5, float m6) {
	_motors_state.M1Force = m1;
	_motors_state.M2Force = m2;
	_motors_state.M3Force = m3;
	_motors_state.M4Force = m4;
	_motors_state.M5Force = m5;
	_motors_state.M6Force = m6;
	_movement_control_type = MCT_DIRECT;
}

void SimpleCommunicator_t::SetMovementForce(float x, float y) {
	_movement_force.x_force = x;
	_movement_force.y_force = y;
	_movement_control_type = MCT_VECTOR;
}

void SimpleCommunicator_t::SetSinkingForce(float z) {
	_sinking_force = z;
	_depth_control_type = CT_DIRECT;
}

void SimpleCommunicator_t::SetDepth(float depth) {
	_depth = depth;
	_depth_control_type = CT_AUTO;
}

void SimpleCommunicator_t::SetPitchForce(float pitch_force) {
	_pitch_force = pitch_force;
	_pitch_control_type = CT_DIRECT;
}

void SimpleCommunicator_t::SetPitch(float pitch) {
	_pitch = pitch;
	_pitch_control_type = CT_AUTO;
}

void SimpleCommunicator_t::SetYawForce(float yaw_force) {
	_yaw_force = yaw_force;
	_yaw_control_type = CT_AUTO;
}

void SimpleCommunicator_t::SetYaw(float yaw) {
	_yaw = yaw;
	_yaw_control_type = CT_DIRECT;
}

void SimpleCommunicator_t::ScanI2C() {
	_last_i2c_scan++;
}

void SimpleCommunicator_t::SetFlashlightState(bool flashlight_state) {
	_state.FlashlightState = flashlight_state;
}

void SimpleCommunicator_t::SetReadBluetoothState(bool read) {
	_state.ReadBluetooth = read;
}

void SimpleCommunicator_t::SetDepthPid(float p, float i, float d) {
	_depth_pid.P = p;
	_depth_pid.I = i;
	_depth_pid.D = d;
	_UpdatePidHash();
}

void SimpleCommunicator_t::SetPitcPid(float p, float i, float d) {
	_pitch_pid.P = p;
	_pitch_pid.I = i;
	_pitch_pid.D = d;
	_UpdatePidHash();
}

void SimpleCommunicator_t::SetYawPid(float p, float i, float d) {
	_yaw_pid.P = p;
	_yaw_pid.I = i;
	_yaw_pid.D = d;
	_UpdatePidHash();
}

void SimpleCommunicator_t::SetReceiveRawSensorData(bool receive) {
	_state.SendRawSensorData = receive;
}

void SimpleCommunicator_t::SetReceiveCalibratedSensorData(bool receive) {
	_state.SendCalibratedSensorData = receive;
}

void SimpleCommunicator_t::SetRescanI2CDevices() {
	_last_i2c_scan++;
}

void SimpleCommunicator_t::OnConnectionStateChange(std::function<void (bool)> on_connection_state_change) {
	_on_connection_state_change = on_connection_state_change;
}

void SimpleCommunicator_t::OnPacketsLeak(std::function<void(int, int)> on_packets_leak) {
	_on_packets_leak = on_packets_leak;
}

void SimpleCommunicator_t::OnRobotRestart(std::function<void()> on_robot_restart) {
	_on_robot_restart = on_robot_restart;
}

void SimpleCommunicator_t::_UpdatePidHash() {
	_pid_hash = HashLy(_depth_pid, 0);
	_pid_hash = HashLy(_pitch_pid, _pid_hash);
	_pid_hash = HashLy(_yaw_pid, _pid_hash);
}

void SimpleCommunicator_t::_Receiver() {
	while (true) {
		if (!_updating) {
			return;
		}

		int readed_bytes = 0;
		if (readed_bytes = _connection_provider->Receive()) {
			try {
				auto dr = DataReader_t(_connection_provider->ReceiveBuffer(), readed_bytes);

				uint32_t msg_number = dr.GetUInt32();
				uint16_t remote_packets_leak = dr.GetUInt16();

				if (msg_number < _last_received_msg_number) {
					if (_on_robot_restart) {
						_on_robot_restart();
					}
				}
				else {
					int send_leak = remote_packets_leak > _remote_packets_leak && _remote_packets_leak ? remote_packets_leak - _remote_packets_leak : 0;;
					int receive_leak = _last_received_msg_number ? msg_number - _last_received_msg_number - 1 : 0;

					if ((send_leak || receive_leak) && _on_packets_leak) {
						_on_packets_leak(send_leak, receive_leak);
					}

					_remote_packets_leak = remote_packets_leak;
				}

				_last_received_msg_number = msg_number;

				while (dr.Available()) {
					RECEIVE_BLOCK_IDS block_id = static_cast<RECEIVE_BLOCK_IDS>(dr.GetUInt8());
					switch (block_id) {
					case RBI_STATE: {
						auto state = dr.GetVar<State_t>();
						if (cmp(state, _current_remote_state) && _on_state_change) {
							_on_state_change(state);
						}
						_current_remote_state = state;

						auto i2c_scan_token = dr.GetUInt8();
						auto i2c_devices = dr.GetVar<I2CDevices_t>();

						if (i2c_scan_token != _last_i2c_scan_remote && _on_i2c_devices_receive) {
							_on_i2c_devices_receive(i2c_devices);
						}
						_last_i2c_scan_remote = i2c_scan_token;

						_remote_pid_hash = dr.GetVar<uint32_t>();
					}
					break;
					case RBI_SENSOR_DATA: {
						Orientation_t orientation;
						orientation.q1 = dr.GetFloat();
						orientation.q2 = dr.GetFloat();
						orientation.q3 = dr.GetFloat();
						orientation.q4 = dr.GetFloat();
						float depth = dr.GetFloat();

						if (_on_orientation_receive) {
							_on_orientation_receive(orientation);
						}

						if (_on_depth_receive) {
							_on_depth_receive(depth);
						}
					}
					break;
					case RBI_RAW_SENSOR_DATA: {
						RawSensorData_t raw_sendor_data;

						raw_sendor_data.Ax = dr.GetInt16();
						raw_sendor_data.Ay = dr.GetInt16();
						raw_sendor_data.Az = dr.GetInt16();

						raw_sendor_data.Gx = dr.GetInt16();
						raw_sendor_data.Gy = dr.GetInt16();
						raw_sendor_data.Gz = dr.GetInt16();

						raw_sendor_data.Mx = dr.GetInt16();
						raw_sendor_data.My = dr.GetInt16();
						raw_sendor_data.Mz = dr.GetInt16();

						raw_sendor_data.Depth = dr.GetFloat();
						if (_on_raw_sensor_data_receive) {
							_on_raw_sensor_data_receive(raw_sendor_data);
						}
					}
					break;
					case RBI_CALIBRATED_SENSOR_DATA: {
						CalibratedSensorData_t calibrated_sensor_data;

						calibrated_sensor_data.Ax = dr.GetFloat();
						calibrated_sensor_data.Ay = dr.GetFloat();
						calibrated_sensor_data.Az = dr.GetFloat();

						calibrated_sensor_data.Gx = dr.GetFloat();
						calibrated_sensor_data.Gy = dr.GetFloat();
						calibrated_sensor_data.Gz = dr.GetFloat();

						calibrated_sensor_data.Mx = dr.GetFloat();
						calibrated_sensor_data.My = dr.GetFloat();
						calibrated_sensor_data.Mz = dr.GetFloat();

						calibrated_sensor_data.Depth = dr.GetFloat();

						if (_on_calibrated_sensor_data_receive) {
							_on_calibrated_sensor_data_receive(calibrated_sensor_data);
						}
					}
					break;
					case RBI_BLUETOOTH_MSG_RECEIVE: {
						const char* msg = reinterpret_cast<const char*>(dr.GetBytes(7));

						if (_on_bluetooth_msg_receive) {
							_on_bluetooth_msg_receive(std::string(msg, 7));
						}
					}
					break;
					case RBI_PID_STATE_RECEIVE: {
						PidState_t depth_pid;

						depth_pid.In = dr.GetFloat();
						depth_pid.Target = dr.GetFloat();
						depth_pid.Out = dr.GetFloat();

						PidState_t pitch_pid;

						pitch_pid.In = dr.GetFloat();
						pitch_pid.Target = dr.GetFloat();
						pitch_pid.Out = dr.GetFloat();

						PidState_t yaw_pid;

						yaw_pid.In = dr.GetFloat();
						yaw_pid.Target = dr.GetFloat();
						yaw_pid.Out = dr.GetFloat();

						if (_on_pid_state_receive) {
							_on_pid_state_receive(depth_pid, pitch_pid, yaw_pid);
						}
					}
					break;
					case RBI_MOTORS_STATE_RECEIVE: {
						MotorsState_t motors_state;

						motors_state.M1Force = dr.GetFloat();
						motors_state.M2Force = dr.GetFloat();
						motors_state.M3Force = dr.GetFloat();
						motors_state.M4Force = dr.GetFloat();
						motors_state.M5Force = dr.GetFloat();
						motors_state.M6Force = dr.GetFloat();

						if (_on_motors_state_receive) {
							_on_motors_state_receive(motors_state);
						}
					}
					break;
					default:;
					}
				}
			}
			catch (DataReader_t::too_short_buffer_exception_t e) {
				printf("Too short buffer\n");
			}

			_last_received_msg_time = std::chrono::system_clock::now();
		}

		if (std::chrono::system_clock::now() - _last_received_msg_time > _receive_time_out) {
			if (_connected && _on_connection_state_change) {
				_on_connection_state_change(_connected = false);
			}
		}
		else {
			if (!_connected && _on_connection_state_change) {
				_on_connection_state_change(_connected = true);
			}
		}
	}
}

void SimpleCommunicator_t::_Sender() {
	while (true) {
		if (!_updating) {
			return;
		}

		//printf("%lld\n", std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - _last_sended_msg_time).count());
		_connection_provider->BeginPacket()
			->WriteUInt32(++_last_sended_msg_number)
			->WriteUInt8(SBI_STATE)
			->WriteVar(_state)
			->WriteUInt8(_last_i2c_scan)
			->WriteInt8(SBI_DEVICES_STATE)
			->WriteFloat(_manipulator_state.ArmPos)
			->WriteFloat(_manipulator_state.HandPos)
			->WriteFloat(_manipulator_state.M1)
			->WriteFloat(_manipulator_state.M2)
			->WriteFloat(_camera1_pos)
			->WriteFloat(_camera2_pos);

		switch (_movement_control_type) {
		case MCT_DIRECT:
			_connection_provider
				->WriteUInt8(SBI_MOTORS_STATE)
				->WriteFloat(_motors_state.M1Force)
				->WriteFloat(_motors_state.M2Force)
				->WriteFloat(_motors_state.M3Force)
				->WriteFloat(_motors_state.M4Force)
				->WriteFloat(_motors_state.M5Force)
				->WriteFloat(_motors_state.M6Force);
			break;
		case MCT_VECTOR:
			_connection_provider
				->WriteUInt8(SBI_MOVEMENT)
				->WriteUInt8((_depth_control_type == CT_AUTO) | (_yaw_control_type == CT_AUTO) << 1 | (_pitch_control_type == CT_AUTO) << 2)
				->WriteFloat(_movement_force.x_force)
				->WriteFloat(_movement_force.y_force)
				->WriteFloat(_depth_control_type == CT_AUTO ? _depth : _sinking_force)
				->WriteFloat(_yaw_control_type == CT_AUTO ? _yaw : _yaw_force)
				->WriteFloat(_pitch_control_type == CT_AUTO ? _pitch : _pitch_force);
			break;
		default:;
		}

		if (_remote_pid_hash != _pid_hash) {
			_connection_provider
				->WriteUInt8(SBI_PID)

				->WriteFloat(_depth_pid.P)
				->WriteFloat(_depth_pid.I)
				->WriteFloat(_depth_pid.D)

				->WriteFloat(_pitch_pid.P)
				->WriteFloat(_pitch_pid.I)
				->WriteFloat(_pitch_pid.D)

				->WriteFloat(_yaw_pid.P)
				->WriteFloat(_yaw_pid.I)
				->WriteFloat(_yaw_pid.D);
		}

		_connection_provider->EndPacket();

		_last_sended_msg_time = std::chrono::system_clock::now();

		std::this_thread::sleep_for(_send_frequency);
	}
}

void SimpleCommunicator_t::OnStateChange(std::function<void(State_t)> on_state_change) {
	_on_state_change = on_state_change;
}

void SimpleCommunicator_t::OnI2CDevicesReceive(std::function<void(I2CDevices_t)> on_i2c_devices_receive) {
	_on_i2c_devices_receive = on_i2c_devices_receive;
}

void SimpleCommunicator_t::OnBluetoothMsgReceive(std::function<void(std::string)> on_bluetooth_msg_receive) {
	_on_bluetooth_msg_receive = on_bluetooth_msg_receive;
}

void SimpleCommunicator_t::OnOrientationReceive(std::function<void(Orientation_t)> on_orientation_receive) {
	_on_orientation_receive = on_orientation_receive;
}

void SimpleCommunicator_t::OnDepthReceive(std::function<void(float)> on_depth_receive) {
	_on_depth_receive = on_depth_receive;
}

void SimpleCommunicator_t::OnRawSensorDataReceive(std::function<void(RawSensorData_t)> on_raw_sensor_data_receive) {
	_on_raw_sensor_data_receive = on_raw_sensor_data_receive;
}

void SimpleCommunicator_t::OnCalibratedSensorDataReceive(std::function<void(CalibratedSensorData_t)> on_calibrated_sensor_data_receive) {
	_on_calibrated_sensor_data_receive = on_calibrated_sensor_data_receive;
}

void SimpleCommunicator_t::OnMotorsStateReceive(std::function<void(MotorsState_t)> on_motors_state_receive) {
	_on_motors_state_receive = on_motors_state_receive;
}

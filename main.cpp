#include "ConnectionProvider.h"
#include "UARTConnectionProviderWindows.h"
#include "SimpleCommunicator.h"

#include <iostream>
#include "Utils.h"

#define _CRT_SECURE_NO_WARNINGS

int main() {
	std::string com_port_name;

	std::cin >> com_port_name;

	ConnectionProvider_t* connection_provider = new UARTConnectionProvider_t(com_port_name, 19200, 1024, 1024);
	SimpleCommunicator_t* communicator = new SimpleCommunicator_t(connection_provider);

	bool orientation = false;

	try {
		connection_provider->Begin();
		communicator->OnRobotRestart([]()
		{
			printf("Arduino was restarted\n");
		});
		communicator->OnPacketsLeak([](int send, int receive)
		{
			printf("Leak: send %d, receive %d\n", send, receive);
		});
		communicator->OnConnectionStateChange([](bool connected)
		{
			if (connected) {
				printf("Connected\n");
			} else {
				printf("Disconnected\n");
			}
		});
		communicator->OnOrientationReceive([&](SimpleCommunicator_t::Orientation_t o)
		{
			float angles[3];
			if (orientation) {
				angles[0] = atan2(2 * o.q2 * o.q3 - 2 * o.q1 * o.q4, 2 * o.q1 * o.q1 + 2 * o.q2 * o.q2 - 1); // psi
				angles[1] = -asin(2 * o.q2 * o.q4 + 2 * o.q1 * o.q3); // theta
				angles[2] = atan2(2 * o.q3 * o.q4 - 2 * o.q1 * o.q2, 2 * o.q1 * o.q1 + 2 * o.q4 * o.q4 - 1); // phi
				printf("Orientation: %f, %f, %f\r", angles[0], angles[1], angles[2]);
			}
		});
		float gyro_x = 0, gyro_y = 0, gyro_z = 0;
		int n = 0;
		communicator->OnRawSensorDataReceive([&](SimpleCommunicator_t::RawSensorData_t raw_sensor_data)
		{
			//printf("Raw sensor data: %d, %d, %d\r", raw_sensor_data.Gx, raw_sensor_data.Gy, raw_sensor_data.Gz);
			gyro_x += raw_sensor_data.Gx;
			gyro_y += raw_sensor_data.Gy;
			gyro_z += raw_sensor_data.Gz;

			n++;

			printf("%f, %f, %f\r", gyro_x / n, gyro_y / n, gyro_z / n);
		});
		int len = 0;
		communicator->OnCalibratedSensorDataReceive([&](SimpleCommunicator_t::CalibratedSensorData_t calibrated_sensor_data)
		{
			printf("Calibrated sensor data: %f, %f, %f  %f\r", calibrated_sensor_data.Gx, calibrated_sensor_data.Gy, calibrated_sensor_data.Gz, calibrated_sensor_data.Depth);
		});
		communicator->OnI2CDevicesReceive([](SimpleCommunicator_t::I2CDevices_t devices)
		{
			printf("PCA1: ");
			printf(devices.PCA1 ? "connected\n" : "disconnected\n");

			printf("PCA2: ");
			printf(devices.PCA2 ? "connected\n" : "disconnected\n");

			printf("ADXL345: ");
			printf(devices.ADXL345 ? "connected\n" : "disconnected\n");

			printf("HMC58X3: ");
			printf(devices.HMC58X3 ? "connected\n" : "disconnected\n");

			printf("ITG3200: ");
			printf(devices.ITG3200 ? "connected\n" : "disconnected\n");

			printf("BMP085: ");
			printf(devices.BMP085 ? "connected\n" : "disconnected\n");

			printf("MS5803: ");
			printf(devices.MS5803 ? "connected\n" : "disconnected\n");
		});
		communicator->OnBluetoothMsgReceive([](std::string msg)
		{
			printf("Bluetooth message: %s\n", msg.c_str());
		});
		communicator->OnPidStateReceive([](SimpleCommunicator_t::PidState_t depth, SimpleCommunicator_t::PidState_t yaw, SimpleCommunicator_t::PidState_t pitch)
		{
			printf("Yaw pid state: input: %f, target: %f, output: %f\r", yaw.In, yaw.Target, yaw.Out);
		});

		communicator->OnStop([](std::string error)
		{
			printf("Error: %s", error.c_str());
		});

		communicator->OnMessageReceive([](unsigned long ping)
		{
			//printf("Ping: %ld\r", ping);
		});

		communicator->OnRemoteProcessorLoad([](unsigned long loop_frequency) 
		{
			printf("Loop frequency: %ld          \r", loop_frequency);
		});
		char c;
		bool flashlight_state = false;
		bool receive_raw_sensor_data = false;
		bool receive_calibrated_sensor_data = false;
		bool read_bluetooth = false;
		bool receive_pid = false;
		bool receive_loop = false;

		float thrust[6] = { 0, 0, 0, 0, 0, 0 };
		float manipulator[2] = { 0, 0 };

		float x = 0, y = 0;

		communicator->SetPitcPid(0.5, 0, 0);
		communicator->SetSendMessageFrequency(50);

		communicator->Begin();
		while (std::cin >> c) {
			switch (c) {
			case 'f': communicator->SetFlashlightState(flashlight_state = !flashlight_state); break;
			case 'r': communicator->SetReceiveRawSensorData(receive_raw_sensor_data = !receive_raw_sensor_data); break;
			case 'c': communicator->SetReceiveCalibratedSensorData(receive_calibrated_sensor_data = !receive_calibrated_sensor_data); break;
			case 's': communicator->SetRescanI2CDevices(); break;
			case 'm': {
				int m_id;
				std::cin >> m_id;
				std::cin >> thrust[m_id];
				communicator->SetMotorsState(thrust[0], thrust[1], thrust[2], thrust[3], thrust[4], thrust[5]);
			} break;
			case 'd': {
				char d;
				std::cin >> d;
				float v;
				std::cin >> v;
				if (d == 'x') {
					x = v;
				} else if (d == 'y') {
					y = v;
				}
				else if (d == 'z') {
					communicator->SetSinkingForce(v);
				}

				communicator->SetMovementForce(x, y);
			} break;
			case 'a': {
				char r;
				std::cin >> r;
				float v;
				std::cin >> v;
				if (r == 'y') {
					communicator->SetYaw(v);
				}
				else if (r == 'p') {
					communicator->SetPitch(v);
				}
				else if (r == 'd') {
					communicator->SetDepth(v);
				}
			} break;
			case 'q': {
				char r;
				std::cin >> r;
				float v;
				std::cin >> v;
				if (r == 'y') {
					communicator->SetYawForce(v);
				}
				else if (r == 'p') {
					communicator->SetPitchForce(v);
				}
			} break;
			case 'g': {
				int c_id;
				std::cin >> c_id;
				float pos;
				std::cin >> pos;
				if (c_id == 1) {
					communicator->SetCamera1LocalPos(pos);
				}
				else if (c_id == 2) {
					communicator->SetCamera1LocalPos(pos);
				}
			} break;
			case 't': {
				int d_id;
				std::cin >> d_id;
				std::cin >> manipulator[d_id];
				communicator->SetManipulatorState(manipulator[0], manipulator[1], manipulator[2], manipulator[3]);
			} break;
			case 'b': {
				communicator->SetReadBluetoothState(read_bluetooth = !read_bluetooth);
			} break;
			case 'o': {
				orientation = !orientation;
			} break;
			case 'p': {
				communicator->SetReceivePidState(receive_pid = !receive_pid);
			} break;
			case 'i': {
				char v;
				unsigned long t;
				std::cin >> v;
				std::cin >> t;
				if (v == 's') {
					communicator->SetSendMessageFrequency(t);
				} else if (v == 'r') {
					communicator->SetRemoteSendMessageFrequency(t);
				}
			} break;
			case 'l': {
				receive_loop = !receive_loop;
			} break;
			case 'y': {
				communicator->SetMotorsPositions(0, 1, 2, 3, 4, 5);
				communicator->SetMotorsMultiplier(0, 1, -1, -0.5, 0.3, 0.9);
			} break;
			}
		}
	} catch(ControllerException_t& e) {
		printf("%s\n", e.error_message.c_str());
	}

	communicator->Stop();
	connection_provider->Stop();

	system("pause");

	delete communicator;
	delete connection_provider;

	return 0;
}

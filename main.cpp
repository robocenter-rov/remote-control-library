#include "ConnectionProvider.h"
#include "UARTConnectionProviderWindows.h"
#include "SimpleCommunicator.h"

#include <iostream>
#include "Utils.h"

#define _CRT_SECURE_NO_WARNINGS

int main() {
	std::string com_port_name;

	std::cin >> com_port_name;

	ConnectionProvider_t* connection_provider = new UARTConnectionProvider_t(com_port_name, 115200, 1024, 1024);
	SimpleCommunicator_t* communicator = new SimpleCommunicator_t(connection_provider);

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
		communicator->OnOrientationReceive([](SimpleCommunicator_t::Orientation_t o)
		{
			float angles[3];
			angles[0] = atan2(2 * o.q2 * o.q3 - 2 * o.q1 * o.q4, 2 * o.q1 * o.q1 + 2 * o.q2 * o.q2 - 1); // psi
			angles[1] = -asin(2 * o.q2 * o.q4 + 2 * o.q1 * o.q3); // theta
			angles[2] = atan2(2 * o.q3 * o.q4 - 2 * o.q1 * o.q2, 2 * o.q1 * o.q1 + 2 * o.q4 * o.q4 - 1); // phi
			//printf("Orientation: %f, %f, %f\r", angles[0], angles[1], angles[2]);
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

		communicator->OnStop([](std::string error)
		{
			printf("Error: %s", error.c_str());
		});
		char c;
		bool flashlight_state = false;
		bool receive_raw_sensor_data = false;
		bool receive_calibrated_sensor_data = false;
		bool read_bluetooth = false;


		float thrust[6] = { 0, 0, 0, 0, 0, 0 };
		float manipulator[2] = { 0, 0 };

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
			case 'g': {
				int c_id;
				std::cin >> c_id;
				float pos;
				std::cin >> pos;
				if (c_id == 1) {
					communicator->SetCamera1Pos(pos);
				}
				else if (c_id == 2) {
					communicator->SetCamera2Pos(pos);
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
			}
		}
	} catch(ControllerException_t& e) {
		printf("%s\n", e.error_message.c_str());
	}

	system("pause");

	delete communicator;
	delete connection_provider;

	return 0;
}

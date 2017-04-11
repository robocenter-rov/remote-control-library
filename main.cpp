#include "ConnectionProvider.h"
#include "UARTConnectionProviderWindows.h"
#include "SimpleCommunicator.h"

#include <iostream>
#include "Utils.h"

#define _CRT_SECURE_NO_WARNINGS

int main() {
	ConnectionProvider_t* connection_provider = new UARTConnectionProvider_t("COM3", 115200, 200, 200);
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
		});/*
		communicator->OnOrientationReceive([](SimpleCommunicator_t::Orientation_t orientation)
		{
			printf("Orientation: %f, %f, %f, %f\n", orientation.q1, orientation.q2, orientation.q3, orientation.q4);
		});*/
		communicator->OnRawSensorDataReceive([](SimpleCommunicator_t::RawSensorData_t raw_sensor_data)
		{
			printf("\rRaw sensor data: %d, %d, %d, Depth: %f", raw_sensor_data.Ax, raw_sensor_data.Ay, raw_sensor_data.Az, raw_sensor_data.Depth);
		});
		int len = 0;
		communicator->OnCalibratedSensorDataReceive([&](SimpleCommunicator_t::CalibratedSensorData_t calibrated_sensor_data)
		{
			printf("\rCalibrated sensor data: %f, %f, %f  %f", calibrated_sensor_data.Mx, calibrated_sensor_data.My, calibrated_sensor_data.Mz, calibrated_sensor_data.Depth);
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
		char c;
		bool flashlight_state = false;
		bool receive_raw_sensor_data = false;
		bool receive_calibrated_sensor_data = false;

		communicator->Begin();
		while (std::cin >> c) {
			switch (c) {
			case 'f': communicator->SetFlashlightState(flashlight_state = !flashlight_state); break;
			case 'r': communicator->SetReceiveRawSensorData(receive_raw_sensor_data = !receive_raw_sensor_data); break;
			case 'c': communicator->SetReceiveCalibratedSensorData(receive_calibrated_sensor_data = !receive_calibrated_sensor_data); break;
			case 's': communicator->SetRescanI2CDevices(); break;
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

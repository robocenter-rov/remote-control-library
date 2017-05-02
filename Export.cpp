#include "SimpleCommunicatorWrapper.h"
#include "UARTConnectionProviderWindows.h"
SimpleCommunicatorWrapper_t* simple_communicator_wrapper;

extern "C" {
	__declspec(dllexport) void Init(char* com_port_name, int baud_rate) {
		auto connection_provider = new UARTConnectionProvider_t(com_port_name, baud_rate, 1 << 20, 1 << 20);
		simple_communicator_wrapper = new SimpleCommunicatorWrapper_t(new SimpleCommunicator_t(connection_provider), connection_provider);
	}

	__declspec(dllexport) void Begin() {
		simple_communicator_wrapper->Begin();
	}

	__declspec(dllexport) void Stop() {
		simple_communicator_wrapper->Stop();
	}

	__declspec(dllexport) void SetMotorsDirection(bool m1, bool m2, bool m3, bool m4, bool m5, bool m6) {
		simple_communicator_wrapper->SetMotorsDirection(m1, m2, m3, m4, m5, m6);
	}

	__declspec(dllexport) void SetManipulatorState(float arm_pos, float hand_pos, float m1, float m2) {
		simple_communicator_wrapper->SetManipulatorState(arm_pos, hand_pos, m1, m2);
	}

	__declspec(dllexport) void SetCamera1Pos(float camera1) {
		simple_communicator_wrapper->SetCamera1Pos(camera1);
	}

	__declspec(dllexport) void SetCamera2Pos(float camera2) {
		simple_communicator_wrapper->SetCamera2Pos(camera2);
	}

	__declspec(dllexport) void SetMotorsState(float m1, float m2, float m3, float m4, float m5, float m6) {
		simple_communicator_wrapper->SetMotorsState(m1, m2, m3, m4, m5, m6);
	}

	__declspec(dllexport) void SetMovementForce(float x, float y) {
		simple_communicator_wrapper->SetMovementForce(x, y);
	}

	__declspec(dllexport) void SetSinkingForce(float z) {
		simple_communicator_wrapper->SetSinkingForce(z);
	}

	__declspec(dllexport) void SetDepth(float depth) {
		simple_communicator_wrapper->SetDepth(depth);
	}

	__declspec(dllexport) void SetPitchForce(float pitch_force) {
		simple_communicator_wrapper->SetPitchForce(pitch_force);
	}

	__declspec(dllexport) void SetPitch(float pitch) {
		simple_communicator_wrapper->SetPitch(pitch);
	}

	__declspec(dllexport) void SetYawForce(float yaw_force) {
		simple_communicator_wrapper->SetYawForce(yaw_force);
	}

	__declspec(dllexport) void SetYaw(float yaw) {
		simple_communicator_wrapper->SetYaw(yaw);
	}

	__declspec(dllexport) void ScanI2C() {
		simple_communicator_wrapper->ScanI2C();
	}

	__declspec(dllexport) void SetFlashlightState(bool state) {
		simple_communicator_wrapper->SetFlashlightState(state);
	}

	__declspec(dllexport) void SetReadBluetoothState(int read) {
		simple_communicator_wrapper->SetReadBluetoothState(read);
	}

	__declspec(dllexport) void SetDepthPid(float p, float i, float d) {
		simple_communicator_wrapper->SetDepthPid(p, i, d);
	}

	__declspec(dllexport) void SetPitcPid(float p, float i, float d) {
		simple_communicator_wrapper->SetPitcPid(p, i, d);
	}

	__declspec(dllexport) void SetYawPid(float p, float i, float d) {
		simple_communicator_wrapper->SetYawPid(p, i, d);
	}

	__declspec(dllexport) void SetReceiveRawSensorData(bool receive) {
		simple_communicator_wrapper->SetReceiveRawSensorData(receive);
	}

	__declspec(dllexport) void SetReceiveCalibratedSensorData(bool receive) {
		simple_communicator_wrapper->SetReceiveCalibratedSensorData(receive);
	}

	__declspec(dllexport) void SetRescanI2CDevices() {
		simple_communicator_wrapper->SetRescanI2CDevices();
	}

	__declspec(dllexport) void ConnectionState(bool& state) {
		simple_communicator_wrapper->ConnectionState(state);
	}

	__declspec(dllexport) void PacketsLeak(int& receive, int& send) {
		simple_communicator_wrapper->PacketsLeak(receive, send);
	}

	__declspec(dllexport) void RobotRestarted(int& restarted) {
		simple_communicator_wrapper->RobotRestarted(restarted);
	}

	__declspec(dllexport) void State(bool& flashlight_state) {
		simple_communicator_wrapper->State(flashlight_state);
	}

	__declspec(dllexport) void I2CDevices(int& pca1, int& pca2, int& hmc58x3, int& itg3200, int& adxl345, int& bmp085, int& ms5803, int& rescanned) {
		simple_communicator_wrapper->I2CDevices(pca1, pca2, hmc58x3, itg3200, adxl345, bmp085, ms5803, rescanned);
	}

	__declspec(dllexport) void BluetoothMessage(char* msg) {
		simple_communicator_wrapper->BluetoothMessage(msg);
	}

	__declspec(dllexport) void Orientation(float& q0, float& q1, float& q2, float& q3) {
		simple_communicator_wrapper->Orientation(q0, q1, q2, q3);
	}

	__declspec(dllexport) void Depth(float& depth) {
		simple_communicator_wrapper->Depth(depth);
	}

	__declspec(dllexport) void RawSensorData(int& ax, int& ay, int& az, int& gx, int& gy, int& gz, int& mx, int& my, int& mz, float& depth) {
		RawSensorData(ax, ay, az, gx, gy, gz, mx, my, mz, depth);
	}

	__declspec(dllexport) void CalibratedSensorData(float& ax, float& ay, float& az, float& gx, float& gy, float& gz, float& mx, float& my, float& mz, float& depth) {
		CalibratedSensorData(ax, ay, az, gx, gy, gz, mx, my, mz, depth);
	}

	__declspec(dllexport) void MotorsState(float& m1, float& m2, float& m3, float& m4, float& m5, float& m6) {
		simple_communicator_wrapper->MotorsState(m1, m2, m3, m4, m5, m6);
	}

	__declspec(dllexport) int Error(char* error_msg, int buff_size) {
		return simple_communicator_wrapper->Error(error_msg, buff_size);
	}
}
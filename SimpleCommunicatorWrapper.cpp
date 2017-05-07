#include "SimpleCommunicatorWrapper.h"

SimpleCommunicatorWrapper_t::SimpleCommunicatorWrapper_t(SimpleCommunicator_t* simple_communicator, ConnectionProvider_t* connection_provider) {
	_simple_communicator = simple_communicator;
	_connection_provider = connection_provider;
	_connection_state = false;
	_robot_restarted = false;
	_i2c_rescanned = false;
	_depth = 0;

	_simple_communicator->OnConnectionStateChange([&](bool state)
	{
		_connection_state = state;
	});

	_simple_communicator->OnCalibratedSensorDataReceive([&](SimpleCommunicator_t::CalibratedSensorData_t sd)
	{
		_calibrated_sensor_data = sd;
	});

	_simple_communicator->OnBluetoothMsgReceive([&](std::string str)
	{
		_bluetooth_msg = str;
	});

	_simple_communicator->OnDepthReceive([&](float depth)
	{
		_depth = depth;
	});

	_simple_communicator->OnI2CDevicesReceive([&](SimpleCommunicator_t::I2CDevices_t d)
	{
		_i2c_rescanned = true;
		_scanned_devices = d;
	});

	_simple_communicator->OnOrientationReceive([&](SimpleCommunicator_t::Orientation_t o)
	{
		_orientation = o;
	});

	_simple_communicator->OnStateChange([&](SimpleCommunicator_t::State_t state)
	{
		_state = state;
	});
}

void SimpleCommunicatorWrapper_t::Begin() {
	try {
		_connection_provider->Begin();
		_simple_communicator->Begin();
	} catch(ControllerException_t e) {
		_error = e.error_message;
	}
}

void SimpleCommunicatorWrapper_t::Stop() const {
	_simple_communicator->Stop();
	_connection_provider->Stop();
}

void SimpleCommunicatorWrapper_t::SetMotorsMultiplier(float m1, float m2, float m3, float m4, float m5, float m6) const {
	_simple_communicator->SetMotorsMultiplier(m1, m2, m3, m4, m5, m6);
}

void SimpleCommunicatorWrapper_t::SetMotorsPositions(int m1, int m2, int m3, int m4, int m5, int m6) const {
	_simple_communicator->SetMotorsPositions(m1, m2, m3, m4, m5, m6);
}

void SimpleCommunicatorWrapper_t::SetManipulatorState(float arm_pos, float hand_pos, float m1, float m2) const {
	_simple_communicator->SetManipulatorState(arm_pos, hand_pos, m1, m2);
}

void SimpleCommunicatorWrapper_t::SetCamera1Pos(float camera1) const {
	_simple_communicator->SetCamera1Pos(camera1);
}

void SimpleCommunicatorWrapper_t::SetCamera2Pos(float camera2) const {
	_simple_communicator->SetCamera2Pos(camera2);
}

void SimpleCommunicatorWrapper_t::SetMotorsState(float m1, float m2, float m3, float m4, float m5, float m6) const {
	_simple_communicator->SetMotorsState(m1, m2, m3, m4, m5, m6);
}

void SimpleCommunicatorWrapper_t::SetMovementForce(float x, float y) const {
	_simple_communicator->SetMovementForce(x, y);
}

void SimpleCommunicatorWrapper_t::SetSinkingForce(float z) const {
	_simple_communicator->SetSinkingForce(z);
}

void SimpleCommunicatorWrapper_t::SetDepth(float depth) const {
	_simple_communicator->SetDepth(depth);
}

void SimpleCommunicatorWrapper_t::SetPitchForce(float pitch_force) const {
	_simple_communicator->SetPitchForce(pitch_force);
}

void SimpleCommunicatorWrapper_t::SetPitch(float pitch) const {
	_simple_communicator->SetPitch(pitch);
}

void SimpleCommunicatorWrapper_t::SetYawForce(float yaw_force) const {
	_simple_communicator->SetYawForce(yaw_force);
}

void SimpleCommunicatorWrapper_t::SetYaw(float yaw) const {
	_simple_communicator->SetYaw(yaw);
}

void SimpleCommunicatorWrapper_t::ScanI2C() const {
	_simple_communicator->ScanI2C();
}

void SimpleCommunicatorWrapper_t::SetFlashlightState(bool state) const {
	_simple_communicator->SetFlashlightState(state);
}

void SimpleCommunicatorWrapper_t::SetReadBluetoothState(bool read) const {
	_simple_communicator->SetReadBluetoothState(read);
}

void SimpleCommunicatorWrapper_t::SetDepthPid(float p, float i, float d) const {
	_simple_communicator->SetDepthPid(p, i, d);
}

void SimpleCommunicatorWrapper_t::SetPitcPid(float p, float i, float d) const {
	_simple_communicator->SetPitcPid(p, i, d);
}

void SimpleCommunicatorWrapper_t::SetYawPid(float p, float i, float d) const {
	_simple_communicator->SetYawPid(p, i, d);
}

void SimpleCommunicatorWrapper_t::SetReceiveRawSensorData(bool receive) const {
	_simple_communicator->SetReceiveRawSensorData(receive);
}

void SimpleCommunicatorWrapper_t::SetReceiveCalibratedSensorData(bool receive) const {
	_simple_communicator->SetReceiveCalibratedSensorData(receive);
}

void SimpleCommunicatorWrapper_t::SetRescanI2CDevices() const {
	_simple_communicator->SetRescanI2CDevices();
}

void SimpleCommunicatorWrapper_t::ConnectionState(bool& state) const {
	state = _connection_state;
}

void SimpleCommunicatorWrapper_t::PacketsLeak(int& receive, int& send) {
	receive = _packets_leaked.receive;
	send = _packets_leaked.send;
	_packets_leaked.receive = _packets_leaked.send = 0;
}

void SimpleCommunicatorWrapper_t::RobotRestarted(int& restarted) {
	restarted = _robot_restarted;
	_robot_restarted = false;
}

void SimpleCommunicatorWrapper_t::State(bool& flashlight_state) const {
	flashlight_state = _state.FlashlightState;
}

void SimpleCommunicatorWrapper_t::I2CDevices(int& pca1, int& pca2, int& hmc58x3, int& itg3200, int& adxl345, int& bmp085, int& ms5803, int& rescanned) {
	pca1 = _scanned_devices.PCA1;
	pca2 = _scanned_devices.PCA2;
	hmc58x3 = _scanned_devices.HMC58X3;
	itg3200 = _scanned_devices.ITG3200;
	adxl345 = _scanned_devices.ADXL345;
	bmp085 = _scanned_devices.BMP085;
	ms5803 = _scanned_devices.MS5803;

	rescanned = _i2c_rescanned;
	_i2c_rescanned = false;
}

void SimpleCommunicatorWrapper_t::BluetoothMessage(char* msg) const {
	strcpy_s(msg, 8, _bluetooth_msg.c_str());
}

void SimpleCommunicatorWrapper_t::Orientation(float& q0, float& q1, float& q2, float& q3) const {
	q0 = _orientation.q1;
	q1 = _orientation.q2;
	q2 = _orientation.q3;
	q3 = _orientation.q4;
}

void SimpleCommunicatorWrapper_t::Depth(float& depth) const {
	depth = _depth;
}

void SimpleCommunicatorWrapper_t::RawSensorData(int& ax, int& ay, int& az, int& gx, int& gy, int& gz, int& mx, int& my, int& mz, float& depth) const {
	ax = _raw_sensor_data.Ax;
	ay = _raw_sensor_data.Ay;
	az = _raw_sensor_data.Az;

	gx = _raw_sensor_data.Gx;
	gy = _raw_sensor_data.Gy;
	gz = _raw_sensor_data.Gz;

	mx = _raw_sensor_data.Mx;
	my = _raw_sensor_data.My;
	mz = _raw_sensor_data.Mz;

	depth = _raw_sensor_data.Depth;
}

void SimpleCommunicatorWrapper_t::CalibratedSensorData(float& ax, float& ay, float& az, float& gx, float& gy, float& gz, float& mx, float& my, float& mz, float& depth) const {
	ax = _calibrated_sensor_data.Ax;
	ay = _calibrated_sensor_data.Ay;
	az = _calibrated_sensor_data.Az;

	gx = _calibrated_sensor_data.Gx;
	gy = _calibrated_sensor_data.Gy;
	gz = _calibrated_sensor_data.Gz;

	mx = _calibrated_sensor_data.Mx;
	my = _calibrated_sensor_data.My;
	mz = _calibrated_sensor_data.Mz;

	depth = _calibrated_sensor_data.Depth;
}

void SimpleCommunicatorWrapper_t::MotorsState(float& m1, float& m2, float& m3, float& m4, float& m5, float& m6) const {
	m1 = _motors_state.M1Force;
	m2 = _motors_state.M2Force;
	m3 = _motors_state.M3Force;
	m4 = _motors_state.M4Force;
	m5 = _motors_state.M5Force;
	m6 = _motors_state.M6Force;
}

int SimpleCommunicatorWrapper_t::Error(char* error_msg, int buff_size) {
	strcpy_s(error_msg, buff_size, _error.c_str());
	return !_error.empty();
}

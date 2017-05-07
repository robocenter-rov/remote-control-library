#pragma once
#include <exception>
#include "Exception.h"
#include "Utils.h"

class ConnectionProviderException_t : public ControllerException_t {
public:
	using ControllerException_t::ControllerException_t;
};

class ConnectionProvider_t {
protected:
	uint8_t* _receive_buffer;
	size_t _receive_buffer_size;
public:
	ConnectionProvider_t(size_t buffer_size);

	virtual ~ConnectionProvider_t();
	virtual void Begin() {}
	virtual void Stop() {}
	virtual ConnectionProvider_t* WriteBool(bool val);
	ConnectionProvider_t* WriteInt8(int8_t val);
	virtual ConnectionProvider_t* WriteUInt8(uint8_t val) = 0;
	ConnectionProvider_t* WriteInt16(int16_t val);
	ConnectionProvider_t* WriteUInt16(uint16_t val);
	ConnectionProvider_t* WriteUInt32(uint32_t val);
	ConnectionProvider_t* WriteFloat(float val);
	virtual ConnectionProvider_t* Write(const void* buffer, size_t size) = 0;

	template<typename T>
	ConnectionProvider_t* WriteVar(T var);

	template<typename T>
	ConnectionProvider_t* WriteFloatAs(float val, float in_min, float in_max);

	virtual ConnectionProvider_t* BeginPacket() = 0;
	virtual ConnectionProvider_t* EndPacket() = 0;
	virtual ConnectionProvider_t* Send(const void* buffer, size_t size);
	virtual int Receive() = 0;
	virtual const void* ReceiveBuffer();
	virtual bool Connected() = 0;
};

template <typename T>
ConnectionProvider_t* ConnectionProvider_t::WriteVar(T var) {
	//printf("WriteVar, size: %d\n", sizeof(T));
	return Write(&var, sizeof(T));
}

template <typename T>
ConnectionProvider_t* ConnectionProvider_t::WriteFloatAs(float val, float in_min, float in_max) {
	val = constrain(val, in_min, in_max);

	T out_min = std::numeric_limits<T>::min();
	T out_max = std::numeric_limits<T>::max();

	T res = (val - in_min) / (in_max - in_min) * (out_max - out_min) + out_min;

	return WriteVar(res);
}

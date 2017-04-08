#pragma once
#include <exception>
#include "Exception.h"

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

	virtual ConnectionProvider_t* BeginPacket() = 0;
	virtual ConnectionProvider_t* EndPacket() = 0;
	virtual ConnectionProvider_t* Send(const void* buffer, size_t size);
	virtual int Receive() = 0;
	virtual const void* ReceiveBuffer();
	virtual bool Connected() = 0;
};

template <typename T>
ConnectionProvider_t* ConnectionProvider_t::WriteVar(T var) {
	return Write(&var, sizeof(T));
}

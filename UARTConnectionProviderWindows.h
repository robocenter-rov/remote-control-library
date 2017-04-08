#pragma once
#include "ConnectionProvider.h"
#include <string>
#include <windows.h>
#include "CircularBuffer.h"
#include "RingBuffer.h"

class CantOpenPortException_t : public ConnectionProviderException_t {
	DWORD error_code;
public:
	std::string port_name;
	CantOpenPortException_t(std::string port_name, DWORD error_code);
};

class CantClosePortException_t : public ConnectionProviderException_t {
	DWORD error_code;
public:
	std::string port_name;
	CantClosePortException_t(std::string port_name, DWORD error_code);
};

class PortClosedException_t : public ConnectionProviderException_t {
public:
	std::string port_name;
	PortClosedException_t(std::string port_name);
};

class SendBufferLimitExceeded_t : public ConnectionProviderException_t {
public:
	SendBufferLimitExceeded_t();
};

class ReceiveBufferLimitExceeded_t : public ConnectionProviderException_t {
public:
	ReceiveBufferLimitExceeded_t();
};

class UARTConnectionProvider_t : public ConnectionProvider_t {
protected:
	HANDLE _h_com_port;
	int _baud_rate;
	std::string _com_port_name;

	uint8_t* _send_buffer;
	size_t _send_buffer_size;
	size_t _send_head;
	uint32_t _current_hash;

	RingBuffer_t _raw_receive_buffer;
	size_t _received = 0;
	bool _in_esc;

	void WriteToSendBuffer(uint8_t val);
public:
	UARTConnectionProvider_t(std::string com_port_name, int baud_rate, size_t send_buffer_size, size_t receive_buffer_size);

	~UARTConnectionProvider_t() override;
	void Begin() override;
	void Stop() override;
	ConnectionProvider_t* Write(const void* buffer, size_t size) override;
	ConnectionProvider_t* WriteUInt8(uint8_t val) override;
	ConnectionProvider_t* BeginPacket() override;
	ConnectionProvider_t* EndPacket() override;
	int Receive() override;
	bool Connected() override;
};

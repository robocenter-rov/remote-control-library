#include "UARTConnectionProviderWindows.h"
#include "Utils.h"
#include <chrono>

#define END             0300    /* indicates end of packet */
#define ESC             0333    /* indicates byte stuffing */
#define ESC_END         0334    /* ESC ESC_END means END data byte */
#define ESC_ESC         0335    /* ESC ESC_ESC means ESC data byte */

std::string GetErrorMessage(const char* msg, DWORD error_code) {
	std::string  error_message;
	char* msg_buffer;
	if (!FormatMessage(
		FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_IGNORE_INSERTS,
		nullptr,
		error_code,
		LANG_ENGLISH,
		(LPTSTR)&msg_buffer,
		0,
		nullptr
	)) {
		error_message = msg;
	}
	else {
		error_message = msg + std::string(" Error: ") + msg_buffer;
		LocalFree(msg_buffer);
	}
	return error_message;
}

CantOpenPortException_t::CantOpenPortException_t(std::string port_name, DWORD error_code):
	ConnectionProviderException_t(GetErrorMessage("Can't open port.", error_code)), error_code(error_code), port_name(port_name)
{}

CantClosePortException_t::CantClosePortException_t(std::string port_name, DWORD error_code):
	ConnectionProviderException_t(GetErrorMessage("Can't close port.", error_code)), error_code(error_code), port_name(port_name) {}

PortClosedException_t::PortClosedException_t(std::string port_name):
	ConnectionProviderException_t("Port is closed"), port_name(port_name) {}

SendBufferLimitExceeded_t::SendBufferLimitExceeded_t() : ConnectionProviderException_t("Send buffer limit exceeded") {}
ReceiveBufferLimitExceeded_t::ReceiveBufferLimitExceeded_t() : ConnectionProviderException_t("Receive buffer limit exceeded") {}

void UARTConnectionProvider_t::WriteToSendBuffer(uint8_t val) {
	if (_send_head <= _send_buffer_size) {
		_send_buffer[_send_head++] = val;
		//printf("Write: %X\n", (uint32_t)val);
	} else {
		throw SendBufferLimitExceeded_t();
	}
}

UARTConnectionProvider_t::UARTConnectionProvider_t(std::string com_port_name, int baud_rate, size_t send_buffer_size, size_t receive_buffer_size)
	: ConnectionProvider_t(receive_buffer_size), 
	_h_com_port(nullptr), 
	_baud_rate(baud_rate), 
	_com_port_name(com_port_name), 
	_send_buffer_size(send_buffer_size), 
	_send_head(0), 
	_current_hash(0), 
	_raw_receive_buffer(receive_buffer_size), 
	_in_esc(false) 
{
	_send_buffer = static_cast<uint8_t*>(malloc(_send_buffer_size));
}

UARTConnectionProvider_t::~UARTConnectionProvider_t() {
	Stop();
	if (_send_buffer) {
		free(_send_buffer);
	}
}

void UARTConnectionProvider_t::Begin() {
    if (_h_com_port) {
        return;
    }

#ifdef UNICODE
    wchar_t buffer[256];
    swprintf(buffer, L"\\\\.\\%s", _com_port_name.c_str());
	_h_com_port = CreateFile(
        static_cast<LPCWSTR>(buffer),
#else
    char buffer[256];
    sprintf_s(buffer, "\\\\.\\%s", _com_port_name.c_str());
    _h_com_port = CreateFile(
        static_cast<LPCSTR>(buffer),
#endif
		GENERIC_READ | GENERIC_WRITE,
		0,
		nullptr,
		OPEN_EXISTING,
		FILE_FLAG_OVERLAPPED,
		nullptr
	);
	if (_h_com_port == INVALID_HANDLE_VALUE) {
		throw CantOpenPortException_t(_com_port_name, GetLastError());
	}
	DCB serialParams;
	serialParams.DCBlength = sizeof(serialParams);

	if (!GetCommState(_h_com_port, &serialParams)) {
		throw CantOpenPortException_t(_com_port_name, GetLastError());
	}
	serialParams.BaudRate = _baud_rate;
	serialParams.fBinary = 1;
	serialParams.fParity = 0;
	serialParams.fOutxCtsFlow = 0;
	serialParams.fOutxDsrFlow = 0;
	serialParams.fDtrControl = 1;
	serialParams.fDsrSensitivity = 0;
	serialParams.fTXContinueOnXoff = 0;
	serialParams.fOutX = 0;
	serialParams.fInX = 0;
	serialParams.fErrorChar = 0;
	serialParams.fNull = 0;
	serialParams.fRtsControl = 1;
	serialParams.fAbortOnError = 0;
	serialParams.fDummy2 = 0;
	serialParams.wReserved = 0;
	serialParams.XonLim = 2048;
	serialParams.XoffLim = 512;
	serialParams.ByteSize = 8;
	serialParams.Parity = 0;
	serialParams.StopBits = 0;
	serialParams.XonChar = 17;
	serialParams.XoffChar = 19;
	serialParams.ErrorChar = 0;
	serialParams.EofChar = 26;
	serialParams.EvtChar = 26;
	if (!SetCommState(_h_com_port, &serialParams)) {
		throw CantOpenPortException_t(_com_port_name, GetLastError());
	}

	COMMTIMEOUTS timeouts;
	if (!GetCommTimeouts(_h_com_port, &timeouts)) {
		throw CantOpenPortException_t(_com_port_name, GetLastError());
	}

	timeouts.ReadIntervalTimeout = 10;
	timeouts.ReadTotalTimeoutMultiplier = 3;
	timeouts.ReadTotalTimeoutConstant = 10;
	timeouts.WriteTotalTimeoutMultiplier = 0;
	timeouts.WriteTotalTimeoutConstant = 0;

	if (!SetCommTimeouts(_h_com_port, &timeouts)) {
		throw CantOpenPortException_t(_com_port_name, GetLastError());
	}
	
	if (!PurgeComm(_h_com_port, PURGE_RXCLEAR | PURGE_TXCLEAR)) {
		throw CantOpenPortException_t(_com_port_name, GetLastError());
	}
}

void UARTConnectionProvider_t::Stop() {
	if (_h_com_port) {
		if (!CloseHandle(_h_com_port)) {
			throw CantClosePortException_t(_com_port_name, GetLastError());
		}
	}
	_h_com_port = nullptr;
}

ConnectionProvider_t* UARTConnectionProvider_t::Write(const void* buffer, size_t size) {
	if (_send_buffer_size < _send_head + size) {
		_send_head = 0;
		throw SendBufferLimitExceeded_t();
	}
	const uint8_t* curr_byte = static_cast<const uint8_t*>(buffer);
	int receivebytes = 0;
	while (size--) {
		UARTConnectionProvider_t::WriteUInt8(*curr_byte);
		curr_byte++;
	}
	return this;
}

ConnectionProvider_t* UARTConnectionProvider_t::WriteUInt8(uint8_t val) {
	switch (val) {
	case END:
		WriteToSendBuffer(ESC);
		WriteToSendBuffer(ESC_END);
		break;
	case ESC:
		WriteToSendBuffer(ESC);
		WriteToSendBuffer(ESC_ESC);
		break;
	default:
		WriteToSendBuffer(val);
	}
	_current_hash = HashLy(val, _current_hash);
	return this;
}

ConnectionProvider_t* UARTConnectionProvider_t::BeginPacket() {
	_send_head = 0;
	_current_hash = 0;
	return this;
}

ConnectionProvider_t* UARTConnectionProvider_t::EndPacket() {
	uint32_t hash = _current_hash;
	//printf("Write hash\n");
	Write(&hash, sizeof(uint32_t));
	//printf("Write END\n");
	WriteToSendBuffer(END);
	DWORD written_bytes;

	auto t = std::chrono::system_clock::now();

	OVERLAPPED osWrite = { 0 };
	DWORD dwWritten;
	BOOL fRes;

	// Create this writes OVERLAPPED structure hEvent.
	osWrite.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
	if (osWrite.hEvent == NULL)
		// Error creating overlapped event handle.
		return FALSE;

	// Issue write.
	if (!WriteFile(_h_com_port, _send_buffer, _send_head, &written_bytes, &osWrite)) {
		if (GetLastError() != ERROR_IO_PENDING || !GetOverlappedResult(_h_com_port, &osWrite, &written_bytes, TRUE)) {
			throw PortClosedException_t(_com_port_name);
		}
	}

	CloseHandle(osWrite.hEvent);

	//printf("%lld\n", std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - t).count());

	_send_head = 0;
	return this;
}

int UARTConnectionProvider_t::Receive() {
	if (!_raw_receive_buffer.AvailableRead()) {
		const size_t temp_recv_buff_size = 128;
		uint8_t buffer[temp_recv_buff_size];
		DWORD readed_bytes = 0;

		OVERLAPPED osReader = { 0 };

		// Create the overlapped event. Must be closed before exiting
		// to avoid a handle leak.
		osReader.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

		if (osReader.hEvent == NULL) {
			throw PortClosedException_t(_com_port_name);
		}

		if (!ReadFile(_h_com_port, buffer, temp_recv_buff_size, &readed_bytes, &osReader)) {
			if (GetLastError() != ERROR_IO_PENDING || !GetOverlappedResult(_h_com_port, &osReader, &readed_bytes, TRUE)) {
				throw PortClosedException_t(_com_port_name);
			} 
		}

		CloseHandle(osReader.hEvent);

		try {
			_raw_receive_buffer.Write(buffer, readed_bytes);
		} catch (OutOfRangeException_t) {
			_raw_receive_buffer.ClearWrite();
		}
	}

	int c;
	while((c = _raw_receive_buffer.GetChar()) >= 0) {
		//printf("Received: %X\n", (uint32_t)c);

		if (c == ESC) {
			_in_esc = true;
			continue;
		}

		if (_in_esc) {
			switch (c) {
				case ESC_END:
					c = END;
					break;
				case ESC_ESC:
					c = ESC;
					break;
				default: break;
			}
			_in_esc = false;
		} else if (c == END) {
			if (_received < 5) {
				_received = 0;
				return 0;
			}

			uint32_t* hash_ptr = reinterpret_cast<uint32_t*>(_receive_buffer + _received - sizeof(uint32_t));
			uint32_t msg_hash = 0;

			for (int i = 0; i < _received - sizeof(uint32_t); i++) {
				msg_hash = HashLy(_receive_buffer[i], msg_hash);
			}

			size_t t = _received - sizeof(uint32_t);
			_received = 0;

			return msg_hash == *hash_ptr ? t : 0;
		}
		if (_received < _receive_buffer_size) {
			_receive_buffer[_received++] = c;
		} else {
			throw ReceiveBufferLimitExceeded_t();
		}
	}

	return 0;
}

bool UARTConnectionProvider_t::Connected() {
	return _h_com_port;
}

#include "ConnectionProvider.h"
#include "Utils.h"

ConnectionProvider_t::ConnectionProvider_t(char* buffer, size_t buffer_size): _buffer(buffer),
                                                                              _buffer_size(buffer_size) {}

ConnectionProvider_t* ConnectionProvider_t::Write(bool val) {
	return Write(static_cast<char>(val));
}

ConnectionProvider_t* ConnectionProvider_t::Write(int val) {
	char buffer[sizeof(uint16_t)];
	set_le_uint16(buffer, val);
	return Write(buffer, sizeof buffer);
}

ConnectionProvider_t* ConnectionProvider_t::Write(unsigned int val) {
	return Write(static_cast<int>(val));
}

ConnectionProvider_t* ConnectionProvider_t::Write(float val) {
	char buffer[sizeof(uint16_t)];
	set_le_uint16(buffer, float_to_int(val));
	return Write(buffer, sizeof buffer);
}

ConnectionProvider_t* ConnectionProvider_t::Send(char* buffer, int count) {
	BeginPacket();
	Write(buffer, count);
	EndPacket();
	return this;
}

void ConnectionProvider_t::DataReader_t::CheckBuffSize(size_t read_size) const {
	if (read_size > _buffer_size - _readed) {
		throw too_short_buffer_exception_t();
	}
}

ConnectionProvider_t::DataReader_t::DataReader_t(const char* buffer, size_t buffer_size): _buffer(buffer),
																						  _buffer_size(buffer_size),
																					      _readed(0) {}

char ConnectionProvider_t::DataReader_t::GetByte() {
	CheckBuffSize(1);
	return *(_buffer + _readed++);
}

bool ConnectionProvider_t::DataReader_t::GetBool() {
	return GetByte();
}

int ConnectionProvider_t::DataReader_t::GetInt() {
	CheckBuffSize(sizeof(uint16_t));
	auto res = get_be_uint16(_buffer + _readed);
	_readed += sizeof(uint16_t);
	return res;
}

unsigned ConnectionProvider_t::DataReader_t::GetUInt() {
	return GetInt();
}

float ConnectionProvider_t::DataReader_t::GetFloat() {
	return int_to_float(GetInt());
}

const char* ConnectionProvider_t::DataReader_t::GetBytes(size_t count) {
	CheckBuffSize(count);
	auto res = _buffer + _readed;
	_readed += count;
	return res;
}

int ConnectionProvider_t::DataReader_t::Readed() const {
	return _readed;
}

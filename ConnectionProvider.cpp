#include "ConnectionProvider.h"
#include "Utils.h"

ConnectionProvider_t::ConnectionProvider_t(size_t receive_buffer_size): _receive_buffer_size(receive_buffer_size) {
	_receive_buffer = static_cast<uint8_t*>(malloc(receive_buffer_size));
}

ConnectionProvider_t::~ConnectionProvider_t() {
	free(_receive_buffer);
}

ConnectionProvider_t* ConnectionProvider_t::WriteBool(bool val) {
	return WriteUInt8(static_cast<uint8_t>(val));
}

ConnectionProvider_t* ConnectionProvider_t::WriteInt8(int8_t val) {
	return WriteUInt8(static_cast<uint8_t>(val));
}

ConnectionProvider_t* ConnectionProvider_t::WriteUInt16(uint16_t val) {
	auto t = swap_endian(val);
	return WriteVar(t);
}

ConnectionProvider_t* ConnectionProvider_t::WriteUInt32(uint32_t val) {
	auto t = swap_endian(val);
	return WriteVar(t);
}

ConnectionProvider_t* ConnectionProvider_t::WriteFloat(float val) {
	return WriteVar(val);
}

ConnectionProvider_t* ConnectionProvider_t::Send(const void* buffer, size_t size) {
	BeginPacket();
	Write(buffer, size);
	EndPacket();
	return this;
}

const void* ConnectionProvider_t::ReceiveBuffer() {
	return _receive_buffer;
}

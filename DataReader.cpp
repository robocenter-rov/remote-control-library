#include "DataReader.h"
#include "Utils.h"

void DataReader_t::CheckBuffSize(size_t read_size) const {
	if (read_size > _buffer_size - _readed) {
		throw too_short_buffer_exception_t();
	}
}

DataReader_t::DataReader_t(const void* buffer, size_t buffer_size) : _buffer(static_cast<const uint8_t*>(buffer)),
_buffer_size(buffer_size),
_readed(0) {}

uint8_t DataReader_t::GetUInt8() {
	CheckBuffSize(1);
	return *(_buffer + _readed++);
}

bool DataReader_t::GetBool() {
	return GetUInt8();
}

int16_t DataReader_t::GetInt16() {
	return swap_endian(GetVar<uint16_t>());
}

uint16_t DataReader_t::GetUInt16() {
	return GetInt16();
}

int32_t DataReader_t::GetInt32() {
	return swap_endian(GetVar<uint32_t>());
}

uint32_t DataReader_t::GetUInt32() {
	return GetInt32();
}

float DataReader_t::GetFloat() {
	return int_to_float(GetInt16());
}

const uint8_t* DataReader_t::GetBytes(size_t count) {
	CheckBuffSize(count);
	auto res = _buffer + _readed;
	_readed += count;
	return res;
}

int DataReader_t::Readed() const {
	return _readed;
}

int DataReader_t::Available() const {
	return _buffer_size - _readed;
}

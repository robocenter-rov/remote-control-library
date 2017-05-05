#include "RingBuffer.h"
#include <cstdlib>
#include <cstring>

RingBuffer_t::RingBuffer_t(size_t buffer_size) {
	_buffer = static_cast<char*>(malloc(buffer_size));
	_buffer_size = buffer_size;
	_write_head = _read_head = 0;
}

RingBuffer_t::~RingBuffer_t() {
	free(_buffer);
}

int RingBuffer_t::GetChar() {
	if (_read_head == _write_head) {
		return -1;
	}
	return (unsigned char)_buffer[_read_head++ % _buffer_size];
}

void RingBuffer_t::Write(void* buffer, size_t size) {
	if (_buffer_size + _write_head - _read_head < size) {
		throw OutOfRangeException_t();
	}
	size_t i = _write_head % _buffer_size;

	if (size > _buffer_size - i) {
		memcpy(_buffer + i, buffer, _buffer_size - i);
		memcpy(_buffer, (char*)buffer + _buffer_size - i, size - _buffer_size + i);
	} else {
		memcpy(_buffer + i, buffer, size);
	}

	_write_head += size;
}

void RingBuffer_t::ClearWrite() {
	_write_head = _read_head;
}

void RingBuffer_t::ClearRead() {
	_read_head = _write_head;
}

size_t RingBuffer_t::AvailableWrite() const {
	return _read_head + _buffer_size - _write_head;
}

size_t RingBuffer_t::AvailableRead() const {
	return _write_head - _read_head;
}

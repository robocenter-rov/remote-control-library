#pragma once

class OutOfRangeException_t {};

class RingBuffer_t {
private:
	char* _buffer;
	size_t _buffer_size;
	size_t _read_head;
	size_t _write_head;
public:
	RingBuffer_t(size_t buffer_size);
	~RingBuffer_t();
	int GetChar();
	void Write(void* buffer, size_t size);
	size_t AvailableWrite() const;
	size_t AvailableRead() const;
};

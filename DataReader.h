#pragma once
#include <cstdint>
#include <exception>

class DataReader_t {
protected:
	const uint8_t* _buffer;
	size_t _buffer_size;
	unsigned int _readed;

	void CheckBuffSize(size_t read_size) const;
public:
	DataReader_t(const void* buffer, size_t buffer_size);

	class too_short_buffer_exception_t : public std::exception {};

	bool GetBool(); 
	uint8_t GetUInt8();
	int16_t GetInt16();
	uint16_t GetUInt16();
	int32_t GetInt32();
	uint32_t GetUInt32();
	float GetFloat();
	const uint8_t* GetBytes(size_t count);
	
	template<typename T>
	T GetVar() {
		CheckBuffSize(sizeof(T));
		T res = *(reinterpret_cast<const T*>(_buffer + _readed));
		_readed += sizeof(T);
		return res;
	}

	int Readed() const;
	int Available() const;
};

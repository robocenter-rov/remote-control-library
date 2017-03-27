#pragma once
#include <exception>

class ConnectionProvider_t {
protected:
	char* _buffer;
	size_t _buffer_size;
public:
	ConnectionProvider_t(char* buffer, size_t buffer_size);

	virtual ~ConnectionProvider_t() {}
	virtual void Begin() {}
	virtual void Stop() {}
	virtual ConnectionProvider_t* Write(bool val);
	virtual ConnectionProvider_t* Write(char val) = 0;
	virtual ConnectionProvider_t* Write(int val);
	virtual ConnectionProvider_t* Write(unsigned int val);
	virtual ConnectionProvider_t* Write(float val);
	virtual ConnectionProvider_t* Write(char* buffer, int count) = 0;
	virtual ConnectionProvider_t* BeginPacket() = 0;
	virtual ConnectionProvider_t* EndPacket() = 0;
	virtual ConnectionProvider_t* Send(char* buffer, int bytes);
	virtual int Receive(const char** buffer) = 0;
	virtual bool Connected() = 0;

	class DataReader_t {
	protected:
		const char* _buffer;
		size_t _buffer_size;
		unsigned int _readed;

		void CheckBuffSize(size_t read_size) const;
	public:
		DataReader_t(const char* buffer, size_t buffer_size);

		class too_short_buffer_exception_t : public std::exception {};

		char GetByte();
		bool GetBool();
		int GetInt();
		unsigned int GetUInt();
		float GetFloat();
		const char* GetBytes(size_t count);

		int Readed() const;
	};
};

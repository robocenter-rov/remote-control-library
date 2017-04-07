#pragma once
#include "ConnectionProvider.h"
#include "boost/asio.hpp"

using namespace boost::asio;

class UdpConnectionProvider_t : public ConnectionProvider_t {
public:
	UdpConnectionProvider_t(ip::udp::endpoint remote_endpoint);
	
	void Begin() override;
	void Send(char* buffer, int bytes) override;
	void Receive(char* buffer, int bytes) override;
	bool Connected() override;

	~UdpConnectionProvider_t() override;
private:
	ip::udp::socket _udpSocket;
	ip::udp::endpoint _remoteEndPoint;
};

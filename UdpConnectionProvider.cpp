#include "UdpConnectionProvider.h"

UdpConnectionProvider_t::UdpConnectionProvider_t(ip::udp::endpoint remote_endpoint) : _udpSocket(io_service()) {
	_remoteEndPoint = remote_endpoint;
}

void UdpConnectionProvider_t::Begin() {
	_udpSocket.open(ip::udp::v4());
}

void UdpConnectionProvider_t::Send(char* buffer_ptr, int bytes) {
	_udpSocket.send_to(buffer(buffer_ptr, bytes), _remoteEndPoint);
}

void UdpConnectionProvider_t::Receive(char* buffer_ptr, int bytes) {
	_udpSocket.receive(mutable_buffer(buffer_ptr, bytes));
}

bool UdpConnectionProvider_t::Connected() {
	return true;
}

UdpConnectionProvider_t::~UdpConnectionProvider_t() {}

#include "UDPConnectionProvider.h"

UDPConnectionProvider_t::UDPConnectionProvider_t(const QHostAddress &address, uint16_t port, size_t receive_buffer_size)
    : ConnectionProvider_t(receive_buffer_size),
    _address(address),
    _port(port),
    _udp_socket(new QUdpSocket())
{
    _received = 0;
    QObject::connect(
        _udp_socket,
        &QUdpSocket::readyRead,
        [&]() {
            _received = _udp_socket->pendingDatagramSize();
            QHostAddress *address = new QHostAddress();
            _udp_socket->readDatagram((char*)_receive_buffer, _receive_buffer_size, address);
            if (_received > _receive_buffer_size) {
                _received = 0;
                return;
            }
        }
    );
}

UDPConnectionProvider_t::~UDPConnectionProvider_t() {
    Stop();
    delete _udp_socket;
}

void UDPConnectionProvider_t::Begin() {
     _udp_socket->bind(_address, _port);
}

void UDPConnectionProvider_t::Stop() {
    _udp_socket->close();
}

ConnectionProvider_t* UDPConnectionProvider_t::Write(const void *buffer, size_t size) {
    _send_buffer.append((char *)buffer, size);
    return this;
}

ConnectionProvider_t* UDPConnectionProvider_t::WriteUInt8(uint8_t c) {
    _send_buffer.append(c);
    return this;
}

ConnectionProvider_t* UDPConnectionProvider_t::BeginPacket() {
    _received = 0;
    _send_buffer.clear();
    return this;
}

ConnectionProvider_t* UDPConnectionProvider_t::EndPacket() {
    _udp_socket->writeDatagram(_send_buffer, _address, _port);
    return this;
}

int UDPConnectionProvider_t::Receive() {
    return _received;
}

bool UDPConnectionProvider_t::Connected() {
    return _udp_socket->state() == QAbstractSocket::ConnectedState;
}

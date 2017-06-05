#include "UDPConnectionProvider.h"

UDPConnectionProvider_t::UDPConnectionProvider_t(const QHostAddress &address, uint16_t port, size_t receive_buffer_size)
    : ConnectionProvider_t(receive_buffer_size),
    _address(address),
    _port(port),
    _udp_socket(new QUdpSocket())
{
    connect(_udp_socket, SIGNAL(readyRead()), this, SLOT(readPendingDatagrams()));
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
    _send_buffer = QByteArray((char *)buffer, size);
    return this;
}

ConnectionProvider_t* UDPConnectionProvider_t::WriteUInt8(uint8_t) {
    return this;
}

ConnectionProvider_t* UDPConnectionProvider_t::BeginPacket() {
    _send_buffer.clear();
    return this;
}

ConnectionProvider_t* UDPConnectionProvider_t::EndPacket() {
    _udp_socket->writeDatagram(_send_buffer, _address, _port);
    return this;
}

int UDPConnectionProvider_t::Receive() {
    return _receive_buffer_size;
}

bool UDPConnectionProvider_t::Connected() {
    return _udp_socket->state() == QAbstractSocket::ConnectedState;
}

void UDPConnectionProvider_t::readPendingDatagrams() {
    QByteArray receivedData;
    receivedData.resize(_udp_socket->pendingDatagramSize());
    QHostAddress *address = new QHostAddress();
    _udp_socket->readDatagram(receivedData.data(), receivedData.size(), address);

    _receive_buffer_size = receivedData.size();
    for (int i = 0; i < receivedData.size(); i++) {
        _receive_buffer[i] = static_cast<uint8_t>(receivedData[i]);
    }
}

#include "UDPConnectionProvider.h"

WriteInvoker_t::WriteInvoker_t(QHostAddress address, uint16_t port, QUdpSocket *udp_socket) {
    _address = address;
    _port = port;
    _udp_socket = udp_socket;
    connect(this, SIGNAL(Send(uint8_t*,int)), this, SLOT(Sender(uint8_t*,int)));
}

WriteInvoker_t::~WriteInvoker_t() {
    delete _udp_socket;
}

void WriteInvoker_t::Sender(uint8_t *buffer, int size) {
    _udp_socket->writeDatagram((char*)buffer, size, _address, _port);
}

UDPConnectionProvider_t::UDPConnectionProvider_t(const QHostAddress &address, uint16_t port, size_t receive_buffer_size, size_t send_buffer_size)
    : ConnectionProvider_t(receive_buffer_size),
    _write_invoker(address, port, new QUdpSocket()),
    _send_buffer(new uint8_t[send_buffer_size]),
    _send_buffer_size(send_buffer_size)
{
    _received = 0;
    QObject::connect(
        _write_invoker._udp_socket,
        &QUdpSocket::readyRead,
        [&]() {
            _received = _write_invoker._udp_socket->pendingDatagramSize();
            QHostAddress *address = new QHostAddress();
            _write_invoker._udp_socket->readDatagram((char*)_receive_buffer, _receive_buffer_size, address);
            if (_received > _receive_buffer_size) {
                _received = 0;
                return;
            }
        }
    );
}

UDPConnectionProvider_t::~UDPConnectionProvider_t() {
    Stop();
    delete[] _send_buffer;
}

void UDPConnectionProvider_t::Begin() {
     _write_invoker._udp_socket->bind(_write_invoker._address, _write_invoker._port);
}

void UDPConnectionProvider_t::Stop() {
    _write_invoker._udp_socket->close();
}

ConnectionProvider_t* UDPConnectionProvider_t::Write(const void *buffer, size_t size) {
    if (_send_buffer_size < _send_head + size) {
        _send_head = 0;
        throw SendBufferLimitExceeded_t();
    }
    memcpy(_send_buffer + _send_head, buffer, size);
    _send_head += size;
    return this;
}

ConnectionProvider_t* UDPConnectionProvider_t::WriteUInt8(uint8_t c) {
    if (_send_head >= _send_buffer_size) {
        _send_head = 0;
        throw SendBufferLimitExceeded_t();
    }
    _send_buffer[_send_head++] = c;
    return this;
}

ConnectionProvider_t* UDPConnectionProvider_t::BeginPacket() {
    _send_head = 0;
    return this;
}

ConnectionProvider_t* UDPConnectionProvider_t::EndPacket() {
    _write_invoker.Send(_send_buffer, _send_head);
    _send_head = 0;
    return this;
}

int UDPConnectionProvider_t::Receive() {
    size_t t = _received;
    _received = 0;
    return t;
}

bool UDPConnectionProvider_t::Connected() {
    return _write_invoker._udp_socket->state() == QAbstractSocket::ConnectedState;
}

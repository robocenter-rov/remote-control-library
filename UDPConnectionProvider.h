#ifndef UDPCONNECTIONPROVIDER_H
#define UDPCONNECTIONPROVIDER_H

#include "ConnectionProvider.h"
#include <QHostAddress>
#include <QUdpSocket>
#include <QByteArray>
#include <QObject>

class UDPConnectionProvider_t : public ConnectionProvider_t
{
private:
    QHostAddress _address;
    uint16_t _port;
    QUdpSocket* _udp_socket;
    uint8_t* _send_buffer;
    size_t _send_head;
    size_t _send_buffer_size;
    size_t _received;
public:
    explicit UDPConnectionProvider_t(const QHostAddress &address, uint16_t port, size_t receive_buffer_size, size_t send_buffer_size);
    ~UDPConnectionProvider_t() override;
    void Begin() override;
    void Stop() override;
    ConnectionProvider_t* Write(const void* buffer, size_t size) override;
    ConnectionProvider_t* WriteUInt8(uint8_t val) override;
    ConnectionProvider_t* BeginPacket() override;
    ConnectionProvider_t* EndPacket() override;
    int Receive() override;
    bool Connected() override;
};

#endif // UDPCONNECTIONPROVIDER_H

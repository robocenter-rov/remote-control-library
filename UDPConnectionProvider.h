#ifndef UDPCONNECTIONPROVIDER_H
#define UDPCONNECTIONPROVIDER_H

#include "ConnectionProvider.h"
#include <QHostAddress>
#include <QUdpSocket>
#include <QByteArray>
#include <QObject>

class UDPConnectionProvider_t : public ConnectionProvider_t, public QObject
{
    Q_OBJECT
private:
    QHostAddress _address;
    uint16_t _port;
    QUdpSocket* _udp_socket;
    QByteArray _send_buffer;
private slots:
    void readPendingDatagrams();
public:
    explicit UDPConnectionProvider_t(const QHostAddress &address, uint16_t port, size_t receive_buffer_size);
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

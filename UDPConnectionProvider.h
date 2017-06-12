#ifndef UDPCONNECTIONPROVIDER_H
#define UDPCONNECTIONPROVIDER_H

#include "ConnectionProvider.h"
#include <QHostAddress>
#include <QUdpSocket>
#include <QByteArray>
#include <QObject>

class WriteInvoker_t : public QObject {
    Q_OBJECT
private:
public:
    QHostAddress _address;
    uint16_t _port;
    QUdpSocket *_udp_socket;
    WriteInvoker_t(QHostAddress address, uint16_t port, QUdpSocket *udp_socket);
    ~WriteInvoker_t();
signals:
    void Send(uint8_t* buffer, int size);
private slots:
    void Sender(uint8_t* buffer, int size);
};

class UDPConnectionProvider_t : public ConnectionProvider_t
{
private:
    uint8_t* _send_buffer;
    size_t _send_head;
    size_t _send_buffer_size;
    size_t _received;

    WriteInvoker_t _write_invoker;
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

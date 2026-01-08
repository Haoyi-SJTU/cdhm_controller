#ifndef SERIAL_H
#define SERIAL_H
#include<QTcpSocket>
#include<QMessageBox>
#include <QMessageBox>
#include <iostream>

#define serial_Ip "192.168.1.254"  //485转ETH IP
#define serial_Port 4196

// 力传感器读取相关

class Serial:public QObject
{
    Q_OBJECT

public:
    Serial();
    bool connectSerialServer();
    void sendSerialMsg(QString msg, QByteArray data);//MainWindow::forceRead()调用
    bool isConnected();

    QByteArray HexStringToByteArray(QString HexString);
    qint32 QByteArrayToqint32(QByteArray input);

private:
    QTcpSocket *m_SerialSocket;

// 声明一个​​信号​​
signals:
    void returnForce(QByteArray,int);
    //表示该类可以发出一个名为returnForce的信号，附带QByteArray和int两个参数

private slots:
    void onReadyRead();

};

#endif // SERIAL_H

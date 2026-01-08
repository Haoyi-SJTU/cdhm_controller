#include "serial.h"

// 力传感器读取相关

using namespace std;

Serial::Serial()
{
    m_SerialSocket = new QTcpSocket(this);
}


bool Serial::connectSerialServer()
{
    //连接485
    if(m_SerialSocket->state() == QAbstractSocket::UnconnectedState)//第二步再判断tcpClient套接字的状态是否建立连接，这里判断是没有建立连接
    {
        //主动和服务器进行连接
        m_SerialSocket->connectToHost(serial_Ip, serial_Port);
        //连接成功进入if语句
        if(m_SerialSocket->waitForConnected(1000))
        {
            connect(m_SerialSocket,&QTcpSocket::readyRead,this,&Serial::onReadyRead);
            cout<<"485 connected!"<<endl;
            return true;
        }
        else
        {
            cout<<"485 connect fail 1"<<endl;
            return false;
        }
    }
    else// 主动断开连接485
    {
        m_SerialSocket->disconnectFromHost();
        return false;
    }

}

bool Serial::isConnected(){

    if(m_SerialSocket->state() == QAbstractSocket::ConnectedState){
        cout<<"485 connected!"<<endl;
        return true;
    }
    else{
        cout<<"485 connect fail 3"<<endl;
        return false;
    }

}

// 将16进制字符串转换为对应的字节序列
QByteArray Serial::HexStringToByteArray(QString HexString)
{
    bool ok;
    QByteArray ret;
    HexString = HexString.trimmed();
    HexString = HexString.simplified();
    QStringList sl = HexString.split(" ");

    foreach (QString s, sl)
    {
        if(!s.isEmpty())
        {
            char c = s.toInt(&ok,16)&0xFF;
            if(ok){
                ret.append(c);
            }else{
                qDebug()<<"非法的16进制字符："<<s;
                QMessageBox::warning(0,tr("错误："),QString("非法的16进制字符: \"%1\"").arg(s));
            }
        }
    }
    qDebug()<<ret;
    return ret;
}


// MainWindow::forceRead()调用
void Serial:: sendSerialMsg(QString msg,QByteArray data)
{
    if(msg!=""){
        QByteArray hexByteArray=  QByteArray::fromHex(msg.toUtf8());
        qDebug()<<"发送"<<hexByteArray;
        cout<<"sent to Fsensor"<<(string)data<<endl;
        m_SerialSocket->write(hexByteArray);
    }
    else{
        // data写入套接字的发送缓冲区，​​异步​​通过TCP协议发送到传感器
        m_SerialSocket->write(data);
        // cout<<"2 sent to Fsensor"<<(string)data<<endl;
        // qDebug()<<"发送"<<data;
    }
}


// readyRead()信号槽函数
void Serial::onReadyRead()
{
    //读取传感器返回的数据,获取套接字中的内容
    QByteArray tmp =m_SerialSocket->readAll();
    //qDebug()<<"原始数据："<<QString(tmp.toHex());
    if(!tmp.isEmpty()&&tmp.length()>6){
        int index=QString::number(tmp.at(0)).toInt();//根据串口返回数据的站号(01/02/03/04/05/06/07/08)确定触发的信号
        int data_len=QString::number(tmp.at(2)).toInt();//根据串口返回数据的长度确认是否是编码器数据

        switch (index) {
        case 1://拉力传感器 转接板 1
        {
            if(data_len==36)
            {
                QByteArray data_new= tmp.mid(3,36);
                //qDebug()<<"拉力传感器A数据：" <<data_new.toHex().toUpper();
                emit(returnForce(data_new,1));// 通过emit关键字触发信号,串口数据解析完成后通知界面更新
            }
            break;
        }

        // case 2://拉力传感器 转接板 2
        // {
        //     if(data_len==36)
        //     {
        //         QByteArray data_new= tmp.mid(3,36);
        //         //qDebug()<<"拉力传感器B数据：" <<data_new.toHex().toUpper();
        //         emit(returnForce(data_new,2));
        //     }
        //     break;
        // }

        // case 3://拉力传感器 转接板 3
        // {
        //     if(data_len==36)
        //     {
        //         QByteArray data_new= tmp.mid(3,36);
        //         //qDebug()<<"拉力传感器C数据：" <<data_new.toHex().toUpper();
        //         emit(returnForce(data_new,3));
        //     }
        //     break;
        // }
        // case 4://拉力传感器 转接板 4
        // {
        //     if(data_len==36)
        //     {
        //         QByteArray data_new= tmp.mid(3,36);
        //         //qDebug()<<"拉力传感器C数据：" <<data_new.toHex().toUpper();
        //         emit(returnForce(data_new,4));
        //     }
        //     break;
        // }
        default:
            break;
        }
    }
}



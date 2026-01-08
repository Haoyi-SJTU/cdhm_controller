#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <QUdpSocket>
#include <QTcpSocket>
#include <QHostAddress>
#include <QMessageBox>
#include <fstream>
#include <QDateTime>
#include <QString>
#include <iostream>

//包含eigen矩阵库头文件
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

using namespace std;

const int SECTION_NUM = 3; //总关节数
const int SEC_START_INDEX = 0;//关节范围限制：start 0  end 11
const int SEC_END_INDEX = 2;

extern int CLB_FLAG;

class Communication : public QObject
{
    Q_OBJECT
public:
    Communication();

    void isConnected();
private:

    QTcpSocket *TcpClientA = new QTcpSocket(this); //定义
    // QTcpSocket *TcpClientB = new QTcpSocket(this); //定义
    // QTcpSocket *TcpClientC = new QTcpSocket(this); //定义
    // QTcpSocket *TcpClientD = new QTcpSocket(this); //定义

    QByteArray SendDatagram_motor_value;
    Eigen::MatrixXd Cur_Motor_Pos1;
    Eigen::MatrixXd Cur_Ang1;
    double sec9_Hor_Ang_last;//第9节水平
    bool first_flag;
    bool Board_A_Flag;
    std::ofstream clb_of;
    bool clb_flag;
    bool ConnSuccess;

    void Tcp_close();
    void Recv_All();
    void Tcp_connect();
    void SendTcpData(QByteArray msg);


signals:
    void Return_Cur_Motor_Pos(Eigen::MatrixXd, Eigen::MatrixXd);//电机位置信号和关节编码器信号

private slots:
    //客户端槽函数
    void ReadData_A();
    // void ReadData_B();
    // void ReadData_C();
    // void ReadData_D();

    void Send_CON();
    void Send_POS(Eigen::MatrixXd Len_Diff,double base_Diff);
    void Send_STP();
    void Send_Current(int sec_index, int sec_Endindex,int cur_speed_level, int cur_level);
    void Send_SingleCurrent(int sec_index,int rope1_cur_level,int rope2_cur_level,int rope3_cur_level, int cur_speed_level);
    void Send_Record_Angle(int angle_flag);
    void Send_PPM();
    void Send_ClearErr();
    void Send_Calibration();

public slots:
    void clb_stop();
    void clb_start(char *clb_filename);

};

#endif // COMMUNICATION_H

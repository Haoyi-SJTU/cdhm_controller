#include <communication.h>



Communication::Communication() : QObject()
{
    Cur_Motor_Pos1=Eigen::MatrixXd::Zero(12,3);
    Cur_Ang1=Eigen::MatrixXd::Zero(12,2);

    Board_A_Flag = false;//标志 接收到的TCP数据包非空
    ConnSuccess = false;//连接F407标志位
}


// 连接服务器
void Communication:: Tcp_connect(){
    //初始化TCP
    TcpClientA->abort();                 //取消原有连接
    TcpClientA->setSocketOption(QAbstractSocket::LowDelayOption,1);
    TcpClientA->connectToHost("192.168.1.251",80); //IP以及端口号

    ConnSuccess=TcpClientA->waitForConnected(30000);
    connect(TcpClientA,SIGNAL(readyRead()),this,SLOT(ReadData_A()));
}



void Communication::isConnected(){

    QAbstractSocket::SocketState state_A = TcpClientA->state();
    qDebug()<<"A---" << state_A;


}

// 断开所有服务器！
void Communication::Tcp_close()
{
    TcpClientA->disconnectFromHost();                 //取消原有连接
    qDebug() << "断开所有服务器！" ;
}



// 向F407发送统一的数据
void Communication::SendTcpData(QByteArray msg){
    if((TcpClientA->isOpen())&&ConnSuccess)
    {
        TcpClientA->write(msg);
        TcpClientA->flush();
    }
    else{
        ConnSuccess=false;
    }
}



// 连接STM32
void Communication::Send_CON(){
    //TCP通讯
    Tcp_connect();
    if(ConnSuccess){
        qDebug() << "已连接至全部F407" ;
        quint8 TCP_SendData[12]={0};
        *(char *) (TCP_SendData+0) = '#';
        *(char *) (TCP_SendData+1) = 'C';
        *(char *) (TCP_SendData+2) = 'O';
        *(char *) (TCP_SendData+3) = 'N';
        SendDatagram_motor_value.resize(12);
        memcpy(SendDatagram_motor_value.data(),TCP_SendData,sizeof(TCP_SendData));
        SendTcpData(SendDatagram_motor_value);
    }
    else {
        qDebug() << "未全部连接至F407" ;
    }

}

// 断开STM32连接(电机驱动量为0)
void Communication::Send_STP(){
    //TCP通信
    if(ConnSuccess){
        quint8 TCP_SendData[4]={0};
        *(char *) (TCP_SendData+0) = '#';
        *(char *) (TCP_SendData+1) = 'S';
        *(char *) (TCP_SendData+2) = 'T';
        *(char *) (TCP_SendData+3) = 'P';
        SendDatagram_motor_value.resize(4);
        memcpy(SendDatagram_motor_value.data(),TCP_SendData,sizeof(TCP_SendData));
        SendTcpData(SendDatagram_motor_value);
    }
    else{
        qDebug() << "Send_STP 失败!" ;
    }

}

// Len_Diff 12*3矩阵 存放12个关节的每根绳的驱动量
// 驱动maxon电机运动
void Communication::Send_POS(Eigen::MatrixXd Len_Diff,double base_Diff)
{
    // debug
    // cout <<"发出去的绳子变化量:";
    // for (int row = 0; row < 3; row++)
    // {
    //     for (int col = 0; col < 3; col++)
    //         cout << Len_Diff(row,col) << " ";
    // }
    // cout << endl;

    //TCP通信
    if(ConnSuccess)
    {
        quint8 TCP_SendData[400]={0};
        quint8 i,j;
        *(char *) (TCP_SendData+0) = '#';
        *(char *) (TCP_SendData+1) = 'P';
        *(char *) (TCP_SendData+2) = 'O';
        *(char *) (TCP_SendData+3) = 'S';

        for(i = SEC_START_INDEX; i <SECTION_NUM; i++)
        {
            for(j = 0; j < 3; j++)
            {
                *(double *) (TCP_SendData+( SECTION_NUM*j + i )*8 + 4) = Len_Diff(i,j);// SECTION_NUM为关节数
            }
            // *(double *) (TCP_SendData+SECTION_NUM*3 * 8 + 4) = base_Diff;//底座驱动量
        }
        SendDatagram_motor_value.resize(400);
        memcpy(SendDatagram_motor_value.data(),TCP_SendData,sizeof(TCP_SendData));
        SendTcpData(SendDatagram_motor_value);
    }
    else{
        qDebug() << "Send_POS 失败！" ;
    }
}


 // 修改maxon电机控制模式为电流模式
void Communication::Send_Current(int sec_Startindex,int sec_Endindex, int cur_speed_level, int cur_level){
    //TCP通信
    if(ConnSuccess){
        quint8 TCP_SendData[400]={0};
        *(char *) (TCP_SendData+0) = '#';
        *(char *) (TCP_SendData+1) = 'C';
        *(char *) (TCP_SendData+2) = 'U';
        *(char *) (TCP_SendData+3) = 'R';
        *(int *) (TCP_SendData+4) = sec_Startindex; //控制的起始电机关节数
        *(int *) (TCP_SendData+8) = sec_Endindex;//控制的结束电机关节数
        *(int *) (TCP_SendData+12) = cur_speed_level;
        *(int *) (TCP_SendData+16) = cur_level; // curlevel=10是默认值，=1是原电流的1/10
        qDebug() << "Current!!!";
        SendDatagram_motor_value.resize(400);
        memcpy(SendDatagram_motor_value.data(),TCP_SendData,sizeof(TCP_SendData));
        SendTcpData(SendDatagram_motor_value);
    }
    else{
        qDebug() << "Send_CUR 失败！" ;
    }
}

// 修改maxon电机控制模式为 单电机多电流模式
void Communication::Send_SingleCurrent(int sec_index,int rope1_cur_level,int rope2_cur_level,int rope3_cur_level, int cur_speed_level){
    //TCP通信
    if(ConnSuccess){
        quint8 TCP_SendData[300]={0};
        *(char *) (TCP_SendData+0) = '#';
        *(char *) (TCP_SendData+1) = 'S';
        *(char *) (TCP_SendData+2) = 'C';
        *(char *) (TCP_SendData+3) = 'R';
        *(int *) (TCP_SendData+4) = sec_index;
        *(int *) (TCP_SendData+8) = rope1_cur_level;
        *(int *) (TCP_SendData+12) = rope2_cur_level;
        *(int *) (TCP_SendData+16) = rope3_cur_level; // curlevel=10是默认值，=1是原电流的1/10
        *(int *) (TCP_SendData+20) = cur_speed_level;
        qDebug() << "Single Current!!!";
        SendDatagram_motor_value.resize(400);
        memcpy(SendDatagram_motor_value.data(),TCP_SendData,sizeof(TCP_SendData));
        SendTcpData(SendDatagram_motor_value);
    }
    else{
        qDebug() << "Send_SingleCurrent 失败！" ;
    }
}



void Communication::Send_Record_Angle(int angle_flag)
{
    //TCP通信
    if(ConnSuccess)
    {
        quint8 TCP_SendData[12]={0};
        *(char *) (TCP_SendData+0) = '#';
        *(char *) (TCP_SendData+1) = 'A';
        *(char *) (TCP_SendData+2) = 'N';
        *(char *) (TCP_SendData+3) = 'G';
        *(int *) (TCP_SendData+4) = angle_flag;
        // qDebug() << "Record_Angle_Flag = "<< angle_flag;
        SendDatagram_motor_value.resize(400);
        memcpy(SendDatagram_motor_value.data(),TCP_SendData,sizeof(TCP_SendData));
        SendTcpData(SendDatagram_motor_value);
    }
    else
    {
        qDebug() << "TCP读角度失败! angle_flag=0" ;
    }
}

// 修改maxon电机控制模式为位置模式
void Communication::Send_PPM()
{
    //TCP通信
    if(ConnSuccess){
        quint8 TCP_SendData[4]={0};
        *(char *) (TCP_SendData+0) = '#';
        *(char *) (TCP_SendData+1) = 'P';
        *(char *) (TCP_SendData+2) = 'P';
        *(char *) (TCP_SendData+3) = 'M';
        qDebug() << "PPM!!!!";
        SendDatagram_motor_value.resize(4);
        memcpy(SendDatagram_motor_value.data(),TCP_SendData,sizeof(TCP_SendData));
        SendTcpData(SendDatagram_motor_value);//"#PPM"发送给电机
        PPM_Flag = true;
    }
    else
    {
        qDebug() << "Send_PPM 失败！" ;
    }
}

//未使用
void Communication::Send_ClearErr()
{
    //TCP通信
    if(ConnSuccess){
        quint8 TCP_SendData[4]={0};
        *(char *) (TCP_SendData+0) = '#';
        *(char *) (TCP_SendData+1) = 'C';
        *(char *) (TCP_SendData+2) = 'L';
        *(char *) (TCP_SendData+3) = 'E';
        qDebug() << "CLEAR MOTOR ERROR!!!!";
        SendDatagram_motor_value.resize(4);
        memcpy(SendDatagram_motor_value.data(),TCP_SendData,sizeof(TCP_SendData));
        SendTcpData(SendDatagram_motor_value);
    }
    else{
        qDebug() << "CLEAR MOTOR ERROR 失败，请检查连接状态！" ;
    }
}

//
void Communication::Send_Calibration()
{
    //TCP通信
    if(ConnSuccess){
        quint8 TCP_SendData[12]={0};
        *(char *) (TCP_SendData+0) = '#';
        *(char *) (TCP_SendData+1) = 'C';
        *(char *) (TCP_SendData+2) = 'L';
        *(char *) (TCP_SendData+3) = 'B';
        *(int *) (TCP_SendData+4) = CLB_FLAG;
        qDebug() << "Calibration!!!";
        SendDatagram_motor_value.resize(400);
        memcpy(SendDatagram_motor_value.data(),TCP_SendData,sizeof(TCP_SendData));
        SendTcpData(SendDatagram_motor_value);
    }
    else{
        qDebug() << "Send_Calibration 失败！" ;
    }

}


// 读电机位置和关节编码器 发布为信号Return Cur Motor Pos
void Communication::Recv_All()
{
    // int i,j;
    if(Board_A_Flag)
    {
        Board_A_Flag=false;
        emit(Return_Cur_Motor_Pos(Cur_Motor_Pos1, Cur_Ang1));
        Cur_Motor_Pos1=Eigen::MatrixXd::Zero(12,3);
    }
}


void Communication::ReadData_A()
{
    QByteArray RecvData = TcpClientA->readAll();
    quint8 TCP_RecvData[400]={0};
    int i,j;

    //TCP
    if(!RecvData.isEmpty())
    {
        Board_A_Flag=true;
        memcpy(&TCP_RecvData,RecvData.data(),RecvData.size());//RecvData复制给TCP_RecvData
        for(int k=0;k<9;k++){
            Cur_Motor_Pos1(k,0) = *(double *)(TCP_RecvData+k*8);//TCP关节角数据解包 转double类型
        }
        for(int k=0; k<6; k++){
            i = k / 2; // 行索引：0,0,1,1,2,2
            j = k % 2; // 列索引：0,1,0,1,0,1
            Cur_Ang1(i,j) = *(double *)(TCP_RecvData+(k+9)*8);
            // cout<<"Cur_Ang1("<<i<<","<<j<<")="<<Cur_Ang1(i,j)<<endl;
        }
    }

    Recv_All();
}


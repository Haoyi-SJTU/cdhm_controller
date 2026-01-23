#include "mainwindow.h"
#include "ui_mainwindow.h"

//old

int PLAN_FRAME;
int CLB_FLAG;
bool PPM_Flag = false; //标志 电机切换为PPM位置模式

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    parm_init();
    function_connect();
    ui_init();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::ui_init()
{
    ui->forceRead_pushButton->setEnabled(false);
    ui->SaveData_pushButton->setEnabled(false);
    ui->Record_Angle_Open_pushButton->setEnabled(false);
    // ui->Cur_Base_Pos_lineEdit->setText(QString::number(Cur_Base_Pos));

    for (int i = 0; i < 3; ++i)
    {
        QLineEdit* cur_bias_h = this->findChild<QLineEdit*>("Cur_Hor_Ang_Bias"+QString::number(i+1));
        QLineEdit* cur_bias_v = this->findChild<QLineEdit*>("Cur_Ver_Ang_Bias"+QString::number(i+1));
        cur_bias_h->setText(QString::number(Tar_Bias[i][0],'f',2));
        cur_bias_v->setText(QString::number(Tar_Bias[i][1],'f',2));
    }
}

void MainWindow::parm_init()
{
    //bool 标志位
    STM32_Flag = false;
    Total_Stop_Flag = false;
    Record_Angle_Flag = false;
    Feedback_Recv_Flag = false;//下位机数据接收完成标志
    CurMode_PPM_Flag = false;
    MultiMotor_CurMode_Flag=false;
    Setzero_Move_Flag=false;//回零标志位
    Joint_Move_Flag=false;
    Tension_alert_send_flag=false;
    Tele_Ready_Flag = false;
    Planned_Motion_Flag=false;
    Save_Force_Flag = false;//1 导出力传感器数据 0 停止导出

    //控制参数
    memset(Cur_Ang,0,sizeof(Cur_Ang));//当前关节角度
    memset(Tar_Ang,0,sizeof(Tar_Ang));//目标关节角度
    memset(JointMove_TarAng,0,sizeof(JointMove_TarAng));;//关节运动目标角度
    memset(Cur_Motor_Pos,0,sizeof(Cur_Motor_Pos));;//Maxon当前电机绝对位置
    memset(Serial_dispCH, 0, sizeof(Serial_dispCH));//拉力传感器串口原始数据
    memset(Force_array, 0, sizeof(Force_array));//拉力传感器数据 以关节绳号存储
    memset(Force_array_last, 0, sizeof(Force_array_last));

    memset(Pre_Angle_Diff, 0, sizeof(Pre_Angle_Diff));
    memset(Tar_Bias, 0, sizeof(Tar_Bias));//角度零位偏移量

    //角度零位偏移量
    Tar_Bias[0][0] = -0.63;//1a 水平
    Tar_Bias[0][1] = 0.0;//2b 竖直
    Tar_Bias[1][0] = 0.25;//2a 水平
    Tar_Bias[1][1] = -0.74;//2b 竖直
    Tar_Bias[2][0] = -1.23; //3a 水平
    Tar_Bias[2][1] = -2.27; //3b 竖直

    // 力传感器零位偏移量
    Force_array_bias[0][0] = 0;//1-1
    Force_array_bias[0][1] = 0;
    Force_array_bias[0][2] = 0;
    Force_array_bias[1][0] = 0;//2-1
    Force_array_bias[1][1] = 0;
    Force_array_bias[1][2] = 0;
    Force_array_bias[2][0] = 0;//3-1
    Force_array_bias[2][1] = 0;
    Force_array_bias[2][2] = 0;

    //角度回零
    P_ZeroForce=0.00001;      //角度回零后，零位保持阈值
    setZero_cur_stage=0;      //显示当前进行回零的关节号
    Sec_Setzero_Startindex=0; //回零起始范围
    Sec_Setzero_Endindex=0;

    control_mode = 1;

    //力回零参数
    P_Force=0.0005;//角度回零 与力配合 p参数
    Max_Tension=195; //最大拉力
    best_force_tar[0]=120; //零位时驱动绳理想张力 关节0
    best_force_tar[1]=100; //零位时驱动绳理想张力 关节1
    best_force_tar[2]= 80; //零位时驱动绳理想张力 关节2
    Force_Threshold=10;//力平衡阈值

    // 机械臂运动 角度PID参数
    tarAng_delta_thr=0.3;//角度阈值 单位度
    P_Angle = 0.7;
    I_Angle = 0.005;
    Pre_Angle_Diff_Max = 2; //积分I项 角度误差累积限幅 2度


    Ang_Tele = Eigen::VectorXd::Zero(25,1);
    trajectory_count = 0;// 离线路径 计数器


    PLAN_FRAME=20;
    Planned_Frame=0;

    CLB_FLAG=0;
    //定时器
    Timer_secMotor  = new QTimer(this);
    Timer_secMotor->setTimerType(Qt::PreciseTimer);

    Timer_Reach_Target = new QTimer(this);
    Timer_Reach_Target->setTimerType(Qt::PreciseTimer);

    Timer_forceRead=new QTimer(this);
    Timer_forceRead->setTimerType(Qt::PreciseTimer);

    //类
    m_serial  = new Serial();
    communication = new Communication();
    Planner = new MotionPlan();
}

// 连接信号与槽
void MainWindow::function_connect()
{
    connect(this,SIGNAL(STM32_Init()),communication,SLOT(Send_CON()));
    connect(this,SIGNAL(STM32_Discon()),communication,SLOT(Tcp_close()));
    connect(this,SIGNAL(sig_angle(int)),communication,SLOT(Send_Record_Angle(int)));//角度传感器开启控制
    connect(this,SIGNAL(Send_Data(Eigen::MatrixXd,double)),communication,SLOT(Send_POS(Eigen::MatrixXd,double)));// 接收Send Data信号 向电机发送控制量
    //电机位置和关节编码器信号Return Cur Motor Pos  槽函数Recv Cur Motor Pos
    connect(communication,SIGNAL(Return_Cur_Motor_Pos(Eigen::MatrixXd,Eigen::MatrixXd)),this,SLOT(Recv_Cur_Motor_Pos(Eigen::MatrixXd,Eigen::MatrixXd)));
    connect(this,SIGNAL(sig_current(int,int,int,int)),communication,SLOT(Send_Current(int,int,int,int)));
    connect(this,SIGNAL(sig_singleCurrent(int,int,int,int,int)),communication,SLOT(Send_SingleCurrent(int,int,int,int,int)));
    connect(this,SIGNAL(sig_PPM()),communication,SLOT(Send_PPM()));//修改maxon电机控制模式为位置模式
    connect(Planner,SIGNAL(ReturnAng(Eigen::VectorXd)),this,SLOT(Recv_Ang_Path(Eigen::VectorXd)));//planning.cpp使用 发布规划结果
    connect(Timer_secMotor, &QTimer::timeout, this, &MainWindow::SecMotor_Send);//maxon 电机调试定时器
    connect(m_serial,&Serial::returnForce,this,&MainWindow::SetForce);//力传感信号returnForce 槽函数SetForce
    connect(Timer_forceRead, SIGNAL(timeout()), this, SLOT(forceRead()));//定时器触发 读力传感器
    connect(Timer_Reach_Target, SIGNAL(timeout()), this, SLOT(Reach_Target()));
    connect(this,SIGNAL(Tele_Caculate(double, double, const bool, const bool)),Planner,SLOT(Tele_operation(double, double, const bool, const bool)));
}

// 连接 电机位置和关节编码器信号Return_Cur_Motor_Pos  槽函数Recv Cur Motor Pos
// 该信号在communication.cpp中定义
void MainWindow::Recv_Cur_Motor_Pos(Eigen::MatrixXd Read_Cur_Motor_Pos, Eigen::MatrixXd Read_Cur_Ang)
{
    //读当前电机位置
    for(int i=SEC_START_INDEX;i<SECTION_NUM;i++)
    {
        Cur_Motor_Pos[i][0]=Read_Cur_Motor_Pos(i,0);
        Cur_Motor_Pos[i][1]=Read_Cur_Motor_Pos(i,1);
        Cur_Motor_Pos[i][2]=Read_Cur_Motor_Pos(i,2);
    }
    ui->STM32_Con_Button->setText("断开");
    // stm32连接成功
    if(!STM32_Flag){
        STM32_Flag=true;
        SetTip("STM32F407连接成功！");
        emit(sig_PPM());
        Total_Start();//开始触发reach_target函数

        ui->Record_Angle_Open_pushButton->setEnabled(true);
        ui->secMotor_Pull_Button->setEnabled(true);
        ui->secMotor_Loose_Button->setEnabled(true);
        ui->secMotor_ContinuePull_Button->setEnabled(true);
        ui->secMotor_ContinueLoose_Button->setEnabled(true);
        ui->secMotor_Stop_Button->setEnabled(true);
        ui->angSet_pushButton->setEnabled(true);
        ui->angStart_pushButton->setEnabled(true);
        ui->SetZero_pushButton->setEnabled(true);
        ui->change_controller->setEnabled(true);
        ui->set_P_ZeroForce_btn->setEnabled(true);
        ui->set_Ang_Threshold_btn->setEnabled(true);
        ui->set_Max_Tension_btn->setEnabled(true);
        ui->set_P_Angle_btn->setEnabled(true);
        ui->set_P_Force_btn->setEnabled(true);
    }

    //读6个关节编码器 关节角度 = 关节编码器数据 - 偏置
    Cur_Ang[0][0] = -(Read_Cur_Ang(0,0) - Tar_Bias[0][0]); //1a
    Cur_Ang[0][1] = -(Read_Cur_Ang(0,1) - Tar_Bias[0][1]); //1b
    Cur_Ang[1][0] = -(Read_Cur_Ang(1,0) - Tar_Bias[1][0]); //2a
    Cur_Ang[1][1] = -(Read_Cur_Ang(1,1) - Tar_Bias[1][1]); //2b
    Cur_Ang[2][0] = -(Read_Cur_Ang(2,0) - Tar_Bias[2][0]); //3a
    Cur_Ang[2][1] =   Read_Cur_Ang(2,1) - Tar_Bias[2][1];  //3b

    Feedback_Recv_Flag = true;//下位机数据接收完成

    setAngInfo();//更新显示​​关节角,关节角超限检测
    setMotorInfo();//更新显示电机位置​​

    if (CurMode_PPM_Flag)//电流模式下切回位置模式
    {
        CurMode_PPM_Flag = false;
        ui->MultiMotor_Current_pushButton->setEnabled(true);
        ui->singleRope_Current_pushButton->setEnabled(true);
        SetTip("从电流模式切换回位置模式完成!");
    }
}

void MainWindow::SetTip(const string& tip) {
    static string last_tip = "";
    if (last_tip != tip) {  // 避免重复
        auto now = chrono::system_clock::now();// 获取当前时间
        auto time = chrono::system_clock::to_time_t(now);
        stringstream ss;
        ss << put_time(std::localtime(&time), "%Y.%m.%d %H:%M:%S");
        cout << ss.str() << " ---" << tip << endl;
        last_tip = tip;
    }
}

// 更新显示电机位置
void MainWindow::setMotorInfo()
{
    for (int i = 0; i < 9;i++) {
        QLineEdit* text_disp = this->findChild<QLineEdit*>("Cur_Motor_Pos"+QString::number(i+1));
        text_disp->setText(QString::number(Cur_Motor_Pos[i/3][i%3],'f',3));
    }
}

//更新显示​​关节角,关节角超限检测
void MainWindow::setAngInfo()
{
    //界面更新​​：3个关节的水平和垂直角显示
    for (int i = 0; i < 3; ++i)
    {
        QLineEdit* text_disp_h = this->findChild<QLineEdit*>("Cur_Hor_Ang"+QString::number(i+1));
        QLineEdit* text_disp_v = this->findChild<QLineEdit*>("Cur_Ver_Ang"+QString::number(i+1));
        text_disp_h->setText(QString::number(Cur_Ang[i][0],'f',2));
        text_disp_v->setText(QString::number(Cur_Ang[i][1],'f',2));
        // cout<<"joint "<<i<<"  "<<Cur_Ang[i][0]<<"  "<<Cur_Ang[i][1]<<endl;
    }
    //关节角度超限检测
    for(int i=SEC_START_INDEX; i< SECTION_NUM; i++)
    {
        for(int j=0; j<2; j++){
            if(fabs(Cur_Ang[i][j]) > MAX_ANG){
                SetTip("关节角度过大!!");
                Total_Stop();
            }
        }
    }
}

//禁止运动:关节角度超限 或 到达目标    停止触发reach_target函数
void MainWindow::Total_Stop()
{
    Total_Stop_Flag = true;
    ui->Total_Stop_Button->setText("START"); //按钮设置为启动字样 下次按下则全部启动
    if(Timer_Reach_Target->isActive()){
        Timer_Reach_Target->stop();//停止触发reach_target函数
    }
}

//允许运动：按钮在START字样时被按下 或     触发reach_target函数
void MainWindow::Total_Start()
{
    Total_Stop_Flag = false;
    ui->Total_Stop_Button->setText("TOTALSTOP"); //按钮设置为stop字样 下次按下则禁止运动
    if(!Timer_Reach_Target->isActive()){
        Timer_Reach_Target->start(CONTROL_PERIOD);//触发reach_target函数
    }
}

// 若为start字样 则开始运动 若为total_stop字样，则结束运动
void MainWindow::on_Total_Stop_Button_clicked()
{
    if(!Total_Stop_Flag)
        Total_Stop();
    else
        Total_Start();
}

void MainWindow::msecSleep(int msec)
{
    QEventLoop loop;
    QTimer::singleShot(msec, &loop, SLOT(quit()));
    loop.exec();
}

// 根据角度a和b变换，输出变换前后两点距离
double MainWindow::Cal_Point_Dis(double *P, double a, double b)
{
    Eigen::Vector4d P1(P[0],P[1],P[2],1);
    Eigen::Vector4d P2;
    Eigen::Matrix4d Tran;
    double Ponit_Dis;
    a=a*PI/180.0;
    b=b*PI/180.0;
    //变换矩阵赋值
    Tran << cos(b),         0,       sin(b),         -h2*sin(b),
            sin(a)*sin(b),  cos(a),  0,              -h2*sin(a)*cos(b),
            -cos(a)*sin(b), sin(a),  cos(a)*cos(b),  h1+h2*cos(a)*cos(b),
            0 ,             0,       0 ,             1;
    P2=Tran*P1;
    Ponit_Dis=sqrt(pow((P1(0)-P2(0)),2)+pow((P1(1)-P2(1)),2)+pow((P1(2)-P2(2)),2)+pow((P1(3)-P2(3)),2));
    return Ponit_Dis;
}

// 根据角度a和b和导绳盘间距，输出上下两个过绳孔之距离 见郑阳论文P45
// 输入：Cur_Ang_Local[3][2] 3个关节上的转角a和b
// 输出：Len_Temp[3][3]
void MainWindow::Cal_Len(double Cur_Ang_Local[][2], double Len_Temp[][3])
{
    double Sec_Phi[3] = {360,320,280};
    for(int i=SEC_START_INDEX; i < SECTION_NUM; i++)
        for(int j=0;j<3;j++)
            Len_Temp[i][j] = 0;

    double P[3]={0};
    quint8 i,j,k;
    for(i=1; i<=SECTION_NUM; i++)//关节1-3
    {
        for(k=0;k<3;k++)//每个关节3根钢丝
        {
            for(j=1;j<=i;j++)//关节1-i的累计偏转
            {
                P[0]=Plate_Radius*sin(Sec_Phi[i-1]/180*PI - 2.0/3*PI*k);
                P[1]=0;
                P[2]=Plate_Radius*cos(Sec_Phi[i-1]/180*PI - 2.0/3*PI*k);
                Len_Temp[i-1][k]+=Cal_Point_Dis(P,Cur_Ang_Local[j-1][0],Cur_Ang_Local[j-1][1]);
            }
        }
    }
}


//计算控制第i个关节的第k根绳在第j个关节的绳长变化量(运动学)
// 从Cur_Ang_Local 到 Tar_Ang_Local 绳长变化量
Eigen::MatrixXd MainWindow::Cal_Len_Diff(double Tar_Ang_Local[][2], double Cur_Ang_Local[][2])
{
    double Sec_Phi[3] = {360,320,280};
    Eigen::MatrixXd Len_Diff=Eigen::MatrixXd::Zero(3,3);
    double P[3]={0};
    double Len_Temp=0;
    int i,j,k;
    for(i=0; i<SECTION_NUM; i++)//关节1-3
    {
        for(k=0;k<3;k++)//每个关节3根钢丝
        {
            for(j=0; j<=i; j++)//关节1-i的累计偏转
            {
                P[0]=Plate_Radius*cos(Sec_Phi[i]/180.0*PI - 2.0/3*PI*k);//.0不能省 否则按int计算
                P[1]=Plate_Radius*sin(Sec_Phi[i]/180.0*PI - 2.0/3*PI*k);
                P[2]=0;
                Len_Temp -= Cal_Point_Dis(P,-Tar_Ang_Local[j][0],Tar_Ang_Local[j][1]) - Cal_Point_Dis(P,-Cur_Ang_Local[j][0],Cur_Ang_Local[j][1]);
            }
            Len_Diff(i,k)=Len_Temp;
            Len_Temp=0;
        }
    }
    return Len_Diff;
}




// 导出 力传感器和编码器 数据
void MainWindow::saveDataToFile(const QString& filename)
{
    // std::ofstream outFile("../data_output/force_joint_data.txt", std::ios::app);// 以追加模式打开文件
    // if (!outFile.is_open())
    // {
    //     std::cerr << "错误：无法打开文件 " << std::endl;
    //     return;
    // }
    // outFile << std::fixed << std::setprecision(6);// 设置输出格式：固定小数点，保留6位小数
    // for (int i = 0; i < 3; ++i)
    //     outFile << forceArray[i][0]<< " " << forceArray[i][1]<< " " << forceArray[i][2]<< " ";
    // for (int i = 0; i < 3; ++i)
    //     outFile << Cur_Ang[i][0]<< " " << Cur_Ang[i][1];
    // outFile << std::endl;// 完成一行后换行
    // outFile.close();// 关闭文件
    // cout<<"success write"<<endl;


    if (DataContainer.empty())
    {
        qWarning() << "数据容器为空，没有数据可保存";
        return;
    }

    QFile file(filename);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        qWarning() << "无法打开文件:" << filename;
        return;
    }

    QTextStream stream(&file);
    stream.setRealNumberPrecision(6);  // 设置6位小数

    // 写入数据头（可选）
    // stream << "# 数据保存时间: "<< QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss")<< "\n";
    // stream << "# 总记录数: " << forceDataContainer.size() << "\n";

    // 写入所有数据行
    for (const auto& row : DataContainer)
    {
        for (size_t i = 0; i < row.size(); ++i)
        {
            stream << row[i];
            if (i < row.size() - 1)
                stream << " "; // 空格分隔
        }
        stream << "\n";
    }
    file.close();

    qDebug() << "已保存" << DataContainer.size() << "组数据到文件:" << filename;
    DataContainer.clear(); // 可选：保存后清空容器
}

// 添加数据到容器
void MainWindow::accumulateForceData()
{
    std::array<double, 15> rowData;
    // force
    for (int i = 0; i < 3; ++i)
    {
        rowData[i * 3] = Force_array[i][0];
        rowData[i * 3 + 1] = Force_array[i][1];
        rowData[i * 3 + 2] = Force_array[i][2];
    }
    // joint angle
    for (int i = 0; i < 3; ++i)
    {
        rowData[i * 2 + 9] = Cur_Ang[i][0];
        rowData[i * 2 + 10] = Cur_Ang[i][1];
    }
    DataContainer.push_back(rowData);// 添加到容器
    qDebug() << "已添加第" << DataContainer.size() << "组数据到容器";
}


// 检查项: 角度传感器 传感器接收 PPM模式 力超限
inline bool MainWindow::check_all()
{
    if (!Record_Angle_Flag)
    {
        SetTip("不能运动，未开启角度传感器");
        return false;
    }
    if(!PPM_Flag)//PPM模式
    {
        SetTip("不能运动，电机未进入PPM模式");
        return false;
    }
    if(!Feedback_Recv_Flag)//数据接收完整
    {
        SetTip("不能运动，数据接收不完整");
        return false;
    }
    if(!Force_Check() )//拉力监控
    {
        SetTip("不能运动，张力超限");
        return false;
    }
    return true;
}

// 角度回零后对驱动绳张力重新调整 仅在回零时启动
bool MainWindow::force_refine(Eigen::MatrixXd& secDrivenLength,int i)
{
    bool Force_flag = true;
    double target_force=best_force_tar[i];// 理想的目标拉力(随关节位置递减): 80N 100N 120N
    double min_force_tar=target_force-2*Force_Threshold; // 目标拉力的下限 Force_Threshold力平衡阈值 10
    double max_force_tar=target_force+2*Force_Threshold; // 目标拉力的上限
    for (int j = 0; j < 3; j++)
    {
        double force = fabs(Force_array[i][j]);
        if(force < min_force_tar)//若最小拉力钢丝绳不满足力平衡  对最小拉力钢丝绳进行驱动
        {
            secDrivenLength(i,j) =P_Force * ( target_force - force );
            Force_flag = false;
        }
        else if (force > max_force_tar)
        {
            secDrivenLength(i,j) = -P_Force * ( force - target_force );
            Force_flag = false;
        }
    }
    return Force_flag;
}

//从离线路径中加载单步运动目标
inline void MainWindow::read_trajectory_1step()
{
    if (trajectory_count < Planned_Frame)//Planned_Frame存放离线路径的总帧数
    {
        for (int i=0;i<SECTION_NUM;i++)//Motion_Data 6个关节规划值
        {
            JointMove_TarAng[i][0] = Motion_Data[trajectory_count][2*i] * 180.0 / PI;
            JointMove_TarAng[i][1] = Motion_Data[trajectory_count][2*i+1] * 180.0 / PI;
        }
        ui->Planned_Frame_Num->setText(QString::number(trajectory_count+1));
        trajectory_count++;
        Joint_Move_Flag = true;
    }
    else
    {
        SetTip("离线路径读取完毕");
        trajectory_count = 0;
        Planned_Motion_Flag = false;
        Joint_Move_Flag = false;
    }
}

// 控制机械臂到达某个目标 Timer_Reach_Target定时器按照 触发
void MainWindow::Reach_Target()
{
    if (!check_all())// 检查项: 角度传感器 传感器接收 PPM模式 力超限
        return;

    ui->label_Ang_Threshold_read->setText(QString::number(tarAng_delta_thr,'f',3));
    ui->label_P_ZeroForce_read->setText(QString::number(P_ZeroForce,'f',5));
    ui->label_P_Angle_read->setText(QString::number(P_Angle,'f',4));
    ui->label_P_Force_read->setText(QString::number(P_Force,'f',4));
    ui->label_Max_Tension_read->setText(QString::number(Max_Tension,'f',2)); //最大拉力
    ui->label_Force_Threshold_read->setText(QString::number(Force_Threshold,'f',2));//力平衡阈值
    ui->setZero_cur_stage_lable->setText(QString::number(setZero_cur_stage));//显示当前进行回零的关节号

    quint16 i, j;
    double Ang_Diff_Max = 7.0/ (1000.0/CONTROL_PERIOD); //最大运动速度7度/s，折算出每帧最大运动量 7 degree/ 20Hz
    Eigen::MatrixXd secDrivenLength = Eigen::MatrixXd::Zero(SECTION_NUM,3);//钢丝绳驱动长度 默认设置钢丝绳驱动量为0（不需要驱动）
    double Tar_Ang_P[SECTION_NUM][2];
    memset(Tar_Ang_P, 0, sizeof (Tar_Ang_P));
    bool Emit_Flag = false;//防止多次Emit信号的标志位

    // 离线路径
    if(Planned_Motion_Flag)
        read_trajectory_1step(); //从离线路径中加载单步运动目标

    //------ 运动模式 -------：关节运动到JointMove TarAng位置
    if(Joint_Move_Flag)
    {
        for(i=SEC_START_INDEX; i<SECTION_NUM; i++)
        {
            Tar_Ang[i][0] = (fabs(JointMove_TarAng[i][0] - Cur_Ang[i][0]) > tarAng_delta_thr) ? JointMove_TarAng[i][0] : Cur_Ang[i][0];
            Tar_Ang[i][1] = (fabs(JointMove_TarAng[i][1] - Cur_Ang[i][1]) > tarAng_delta_thr) ? JointMove_TarAng[i][1] : Cur_Ang[i][1];
        }
    }
    //------ 回零模式 ---------：指定范围(Sec Setzero Startindex 到 Sec Setzero Endindex)的关节运动至0位
    else if (Setzero_Move_Flag)
    {
        for(i=SEC_START_INDEX; i<SECTION_NUM; i++)
        {
            if (i >= Sec_Setzero_Startindex && i <= setZero_cur_stage)//当前正在处理的关节 和已经回零的关节 保持零位不动
            {
                JointMove_TarAng[i][0] = 0;
                JointMove_TarAng[i][1] = 0;
                Tar_Ang[i][0] = (fabs(JointMove_TarAng[i][0] - Cur_Ang[i][0]) > tarAng_delta_thr) ? JointMove_TarAng[i][0] : Cur_Ang[i][0];
                Tar_Ang[i][1] = (fabs(JointMove_TarAng[i][1] - Cur_Ang[i][1]) > tarAng_delta_thr) ? JointMove_TarAng[i][1] : Cur_Ang[i][1];
            }
            else//尚未轮到的关节 保持当前角度不变
            {
                Tar_Ang[i][0] = Cur_Ang[i][0];
                Tar_Ang[i][1] = Cur_Ang[i][1];
            }
        }

        // 递增当前回零阶段（准备处理下一组关节）
        if (setZero_cur_stage <= Sec_Setzero_Endindex)
            if(fabs(Cur_Ang[setZero_cur_stage][0]) <= tarAng_delta_thr && fabs(Cur_Ang[setZero_cur_stage][1]) <= tarAng_delta_thr)
                setZero_cur_stage++;

        //处于回零模式 且 所有需回零关节均已到位
        if (setZero_cur_stage > Sec_Setzero_Endindex)
        {
            bool Zero_Reached_Flag = true;
            int failed_index = -1;
            for(i = Sec_Setzero_Startindex; i <= Sec_Setzero_Endindex; i++)
            {
                if(fabs(Cur_Ang[i][0]) > tarAng_delta_thr || fabs(Cur_Ang[i][1]) > tarAng_delta_thr)
                {
                    Zero_Reached_Flag = false;
                    failed_index = i;
                    break;
                }
                if(control_mode == 2)
                    force_refine(secDrivenLength, i);// 角度回零后对驱动绳张力重新调整 仅在回零时启动
            }

            if(Zero_Reached_Flag)// ​全域验证​​：检查所有回零关节段是否真正到达零位
            {
                Setzero_Move_Flag = false;
                SetTip("角度回零已完成");
                on_SetZero_pushButton_clicked();
            }
            else
                setZero_cur_stage = failed_index;
        }
    }
    //------ 无模式 保持现状 -------
    else
    {
        for(i=SEC_START_INDEX; i<SECTION_NUM; i++)
        {
            Tar_Ang[i][0] = Cur_Ang[i][0];
            Tar_Ang[i][1] = Cur_Ang[i][1];
        }
    }

    cout<<"target angle= "<<Tar_Ang[0][0]<<" "<<Tar_Ang[0][1]<<" "<<Tar_Ang[1][0]<<" "<<Tar_Ang[1][1]<<" "<<Tar_Ang[2][0]<<" "<<Tar_Ang[2][1]<<endl;



    // /////////////////P控制位置-----计算驱动量/////////////////////////////
    // 根据目标角度计算P
    double P_dyn = 0;
    for (i = SEC_START_INDEX; i < SECTION_NUM; i++)
    {
        for (j = 0; j < 2; j++)
        {
            double Ang_delta = Tar_Ang[i][j] - Cur_Ang[i][j];
            if(fabs(Ang_delta) >= tarAng_delta_thr && fabs(Ang_delta) < 2)// 屏蔽 大于2度 过小 的角度误差
                P_dyn = P_Angle;//固定比例项系数 0.7
            else
                P_dyn = 0;

            Tar_Ang_P[i][j] =Cur_Ang[i][j] + Ang_delta * P_dyn + Pre_Angle_Diff[i][j] * I_Angle;//I 0.002
            Pre_Angle_Diff[i][j] += Ang_delta;//从时间0到当前时刻 误差的累积
            if(Pre_Angle_Diff[i][j] > Pre_Angle_Diff_Max) Pre_Angle_Diff[i][j] = Pre_Angle_Diff_Max;// 角度误差累积限幅
            if(Pre_Angle_Diff[i][j] < -Pre_Angle_Diff_Max) Pre_Angle_Diff[i][j] = -Pre_Angle_Diff_Max;

            double Tar_Ang_P_delta = Tar_Ang_P[i][j] - Cur_Ang[i][j]; // 调整后的目标位置变化量
            if (fabs(Tar_Ang_P_delta) >= Ang_Diff_Max)// 输出限幅：防止单步变化过大
            {
                const int sign = (Tar_Ang_P_delta > 0) ? 1 : -1;// 计算符号
                Tar_Ang_P[i][j] = Cur_Ang[i][j] + sign * Ang_Diff_Max;;// 限制调整后的目标位置
            }
        }
    }

        // /////////////正运动学////////////////////
        secDrivenLength = Cal_Len_Diff(Tar_Ang_P, Cur_Ang);
        // cout <<"运动学绳子变化量(mm):";
        // for (int row = 0; row < 3; row++)
        //     cout << secDrivenLength(row,0) << " "<< secDrivenLength(row,1) << " "<< secDrivenLength(row,2) << " ";
        // cout << endl;



    if(Save_Force_Flag)
    {
        accumulateForceData();
        // saveDataToFile(); // 导出力传感器数据
    }

    //回零过程中----非控制关节不发送驱动量
    // if(Sec_Setzero_Startindex != 0 && Sec_Setzero_Endindex !=0 && Setzero_Move_Flag)
    // {
    //     for (int i = SEC_START_INDEX; i < SECTION_NUM; i++)
    //     {
    //         for (int j = 0; j < 3; j++)
    //         {
    //             if (i < Sec_Setzero_Startindex-1 || i > Sec_Setzero_Endindex-1)
    //                 secDrivenLength(i,j) = 0;
    //         }
    //     }
    // }

    // /////////////////发送驱动量//////////////////
    if (!Emit_Flag&&STM32_Flag)
    {
        if(Setzero_Move_Flag || Joint_Move_Flag )
            emit (Send_Data(secDrivenLength, 0.0));
        else
        {
            secDrivenLength = Eigen::MatrixXd::Zero(SECTION_NUM,3);
            emit(Send_Data(secDrivenLength,0));
        }
        Feedback_Recv_Flag = false;
        Emit_Flag = true;
    }
    // }

    //条件: F407已连接 不丢包 不回零 不关节运动 不离线路径
    if ( (!Emit_Flag&&STM32_Flag) && Feedback_Recv_Flag && !Setzero_Move_Flag && !Joint_Move_Flag && !Planned_Motion_Flag )
    {
        Feedback_Recv_Flag = false;
        secDrivenLength = Eigen::MatrixXd::Zero(SECTION_NUM,3);
        emit(Send_Data(secDrivenLength,0));
        Emit_Flag = true;
    }

}




// 按下连接stm32F407按钮：更新电机位置​​、更新显示关节角,关节角超限检测
void MainWindow::on_STM32_Con_Button_clicked()
{
    if(!STM32_Flag)
    {
        setMotorInfo();//更新显示电机位置
        setAngInfo();//更新显示​​关节角,关节角超限检测
        // 检查局域网通信是否正常
        // if(QProcess::execute("ping 192.168.1.251 -w 1")||QProcess::execute("ping 192.168.1.252 -w 1")||QProcess::execute("ping 192.168.1.253 -w 1")||QProcess::execute("ping 192.168.1.254 -w 1"))
        // {
        //     QMessageBox::information(NULL, "Information", "Ping faild ,check Stm32!");
        //     return;
        // }
        emit(STM32_Init());
    }
    else //断开链接
    {
        emit(STM32_Discon());
        ui->STM32_Con_Button->setText("STM32_CON");
        ui->Record_Angle_Open_pushButton->setEnabled(false);
        ui->Record_Angle_Open_pushButton->setText("打开");
        Record_Angle_Flag = false;
        STM32_Flag=false;
    }
}

// 按下连接485按钮
void MainWindow::on_serialCon_pushButton_clicked()
{
    if(ui->serialCon_pushButton->text()=="RS485_Con")// 按下按钮 连接485
    {
        // bool flag1= m_serial->connectSerialServer();
        // bool flag2= m_serial->isConnected();
        if(m_serial->connectSerialServer() && m_serial->isConnected())
        {
            ui->serialCon_pushButton->setText("断开");
            SetTip("远程485模块连接成功！");
            ui->forceRead_pushButton->setEnabled(true);
        }
    }
    else//按下按钮 断开485连接
    {
        if(!m_serial->connectSerialServer()){
            ui->serialCon_pushButton->setText("RS485_Con");

            Timer_forceRead->stop();//关闭定时器 不再触发读力传感器
            ui->forceRead_pushButton->setText("读取");
            SetTip("远程485模块主动断开连接！");
            ui->forceRead_pushButton->setEnabled(false);
        }

    }
}

// 按下读力传感器按钮
void MainWindow::on_forceRead_pushButton_clicked()
{
    if(ui->serialCon_pushButton->text()=="断开")//已连接485 此时按钮切换为断开
    {
        if( ui->forceRead_pushButton->text()=="读取"){
            Timer_forceRead->start(FORCE_PERIOD);//定时器开启 每50ms触发forceread()
            ui->forceRead_pushButton->setText("关闭");

        }else{
            Timer_forceRead->stop();
            ui->forceRead_pushButton->setText("读取");
            //ui->force_lineEdit->clear();
        }
    }
    else{//尚未连接485 按钮仍然是连接485
        QMessageBox::information(nullptr,"提示","请先连接远程485串口！");
    }
}

// 读变送器  发送“读取CH1-9的所有数据”的对应指令  1个变送器，每个上CH1-9共9个力传感器
// 由定时器触发
void MainWindow::forceRead()
{
    // 读第一块变送器  发送“读取CH1-9的所有数据”的对应指令
    QByteArray data1= QByteArray::fromHex("010303000012C583");
    m_serial->sendSerialMsg("",data1);
    // msecSleep(100);
}

//接收力传感器信号的槽函数
void MainWindow::SetForce(QByteArray buffer,int index)
{
    // quint8 sensor_index[SECTION_NUM*3]={1,2,3,4,5,6,7,8,9};//根据电机排布定义

    // 先不管是哪块转接板，先读下来数据
    double forceArry[9];//每个转接板上有9个力传感器数据
    QString str=QString(buffer.toHex());
    bool ok;
    for (int i = 0; i < 9; i++)
    {
        QString tmp_str =str.mid(i*8,8);//将力传感器信号存入tmp_str

        if(tmp_str.length()==8)
        {
            int  num = tmp_str.toLong(&ok,16);
            forceArry[i]=num;
        }
        else{
            return;
        }
    }

    //按照转接板编号存储
        for (int i = 0; i < 9; i++)
        {
            Serial_dispCH[i]=forceArry[i]/100;//因为读到的long型数据没带小数点
        }


    for(int i =SEC_START_INDEX;i < SECTION_NUM;i++)
    {
        for(int j=0;j<3;j++)
        {
            Force_array[i][j] =Serial_dispCH[j+i*SECTION_NUM]*10 -Force_array_bias[i][j];//原始数据为kg 这边转换为N
            QLineEdit* text_disp = this->findChild<QLineEdit*>("force_lineEdit_"+QString::number(i+j*SECTION_NUM+1));
            text_disp->setText(QString::number(Force_array[i][j],'f',3));
        }
    }

    // QVector<double> vec;
    // vec.reserve(9);
    // for (int i = SEC_START_INDEX; i < 3; i++)
    //     for (int j = 0; j < 3; j++)
    //         vec.append(Force_array[i][j]);
    // qDebug() << "Force_array:" << vec;

}

//张力安全检测
bool MainWindow::Force_Check()
{
    // 进行本轮张力监控
    bool Tension_alert_flag=false;
    for (int i=0; i<9; i++) {
        if(fabs(Force_array[i%3][i/3])>Max_Tension)//力超限
        {
            QLineEdit* force_lineEdit = this->findChild<QLineEdit*>("force_lineEdit_"+QString::number(i+1));
            force_lineEdit->setStyleSheet("QLineEdit {color: red;}");
            Tension_alert_flag=true;

        }
        else
        {
            QLineEdit* force_lineEdit = this->findChild<QLineEdit*>("force_lineEdit_"+QString::number(i+1));
            force_lineEdit->setStyleSheet("QLineEdit {color: black;}");
        }
    }

    if(!Tension_alert_flag)// 所有张力正常
    {
        if(Tension_alert_send_flag)//上次张力监控时张力超限
        {
            Tension_alert_send_flag=false;
            SetTip("张力恢复，运动继续！");
        }
        return true;
    }
    else// 存在超限张力
    {
        if(!Tension_alert_send_flag)//上次张力监控时张力没超限，这是首次超限
        {
            QMessageBox::warning(NULL, "警告", "最大张力超限！");
            Sec_Setzero_Startindex=0;// 重置回零运动参数
            Sec_Setzero_Endindex=0;
            ui->SetZero_pushButton->setText("启动");

            Setzero_Move_Flag=false;//回零运动状态
            setZero_cur_stage = Sec_Setzero_Startindex;
            ui->change_controller->setEnabled(true);
            SetTip("最大张力超限，运动已停止！");
            Tension_alert_send_flag=true;
        }
        return false;
    }
}

void MainWindow::SecMotor_Send()
{
    int secNum = ui->Sec_Num_spinBox->text().toInt();
    int ropeNum = ui->Rope_Num_spinBox->text().toInt();
    int secNum_Start = ui->Start_Sec_spinBox->value();
    int secNum_End = ui->End_Sec_spinBox->value();


    //持续拉绳
    if (Continue_Pull_Flag||Continue_Loose_Flag){
        Eigen::MatrixXd Len_Temp=Eigen::MatrixXd::Zero(SECTION_NUM,3);
        int speed_level = ui->Test_Mortor_Speed_Level->text().toInt();

        if (Continue_Pull_Flag && !Continue_Loose_Flag){
            if (ui->Test_Single_checkBox->isChecked()){
                Len_Temp(secNum-1,ropeNum-1)=Step_Test*speed_level;//Len_Temp相对位置驱动
            }
            else if (ui->Test_Multiple_checkBox->isChecked()){
                for(int i = secNum_Start - 1; i < secNum_End; i++)
                    for(int j = 0; j < 3; j++)
                        Len_Temp(i, j) = Step_Test*speed_level;
            }
        }
        else if (!Continue_Pull_Flag && Continue_Loose_Flag){
            if (ui->Test_Single_checkBox->isChecked()){
                Len_Temp(secNum-1,ropeNum-1)=-Step_Test*speed_level;//Len_Temp相对位置驱动
            }
            else if (ui->Test_Multiple_checkBox->isChecked()){
                for(int i = secNum_Start - 1; i < secNum_End; i++)
                    for(int j = 0; j < 3; j++)
                        Len_Temp(i, j) = -Step_Test*speed_level;
            }
        }
        emit(Send_Data(Len_Temp,0));

    }
}


// 右上角开启角度传感器按钮
void MainWindow::on_Record_Angle_Open_pushButton_clicked()
{
    if (Record_Angle_Flag){
        emit(sig_angle(0));
        ui->Record_Angle_Open_pushButton->setText("打开");
        Record_Angle_Flag = false;
    }
    else if(!Record_Angle_Flag) {
        emit(sig_angle(1));
        ui->Record_Angle_Open_pushButton->setText("关闭");
        Record_Angle_Flag = true;
    }
}





// --------------------- 辅助功能相关 -------------------------
// 第2页 辅助功能 点动拉伸
void MainWindow::on_secMotor_Pull_Button_clicked()
{
    if(ui->forceRead_pushButton->text()=="读取")
    {
        QMessageBox::information(nullptr,"警告","请打开拉力传感器后再试！");
        return;
    }

    double single_step_Length = ui->Actuation_Val_doubleSpinBox->text().toDouble();
    int secNum = ui->Sec_Num_spinBox->text().toInt();
    int ropeNum = ui->Rope_Num_spinBox->text().toInt();
    int secNum_Start = ui->Start_Sec_spinBox->value();
    int secNum_End = ui->End_Sec_spinBox->value();

    if((single_step_Length > 2)||(single_step_Length < 0))
    {
        QMessageBox::information(NULL, "Information", "点动长度应在0-2mm范围内");
    }
    else
    {
        int delay_ms =200 + (int)(single_step_Length * 500.0); //1000;
        Timer_Reach_Target->stop();
        ui->secMotor_Pull_Button->setEnabled(false);
        ui->secMotor_Loose_Button->setEnabled(false);

        Eigen::MatrixXd Len_Temp = Eigen::MatrixXd::Zero(SECTION_NUM,3);
        if (ui->Test_Single_checkBox->isChecked()){
            Len_Temp(secNum-1,ropeNum-1)=single_step_Length;//Len_Temp相对位置驱动
        }
        else if (ui->Test_Multiple_checkBox->isChecked()){
            for(int i = secNum_Start - 1; i < secNum_End; i++)
                for(int j = 0; j < 3; j++)
                    Len_Temp(i, j) = single_step_Length;
        }

        emit(Send_Data(Len_Temp,0));

        msecSleep(delay_ms);
        ui->secMotor_Pull_Button->setEnabled(true);
        ui->secMotor_Loose_Button->setEnabled(true);
        Total_Start();//开始触发reach_target函数
    }
}

// 第2页 辅助功能 点动拉伸
void MainWindow::on_secMotor_Loose_Button_clicked()
{
    if(ui->forceRead_pushButton->text()=="读取")
    {
        QMessageBox::information(nullptr,"警告","请打开拉力传感器后再试！");
        return;
    }

    double single_step_Length = ui->Actuation_Val_doubleSpinBox->text().toDouble();
    int secNum = ui->Sec_Num_spinBox->text().toInt();
    int ropeNum = ui->Rope_Num_spinBox->text().toInt();
    int secNum_Start = ui->Start_Sec_spinBox->value();
    int secNum_End = ui->End_Sec_spinBox->value();

    if((single_step_Length > 2)||(single_step_Length < 0))
    {
        QMessageBox::information(NULL, "Information", "点动长度应在0-2mm范围内");
    }
    else
    {
        int delay_ms =200 + (int)(single_step_Length * 500.0); //1000;
        Timer_Reach_Target->stop();
        ui->secMotor_Pull_Button->setEnabled(false);
        ui->secMotor_Loose_Button->setEnabled(false);

        Eigen::MatrixXd Len_Temp = Eigen::MatrixXd::Zero(SECTION_NUM,3);

        if (ui->Test_Single_checkBox->isChecked())
        {
            Len_Temp(secNum-1,ropeNum-1)=-single_step_Length;//Len_Temp相对位置驱动
        }
        else if (ui->Test_Multiple_checkBox->isChecked())
        {
            for(int i = secNum_Start - 1; i < secNum_End; i++)
                for(int j = 0; j < 3; j++)
                    Len_Temp(i, j) = -single_step_Length;
        }

        emit(Send_Data(Len_Temp,0));

        msecSleep(delay_ms);
        ui->secMotor_Pull_Button->setEnabled(true);
        ui->secMotor_Loose_Button->setEnabled(true);
        Total_Start();//开始触发reach_target函数
    }
}


// 第2页 辅助功能 持续拉伸
void MainWindow::on_secMotor_ContinuePull_Button_clicked()
{
    if(ui->forceRead_pushButton->text()=="读取")
    {
        QMessageBox::information(nullptr,"警告","请打开拉力传感器后再试！");
        return;
    }

    Continue_Pull_Flag = true;
    Continue_Loose_Flag = false;

    Timer_Reach_Target->stop();
    Timer_secMotor->start(CONTROL_PERIOD);
}

// 第2页 辅助功能 持续放松
void MainWindow::on_secMotor_ContinueLoose_Button_clicked()
{
    if(ui->forceRead_pushButton->text()=="读取")
    {
        QMessageBox::information(nullptr,"警告","请打开拉力传感器后再试！");
        return;
    }

    Continue_Pull_Flag = false;
    Continue_Loose_Flag = true;

    Timer_Reach_Target->stop();
    Timer_secMotor->start(CONTROL_PERIOD);
}

// 第2页 辅助功能 停止持续放松
void MainWindow::on_secMotor_Stop_Button_clicked()
{
    Continue_Pull_Flag = false;
    Continue_Loose_Flag = false;
    Timer_secMotor->stop();
    Total_Start();//开始触发reach_target函数
}

// 第2页辅助功能 设定角度后,将该角度设为当前值
void MainWindow::on_angSet_pushButton_clicked()
{
    if(!Record_Angle_Flag)
    {
        QMessageBox::information(nullptr,"警告","请先打开角度传感器，确保角度数据读取正常后再试！");
        return;
    }

    for (int i = SEC_START_INDEX; i < SECTION_NUM; ++i) {
        QLineEdit* text_disp_h = this->findChild<QLineEdit*>("Tar_Hor_Ang"+QString::number(i+1));
        QLineEdit* text_disp_v = this->findChild<QLineEdit*>("Tar_Ver_Ang"+QString::number(i+1));
        text_disp_h->setText(QString::number(Cur_Ang[i-1][0],'f',4));
        text_disp_v->setText(QString::number(Cur_Ang[i-1][1],'f',4));
    }
}

// 第2页辅助功能 设定角度后,运动至该目标 按钮
void MainWindow::on_angStart_pushButton_clicked()
{
    if (!check_all())// 检查项: 角度传感器 传感器接收 PPM模式 力超限
        return;

    if(ui->angStart_pushButton->text()=="运动至目标")
    {
        for (int i = SEC_START_INDEX; i < SECTION_NUM; ++i) {

            QLineEdit* tarAng_h = this->findChild<QLineEdit*>("Tar_Hor_Ang"+QString::number(i+1));
            QLineEdit* tarAng_v = this->findChild<QLineEdit*>("Tar_Ver_Ang"+QString::number(i+1));

            JointMove_TarAng[i][0]=tarAng_h->text().toDouble();
            JointMove_TarAng[i][1]=tarAng_v->text().toDouble();
        }

        emit(sig_PPM());
        Joint_Move_Flag=true;
        ui->angStart_pushButton->setText("停止");
    }
    else
    {
        for(int i=SEC_START_INDEX;i<SECTION_NUM;i++)
        {
            for(int j=0;j<1;j++)
            {
                JointMove_TarAng[i][j]=Cur_Ang[i][j];
            }
        }

        Joint_Move_Flag=false;
        ui->angStart_pushButton->setText("运动至目标");
    }


}

// --------------------- 回零按钮相关 -------------------------
// 按下 切换回零模式 按钮 更改control_mode变量
void MainWindow::on_change_controller_clicked()
{
    // 选择 角度回零 还是 力平衡回零
    if(ui->current_controller->text()=="角度回零")
    {
        ui->current_controller->setText("力平衡");
        control_mode=2;
    }
    else
    {
        ui->current_controller->setText("角度回零");
        control_mode=1;
    }
}

//按下回零 启动 按钮触发该函数
void MainWindow::on_SetZero_pushButton_clicked()
{

    emit(sig_PPM());//回零前先修改maxon电机控制模式为位置模式
    msecSleep(50);

    if (!check_all())// 检查项: 角度传感器 传感器接收 PPM模式 力超限
        return;

    //开始回零
    if(ui->SetZero_pushButton->text()=="启动")
    {
        Sec_Setzero_Startindex = ui->secNumStart_spinBox->text().toInt();//回零起始范围
        Sec_Setzero_Endindex = ui->secNumEnd_spinBox->text().toInt();
        ui->SetZero_pushButton->setText("停止");//“启动”按钮改为"停止"

        Setzero_Move_Flag=true;//回零运动状态
        setZero_cur_stage = Sec_Setzero_Startindex;
        // Zero_or_autoForce_Switch = false;

        if(ui->current_controller->text()=="角度回零")
            control_mode=1;
        else if(ui->current_controller->text()=="力平衡")
            control_mode=2;
        ui->change_controller->setEnabled(false);
    }
    else// 停止回零
    {
        Sec_Setzero_Startindex=0;
        Sec_Setzero_Endindex=0;

        ui->SetZero_pushButton->setText("启动");
        // Zero_or_autoForce_Switch = false;
        Setzero_Move_Flag=false;//回零运动状态
        setZero_cur_stage = Sec_Setzero_Startindex;

        ui->cur_control_mode->setText("无");
        ui->setZero_cur_stage_lable->setText("无");
        ui->change_controller->setEnabled(true);
    }
}


// ------------------ 电流模式相关 ------------------------

// 第3页 单电机电流模式
void MainWindow::on_singleRope_Current_pushButton_clicked()
{
    int Sec_Num = ui->singleRope_Cur_SecNum_spinBox->text().toInt();
    int rope1_cur_level = ui->rope1_curLevel_SpinBox->text().toInt();
    int rope2_cur_level = ui->rope2_curLevel_SpinBox->text().toInt();
    int rope3_cur_level = ui->rope3_curLevel_SpinBox->text().toInt();
    int cur_moveLevel = ui->cur_moveLevel_SpinBox->text().toInt();

    if (!MultiMotor_CurMode_Flag){
        emit(sig_singleCurrent(Sec_Num-1,rope1_cur_level, rope2_cur_level, rope3_cur_level,cur_moveLevel));
        ui->singleRope_Current_pushButton->setText("stop");
        MultiMotor_CurMode_Flag=true;
        Timer_Reach_Target->stop();
    }
    else{
        emit(sig_PPM());//修改maxon电机控制模式为位置模式
        CurMode_PPM_Flag = true;
        ui->singleRope_Current_pushButton->setEnabled(false);
        ui->singleRope_Current_pushButton->setText("电流搓紧");
        MultiMotor_CurMode_Flag=false;
        Total_Start();//开始触发reach_target函数
    }
}

// 第3页 多电机电流模式
void MainWindow::on_MultiMotor_Current_pushButton_clicked()
{
    int Sec_StartNum = ui->Cur_SecStartNum_spinBox->text().toInt();
    int Sec_EndNum = ui->Cur_SecEndNum_spinBox->text().toInt();
    int cur_level = ui->cur_level_SpinBox->text().toInt();
    int cur_speed_level= ui->cur_moveLevel_SpinBox_2->text().toInt();

    if (!MultiMotor_CurMode_Flag){
        emit(sig_current(Sec_StartNum-1,Sec_EndNum-1, cur_speed_level, cur_level));//修改maxon电机控制模式为电流模式
        ui->MultiMotor_Current_pushButton->setText("停止");
        MultiMotor_CurMode_Flag=true;
        Timer_Reach_Target->stop();
    }
    else{
        emit(sig_PPM());//修改maxon电机控制模式为位置模式
        CurMode_PPM_Flag = true;
        ui->MultiMotor_Current_pushButton->setEnabled(false);
        ui->MultiMotor_Current_pushButton->setText("电流拉紧");
        MultiMotor_CurMode_Flag=false;
        Total_Start();//开始触发reach_target函数
    }
}


// ------------------ plan 规划相关 ------------------------


// 接收规划器给出的目标关节角 planning.cpp使用 发布规划结果
void MainWindow::Recv_Ang_Path(Eigen::VectorXd Ang_Desired)
{
    for (int i = 0;i < 24;i++)
    {
        Ang_Tele(i) = Ang_Desired(i) * 180 / PI;
    }
    Ang_Tele(24) = Ang_Desired(24);
    Tele_Ready_Flag = true;
}

// ------------------ 离线路径相关 ------------------------
// debug向终端打印预规划路径
void printMotionData(const std::vector<std::array<double, 6>>& motionData)
{
    // 设置输出格式：固定小数点，保留4位小数
    cout << fixed << setprecision(4);
    // 遍历所有帧数据
    for (size_t frame = 0; frame < motionData.size(); ++frame) {
        cout << "Frame " << setw(3) << frame << ": [";

        // 输出当前帧的6个数值
        for (size_t i = 0; i < 6; ++i) {
            cout << setw(8) << motionData[frame][i];
            if (i < 5) cout << ", ";
        }
        cout << "]" << endl;
    }
}

// 按下Planned_Motion_Button后 加载预规划路径 读取txt planned_motion数据 存入Motion_Data
void MainWindow::on_Planned_Motion_Button_clicked()
{
    //memset(Motion_Data,0,sizeof(Motion_Data));
    Motion_Data.clear();
    // 打开文件对话框，选择运动轨迹文件
    QString textPath = QFileDialog::getOpenFileName(this, "select planned motion file", ".", "text files(*.txt)");


    ifstream iFile(textPath.toStdString(), ios_base::in);
    if (!iFile.is_open())
    {
        SetTip("Failed to read planned motion data!");
        return;
    }
    string lineData;
    Planned_Frame = 0;
    SetTip("加载路径文件");
    while ((getline(iFile, lineData)))
    {
        istringstream is(lineData);// 将行数据转换为字符串流
        string str_value;
        array<double, 6> arr;// 存储每行6个double值的数组
        int count = 0;
        while (is >> str_value)// 逐个提取空格分隔的值
        {
            arr[count] = stod(str_value);
            count++;
        }
        Motion_Data.push_back(arr);
        Planned_Frame++;
    }
    iFile.close();


    printMotionData(Motion_Data);// debug

    QString temp = QString("%1").arg(Planned_Frame);
    ui->Planned_Motion_Start_Button->setEnabled(true);
    ui->Planned_Frame_Total->setText(temp);
}

// 加载预规划路径 开始 按钮 显示路径长度+设置规划flag
void MainWindow::on_Planned_Motion_Start_Button_clicked()
{
    ui->Planned_Motion_Start_Button->setEnabled(false);
    ui->Planned_Motion_Stop_Button->setEnabled(true);
    Planned_Motion_Flag=true;
    Joint_Move_Flag=true;
}

void MainWindow::on_Planned_Motion_Stop_Button_clicked()
{
    ui->Planned_Motion_Start_Button->setEnabled(true);
    ui->Planned_Motion_Stop_Button->setEnabled(false);
    Planned_Motion_Flag = false;
}


void MainWindow::on_set_Ang_Threshold_btn_clicked()
{
    if(!Record_Angle_Flag)
    {
        QMessageBox::information(nullptr,"警告","请打开角度传感器后再试！");
        return;
    }

    double tarAng_delta_thr_tmp = ui->label_Ang_Threshold->text().toDouble();
    tarAng_delta_thr=tarAng_delta_thr_tmp;

}


void MainWindow::on_set_P_ZeroForce_btn_clicked()
{
    if(!Record_Angle_Flag)
    {
        QMessageBox::information(nullptr,"警告","请打开角度传感器后再试！");
        return;
    }

    double P_ZeroForce_tmp = ui->label_P_ZeroForce->text().toDouble();
    P_ZeroForce=P_ZeroForce_tmp;
}


void MainWindow::on_set_P_Angle_btn_clicked()
{
    if(!Record_Angle_Flag)
    {
        QMessageBox::information(nullptr,"警告","请打开角度传感器后再试！");
        return;
    }

    double P_Angle_tmp = ui->label_P_Angle->text().toDouble();
    if(P_Angle_tmp<0||P_Angle_tmp>1){
        QMessageBox::warning(NULL, "警告", "P_Angle超限！");
    }
    else{
        P_Angle=P_Angle_tmp;
    }
}


void MainWindow::on_set_Max_Tension_btn_clicked()
{
    if(!Record_Angle_Flag)
    {
        QMessageBox::information(nullptr,"警告","请打开角度传感器后再试！");
        return;
    }

    double Max_Tension_tmp = ui->label_Max_Tension->text().toDouble();
    if(Max_Tension_tmp<0||Max_Tension_tmp>1000)
    {
        QMessageBox::warning(NULL, "警告", "Max Tension超限！");
    }
    else{
        Max_Tension=Max_Tension_tmp;
    }
}


void MainWindow::on_set_P_Force_btn_clicked()
{
    if(!Record_Angle_Flag)
    {
        QMessageBox::information(nullptr,"警告","请打开角度传感器后再试！");
        return;
    }

    double P_Force_tmp = ui->label_P_Force->text().toDouble();
    if(P_Force_tmp<0||P_Force_tmp>0.01){
        QMessageBox::warning(NULL, "警告", "Kp超限！");
    }
    else{
        P_Force=P_Force_tmp;
    }
}


void MainWindow::on_set_Force_Threshold_btn_clicked()
{
    if(!Record_Angle_Flag)
    {
        QMessageBox::information(nullptr,"警告","请打开角度传感器后再试！");
        return;
    }

    double Force_Threshold_tmp = ui->label_Force_Threshold->text().toDouble();

    if(Force_Threshold_tmp<0||Force_Threshold_tmp>30){
        QMessageBox::warning(NULL, "警告", "力平衡阈值范围0-30！");
    }
    else
    {
        Force_Threshold=Force_Threshold_tmp;
    }
}


//将用户在输入的​​新偏置值​​存入数组Tar_Bias,即时更新界面上显示的"当前偏置值"
void MainWindow::on_SetAngBias_pushButton_clicked()
{

    for (int i = 0; i < 3; ++i)
    {
        QLineEdit* cur_bias_h = this->findChild<QLineEdit*>("Cur_Hor_Ang_Bias"+QString::number(i+1));
        QLineEdit* cur_bias_v = this->findChild<QLineEdit*>("Cur_Ver_Ang_Bias"+QString::number(i+1));

        QLineEdit* new_bias_h = this->findChild<QLineEdit*>("New_Hor_Ang_Bias"+QString::number(i+1));
        QLineEdit* new_bias_v = this->findChild<QLineEdit*>("New_Ver_Ang_Bias"+QString::number(i+1));

        Tar_Bias[i][0]=new_bias_h->text().toDouble();
        Tar_Bias[i][1]=new_bias_v->text().toDouble();

        cur_bias_h->setText(QString::number(Tar_Bias[i][0],'f',2));
        cur_bias_v->setText(QString::number(Tar_Bias[i][1],'f',2));
    }

}

// ------------------ 数据导出相关 ------------------------
// 左下角记录按钮
void MainWindow::on_RecordData_pushButton_clicked()
{
    Save_Force_Flag = 1;
    ui->SaveData_pushButton->setEnabled(true);
    ui->RecordData_pushButton->setEnabled(false);
}

// 左下角存储 按钮
void MainWindow::on_SaveData_pushButton_clicked()
{
    Save_Force_Flag = 0;
    // 生成带时间戳的文件名
    QString timestamp = QDateTime::currentDateTime().toString("yyyyMMddHHmmss");
    QString fileName = QString("../data_output/force_joint_data_%1.txt").arg(timestamp);
    saveDataToFile(fileName);
    ui->RecordData_pushButton->setEnabled(true);
}



#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QProcess>
#include <QImage>
#include <QTimer>
#include <QFile>
#include <QFileDialog>
#include <QMessageBox>
#include <QThread>

#include <communication.h>
#include <serial.h>
#include <motion_planning.h>

//包含eigen矩阵库头文件
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>


#define Step_Test 0.02 //Maxon电机调试运动步长 一档速度
#define CONTROL_PERIOD 50.0 //底层控制帧数 50ms/帧
#define Sec_Dis  25.0// 导绳盘间距
#define h1 12.5      // 导绳盘间距/2 近端
#define h2 12.5      // 导绳盘间距/2 远端
#define Plate_Radius 24.5 //导绳盘半径
#define sec1Cable 0.55 //第一节钢丝绳绳长 单位m
#define Cable_Radius 0.6/1000 //钢丝绳半径 单位m
#define MAX_ANG 8000//关节角度限制


extern const int SEC_START_INDEX;//关节范围限制 0
extern const int SECTION_NUM;//总关节数 3

// extern int CLB_FLAG;

using namespace std;

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void ui_init();
    void parm_init();
    void function_connect();
    void SetTip(QString);
    void setMotorInfo();
    void setAngInfo();//​​3个关节的水平和垂直角界面更新显示,关节角度超限检测
    void Total_Stop();//禁止运动
    void Total_Start();//允许运动
    void msecSleep(int msec);

signals:

    void STM32_Init();
    void STM32_Discon();

    void Send_Data(Eigen::MatrixXd,double);
    void sig_current(int,int,int, int);
    void sig_PPM();
    void sig_angle(int);
    // void sig_clb_start(char *);
    // void sig_clb_stop();
    void sig_singleCurrent(int,int,int,int,int);
    void Tele_Caculate(double, double, const bool, const bool);

private:
    //变量
    Ui::MainWindow *ui;
    //定时器
    QTimer *Timer_secMotor, *Timer_Reach_Target,*Timer_forceRead,*Timer_Clb,*Timer_Base,*Timer_Cam1,*Timer_Cam2;
    //bool 标志位
    bool STM32_Flag;
    bool Total_Stop_Flag;
    bool Record_Angle_Flag;
    // bool PPM_Flag; //标志 电机切换为PPM位置模式
    // bool Ready_Flag; //似乎重复 也是看有没有进入ppm?????????????
    bool Feedback_Recv_Flag; // 下位机数据接收完成标志位
    bool CurMode_PPM_Flag;//电流模式下切回位置模式标志位
    bool MultiMotor_CurMode_Flag;//各电流模式互斥标志位
    bool Planned_Motion_Flag;//离线路径运动标志位
    bool Continue_Pull_Flag; //maoxn 电机调试连续拉绳标志位
    bool Continue_Loose_Flag;//maoxn 电机调试连续放松标志位
    bool Zero_or_autoForce_Switch;//回零过程中 角度回零与力配合切换标志位
    bool Setzero_Move_Flag;//回零标志位
    bool Joint_Move_Flag;//角度运动标志位
    bool Tension_alert_send_flag; //前一次张力监控标识符
    bool Tele_Ready_Flag;//在线规划完成标志位
    bool SetZero_Done_Flag;//回零完成标志位
    bool Save_Force_Flag;//1 导出力传感器数据 0 停止导出

    int control_mode;// 控制模式  1角度  2力
    int setZero_cur_stage;//reach_target 当前回零阶段
    int Sec_Setzero_Startindex; //回零起始范围
    int Sec_Setzero_Endindex;
    int Planned_Frame;//离线路径帧数


    //类实例化
    Serial *m_serial;
    // Joystick *joystick;
    Communication *communication;
    MotionPlan *Planner;

    //控制参数
    double Cur_Ang[3][2];//当前关节角度
    double Tar_Ang[3][2];//目标关节角度
    double JointMove_TarAng[3][2];//关节运动目标角度
    double Cur_Motor_Pos[3][3];//Maxon当前电机绝对位置
    // double Cur_Motor_Pos_Ready[3][3];//预紧后Maxon当前电机绝对位置
    double Tar_Bias[3][2];//角度补偿 行：关节 列：水平α 竖直β

    double Force_array_bias[3][3];//拉力传感器零位偏移量

    double Serial_dispCH[9]; //拉力传感器串口原始数据
    double Force_array[3][3];//拉力传感器数据 以关节绳号存储
     double Force_array_last[12][3];//上一帧拉力传感器数据 用于绳长补偿

    // double Driven_Len[3][3];//当前角度对应的当前关节段绳长

    double Pre_Angle_Diff[3][3];

    double Sec12_Tar_Ang[2];//末端关节水平与竖直角度存储
    double Sec12_Tar_Ang_Init[2];
    double tarAng_delta_thr;//角度回零阈值
    double P_ZeroForce;//角度回零过程中 力补偿P参数
    double P_Angle,P_Force,I_Angle; //角度回零 与力配合 p参数 i参数
    double Pre_Angle_Diff_Max; // 积分累积限幅
    double Max_Tension; //最大拉力
    double best_force_tar; //阶梯力初值
    double best_force_step;//阶梯力阶梯差值
    double Max_Motor_Loose;//放松量限制
    double  Cur_Base_Pos;//底座电机当前位置
    double base_move_length;//底座电机驱动量
    double Force_Threshold;//力平衡阈值

    vector<std::array<double, 15>> DataContainer;//存放将要输出到txt的张力和关节角

    //遥操作相关
    vector<array<double, 6> > Motion_Data;
    Eigen::VectorXd Ang_Tele;
    int Cur_Ang_Path_Frame;
    int Frame_Index_Temp;

    // /////////////////////////////////////////////////函数/////////////////////////////////////////
    bool Force_Check(double maxTension);

    //运动学部分
    double Cal_Point_Dis(double *P ,double a ,double b);
    void Cal_Len(double Cur_Ang_Local[][2], double Len_Temp[][3]);
    Eigen::MatrixXd Cal_Len_Diff(double Tar_Ang_Local[][2], double Cur_Ang_Local[][2]); //计算控制i关节的k绳 在关节j 从Cur_Ang_Local 到 Tar_Ang_Local 绳长变化量
    Eigen::MatrixXd Rot_trans(quint8 axis_idx, double Angx);
    Eigen::Vector3d Cal_Last_Point(Eigen::Vector3d Point1,Eigen::Vector3d Point2,Eigen::Vector3d Point3);
    Eigen::MatrixXd Cal_Pos(Eigen::Vector3d Base,Eigen::VectorXd Ang);
    Eigen::VectorXd Mat_Col(Eigen::MatrixXd matx, int idx);

    void accumulateForceData();// 添加数据到容器
    void saveDataToFile(const QString& filename);// 导出 力传感器和编码器 数据

private slots:

    //功能槽函数
    // 传入电机位置信号和关节编码器信号，接收电机位置信号和关节编码器信号
    void Recv_Cur_Motor_Pos(Eigen::MatrixXd Read_Cur_Motor_Pos, Eigen::MatrixXd Read_Cur_Ang);

    void Reach_Target();
    void forceRead();
    //接收传感器数据,参数类型与serial.h中的returnForce信号一致
    void SetForce(QByteArray buffer,int index);

    void SecMotor_Send();
    void Recv_Ang_Path(Eigen::VectorXd Ang_Desired);

    void Joy_L_order(QString order);



    //ui界面槽函数
    void on_STM32_Con_Button_clicked();
    // void on_calibration_Button_clicked();
    // void on_calibration_Stop_Button_clicked();
    void on_Total_Stop_Button_clicked();
    void on_Record_Angle_Open_pushButton_clicked();
    void on_forceRead_pushButton_clicked();
    // void on_joyStickCon_pushButton_clicked();
    void on_MultiMotor_Current_pushButton_clicked();
    void on_singleRope_Current_pushButton_clicked();
    void on_Planned_Motion_Start_Button_clicked();
    void on_Planned_Motion_Stop_Button_clicked();
    void on_secMotor_Pull_Button_clicked();
    void on_secMotor_Loose_Button_clicked();
    void on_secMotor_ContinuePull_Button_clicked();
    void on_secMotor_ContinueLoose_Button_clicked();
    void on_secMotor_Stop_Button_clicked();
    void on_angSet_pushButton_clicked();
    void on_angStart_pushButton_clicked();
    void on_change_controller_clicked();
    void on_SetZero_pushButton_clicked();
    // void on_Tele_Button_clicked();
    void on_Planned_Motion_Button_clicked();//按下Planned_Motion_Button后 加载预规划路径
    void on_serialCon_pushButton_clicked();
    void on_set_best_force_tar_btn_clicked();
    void on_set_P_ZeroForce_btn_clicked();
    void on_set_P_Angle_btn_clicked();
    void on_set_Ang_Threshold_btn_clicked();
    void on_set_Max_Tension_btn_clicked();
    void on_set_best_force_step_btn_clicked();
    void on_set_P_Force_btn_clicked();
    void on_set_Force_Threshold_btn_clicked();
    void on_Sec12_Move_checkBox_stateChanged(int arg1);
    void on_SetAngBias_pushButton_clicked();//将用户在输入的​​新偏置值​​存入数组Tar_Bias,即时更新界面上显示的"当前偏置值"
    // void on_force_lineEdit_1_windowIconChanged(const QIcon &icon);
    // void on_force_lineEdit_1_cursorPositionChanged(int arg1, int arg2);
    // void on_force_lineEdit_1_textChanged(const QString &arg1);
    void on_RecordData_pushButton_clicked();// 按下 记录 按钮
    void on_SaveData_pushButton_clicked();// 按下 存储 按钮
};
#endif // MAINWINDOW_H

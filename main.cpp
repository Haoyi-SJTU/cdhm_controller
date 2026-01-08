// Qt应用程序入口，主要完成初始化、加载样式表、显示主窗口并启动事件循环


#include "mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    // 创建QApplication对象，用于初始化Qt框架的底层设置（如GUI、事件循环等）。每个Qt应用有且仅有一个QApplication

    QFile styleSheetFile(":/styles/style.qss");
    //Qt资源系统路径，加载样式表
    if(styleSheetFile.open(QFile::ReadOnly))
    {
        QString styleSheet=QLatin1String(styleSheetFile.readAll());
        a.setStyleSheet(styleSheet);//全局样式，影响所有窗口和部件的外观
        styleSheetFile.close();
    }

    MainWindow w;    // 实例化主窗口​​：创建MainWindow类的对象w，通常继承自QMainWindow，包含程序的主界面
    w.show();
    return a.exec();
    //Qt的事件循环。该方法会阻塞直到所有窗口关闭，期间处理用户输入、重绘事件、信号与槽等
}

QT       += core gui network serialport
# QT       += gamepad
QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    # camera.cpp \
    communication.cpp \
    # joystick.cpp \
    main.cpp \
    mainwindow.cpp \
    motion_planning.cpp \
    serial.cpp

HEADERS += \
    # camera.h \
    communication.h \
    # joystick.h \
    mainwindow.h \
    motion_planning.h \
    serial.h

FORMS += \
    mainwindow.ui

INCLUDEPATH +=./eigen3 \
               /usr/local/include \
               # /usr/local/include/opencv2


# LIBS += -L /usr/local/lib/libopencv_highgui.so\
#         /usr/local/lib/libopencv_core.so\
#         /usr/local/lib/libopencv_imgproc.so\
#         /usr/local/lib/libopencv_calib3d.so\
#         /usr/local/lib/libopencv_videoio.so\

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

RESOURCES += \
    zlg.qrc



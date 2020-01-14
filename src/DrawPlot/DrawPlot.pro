#-------------------------------------------------
#
# Project created by QtCreator 2020-01-10T08:58:35
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport

TARGET = DrawPlot
TEMPLATE = app
CONFIG += c++11
# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

INCLUDEPATH += \
    /opt/ros/kinetic/include/opencv-3.3.1-dev \
    /home/fshs/work/trajectRecordWithRos/include \
    /opt/ros/kinetic/include \

LIBS += -L/opt/ros/kinetic/lib/x86_64-linux-gnu -lopencv_highgui3 -lopencv_core3 -lopencv_imgproc3 -lopencv_imgcodecs3
LIBS += -L/opt/ros/kinetic/lib -lroscpp -lroscpp_serialization -lroslib -lrosconsole -lmoveit_move_group_interface -ltf -lrostime

SOURCES += main.cpp\
        mainwindow.cpp \
    qcustomplot.cpp

HEADERS  += mainwindow.h \
    qcustomplot.h

FORMS    += mainwindow.ui

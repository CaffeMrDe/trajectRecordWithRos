#include "mainwindow.h"
#include <QApplication>
#include "ros/ros.h"
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    ros::init(argc,argv,"traject_polot_qt");
    ros::NodeHandle nh_;
    ros::AsyncSpinner spinner(4);
    spinner.start();

    MainWindow position;
    position.InitRosParam(nh_);
    position.show();
    return a.exec();
}

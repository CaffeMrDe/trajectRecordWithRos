#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "qcustomplot.h"
#include "TrajectData.h"
#include "memory"
#include "ros/ros.h"
#include "moveit_msgs/RobotTrajectory.h"
//#include "trajectcontrol.h"
enum TrajectType{
    Position,
    Vectory,
    accelecation,
};

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
public:
    void InitRosParam(ros::NodeHandle& nh_);
private:
    void updateCustomData(QCustomPlot *customPlot, std::shared_ptr<TrajectGroupData>& group,TrajectType type);
    bool loadLocalTrajectData(std::shared_ptr<TrajectGroupData>& group);
    void InitJointCustomPlot(QCustomPlot *customPlot);
    void jointstatesCallback(const trajectory_msgs::JointTrajectoryConstPtr& msg);
    void updateDrawWidget(std::shared_ptr<TrajectGroupData>& group);
private:
    void receiveTrajectData(std::vector<trajectory_msgs::JointTrajectoryPoint> &JointVector);
private slots:
    void on_pbn_load_clicked();

    void on_pbn_updateVal_clicked();
signals:
    void emitTrajectPoint(std::vector<trajectory_msgs::JointTrajectoryPoint> JointVector);
private:
    std::shared_ptr<TrajectGroupData> group;
    std::shared_ptr<TrajectDataIoManager> TrajectManager;
//    std::shared_ptr<trajectControl> traject;
    ros::NodeHandle nh_;
    ros::Subscriber sub;
    std::vector<trajectory_msgs::JointTrajectoryPoint> JointVector;
private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H

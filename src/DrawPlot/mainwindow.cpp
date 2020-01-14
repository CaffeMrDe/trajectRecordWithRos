#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "dbg.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    qRegisterMetaType<std::vector<trajectory_msgs::JointTrajectoryPoint>>("std::vector<trajectory_msgs::JointTrajectoryPoint>");
    ui->setupUi(this);
    InitJointCustomPlot(ui->PositionplotWidget);
    InitJointCustomPlot(ui->vecPlotWidget);
    InitJointCustomPlot(ui->accPlotWidget);

//    connect(this, &MainWindow::emitTrajectPoint, this, &MainWindow::receiveTrajectData);

}

MainWindow::~MainWindow()
{
    delete ui;
}

//update the six axis group data
void MainWindow::updateCustomData(QCustomPlot *customPlot, std::shared_ptr<TrajectGroupData>& group,TrajectType type)
{
    dbg("updateCustomData");
    // generate some points of data (jp0 for first, jp1 for second graph):
    int size = group->size();
    dbg(size);
    QVector<double>  jp0(size), jp1(size),jp2(size),jp3(size),jp4(size), jp5(size), jp6(size);
    QVector<double> x(size);
    std::vector<double> tempData;
    for (int i=0; i<size; ++i)
    {
//        dbg(i);
        switch (type) {
            case Position:
                    tempData = group->getPositionElement(i);
                break;
            case Vectory:
                    tempData = group->getVectoryElement(i);
                break;

            case accelecation:
                    tempData = group->getAccelerationElement(i);
                break;
        }
        x[i] = group->getTimeStampElement(i);
//        dbg("getTimeStampElement -- tempDatasize:",tempData.size());
        jp0[i] = tempData[0]*180/3.1415;       // exponentially decaying cosine
        jp1[i] = tempData[1]*180/3.1415;       // exponential envelope
        jp2[i] = tempData[2]*180/3.1415;
        jp3[i] = tempData[3]*180/3.1415;
        jp4[i] = tempData[4]*180/3.1415;
        jp5[i] = tempData[5]*180/3.1415;
        jp6[i] =0;

        if(type == Position)
            std::cout << "position: "<< jp0[i]<<" "<< jp1[i]<<" "<< jp2[i]<<" "<< jp3[i]<<" "<< jp4[i]<<" "<< jp5[i]<<" "<< jp6[i]<<" "<<std::endl;
    }
  // configure right and top axis to show ticks but no labels:
  // (see QCPAxisRect::setupFullAxesBox for a quicker method to do this)
  customPlot->xAxis2->setVisible(true);
  customPlot->xAxis2->setTickLabels(false);

  customPlot->yAxis2->setVisible(true);
  customPlot->yAxis2->setTickLabels(true);
  // make left and bottom axes always transfer their ranges to right and top axes:
  connect(customPlot->xAxis, SIGNAL(rangeChanged(QCPRange)), customPlot->xAxis2, SLOT(setRange(QCPRange)));
  connect(customPlot->yAxis, SIGNAL(rangeChanged(QCPRange)), customPlot->yAxis2, SLOT(setRange(QCPRange)));

  // pass data points to graphs:
  customPlot->graph(0)->setData(x, jp0);
  customPlot->graph(1)->setData(x, jp1);
  customPlot->graph(2)->setData(x, jp2);
  customPlot->graph(3)->setData(x, jp3);
  customPlot->graph(4)->setData(x, jp4);
  customPlot->graph(5)->setData(x, jp5);
  customPlot->graph(6)->setData(x, jp6);
  // let the ranges scale themselves so graph 0 fits perfectly in the visible area:
  customPlot->graph(0)->rescaleAxes();
  // same thing for graph 1, but only enlarge ranges (in case graph 1 is smaller than graph 0):
  customPlot->graph(0)->rescaleAxes(true);
  customPlot->graph(1)->rescaleAxes(true);
  customPlot->graph(2)->rescaleAxes(true);
  customPlot->graph(3)->rescaleAxes(true);
  customPlot->graph(4)->rescaleAxes(true);
  customPlot->graph(5)->rescaleAxes(true);
  customPlot->graph(6)->rescaleAxes(true);
  // Note: we could have also just called customPlot->rescaleAxes(); instead
  // Allow user to drag axis ranges with mouse, zoom with mouse wheel and select graphs by clicking:
  customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);

  dbg("finish updateCustomData");
}

bool MainWindow::loadLocalTrajectData(std::shared_ptr<TrajectGroupData> &group)
{
    TrajectManager = std::make_shared<TrajectDataIoManager>();
    bool rtn = TrajectManager->readData("/home/fshs/work/trajectRecordWithRos/build/devel/lib/trajectory_test/");

    group = TrajectManager->getDataHandler();

    return rtn;
}

void MainWindow::InitJointCustomPlot(QCustomPlot *customPlot)
{

    QStringList lineNames;
    lineNames << "joint1" << "joint2" << "joint3" << "joint4" << "joint5" << "joint6"<< "joint7";
    QPen pen;
    customPlot->legend->setVisible(true);
    customPlot->legend->setFont(QFont("Helvetica", 9));

    for( int i= 0; i < 7; i++){
        customPlot->addGraph();
        pen.setColor(QColor(qSin(i*1+1.2)*80+80, qSin(i*0.3+0)*80+80, qSin(i*0.3+1.5)*80+80));
        customPlot->graph()->setPen(pen); // line color red for second graph
        customPlot->graph()->setLineStyle(QCPGraph::lsLine);
        customPlot->graph()->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCross, 10));
        customPlot->graph()->setName(lineNames.at(i));

    }

}

void MainWindow::InitRosParam(ros::NodeHandle& nh_)
{
    this->nh_ = nh_;
    //listener must have handler
    sub = nh_.subscribe("/joint_path_command", 1, &MainWindow::jointstatesCallback, this);
    ROS_INFO(" InitRosParam ");

}

void MainWindow::jointstatesCallback(const trajectory_msgs::JointTrajectoryConstPtr& msg)
{

  JointVector = msg->points;
  std::cout <<" jointstatesCallback size: "<<JointVector.size()<<std::endl;
  receiveTrajectData(JointVector);
//  emitTrajectPoint(JointVector);
//  ROS_INFO("time : %s ",msg->header.stamp.toSec());
}

void MainWindow::updateDrawWidget(std::shared_ptr<TrajectGroupData> &group)
{
    std::cout <<" updateDrawWidget group: "<<group->size()<<std::endl;
    updateCustomData(ui->PositionplotWidget, group, Position);
    updateCustomData(ui->vecPlotWidget, group, Vectory);
    updateCustomData(ui->accPlotWidget, group, accelecation);
    ui->PositionplotWidget->replot();
    ui->vecPlotWidget->replot();
    ui->accPlotWidget->replot();
}

void MainWindow::receiveTrajectData(std::vector<trajectory_msgs::JointTrajectoryPoint>& JointVector)
{
    std::cout <<" receiveTrajectData group: "<<JointVector.size()<<std::endl;

    // group data reset
    group = std::make_shared<TrajectGroupData>();

    dbg("break1 .");
    for(auto its : JointVector){
        std::vector<double> pData = its.positions;
        std::vector<double> vData = its.velocities;
        std::vector<double> aData = its.accelerations;
        double time = its.time_from_start.toSec();
        dbg("setTrajectGroupElementData .group: ",group.use_count());
        group->setTrajectGroupElementData(pData, vData, aData, time);
    }
    dbg("break2 .");

    std::cout <<" updateDrawWidget "<<std::endl;
    updateDrawWidget(group);
}

void MainWindow::on_pbn_load_clicked()
{
    if(!loadLocalTrajectData(group)){
        return ;
    }

    updateDrawWidget(group);
}

void MainWindow::on_pbn_updateVal_clicked()
{

}

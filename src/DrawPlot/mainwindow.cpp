#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "dbg.h"
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
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
        jp0[i] = tempData[0];       // exponentially decaying cosine
        jp1[i] = tempData[1];       // exponential envelope
        jp2[i] = tempData[2];
        jp3[i] = tempData[3];
        jp4[i] = tempData[4];
        jp5[i] = tempData[5];
        jp6[i] = tempData[6];
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
    bool rtn = TrajectManager->readData("/home/de/ws_moveit/build/test/devel/lib/trajectory_test/");

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

void MainWindow::on_pbn_load_clicked()
{
    if(!loadLocalTrajectData(group)){
        return ;
    }

    InitJointCustomPlot(ui->PositionplotWidget);
    InitJointCustomPlot(ui->vecPlotWidget);
    InitJointCustomPlot(ui->accPlotWidget);
    updateCustomData(ui->PositionplotWidget,group, Position);
    updateCustomData(ui->vecPlotWidget,group, Vectory);
    updateCustomData(ui->accPlotWidget,group, accelecation);
//    ui->plotWidget->setWindowTitle("QCustomPlot: good ");
    ui->PositionplotWidget->replot();
    ui->vecPlotWidget->replot();
    ui->accPlotWidget->replot();
}

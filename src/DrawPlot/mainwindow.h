#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "qcustomplot.h"
#include "TrajectData.h"
#include "memory"

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
private:
    void updateCustomData(QCustomPlot *customPlot, std::shared_ptr<TrajectGroupData>& group,TrajectType type);
    bool loadLocalTrajectData(std::shared_ptr<TrajectGroupData>& group);
    void InitJointCustomPlot(QCustomPlot *customPlot);
private slots:
    void on_pbn_load_clicked();

private:
    std::shared_ptr<TrajectGroupData> group;
    std::shared_ptr<TrajectDataIoManager> TrajectManager;
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H

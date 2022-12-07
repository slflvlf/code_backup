#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include <QPointF>
#include <QtCharts>
#include <QValueAxis>
#include <QLineSeries>
#include <QPointF>
#include <QDebug>
#include <cstdlib>
#include <QtMath>
#include <QTimer>
#include <QString>
#include <QVector>
#include <QWidget>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTabWidget>
#include <QGroupBox>
#include <QGridLayout>
#include <QTextEdit>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    void setPreasureDisplayUI();//压力显示
    void setAngleDisplayUI();//倾角显示

private:
    Ui::MainWindow *ui;

    //界面上下两个模块，上面一层的水平模块包括压力开关、倾角显示和倾角设置三个横向的模块
    // 下面一层的模块包括压力显示和倾角显示两个横向的模块
    QVBoxLayout *layout_main_ = new QVBoxLayout();//主界面
    QHBoxLayout *layout_upper_ = new QHBoxLayout();
    QHBoxLayout *layout_lower_ = new QHBoxLayout();

    QVBoxLayout *layout_preasure_scope_ = new QVBoxLayout();
    QVBoxLayout *layout_angle_scope_ = new QVBoxLayout();


    //这一部分是layout_upper的部分，也就是上面一层的东西，我随便写的，
    QTextEdit *text1 = new QTextEdit("...............");
    QTextEdit *text2 = new QTextEdit("...............");
    QTextEdit *text3 = new QTextEdit("...............");



    // 示波器
    int max_size_ = 500; //示波器一次显示多少数据点
    int p_counter_ = 0;//压力数据接收次数的计数
    int a_counter_ = 0;

    // 压力显示
    QLabel *label_preasure_ = new QLabel("压力显示");
    QChart *chart_preasure_ = new QChart;
    QChartView *chartview_preasure_ = new QChartView(chart_preasure_);
    QLineSeries *series_p1_ = new QLineSeries();
    QLineSeries *series_p2_ = new QLineSeries();
    QLineSeries *series_p3_ = new QLineSeries();
    QLineSeries *series_p4_ = new QLineSeries();
    QLineSeries *series_p5_ = new QLineSeries();
    QLineSeries *series_p6_ = new QLineSeries();
    QLineSeries *series_p7_ = new QLineSeries();

    QDateTimeAxis *axisX_preasure_time_ = new QDateTimeAxis();//时间轴
    QValueAxis *axisX_preasure_ = new QValueAxis;//坐标轴
    QValueAxis *axisY_preasure_ = new QValueAxis;


    // 倾角显示
    QLabel *label_angle_ = new QLabel("倾角显示");
    QChart *chart_angle_ = new QChart;
    QChartView *chartview_angle_ = new QChartView(chart_angle_);
    QLineSeries *series_a1_ = new QLineSeries();
    QLineSeries *series_a2_ = new QLineSeries();
    QLineSeries *series_a3_ = new QLineSeries();
    QValueAxis *axisX_angle_ = new QValueAxis;//坐标轴

    QDateTimeAxis *axisX_angle_time_ = new QDateTimeAxis();//时间轴
    QValueAxis *axisY_angle_ = new QValueAxis;

public slots:
    void PreasureMessageReceived(const QVector<double>& msg);
    void AngleMessageReceived(const QVector<double>& msg);

};

#endif // MAINWINDOW_H

#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

//    QWidget *widget = new QWidget();
//    this->setCentralWidget(widget);
//    widget->setLayout(layout_main_);

    setPreasureDisplayUI();
    setAngleDisplayUI();

    layout_lower_->addLayout(layout_preasure_scope_);
    layout_lower_->addLayout(layout_angle_scope_);


    /// 这一块是layout_upper_的东西，后面师兄你自己填充
    layout_upper_->addWidget(text1);
    layout_upper_->addWidget(text2);
    layout_upper_->addWidget(text3);



    layout_main_->addLayout(layout_upper_);
    layout_main_->addLayout(layout_lower_);

//    this->setLayout(layout_main_);

    setWindowTitle("风机");
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::setPreasureDisplayUI()
{


    for (int i=0; i<max_size_; i++){
        series_p1_->append(i, 1.0);
        series_p2_->append(i, 2.0);
        series_p3_->append(i, 3.0);
        series_p4_->append(i, 4.0);
        series_p5_->append(i, 5.0);
        series_p6_->append(i, 6.0);
        series_p7_->append(i, 7.0);
    }

    chart_preasure_->addSeries(series_p1_);
    chart_preasure_->addSeries(series_p2_);
    chart_preasure_->addSeries(series_p3_);
    chart_preasure_->addSeries(series_p4_);
    chart_preasure_->addSeries(series_p5_);
    chart_preasure_->addSeries(series_p6_);
    chart_preasure_->addSeries(series_p7_);

    series_p1_->setName("p1");
    series_p2_->setName("p2");
    series_p3_->setName("p3");
    series_p4_->setName("p4");
    series_p5_->setName("p5");
    series_p6_->setName("p6");
    series_p7_->setName("p7");

    chart_preasure_->setAxisX(axisX_preasure_, series_p1_);
    chart_preasure_->setAxisY(axisY_preasure_, series_p1_);
    chart_preasure_->setAxisX(axisX_preasure_, series_p2_);
    chart_preasure_->setAxisY(axisY_preasure_, series_p2_);
    chart_preasure_->setAxisX(axisX_preasure_, series_p3_);
    chart_preasure_->setAxisY(axisY_preasure_, series_p3_);
    chart_preasure_->setAxisX(axisX_preasure_, series_p4_);
    chart_preasure_->setAxisY(axisY_preasure_, series_p4_);
    chart_preasure_->setAxisX(axisX_preasure_, series_p5_);
    chart_preasure_->setAxisY(axisY_preasure_, series_p5_);
    chart_preasure_->setAxisX(axisX_preasure_, series_p6_);
    chart_preasure_->setAxisY(axisY_preasure_, series_p6_);
    chart_preasure_->setAxisX(axisX_preasure_, series_p7_);
    chart_preasure_->setAxisY(axisY_preasure_, series_p7_);

    axisX_preasure_->setRange(0, max_size_);
    axisY_preasure_->setRange(0.0, 14.0);

    ui->layout_preasure->addWidget(chartview_preasure_);

//    layout_preasure_scope_->addWidget(label_preasure_);
//    layout_preasure_scope_->addWidget(chartview_preasure_);
}

void MainWindow::setAngleDisplayUI()
{
    for (int i=0; i<max_size_; i++){
        series_a1_->append(i, 1.0);
        series_a2_->append(i, 2.0);
        series_a3_->append(i, 3.0);
    }

    chart_angle_->addSeries(series_a1_);
    chart_angle_->addSeries(series_a2_);
    chart_angle_->addSeries(series_a3_);

    series_a1_->setName("a1");
    series_a2_->setName("a2");
    series_a3_->setName("a3");

    chart_angle_->setAxisX(axisX_angle_, series_a1_);
    chart_angle_->setAxisY(axisY_angle_, series_a1_);
    chart_angle_->setAxisX(axisX_angle_, series_a2_);
    chart_angle_->setAxisY(axisY_angle_, series_a2_);
    chart_angle_->setAxisX(axisX_angle_, series_a3_);
    chart_angle_->setAxisY(axisY_angle_, series_a3_);

    axisX_angle_->setRange(0, max_size_);
    axisY_angle_->setRange(0.0, 10);


//    layout_angle_scope_->addWidget(label_angle_);
//    layout_angle_scope_->addWidget(chartview_angle_);
    ui->layout_angle->addWidget(chartview_angle_);


}

void MainWindow::PreasureMessageReceived(const QVector<double> &msg)
{
    p_counter_ += 1;
    QVector<QPointF> oldPoint_p1 = series_p1_->pointsVector();
    QVector<QPointF> oldPoint_p2 = series_p2_->pointsVector();
    QVector<QPointF> oldPoint_p3 = series_p3_->pointsVector();
    QVector<QPointF> oldPoint_p4 = series_p4_->pointsVector();
    QVector<QPointF> oldPoint_p5 = series_p5_->pointsVector();
    QVector<QPointF> oldPoint_p6 = series_p6_->pointsVector();
    QVector<QPointF> oldPoint_p7 = series_p7_->pointsVector();

    QVector<QPointF> newPoint_p1,newPoint_p2,newPoint_p3,newPoint_p4,newPoint_p5,newPoint_p6,newPoint_p7;


    for (int i = 0; i < max_size_ - 1; i++){
        newPoint_p1.append(QPointF(i, oldPoint_p1.at(i+1).y()));
        newPoint_p2.append(QPointF(i, oldPoint_p2.at(i+1).y()));
        newPoint_p3.append(QPointF(i, oldPoint_p3.at(i+1).y()));
        newPoint_p4.append(QPointF(i, oldPoint_p4.at(i+1).y()));
        newPoint_p5.append(QPointF(i, oldPoint_p5.at(i+1).y()));
        newPoint_p6.append(QPointF(i, oldPoint_p6.at(i+1).y()));
        newPoint_p7.append(QPointF(i, oldPoint_p7.at(i+1).y()));
    }

    newPoint_p1.append(QPointF(max_size_-1, msg.at(0)));
    newPoint_p2.append(QPointF(max_size_-1, msg.at(1)));
    newPoint_p3.append(QPointF(max_size_-1, msg.at(2)));
    newPoint_p4.append(QPointF(max_size_-1, msg.at(3)));
    newPoint_p5.append(QPointF(max_size_-1, msg.at(4)));
    newPoint_p6.append(QPointF(max_size_-1, msg.at(5)));
    newPoint_p7.append(QPointF(max_size_-1, msg.at(6)));

    series_p1_->replace(newPoint_p1);
    series_p2_->replace(newPoint_p2);
    series_p3_->replace(newPoint_p3);
    series_p4_->replace(newPoint_p4);
    series_p5_->replace(newPoint_p5);
    series_p6_->replace(newPoint_p6);
    series_p7_->replace(newPoint_p7);


}

void MainWindow::AngleMessageReceived(const QVector<double> &msg)
{
    a_counter_ += 1;
    QVector<QPointF> oldPoint_a1 = series_a1_->pointsVector();
    QVector<QPointF> oldPoint_a2 = series_a2_->pointsVector();
    QVector<QPointF> oldPoint_a3 = series_a3_->pointsVector();

    QVector<QPointF> newPoint_a1,newPoint_a2,newPoint_a3;

    for (int i = 0; i < max_size_ - 1; i++){
        newPoint_a1.append(QPointF(i, oldPoint_a1.at(i+1).y()));
        newPoint_a2.append(QPointF(i, oldPoint_a2.at(i+1).y()));
        newPoint_a3.append(QPointF(i, oldPoint_a3.at(i+1).y()));
    }

    newPoint_a1.append(QPointF(max_size_-1, msg.at(0)));
    newPoint_a2.append(QPointF(max_size_-1, msg.at(1)));
    newPoint_a3.append(QPointF(max_size_-1, msg.at(2)));

    series_a1_->replace(newPoint_a1);
    series_a2_->replace(newPoint_a2);
    series_a3_->replace(newPoint_a3);

}

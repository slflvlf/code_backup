#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    this->centralWidget();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_Button_red_clicked()
{

}

void MainWindow::on_Button_blue_clicked(bool checked)
{
    QPalette plet = ui->Text_helloqt->palette();
    if (checked)
        plet.setColor(QPalette::Text, Qt::blue);

    ui->Text_helloqt->setPalette(plet);


}

void MainWindow::on_Button_red_clicked(bool checked)
{

}



void MainWindow::on_checkBox_underline_clicked(bool checked)
{
    QFont font  = ui->Text_helloqt->font();

    font.setUnderline(checked);

    ui->Text_helloqt->setFont(font);
}

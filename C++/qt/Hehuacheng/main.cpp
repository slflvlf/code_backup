#include "mainwindow.h"
#include <QApplication>
#include "data.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    Data data;
    w.show();
    w.showMaximized();


    QObject::connect(&data, &Data::PreasureDataReady, &w, &MainWindow::PreasureMessageReceived);
    QObject::connect(&data, &Data::AngleDataReady, &w, &MainWindow::AngleMessageReceived);




    return a.exec();
}

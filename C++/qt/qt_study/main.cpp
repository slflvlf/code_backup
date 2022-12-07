#include "mainwindow.h"
#include <QApplication>
#include <form.h>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    Form f;
    f.show();
    w.show();
//    w.showFullScreen();

    return a.exec();
}

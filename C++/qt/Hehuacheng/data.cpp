#include "data.h"

Data::Data(QObject *parent) : QObject(parent)
{
    timer = new QTimer;
    timer->setInterval(200);
    timer->setTimerType(Qt::PreciseTimer);
    timer->start();

    connect(timer, &QTimer::timeout, this, &Data::publishData);


}

void Data::publishData()
{
    QVector<double> preasure_msg = {qrand()%2 + 1,
                           qrand()%2+3,
                           qrand()%2+5,
                           qrand()%2+7,
                           qrand()%2+9,
                           qrand()%2+11,
                           qrand()%2 +13};



    QVector<double> angle_msg = {qrand()%2 + 1,
                           qrand()%2+5,
                           qrand()%2+9};

//    qDebug()<<"preasure: "<<preasure_msg.at(0);

    emit PreasureDataReady(preasure_msg);
    emit AngleDataReady(angle_msg);

}


Data::~Data()
{
    timer->deleteLater();
}

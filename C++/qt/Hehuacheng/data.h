#ifndef DATA_H
#define DATA_H
#include <QObject>
#include <QString>
#include <QThread>
#include <QVector>


#include <QDebug>
#include <cstdlib>
#include <QtMath>
#include <QTimer>
#include <QtGlobal>

class Data : public QObject
{
    Q_OBJECT
public:
    explicit Data(QObject *parent = nullptr);
    QTimer *timer;
    ~Data();

signals:
    void PreasureDataReady(const QVector<double> &msg);
    void AngleDataReady(const QVector<double> &msg);

public slots:
    void publishData();
};

#endif // DATA_H

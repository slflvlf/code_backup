#ifndef SERVODRIVER_H
#define SERVODRIVER_H
#include "triopc.h"
#include <QTimer>


class ServoDriver: public QObject
{
    Q_OBJECT
public:
    explicit ServoDriver(QObject *parent = nullptr);
    ~ServoDriver();
//    TrioPC::_DTrioPC *TrioPC;

    TrioPc::_DTrioPC *TrioPC;
    QTimer *timer;
    bool isPowerOn;

    bool Open();
    bool Close();
    void Poweron();
    void Poweroff();
    void SinglePoweron(const int&);
    void SinglePoweroff(const int&);
    void SetSpeed(const int&, const double&);
    void SetAngle(const int&, const double&);
    void SetAngleAbs(const int&, const double&);
    double GetSpeed(const int&);
    double GetAngle(const int&);
    double GetAngleAbs(const int&, const double&);
//    double GetAngle();
};

#endif // SERVODRIVER_H

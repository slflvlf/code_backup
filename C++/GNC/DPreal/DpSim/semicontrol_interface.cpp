#include "semicontrol_interface.h"

Semicontrol_interface::Semicontrol_interface(QObject *parent) : QObject(parent)
{
   ptr_servo_ = new ServoDriver();
   openflag_ = ptr_servo_->Open();
   ptr_servo_->Poweron();
}

void Semicontrol_interface::thrusterReceived(const QVector<double> &msg)
{
    std::vector<double> revo_speed, angle;
        for (int i = 0; i < 8; ++i)
        {
            revo_speed.push_back(sqrt(msg.at(i)*coeff_force_ / coeff_thruster_));
            angle.push_back(msg.at(i + 8) / 2 / 3.1415926);
          if(openflag_)
          {
            ptr_servo_->SetSpeed(i, revo_speed.at(i));
            ptr_servo_->SetAngleAbs(i + 8, angle.at(i));
          }
        }
}

void Semicontrol_interface::servo_close()
{
    ptr_servo_->Poweroff();
    ptr_servo_->Close();
    delete ptr_servo_;
}

/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Bo Li <lither@sjtu.edu.cn>
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bo Li nor the names of its contributors may
 *     be used to endorse or promote products derived from this
 *     software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "observer.h"
#include <QDebug>

Observer::Observer(const QString &mode, QObject *parent)
    : QObject(parent), positioning_mode(mode) {
  // define the observer
  VesselObserverProperty observer_vessel;
  observer_vessel.name = "Oil_rig";
  observer_vessel.mass = {5.2E7, 5.2E7, 7.5E10};
  observer_vessel.added_mass = {1.7E7, 4.9E7, 5.4E10};
  observer_vessel.damping = {2.4E6, 4.8E6, 3.9E7};

  if (positioning_mode.toLower() == "dp") {
    observer_vessel.stiffness = {0, 0, 0};
  } else {
    observer_vessel.stiffness = {87527, 87549, 7.2E8};
  }

  observer_interface = new ObserverInterface(observer_vessel, 0.1, {0, 0, 0},
                                             "C:Code/C++/GNC/semi/results/");
  observer_interface->moveToThread(&observer_thread);

  connect(&observer_thread, &QThread::finished, observer_interface,
          &QObject::deleteLater);
  connect(this, &Observer::plantMessageReceived, observer_interface,
          &ObserverInterface::motionMessageReceived);
  connect(this, &Observer::controllerMessageReceived, observer_interface,
          &ObserverInterface::controlMessageReceived);
  connect(observer_interface, &ObserverInterface::messageReady, this,
          &Observer::messageReady);

  observer_thread.start();
}

Observer::~Observer() {
  observer_thread.quit();
  observer_thread.wait();
  qDebug() << "Observer has exited";
}

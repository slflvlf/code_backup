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

//#include "controller.h"
//#include "observer.h"
//#include "plant.h"
#include <QApplication>
#include <QThread>
//#include <core_window.h>
#include <QMetaType>

#include "env_load_interface.h"

int main(int argc, char *argv[]) {
  QApplication app(argc, argv);

//  CoreWindow widget;

  env_load_interface  env_load;

//  QString mode = "pm"; // if use dp mode, the controller in ControllerInterface
//                       // needs to be changed
//  Plant plant(mode);
//  Controller controller(mode);
//  Observer observer(mode);


  std::array<double, 3> pose;
  pose = {{0, 0, 0}};
  env_load.compute_envload(pose, 3000);

  // register meta type
//  qRegisterMetaType<QVector<double>>("QVector<double>");

//  // establish communication
//  QObject::connect(&controller, &Controller::messageReady, &plant,
//                   &Plant::messageReceived);
//  QObject::connect(&controller, &Controller::messageReady, &observer,
//                   &Observer::controllerMessageReceived);
//  QObject::connect(&observer, &Observer::messageReady, &controller,
//                   &Controller::messageReceived);
//  QObject::connect(&observer, &Observer::messageReady, &widget,
//                   &CoreWindow::messageReceived);
//  QObject::connect(&plant, &Plant::messageReady, &observer,
//                   &Observer::plantMessageReceived);
//  QObject::connect(&app, &QApplication::aboutToQuit, &plant,
//                   &Plant::abortSimulation);

//  widget.showMaximized();

  return app.exec();
}

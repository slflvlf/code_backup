/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Bo Li <lither@sjtu.edu.cn>
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

#include "controller_interface.h"
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
//#include "pm_setpoint_interface.h"

ControllerInterface::ControllerInterface(
    const VesselControlProperty &vessel,
    const ThrustAllocationConfiguration &config,
    const std::array<double, 3> &PositionSetPoint, const double &step_size,
    std::string save_path)
    : step_size_(step_size), output_directory_(std::move(save_path)) {
  // define the backstepping controller
  // ptr_controller_ = std::unique_ptr<BackSteppingController>(
  //     new BackSteppingController(vessel, 1.0, { 0.5, 0.5, 0.5 }, { 1E6, 1E6,
  //     1E8 }, { 1E6, 1E6, 1E8 }));
  // ptr_controller_->setPositionSetpoint(PositionSetPoint);
  // ptr_controller_->setVelocitySetpoint({ 0, 0, 0 });

  // define the pm controller
  std::array<double, 3> Lambda = {{0.5, 0.0, 0.0}};
  std::array<double, 3> Kp = {{1E6, 0, 0}};
  std::array<double, 3> Kd = {{1E6, 1E6, 1E8}};
  // change 1.0 to step_size_???
  ptr_controller_ = std::make_unique<PmController>(vessel, 0.1, Lambda, Kp, Kd);
  ptr_controller_->setStateSetpoint({{10, 0, 0}});
  //    ptr_pm_setpoint_interface_ =
  //            std::unique_ptr<PmSetpointInterface>(new
  //            PmSetpointInterface("setpoint_server", 10, 360, step_size));

  // define the thrust allocator
  ptr_allocator_ = std::make_unique<SQPThrustAllocation>(config, step_size_);

  step_interval_ = (unsigned int)ptr_controller_->getStepSize() / step_size_ + 1;
}

void ControllerInterface::messageReceived(const QVector<double> &msg) {
  std::array<double, 3> position, velocity, acceleration;

  position.at(0) = msg.at(1); // first element is timestamp
  position.at(1) = msg.at(2);
  position.at(2) = msg.at(3);

  velocity.at(0) = msg.at(4);
  velocity.at(1) = msg.at(5);
  velocity.at(2) = msg.at(6);

  acceleration.at(0) = msg.at(7);
  acceleration.at(1) = msg.at(8);
  acceleration.at(2) = msg.at(9);

  ptr_controller_->setVesselPosition(position);
  ptr_controller_->setVesselVelocity(velocity);
  ptr_controller_->setVesselAcceleration(acceleration);

//  controller_state_ =
//      ptr_controller_->getInternalState(); // for backstepping controller

  auto polar_state = ptr_controller_->ComputeState(); // for pm controller only

  ptr_controller_->ComputeControl();
  std::array<double, 3> output = ptr_controller_->getOutput();

  for (size_t i = 0; i < 3; i++) {
    control_output_.at(i) = output.at(i) / 1E3; // unit: kN
  }

  int flag;

  if (qp_counter_ == 0) {
    flag = ptr_allocator_->initQProblem(control_output_);
  } else {
    flag = ptr_allocator_->solveQProblem(control_output_);
  }

  if (flag != 0) {
    std::cerr << "Thrust allocation failed." << std::endl;
  } else {
    thruster_angle_ = ptr_allocator_->getThrusterAngle();
    thruster_force_ = ptr_allocator_->getThrusterForce();
    ptr_allocator_->computeControlOutput(thruster_force_, thruster_angle_);
    allocation_output_ = ptr_allocator_->getControlOutput();
  }

  // save thrusters data
  if (qp_counter_ == 0) {
    ExportData(output_directory_);
  } else {
    ExportData(output_directory_, "app");
  }

  ++qp_counter_;

  static unsigned int counter = 0;
  ++counter;

  if (counter == step_interval_) {
    // publish control output
    QVector<double> control = {allocation_output_.at(0) * 1E3,
                               allocation_output_.at(1) * 1E3,
                               allocation_output_.at(2) * 1E3};
    emit messageReady(control);

    counter = 0;
  }

  //    // send radius and power info to PmSetpointInterface
  //    double total_power = 0;

  //    for (auto ptr = thruster_force_.cbegin(); ptr!= thruster_force_.cend();
  //    ++ptr)
  //    {
  //        total_power += std::pow(*ptr, 1.5);
  //    }

  //    ptr_pm_setpoint_interface_->updateRadiusAndPower(polar_state.at(0),
  //    total_power);

  //    // pm controller setpoint
  //    double rho = (ptr_controller_->getStateSetpoint()).at(0);
  //    double radius_setpoint =
  //    ptr_pm_setpoint_interface_->getRadiusSetpoint(); ROS_INFO("The radius
  //    setpoint is %5.2f\n", rho); if (std::fabs(radius_setpoint - rho) > 1E-2)
  //    {
  //        double increment = step_size_ / 10 * (radius_setpoint - rho);  //
  //        time constant is 10 sec

  //        // limit the speed
  //        if (increment > 0.01)
  //        {
  //            increment = 0.01;
  //        }
  //        else if (increment < -0.01)
  //        {
  //            increment = -0.01;
  //        }

  //        rho += increment;
  //    }
  //    else
  //    {
  //        rho = radius_setpoint;
  //    }
  //    ptr_controller_->setStateSetpoint({ rho, 0, 0 });
}

void ControllerInterface::ExportData(const std::string &save_directory,
                                     const std::string &mode) const {
  const std::string &control_file =
      save_directory + "control_allocation_file.txt";
  const std::string &force_file = save_directory + "thruster_force_file.txt";
  const std::string &angle_file = save_directory + "thruster_angle_file.txt";
  const std::string &controller_file = save_directory + "controller_file.txt";

  std::ofstream control_out, force_out, angle_out;
  std::ofstream controller_out;

  // open the files
  if (mode == "trunc") {
    control_out.open(control_file.c_str());
    force_out.open(force_file.c_str());
    angle_out.open(angle_file.c_str());
    controller_out.open(controller_file.c_str());
  } else {
    control_out.open(control_file.c_str(), std::ofstream::app);
    force_out.open(force_file.c_str(), std::ofstream::app);
    angle_out.open(angle_file.c_str(), std::ofstream::app);
    controller_out.open(controller_file.c_str(), std::ofstream::app);
  }

  // write data to the file
  control_out << std::fixed << std::setprecision(2) << qp_counter_ << '\t';
  control_out << std::setprecision(5);

  for (size_t i = 0; i < 3; i++) {
    control_out << std::scientific << control_output_.at(i) << '\t';
  }
  for (size_t i = 0; i < 3; i++) {
    control_out << std::scientific << allocation_output_.at(i) << '\t';
  }

  control_out << std::endl;

  // write thruster force data to the file
  force_out << std::fixed << std::setprecision(2) << qp_counter_ << '\t';
  force_out << std::setprecision(5);

  for (size_t i = 0; i < thruster_force_.size(); i++) {
    force_out << std::scientific << thruster_force_.at(i) << '\t';
  }

  force_out << std::endl;

  // write thruster angle data to the file
  angle_out << std::fixed << std::setprecision(2) << qp_counter_ << '\t';
  angle_out << std::setprecision(5);

  for (size_t i = 0; i < thruster_angle_.size(); i++) {
    angle_out << std::scientific << thruster_angle_.at(i) << '\t';
  }

  angle_out << std::endl;

  // backstepping data out
  controller_out << std::fixed << std::setprecision(2)
                 << qp_counter_ * step_size_ << '\t';
  controller_out << std::setprecision(5);

  for (size_t i = 0; i < controller_state_.size(); i++) {
    controller_out << std::scientific << controller_state_.at(i) << '\t';
  }

  controller_out << std::endl;

  // close the files
  control_out.close();
  force_out.close();
  angle_out.close();
  controller_out.close();
}

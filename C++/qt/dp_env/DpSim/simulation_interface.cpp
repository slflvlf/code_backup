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

#include "simulation_interface.h"
#include <QDebug>
#include <QThread>
#include <cstdio>
#include <fstream>
#include <iostream>

SimulationInterface::SimulationInterface(
    const SemiSubmersibleProperty &vessel, const MooringSystemProperty &mooring,
    const JonswapSpectrumProperty &wave, const ConstantWindProperty &wind,
    const ConstantCurrentProperty &current, const WindLoadData &wind_load,
    const CurrentLoadData &current_load, const double &build_up_time,
    const double &simulation_time, const double &step_size, std::string mode,
    const std::array<double, 6> &init_position)
    : build_up_time_(build_up_time), simulation_time_(simulation_time),
      time_step_(step_size), mode_(std::move(mode)) {

  // define wind load
  std::array<std::vector<double>, 3> data_list = {
      {wind_load.wind_x, wind_load.wind_y, wind_load.wind_n}};
  ptr_wind_load_ = std::make_unique<WindLoad>(data_list, wind_load.angle_list,
                                              wind_load.wind_speed);

  // define current load
  data_list = {
      {current_load.current_x, current_load.current_y, current_load.current_n}};
  ptr_current_load_ = std::make_unique<CurrentLoad>(
      data_list, current_load.angle_list, current_load.current_speed);

  // define a constant wind
  ptr_wind_ = std::make_unique<ConstantWind>(wind.speed, wind.direction);

  // define a constant current
  ptr_current_ =
      std::make_unique<ConstantCurrent>(current.speed, current.direction);

  // define and initialize Jonswap wave
  ptr_wave_ = std::make_unique<JonswapSpectrum>(wave.Hs, wave.T0, wave.gamma);
  ptr_wave_->initializeWaveComponents(wave.cutoff_frequency,
                                      wave.component_number);

  // define the mooring system
  if (mode_ == "mooring" || mode_ == "pm") {
    qDebug("Mooring system initializing...");
    std::array<double, 3> current_velocity = {
        {current.speed * std::cos(current.direction * M_PI / 180),
         current.speed * std::sin(current.direction * M_PI / 180), 0}};
    ptr_mooring_ = std::make_unique<MooringSimulation>(
        mooring, current_velocity, time_step_);
  }

  // define the semi-submersible
  qDebug("Hydrodynamic Data importing...");

  if (!vessel.is_diagonal) {
    ptr_qtf_ = std::make_unique<QtfDataBase>(vessel.data_directory,
                                             vessel.wave_load_frequency_list);
  } else {
    ptr_qtf_ = std::make_unique<QtfDataBase>(
        vessel.data_directory + "wavedriftdiag.csv",
        vessel.wave_load_frequency_list, vessel.direction_list);
  }

  // initial velocity and position
  std::array<double, 6> init_velocity = {{0, 0, 0, 0, 0, 0}};

  ptr_semi_ = std::make_unique<SemiSubmersible>(
      vessel.name, vessel.mass, vessel.gravity_center, vessel.moment_of_inertia,
      vessel.restoring_vector, vessel.data_directory,
      vessel.radiation_frequency_list, vessel.data_directory + "forcerao.csv",
      vessel.direction_list, vessel.data_directory,
      vessel.wave_load_frequency_list, *ptr_qtf_, *ptr_wind_load_,
      *ptr_current_load_, init_velocity, init_position, false);

  // define output directory
  std::string path = vessel.data_directory;
  output_directory_ = path.replace(path.end() - 5, path.end(), "results/");

  // define the simulation
  /** @todo use newman's approximate method. Full QTF results in weird roll and
   * pitch motion. Check that. */
  ptr_simulation_ = std::make_unique<Simulation>(
      *ptr_semi_, *ptr_wave_, wave.direction, *ptr_wind_, *ptr_current_,
      step_size, 120, output_directory_, false, false);
}

void SimulationInterface::BuildUpSimulation(const double &build_up_time) {
  unsigned int step_number = std::round(build_up_time / time_step_);
  ptr_simulation_->setCurrentTime(-build_up_time);

  // write the current velocity and position data into the output files
  ptr_simulation_->ExportData(output_directory_);

  for (unsigned int counter = 1; counter <= step_number; ++counter) {
    // check abort flag
    if (get_aborted_) {
      qDebug() << "Build-up simulation is aborted, and" << counter - 1
               << "steps have been conducted";
      return;
    }

    std::vector<double> vessel_motion = ptr_simulation_->getVesselPosition();

    // publish vessel motion info
    PublishMessages();

    // save mooring and control force
    if (counter == 1) {
      ExportData(output_directory_);
    } else {
      ExportData(output_directory_, "app");
    }

    // set mooring or control force
    if (mode_ == "dp") {
      ptr_semi_->setExternalForce(control_force_);
    } else if (mode_ == "mooring") {
      std::array<double, 6> vessel_position;
      for (unsigned int i = 0; i < 6; ++i) {
        vessel_position.at(i) = vessel_motion.at(i);
      }

      std::vector<double> mooring_force =
          ptr_mooring_->OneStepSimulation(vessel_position);
      for (size_t i = 0; i < 6; i++) {
        mooring_force_.at(i) = mooring_force.at(i) * 1E3; // unit: N
      }

      // vertical component always zero
      mooring_force_.at(2) = 0;

      ptr_semi_->setExternalForce(mooring_force_);
    } else if (mode_ == "pm") {
      std::array<double, 6> vessel_position;
      for (unsigned int i = 0; i < 6; ++i) {
        vessel_position.at(i) = vessel_motion.at(i);
      }

      std::vector<double> mooring_force =
          ptr_mooring_->OneStepSimulation(vessel_position);
      for (size_t i = 0; i < 6; i++) {
        mooring_force_.at(i) = mooring_force.at(i) * 1E3; // unit: N
      }

      // vertical component always zero
      mooring_force_.at(2) = 0;

      // use mooring force to initialize the external force acting on the semi
      // first
      ptr_semi_->setExternalForce(mooring_force_);

      // add control force to external force
      ptr_semi_->addExternalForce(control_force_);
    } else {
      qDebug("Unkown control mode, please select one in {dp, mooring, pm}.");
    }

    // conduct one-step build up simulation
    ptr_simulation_->OneStepBuildUpSimulation(build_up_time_);

    // print progressbar
    //    PrintProgBar(counter * 100 / step_number);
  }

  //  std::cout << std::endl;
}

void SimulationInterface::RunSimulation(const double &simulation_time) {
  unsigned int step_number = std::round(simulation_time / time_step_);

  for (unsigned int counter = 1; counter <= step_number; ++counter) {
    // check abort flag
    if (get_aborted_) {
      qDebug() << "Main simulation is aborted, and" << counter - 1
               << "steps have been conducted";
      return;
    }

    std::vector<double> vessel_motion = ptr_simulation_->getVesselPosition();

    // publish vessel motion info
    PublishMessages();

    // save mooring and control force
    ExportData(output_directory_, "app");

    // set mooring or control force
    if (mode_ == "dp") {
      ptr_semi_->setExternalForce(control_force_);
    } else if (mode_ == "mooring") {
      std::array<double, 6> vessel_position;
      for (unsigned int i = 0; i < 6; ++i) {
        vessel_position.at(i) = vessel_motion.at(i);
      }

      std::vector<double> mooring_force =
          ptr_mooring_->OneStepSimulation(vessel_position);
      for (size_t i = 0; i < 6; i++) {
        mooring_force_.at(i) = mooring_force.at(i) * 1E3; // unit: N
      }

      // vertical component always zero
      mooring_force_.at(2) = 0;

      ptr_semi_->setExternalForce(mooring_force_);
    } else if (mode_ == "pm") {
      std::array<double, 6> vessel_position;
      for (unsigned int i = 0; i < 6; ++i) {
        vessel_position.at(i) = vessel_motion.at(i);
      }

      std::vector<double> mooring_force =
          ptr_mooring_->OneStepSimulation(vessel_position);
      for (size_t i = 0; i < 6; i++) {
        mooring_force_.at(i) = mooring_force.at(i) * 1E3; // unit: N
      }

      // vertical component always zero
      mooring_force_.at(2) = 0;

      // use mooring force to initialize the external force acting on the semi
      // first
      ptr_semi_->setExternalForce(mooring_force_);

      // add control force to external force
      ptr_semi_->addExternalForce(control_force_);
    } else {
      qDebug("Unkown control mode, please select one in {dp, mooring, pm}.");
    }

    // conduct one-step main simulation
    ptr_simulation_->OneStepSimulation();

    // print progressbar
    //    PrintProgBar(counter * 100 / step_number);
  }

  //  qDebug(" ");
}

void SimulationInterface::StartSimulation() {
  // reset abort flag
  get_aborted_ = false;

  // print simulation summary
  qDebug("\n---- Simulation Summary ----");
  qDebug("Vessel Name: %s", (ptr_semi_->getName()).c_str());
  qDebug("Wave Direction: %5.2f deg", ptr_simulation_->getWaveDirection());
  qDebug("Positioning Mode: %s", mode_.c_str());
  qDebug("Simuluation Time: %5.2f sec", simulation_time_);
  qDebug("============================\n");
  qDebug("Simulation initializing...");

  // initialize a new simulation
  ptr_simulation_->InitializeNewSimulation();

  // build up simulation
  BuildUpSimulation(build_up_time_);
  ptr_simulation_->InitializeMainSimulation();

  // run the main simulation
  qDebug("Simulation running...");
  RunSimulation(simulation_time_);

  qDebug("Simulation is Done.");
}

void SimulationInterface::PublishMessages() {
  std::vector<double> semi_position = ptr_simulation_->getVesselPosition();
  std::vector<double> semi_velocity = ptr_simulation_->getVesselVelocity();

  // publish vessel motion
  QVector<double> motion_msg;
  motion_msg.append(ptr_simulation_->getCurrentTime());
  motion_msg.append(semi_position.at(0));
  motion_msg.append(semi_position.at(1));
  motion_msg.append(semi_position.at(2));
  motion_msg.append(semi_position.at(3));
  motion_msg.append(semi_position.at(4));
  motion_msg.append(semi_position.at(5));

  for (unsigned int i = 0; i < 6; ++i) {
    motion_msg.append(semi_velocity.at(i));
  }
  emit messageReady(motion_msg);
}

void SimulationInterface::PrintProgBar(const size_t &percent) {
  std::string bar;
  for (size_t i = 0; i < 50; ++i) {
    if (i < (percent / 2)) {
      bar.replace(i, 1, "-");
    } else if (i == (percent / 2)) {
      bar.replace(i, 1, ">");
    } else {
      bar.replace(i, 1, " ");
    }
  }
  std::cout << "\r"
               "["
            << bar << "]";
  std::cout.width(3);
  std::cout << percent << "%    " << std::flush;
}

// save velocity and position data into text files
void SimulationInterface::ExportData(const std::string &save_directory,
                                     const std::string &mode) const {
  std::string mooring_file = save_directory + "mooring_file.txt";
  std::string control_file = save_directory + "control_file.txt";

  std::ofstream mooring_out, control_out;

  // open the files
  if (mode == "trunc") {
    mooring_out.open(mooring_file.c_str());
    control_out.open(control_file.c_str());
  } else {
    mooring_out.open(mooring_file.c_str(), std::ofstream::app);
    control_out.open(control_file.c_str(), std::ofstream::app);
  }

  // mooring force
  mooring_out << std::fixed << std::setprecision(2)
              << ptr_simulation_->getCurrentTime()
              << '\t'; // first column is time
  mooring_out << std::setprecision(5);
  for (size_t index = 0; index < 6; ++index) { // the remaining columns are data
    mooring_out << std::scientific << mooring_force_[index] << '\t';
  }
  mooring_out << std::endl;

  // control force
  control_out << std::fixed << std::setprecision(2)
              << ptr_simulation_->getCurrentTime()
              << '\t'; // first column is time
  control_out << std::setprecision(5);
  for (size_t index = 0; index < 6; ++index) { // the remaining columns are data
    control_out << std::scientific << control_force_[index] << '\t';
  }
  control_out << std::endl;

  // close the files
  mooring_out.close();
  control_out.close();
}

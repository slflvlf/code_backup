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

#include "simulation.h"
#include "jonswap_spectrum.h"
#include "qtf_database.h"
#include "radiation_force.h"
#include "semi_submersible.h"
#include "wave_excitation.h"
#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>

Simulation::Simulation(SemiSubmersible &vessel, JonswapSpectrum &wave,
                       const double &wave_direction, ConstantWind &wind,
                       ConstantCurrent &current, const double &step_size,
                       const double &radiation_cutoff_time,
                       std::string out_path, const bool &full_QTF,
                       const bool &motion_RAO)
    : vessel_(vessel), wave_(wave), wave_direction_(wave_direction),
      wind_(wind), current_(current), step_size_(step_size),
      output_directory_(std::move(out_path)), full_qtf_on_(full_QTF),
      motion_rao_on_(motion_RAO) {
  vessel_.InitializeSemiSubmersible(wave, step_size, radiation_cutoff_time);

  // dividing frequency for low frequency primary motion is 0.1 rad/s
  dividing_frequency_ = 0.1;

  // initialize vessel_state_ and primary_state_
  std::array<double, 6> vessel_vel = vessel.getVelocity();
  std::array<double, 6> vessel_pos = vessel.getPosition();

  for (unsigned int i = 0; i < 6; ++i) {
    vessel_state_.push_back(vessel_vel.at(i));
    primary_state_.push_back(vessel_pos.at(i));
  }

  for (unsigned int i = 0; i < 6; ++i) {
    vessel_state_.push_back(vessel_pos.at(i));
    primary_state_.push_back(vessel_vel.at(i));
  }
}

void Simulation::InitializeNewSimulation() {
  wave_vec_.clear();
  rao_based_position_history_.clear();
  rao_based_velocity_history_.clear();
  velocity_radiation_history_.clear();

  wave_vec_.push_back(0);
  rao_based_velocity_history_.push_back(
      std::array<double, 6>{{0, 0, 0, 0, 0, 0}});
  rao_based_position_history_.push_back(
      std::array<double, 6>{{0, 0, 0, 0, 0, 0}});
  velocity_radiation_history_.push_back(
      std::array<double, 6>{{0, 0, 0, 0, 0, 0}});
}

void Simulation::OneStepBuildUpSimulation(const double &build_up_time) {
  double ratio =
      (current_time_ + build_up_time) /
      build_up_time; // the proportion of the build-up stage completed
  double ramping_factor =
      std::pow(ratio, 3) * (6 * std::pow(ratio, 2) - 15 * ratio + 10);

  // low frequency primary motion
  double x_coor = primary_state_.at(0);
  double y_coor = primary_state_.at(1);
  double heading = primary_state_.at(5);

  // compute wave excitation load
  wave_excitation_load_ =
      vessel_.wave_excitation_.getWaveExcitation(current_time_,
                                                 x_coor, //  x coordinate
                                                 y_coor, //  y coordinate
                                                 wave_direction_ * M_PI / 180,
                                                 0, // vessel forward speed
                                                 heading); // vessel heading

  // compute radiation damping load
  radiation_damping_load_ = vessel_.radiation_force_.getRadiationDampingForce(
      velocity_radiation_history_);

  if (full_qtf_on_) {
    // compute wave drift load based on full QTF
    wave_drift_load_ = vessel_.wave_drift_.getWaveDriftForceFullQTF(
        current_time_,
        x_coor, //  x coordinate
        y_coor, //  y coordinate
        wave_direction_ * M_PI / 180,
        0,        // vessel forward speed
        heading); // vessel heading
  } else {
    // compute wave drift load based on Newman's approximation
    wave_drift_load_ = vessel_.wave_drift_.getWaveDriftForceNewmanApproximation(
        current_time_,
        x_coor, //  x coordinate
        y_coor, //  y coordinate
        wave_direction_ * M_PI / 180,
        0,        // vessel forward speed
        heading); // vessel heading
  }

  // total wave loads
  for (unsigned int i = 0; i < 6; ++i) {
    wave_drift_load_.at(i) *= ramping_factor;
    wave_excitation_load_.at(i) *= ramping_factor;
    wave_load_.at(i) = wave_excitation_load_.at(i) + wave_drift_load_.at(i);
    wave_load_.at(i) -= radiation_damping_load_.at(i);
  }

  // add wave load to external force
  // it is important to make sure that external_force_ has been properly set in
  // simulation interface
  vessel_.addExternalForce(wave_load_);

  // add wind load to external force
  auto wind_force = vessel_.wind_force_.getWindLoad(
      wind_.getSpeed(), wind_.getDirection() * M_PI / 180 - heading);
  wind_load_.at(0) = ramping_factor * wind_force.at(0);
  wind_load_.at(1) = ramping_factor * wind_force.at(1);
  wind_load_.at(5) = ramping_factor * wind_force.at(2);

  vessel_.addExternalForce(wind_load_);

  // add current load to external force
  auto current_force = vessel_.current_force_.getCurrentLoad(
      current_.getSpeed(), current_.getDirection() * M_PI / 180 - heading);
  current_load_.at(0) = ramping_factor * current_force.at(0);
  current_load_.at(1) = ramping_factor * current_force.at(1);
  current_load_.at(5) = ramping_factor * current_force.at(2);

  vessel_.addExternalForce(current_load_);

  // call the ode solver
  stepper_.do_step(std::bind(&SemiSubmersible::WaveFrequencySystemFunction,
                             vessel_, std::placeholders::_1,
                             std::placeholders::_2, std::placeholders::_3),
                   vessel_state_, current_time_, step_size_);

  // update time
  current_time_ += step_size_;

  // filter the primary motion
  DividePrimaryMotion();

  wave_vec_.push_back(wave_.getWaveElevation(current_time_, 0, 0,
                                             wave_direction_ * M_PI / 180,
                                             0, // vessel forward speed
                                             0));

  // save vessel velocity into a deque for computing radiation damping force
  velocity_radiation_history_.push_back(std::array<double, 6>{
      {vessel_state_[0], vessel_state_[1], vessel_state_[2], vessel_state_[3],
       vessel_state_[4], vessel_state_[5]}});

  if (velocity_radiation_history_.size() >=
      vessel_.radiation_force_.getImpulseResponseSize()) {
    velocity_radiation_history_.pop_front();
  }

  // write the current velocity and position data into the output files
  ExportData(output_directory_, "app");
}

void Simulation::InitializeMainSimulation() {
  // reset main simulation parameters
  current_time_ = 0;
  step_counter_ = 0;

  // clear all the history vectors
  velocity_history_.clear();
  position_history_.clear();

  // initial wave elevation
  wave_vec_.front() =
      wave_.getWaveElevation(0, 0, 0, wave_direction_ * M_PI / 180,
                             0, // vessel forward speed
                             0);

  // initialize RAO based motion history
  if (motion_rao_on_) {
    std::array<std::array<double, 6>, 2> initial_vessel_state =
        vessel_.motion_rao_.getRaoBasedMotion(
            0, 0, 0, wave_direction_ * M_PI / 180, 0, 0);
    rao_based_velocity_history_.front() = initial_vessel_state.at(0);
    rao_based_position_history_.front() = initial_vessel_state.at(1);
  }

  // initialize motion history
  velocity_history_.push_back(std::array<double, 6>{
      {vessel_state_[0], vessel_state_[1], vessel_state_[2], vessel_state_[3],
       vessel_state_[4], vessel_state_[5]}});
  position_history_.push_back(std::array<double, 6>{
      {vessel_state_[6], vessel_state_[7], vessel_state_[8], vessel_state_[9],
       vessel_state_[10], vessel_state_[11]}});
}

void Simulation::OneStepSimulation() {
  // low frequency primary motion
  double x_coor = primary_state_.at(0);
  double y_coor = primary_state_.at(1);
  double heading = primary_state_.at(5);

  // compute wave excitation load
  wave_excitation_load_ =
      vessel_.wave_excitation_.getWaveExcitation(current_time_,
                                                 x_coor, //  x coordinate
                                                 y_coor, //  y coordinate
                                                 wave_direction_ * M_PI / 180,
                                                 0, // vessel forward speed
                                                 heading); // vessel heading

  if (full_qtf_on_) {
    // compute wave drift load based on full QTF
    wave_drift_load_ = vessel_.wave_drift_.getWaveDriftForceFullQTF(
        current_time_,
        x_coor, //  x coordinate
        y_coor, //  y coordinate
        wave_direction_ * M_PI / 180,
        0,        // vessel forward speed
        heading); // vessel heading
  } else {
    // compute wave drift load based on Newman's approximation
    wave_drift_load_ = vessel_.wave_drift_.getWaveDriftForceNewmanApproximation(
        current_time_,
        x_coor, //  x coordinate
        y_coor, //  y coordinate
        wave_direction_ * M_PI / 180,
        0,        // vessel forward speed
        heading); // vessel heading
  }

  // compute radiation damping load
  radiation_damping_load_ = vessel_.radiation_force_.getRadiationDampingForce(
      velocity_radiation_history_);

  // total wave loads
  for (unsigned int i = 0; i < 6; ++i) {
    wave_load_.at(i) = wave_drift_load_.at(i) + wave_excitation_load_.at(i);
    wave_load_.at(i) -= radiation_damping_load_.at(i);
  }

  // add wave load to external force
  // it is important to make sure that external_force_ has been properly set in
  // simulation interface
  vessel_.addExternalForce(wave_load_);

  // add wind load to external force
  auto wind_force = vessel_.wind_force_.getWindLoad(
      wind_.getSpeed(), wind_.getDirection() * M_PI / 180 - heading);
  wind_load_.at(0) = wind_force.at(0);
  wind_load_.at(1) = wind_force.at(1);
  wind_load_.at(5) = wind_force.at(2);

  vessel_.addExternalForce(wind_load_);

  // add current load to external force
  auto current_force = vessel_.current_force_.getCurrentLoad(
      current_.getSpeed(), current_.getDirection() * M_PI / 180 - heading);
  current_load_.at(0) = current_force.at(0);
  current_load_.at(1) = current_force.at(1);
  current_load_.at(5) = current_force.at(2);

  vessel_.addExternalForce(current_load_);

  // call the ode solver
  stepper_.do_step(std::bind(&SemiSubmersible::WaveFrequencySystemFunction,
                             vessel_, std::placeholders::_1,
                             std::placeholders::_2, std::placeholders::_3),
                   vessel_state_, current_time_, step_size_);

  // update the data members of the object
  ++step_counter_;
  current_time_ += step_size_;
  time_vec_.push_back(current_time_);

  // filter the primary motion
  DividePrimaryMotion();

  // RAO based motion
  if (motion_rao_on_) {
    std::array<std::array<double, 6>, 2> rao_based_motion =
        vessel_.motion_rao_.getRaoBasedMotion(current_time_, x_coor, y_coor,
                                              wave_direction_ * M_PI / 180, 0,
                                              heading);
    rao_based_velocity_history_.push_back(rao_based_motion.at(0));
    rao_based_position_history_.push_back(rao_based_motion.at(1));
  }

  // update vessel velocity and position
  vessel_.velocity_ << vessel_state_[0], vessel_state_[1], vessel_state_[2],
      vessel_state_[3], vessel_state_[4], vessel_state_[5];
  vessel_.position_ << vessel_state_[6], vessel_state_[7], vessel_state_[8],
      vessel_state_[9], vessel_state_[10], vessel_state_[11];
  // wrap the euler angle into [-pi, pi)
  vessel_.position_[3] = remainder(vessel_.position_[3], 2 * M_PI);
  vessel_.position_[4] = remainder(vessel_.position_[4], 2 * M_PI);
  vessel_.position_[5] = remainder(vessel_.position_[5], 2 * M_PI);
  vessel_.euler_ << vessel_.position_[3], vessel_.position_[4],
      vessel_.position_[5];

  velocity_history_.push_back(std::array<double, 6>{
      {vessel_state_[0], vessel_state_[1], vessel_state_[2], vessel_state_[3],
       vessel_state_[4], vessel_state_[5]}});
  position_history_.push_back(std::array<double, 6>{
      {vessel_state_[6], vessel_state_[7], vessel_state_[8], vessel_state_[9],
       vessel_state_[10], vessel_state_[11]}});
  wave_vec_.push_back(wave_.getWaveElevation(current_time_, 0, 0,
                                             wave_direction_ * M_PI / 180,
                                             0, // vessel forward speed
                                             0));

  // save vessel velocity into a deque for computing radiation damping force
  velocity_radiation_history_.push_back(velocity_history_.back());

  if (velocity_radiation_history_.size() >=
      vessel_.radiation_force_.getImpulseResponseSize()) {
    velocity_radiation_history_.pop_front();
  }

  // write the current velocity and position data into the output files
  ExportData(output_directory_, "app");
}

// save velocity and position data into text files
void Simulation::ExportData(const std::string &save_directory,
                            const std::string &mode) const {
  std::string velocity_file = save_directory + "velocity_file.txt";
  std::string position_file = save_directory + "position_file.txt";
  std::string wave_elevation_file = save_directory + "wave_elevation_file.txt";
  std::string wave_load_file = save_directory + "wave_load_file.txt";
  std::string rao_based_motion_file = save_directory + "rao_based_file.txt";

  std::ofstream velocity_out, position_out, wave_elevation_out;
  std::ofstream wave_load_out, rao_position_out;
  // open the files
  if (mode == "trunc") {
    velocity_out.open(velocity_file.c_str());
    position_out.open(position_file.c_str());
    wave_elevation_out.open(wave_elevation_file.c_str());
    wave_load_out.open(wave_load_file.c_str());
    rao_position_out.open(rao_based_motion_file.c_str());
  } else {
    velocity_out.open(velocity_file.c_str(), std::ofstream::app);
    position_out.open(position_file.c_str(), std::ofstream::app);
    wave_elevation_out.open(wave_elevation_file.c_str(), std::ofstream::app);
    wave_load_out.open(wave_load_file.c_str(), std::ofstream::app);
    rao_position_out.open(rao_based_motion_file.c_str(), std::ofstream::app);
  }

  // velocity
  velocity_out << std::fixed << std::setprecision(2) << current_time_
               << '\t'; // first column is time
  velocity_out << std::setprecision(5);
  for (size_t index = 0; index < 6; ++index) { // the remaining columns are data
    velocity_out << std::scientific << vessel_state_[index] << '\t';
  }
  velocity_out << std::endl;

  // position
  position_out << std::fixed << std::setprecision(2) << current_time_ << '\t';
  position_out << std::setprecision(5);
  for (size_t index = 6; index < 12; ++index) {
    position_out << std::scientific << vessel_state_[index] << '\t';
  }
  for (size_t index = 0; index < 6; ++index) {
    position_out << std::scientific << primary_state_[index] << '\t';
  }
  position_out << std::endl;

  // wave elevation
  wave_elevation_out << std::fixed << std::setprecision(2) << current_time_
                     << '\t'; // first column is time
  wave_elevation_out << std::setprecision(5);
  wave_elevation_out << std::scientific << wave_vec_.back() << std::endl;

  // wave load
  wave_load_out << std::fixed << std::setprecision(2) << current_time_ << '\t';
  wave_load_out << std::setprecision(5);
  for (size_t index = 0; index < 6; ++index) {
    wave_load_out << std::scientific << wave_excitation_load_[index] << '\t';
  }
  for (size_t index = 0; index < 6; ++index) {
    wave_load_out << std::scientific << -radiation_damping_load_[index] << '\t';
  }
  for (size_t index = 0; index < 6; ++index) {
    wave_load_out << std::scientific << wave_drift_load_[index] << '\t';
  }
  wave_load_out << std::endl;

  // RAO motion
  rao_position_out << std::fixed << std::setprecision(2) << current_time_
                   << '\t';
  rao_position_out << std::setprecision(5);
  for (size_t index = 0; index < 6; ++index) {
    rao_position_out << std::scientific
                     << rao_based_position_history_.back().at(index) << '\t';
  }
  for (size_t index = 0; index < 6; ++index) {
    rao_position_out << std::scientific
                     << rao_based_velocity_history_.back().at(index) << '\t';
  }
  rao_position_out << std::endl;

  // close the files
  velocity_out.close();
  position_out.close();
  wave_elevation_out.close();
  wave_load_out.close();
  rao_position_out.close();
}

void Simulation::DividePrimaryMotion() {
  stepper_filter_.do_step(std::bind(&Simulation::LowPassFilterSystemFunction,
                                    this, std::placeholders::_1,
                                    std::placeholders::_2,
                                    std::placeholders::_3),
                          primary_state_, current_time_, step_size_);
}

void Simulation::LowPassFilterSystemFunction(const std::vector<double> &y,
                                             std::vector<double> &dy,
                                             const double t) {
  for (size_t i = 0; i < 6; i++) {
    dy[i] = y[i + 6];
  }

  for (size_t i = 6; i < 12; i++) {
    dy[i] = -2 * dividing_frequency_ * 0.4 * y[i] -
            dividing_frequency_ * dividing_frequency_ * y[i - 6] +
            dividing_frequency_ * dividing_frequency_ * vessel_state_.at(i);
  }
}

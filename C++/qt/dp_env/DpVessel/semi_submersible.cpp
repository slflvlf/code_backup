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

#include "semi_submersible.h"

void SemiSubmersible::WaveFrequencySystemFunction(
    const std::vector<double> &state, std::vector<double> &state_derivative,
    const double t) {
  Vector6d velocity, position;
  Vector12d dy;
  velocity << state[0], state[1], state[2], state[3], state[4], state[5];
  position << state[6], state[7], state[8], state[9], state[10], state[11];
  dy = WaveFrequencyStateDerivative(velocity, position, t);

  for (size_t index = 0; index < 12; ++index) {
    state_derivative[index] = dy[index];
  }
}

MarineStructure::Vector6d
SemiSubmersible::RestoringForce(const MarineStructure::Vector6d &vec) const {
  return restoring_matrix_ * vec;
}

MarineStructure::Vector6d SemiSubmersible::InstantaneousRadiationDamping(
    const MarineStructure::Vector6d &vec) const {
  return 0.5 * impulse_response_matrix_at_zero_ * vec *
         radiation_force_.getTimeStepSize();
}

MarineStructure::Vector12d SemiSubmersible::WaveFrequencyStateDerivative(
    const MarineStructure::Vector6d &velocity,
    const MarineStructure::Vector6d &position, const double) const {
  Vector12d state_derivative;
  Vector6d vec1, vec2;
  vec1 = total_mass_matrix_inv_ * (external_force_ - RestoringForce(position) -
                                   InstantaneousRadiationDamping(velocity));
  vec2 = velocity;
  state_derivative << vec1, vec2;

  return state_derivative;
}

SemiSubmersible::SemiSubmersible(
    const std::string &name, const double &mass,
    const std::array<double, 3> &gravity_center,
    const std::array<double, 3> &moment_of_inertia,
    const std::array<double, 3> &restoring_diagonal,
    std::string radiation_directory,
    const std::vector<double> &radiation_frequency, std::string wave_load_path,
    const std::vector<int> wave_load_direction, std::string rao_directory,
    const std::vector<double> &rao_frequency, QtfDataBase &qtf,
    WindLoad &wind_force, CurrentLoad &current_force,
    const std::array<double, 6> &init_velocity,
    const std::array<double, 6> &init_position, const bool &motion_rao_on)
    : MarineStructure(name, mass, gravity_center, moment_of_inertia,
                      init_velocity, init_position),
      wave_drift_(qtf), wind_force_(wind_force), current_force_(current_force),
      motion_rao_on_(motion_rao_on) {
  // define wave_excitation_
  wave_excitation_ =
      WaveExcitation(wave_load_path, rao_frequency, wave_load_direction);
  // define motion_rao_
  if (motion_rao_on)
    motion_rao_ = RaoDataBase(rao_directory, rao_frequency);
  // define radiation force
  radiation_force_ = RadiationForce(radiation_directory, radiation_frequency);
  // define hybrostatic matrix
  restoring_matrix_ =
      Matrix6d(((Vector6d() << 0, 0, restoring_diagonal.at(0),
                 restoring_diagonal.at(1), restoring_diagonal.at(2), 0)
                    .finished())
                   .asDiagonal());
}

void SemiSubmersible::InitializeSemiSubmersible(
    const WaveSpectrum &wave, const double &step_size,
    const double &radiation_cutoff_time) {
  // initialize impulse response function and infinite-frequency added mass
  // matrix
  radiation_force_.InitializeRaditionForce(step_size, radiation_cutoff_time);

  auto added_mass_inf = radiation_force_.getAddedMassInf();
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      added_mass_matrix_inf_(i, j) = added_mass_inf.at(i).at(j);
    }
  }

  total_mass_matrix_ = RigidBodyInertiaMatrix() + added_mass_matrix_inf_;
  total_mass_matrix_inv_ = total_mass_matrix_.inverse();

  // initialize RAO and wave excitation database
  if (motion_rao_on_) {
    motion_rao_.initializeRaoBasedMoton(wave);
  }
  wave_excitation_.initializeWaveExcitation(wave);

  // initialize wave drift
  wave_drift_.initializeWaveDriftForce(wave);

  // initialize K(0)
  auto impulse_response = radiation_force_.getImpulseResponse(0);
  for (int row = 0; row < 6; ++row) {
    for (int col = 0; col < 6; ++col) {
      impulse_response_matrix_at_zero_(row, col) =
          impulse_response.at(row).at(col);
    }
  }
}

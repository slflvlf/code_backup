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

#ifndef SEMI_SUBMERSIBLE_H
#define SEMI_SUBMERSIBLE_H

#include <Eigen>
#include <array>
#include <vector>
#include "marine_structure.h"
#include "qtf_database.h"
#include "radiation_force.h"
#include "rao_database.h"
#include "wave_excitation.h"
#include "wind_load.h"
#include "current_load.h"
#include "dpvessel_global.h"

// forward declaration
class WaveSpectrum;

struct DPVESSELSHARED_EXPORT SemiSubmersibleProperty
{
  std::string name;
  double mass;
  std::array<double, 3> gravity_center;
  std::array<double, 3> buoyancy_center;
  std::array<double, 3> moment_of_inertia;
  std::string data_directory;
  std::vector<int> direction_list;
  std::vector<double> radiation_frequency_list;
  std::vector<double> wave_load_frequency_list;
  std::array<double, 3> restoring_vector;
  bool is_diagonal;
};

class DPVESSELSHARED_EXPORT SemiSubmersible : public MarineStructure
{
  friend class Simulation;

private:
  // physical parameters
  Matrix6d total_mass_matrix_;
  Matrix6d total_mass_matrix_inv_;
  Matrix6d added_mass_matrix_inf_;
  Matrix6d restoring_matrix_;
  Matrix6d impulse_response_matrix_at_zero_; /**< K(0) */

  // motion rao and hydrodynamic data
  RaoDataBase motion_rao_;
  RadiationForce radiation_force_;
  WaveExcitation wave_excitation_;
  QtfDataBase &wave_drift_;

  // wave and current data
  WindLoad &wind_force_;
  CurrentLoad &current_force_;

  // whether motion rao is used to compute wave frequency response
  bool motion_rao_on_;

  // private methods
  virtual Vector6d RestoringForce(const Vector6d &vec) const override;  //!< returns restoring force based on
                                                                        //! hydrostatics

  /*!
  * \brief returns the instantaneous radiation damping force given by 0.5 * K(0) * dot(x)(t) * dt
  */
  Vector6d InstantaneousRadiationDamping(const Vector6d &vec) const;

  Vector12d WaveFrequencyStateDerivative(const Vector6d &velocity, const Vector6d &position,
                                         const double /* t */) const;  //!< time derivative of the system state

public:
  /*!
  * \brief default constructor
  */
  SemiSubmersible() = default;

  /*!
  * \brief a constructor
  */
  SemiSubmersible(const std::string &name, const double &mass, const std::array<double, 3> &gravity_center,
                  const std::array<double, 3> &moment_of_inertia, const std::array<double, 3> &restoring_diagonal,
                  std::string radiation_directory, const std::vector<double> &radiation_frequency,
                  std::string wave_load_path, const std::vector<int> wave_load_direction, std::string rao_directory,
                  const std::vector<double> &rao_frequency, QtfDataBase &qtf, WindLoad &wind_force,
                  CurrentLoad &current_force, const std::array<double, 6> &init_velocity,
                  const std::array<double, 6> &init_position, const bool &motion_rao_on);

  /*!
  * \brief a destructor
  */
  ~SemiSubmersible()
  {
  }

  /*!
  * \brief compute impulse response function and total mass matrix
  * \param wave  wave spectrum used to initialize vessel rao and wave excitation load
  * \param step_size  time step size of the simulation
  * \param radiation_cutoff_time  cutoff time of the cutoff scaling function for impulse response function
  */
  void InitializeSemiSubmersible(const WaveSpectrum &wave, const double &step_size,
                                 const double &radiation_cutoff_time);
  void WaveFrequencySystemFunction(const std::vector<double> &state, std::vector<double> &state_derivative,
                                   const double t);  //!< system function for using odeint stepper
};

#endif  // SEMI_SUBMERSIBLE_H

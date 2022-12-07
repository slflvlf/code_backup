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

#ifndef SIMULATION_H
#define SIMULATION_H

#include <array>
#include <boost/numeric/odeint.hpp>
#include <constant_current.h>
#include <constant_wind.h>
#include <jonswap_spectrum.h>
#include <semi_submersible.h>
#include <string>
#include <vector>

//* class Simulation
/**
 * The class defines a simple time domain simulation of a semi-submersible in a
 * Jonswap wave.
 */
class Simulation {
private:
  SemiSubmersible &vessel_;          /**< @todo use pointer and basic class */
  JonswapSpectrum &wave_;            /**< @todo use pointer and basic class */
  std::vector<double> vessel_state_; /**< total motion of the vessel */
  std::vector<double>
      primary_state_;       /**< low frequency primary motion of the vessel */
  double wave_direction_;   /**< wave direction in the global coordination in
                               degree */
  ConstantWind wind_;       /**< wind in the environment */
  ConstantCurrent current_; /**< current in the environment */

  // variables for simulations
  double step_size_ = 0.0;             /**< step size of the simulation */
  std::size_t step_counter_ = 0;       /**< counter of simulation steps */
  std::vector<double> time_vec_ = {0}; /**< time vector of the simulation */
  std::vector<double> wave_vec_; /**< wave elevation at the vessel origin during
                                    the simulation */
  double current_time_ =
      0.0; /**< elapsed time since the current simulation starts */
  std::vector<std::array<double, 6>>
      velocity_history_; /**< velocity history in the simulation */
  std::vector<std::array<double, 6>>
      position_history_; /**< position history in the simulation */
  std::deque<std::array<double, 6>>
      velocity_radiation_history_; /**< velocity history in the simulation */

  std::vector<std::array<double, 6>> rao_based_velocity_history_;
  std::vector<std::array<double, 6>> rao_based_position_history_;

  std::array<double, 6> wave_load_{{0, 0, 0, 0, 0, 0}};
  std::array<double, 6> wave_excitation_load_{{0, 0, 0, 0, 0, 0}};
  std::array<double, 6> radiation_damping_load_{{0, 0, 0, 0, 0, 0}};
  std::array<double, 6> wave_drift_load_{{0, 0, 0, 0, 0, 0}};
  std::array<double, 6> wind_load_{{0, 0, 0, 0, 0, 0}};
  std::array<double, 6> current_load_{{0, 0, 0, 0, 0, 0}};

  std::string output_directory_; /**< path to save output files */
  bool full_qtf_on_ =
      true; /**< whether full QTF is used to compute wave drift force */
  bool motion_rao_on_ =
      false; /**< whether motion RAO is used to compute wave frequency motion */

  // ode solvers
  boost::numeric::odeint::runge_kutta_dopri5<std::vector<double>> stepper_;
  boost::numeric::odeint::runge_kutta_dopri5<std::vector<double>>
      stepper_filter_;

  /*!
   * \brief filter the primary motion into its low frequency and wave frequency
   * components
   */
  void DividePrimaryMotion();

  double dividing_frequency_; /**< cutoff frequency of the filter to divide low
                               frequency motion from wave frequency motion */
  void LowPassFilterSystemFunction(
      const std::vector<double> &state, std::vector<double> &state_derivative,
      const double t); //!< system function for using odeint stepper

public:
  /*!
   * \brief default constructor
   */
  Simulation() = default;
  /*!
   * \brief a constructor
   */
  Simulation(SemiSubmersible &vessel, JonswapSpectrum &wave,
             const double &wave_direction, ConstantWind &wind,
             ConstantCurrent &current, const double &step_size,
             const double &radiation_cutoff_time, std::string save_path,
             const bool &full_QTF, const bool &motion_RAO);

  /*!
   * \brief destructor
   */
  ~Simulation() = default;

  /*!
   * \brief iniatilizes a new simulation by clearing up variables
   */
  void InitializeNewSimulation();

  /*!
   * \brief iniatilizes simulation parameters and class member variables
   */
  void InitializeMainSimulation();

  /*!
   * \brief conduct a one-step build-up simulation before the main simulation
   * \param build_up_time total build up time
   */
  void OneStepBuildUpSimulation(const double &build_up_time);

  /*!
   * \brief performance a one-step simulation
   */
  void OneStepSimulation();

  /*!
   * \brief conduct an iteration within one time step of build-up simulation
   * before the main simulation when vessel dynamics is coupled with mooring
   * system
   *
   * \param ramping_factor ramping factor in the build-up phase
   * \param counter time step counter
   * \param tolerance termination criterion
   */
  bool OneIterationBuildUpSimulation(const double &ramping_factor,
                                     const int &counter,
                                     const double &tolerance = 1E-7);

  /*!
   * \brief performance an iteration within one time step in the main simulation
   * when vessel dynamics is coupled with mooring system
   *
   * \param counter time step counter
   * \param tolerance termination criterion
   */
  bool OneIterationSimulation(const int &counter,
                              const double &tolerance = 1E-7);

  /*!
   * \brief set current_time
   */
  void setCurrentTime(const double &current_time) {
    current_time_ = current_time;
  }

  /*!
   * \brief get current_time
   */
  double getCurrentTime() { return current_time_; }

  /*!
   * \brief get wave direction
   */
  double getWaveDirection() { return wave_direction_; }

  /*!
   * \brief return vessel motion
   */
  std::vector<double> getVesselPosition() {
    std::vector<double> out;

    for (size_t i = 6; i < 12; i++) {
      out.push_back(vessel_state_.at(i));
    }

    return out;
  }

  /*!
   * \brief return vessel motion
   */
  std::vector<double> getVesselVelocity() {
    std::vector<double> out(vessel_state_.begin(), vessel_state_.begin() + 6);

    return out;
  }

  /*!
   * \brief add external force
   */
  void AddExternalForce(const std::array<double, 6> external_force) {
    vessel_.addExternalForce(external_force);
  }

  // save velocity and position data into text files
  void ExportData(const std::string &save_directory,
                  const std::string &mode = "trunc") const;
};

#endif // SIMULATION_H

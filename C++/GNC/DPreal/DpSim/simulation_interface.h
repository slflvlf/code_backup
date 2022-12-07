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

#ifndef SIMULATION_INTERFACE_H
#define SIMULATION_INTERFACE_H

#include <QObject>
#include <QVector>
#include <constant_current.h>
#include <constant_wind.h>
#include <current_load.h>
#include <jonswap_spectrum.h>
#include <memory>
#include <mooring_simulation.h>
#include <qtf_database.h>
#include <semi_submersible.h>
#include <simulation.h>
#include <string>
#include <vector>
#include <wind_load.h>

//* class SimulationInterface
/**
 * The class defines a time domain simulation interface for a semi-submersible
 * in a Jonswap wave.
 */
class SimulationInterface : public QObject {
  Q_OBJECT

private:
  std::unique_ptr<Simulation> ptr_simulation_; /**< main simulation platfrom */
  std::unique_ptr<MooringSimulation>
      ptr_mooring_;                           /**< mooring system simulation */
  std::unique_ptr<SemiSubmersible> ptr_semi_; /**< semi submersible */
  std::unique_ptr<JonswapSpectrum> ptr_wave_; /**< Jonswap wave */
  std::unique_ptr<QtfDataBase> ptr_qtf_;      /**< QTF of the semi */
  std::unique_ptr<WindLoad> ptr_wind_load_;   /**< wind load of the semi */
  std::unique_ptr<CurrentLoad>
      ptr_current_load_;                   /**< current load of the semi */
  std::unique_ptr<ConstantWind> ptr_wind_; /**< constant wind in simulation */
  std::unique_ptr<ConstantCurrent>
      ptr_current_; /**< constant current in simulation */

  std::array<double, 6> control_force_{
      {0, 0, 0, 0, 0, 0}}; /**< control force from thruster system */
  std::array<double, 6> mooring_force_{
      {0, 0, 0, 0, 0, 0}}; /**< mooring force from mooring node */

  // simulation parameters
  double build_up_time_ = 100.0;
  double simulation_time_ = 1800.0;
  double time_step_ = 0.1;
  std::string mode_;
  std::string output_directory_;

  // flags
  bool get_aborted_ =
      false; /**< whether simulation is requested to terminate */

signals:
  void messageReady(const QVector<double> &msg);

public slots:
  /*!
   * \brief start a new simulation
   */
  void StartSimulation();

public:
  /*!
   * \brief a constructor
   */
  SimulationInterface(const SemiSubmersibleProperty &vessel,
                      const MooringSystemProperty &mooring,
                      const JonswapSpectrumProperty &wave,
                      const ConstantWindProperty &wind,
                      const ConstantCurrentProperty &current,
                      const WindLoadData &wind_load,
                      const CurrentLoadData &current_load,
                      const double &build_up_time, const double &simulaton_time,
                      const double &step_size, std::string mode,
                      const std::array<double, 6> &init_position);

  /*!
   * \brief conduct a build-up simulation before the main simulation
   * \param build_up_time build up time
   */
  void BuildUpSimulation(const double &build_up_time);

  /*!
   * \brief performance a simulation
   * \param simulation_time main simulation time
   */
  void RunSimulation(const double &simulation_time);

  /*!
   * \brief a static member function which prints progress bar on a console
   */
  static void PrintProgBar(const size_t &percent);

  /*!
   * \brief save mooring force data to file
   */
  void ExportData(const std::string &save_directory,
                  const std::string &mode = "trunc") const;

  /*!
   * \brief publish messages
   */
  void PublishMessages();

  /*!
   * \brief set actuation
   * \param msg actuation msg
   */
  void setControlForce(const QVector<double> &msg) {
    control_force_.at(0) = msg.at(0);
    control_force_.at(1) = msg.at(1);
    control_force_.at(5) = msg.at(2);
  }

  /*!
   * \brief set abort flag to true
   */
  void setAbort() { get_aborted_ = true; }
};

#endif // SIMULATION_INTERFACE_H

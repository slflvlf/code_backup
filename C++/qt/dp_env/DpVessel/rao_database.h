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

#ifndef RAODATABASE_H
#define RAODATABASE_H

#include "dpvessel_global.h"
#include <array>
#include <vector>

// forward declaration
class WaveSpectrum;

//* class RaoDataBase
/**
 * RaoDataBase is a class, which stores motion RAOs, and computes rao-based
 * vessel motion
 */
class DPVESSELSHARED_EXPORT RaoDataBase {
private:
  unsigned int frequency_num_ = 0;
  unsigned int direction_num_ = 0;
  std::vector<double> frequency_; /**< RAO frequency list */
  std::vector<int> direction_;    /**< RAO direction list */
  std::array<std::vector<std::vector<double>>, 6>
      amplitude_; /**< RAO amplitude data */
  std::array<std::vector<std::vector<double>>, 6> phase_; /**< RAO phase data */
  std::array<std::vector<std::vector<double>>, 6>
      amplitude_interp_; /**< interpolated amplitude data at each wave
                            frequency */
  std::array<std::vector<std::vector<double>>, 6>
      phase_interp_; /**< interpolated phase data at each wave frequency */

  /*!
   * \brief returns interpolated rao data for single dof at each frequency
   *
   * \param data  RAO data for one dof (amplitude or phase)
   * \param frequency  frequency list at which RAO data are interpolated
   * \return interpolated rao matrix
   */
  std::vector<std::vector<double>>
  interpolateWaveFrequency(std::vector<std::vector<double>> &data,
                           const std::vector<double> &frequency);

  /*!
   * \brief convert phase angle from [-180, 180] to [0, 360]
   *
   * \param data  RAO phase data
   */
  void phaseTo360(std::vector<std::vector<double>> &data);

  // wave input
  std::vector<double> wave_frequency_; /**< frequency list of wave components */
  std::vector<double> wave_phase_;     /**< frequency list of wave components */
  std::vector<double> wave_amplitude_; /**< frequency list of wave components */
  std::vector<double> wave_num_;       /**< frequency list of wave components */
  unsigned int wave_component_num_ = 0;
  double gravity_ = 9.81;

public:
  /*!
   * \brief default constructor
   */
  RaoDataBase() = default;

  /*!
   * \brief a constructor
   */
  RaoDataBase(std::string, const std::vector<double> &);

  /*!
   * \brief destructor
   */
  ~RaoDataBase() {}

  void interpolateWaveFrequency(
      const std::vector<double>); //!< interpolate rao data at each wave
                                  //!< frequency

  /*!
   * \brief returns RAO amplitude and phase
   *
   * \param direction  wave direction relative to the vessel
   * \param frequency  wave frequency
   * \param dof        degree of freedom
   * \return RAO amplitude and phase
   */
  std::array<double, 2> getRAO(const double &direction, const double &frequency,
                               const unsigned int &dof) const;
  void initializeRaoBasedMoton(
      const WaveSpectrum &); //!< interpolate rao data at each wave frequency

  /*!
   * \brief returns RAO-based motion
   *
   * \param t  simulation time
   * \param x  x coordinate in the global reference frame
   * \param y  y coordinate in the global reference frame
   * \param beta  wave propagation direction in the global reference frame
   * \param U  vessel forward speed
   * \param psi  vessel heading direction
   * \return RAO amplitude and phase
   */
  std::array<std::array<double, 6>, 2>
  getRaoBasedMotion(const double &t, const double &x, const double &y,
                    const double &beta, const double &U,
                    const double &psi) const;
};

#endif // RAODATABASE_H

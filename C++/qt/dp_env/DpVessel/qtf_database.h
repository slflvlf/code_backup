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

#ifndef QTF_DATABASE_H
#define QTF_DATABASE_H

#include <array>
#include <string>
#include <vector>
#include "dpvessel_global.h"

// forward declaration
class WaveSpectrum;

class DPVESSELSHARED_EXPORT QtfDataBase
{
private:
  int frequency_num_ = 0;
  int direction_num_ = 0;
  std::vector<double> frequency_;                                         /**< RAO frequency list */
  std::vector<int> direction_;                                            /**< RAO direction list */
  std::array<std::vector<std::vector<std::vector<double>>>, 6> p_;        /**< QTF P data */
  std::array<std::vector<std::vector<std::vector<double>>>, 6> q_;        /**< QTF Q data */
  std::array<std::vector<std::vector<std::vector<double>>>, 6> p_interp_; /**< interpolated P data at
                                                                         each wave
                                                                         frequency pair */
  std::array<std::vector<std::vector<std::vector<double>>>, 6> q_interp_; /**< interpolated Q data at each wave
                                                                              frequency pair */
  std::array<std::vector<std::vector<double>>, 6> p_diag_;                /**< P at the diagonal */
  std::array<std::vector<std::vector<double>>, 6> p_diag_interp_;         /**< interpolated P at the diagonal */

  bool is_diagonal_ = false; /**< whether only diagonal data is given */

  /*!
  * \brief returns interpolated QTF data for single dof at each frequency pair
  *
  * \param data  QTF data for one dof (P or Q)
  * \param frequency  frequency list at which RAO data are interpolated
  * \return interpolated rao matrix
  */
  std::vector<std::vector<std::vector<double>>>
  interpolateWaveFrequency(std::vector<std::vector<std::vector<double>>> &data, const std::vector<double> &frequency);

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
  QtfDataBase() = default;

  /*!
  * \brief a constructor
  */
  QtfDataBase(const std::string &, const std::vector<double> &);

  /*!
  * \brief overloaded constructor
  */
  QtfDataBase(const std::string &, const std::vector<double> &, const std::vector<int> &);

  /*!
  * \brief destructor
  */
  ~QtfDataBase()
  {
  }

  void interpolateWaveFrequency(const std::vector<double> &);  //!< interpolate QTF data at each wave frequency pair
  void initializeWaveDriftForce(const WaveSpectrum &);         //!< interpolate QTF data at each wave frequency pair and
                                                               //! direction

  /*!
  * \brief returns QTF P and Q
  *
  * \param direction  wave direction relative to the vessel
  * \param frequency_1 wave frequency 1
  * \param frequency_2 wave frequency 2
  * \param dof        degree of freedom
  * \return QTF P and Q
  */
  std::array<double, 2> getPandQ(const double &direction, const double &frequency_1, const double &frequency_2,
                                 const int &dof) const;

  /*!
  * \brief returns QTF P at certain direction and frequency when only diagonal P is given
  *
  * \param direction  wave direction relative to the vessel
  * \param frequency  wave frequency
  * \param dof        degree of freedom
  * \return QTF P
  */
  double getDiagonalP(const double &direction, const double &frequency, const int &dof) const;

  /*!
  * \brief returns a vector of wave drift load based on Full QTF.
  *
  * \param t  simulation time
  * \param x  x coordinate in the global reference frame
  * \param y  y coordinate in the global reference frame
  * \param beta  wave propagation direction in the global reference frame
  * \param U  vessel forward speed
  * \param psi  vessel heading direction
  * \return a vector of wave excitation load
  */
  std::array<double, 6> getWaveDriftForceFullQTF(const double &t, const double &x, const double &y, const double &beta,
                                                 const double &U, const double &psi) const;

  /*!
  * \brief returns a vector of wave drift load based on Newman's approximation.
  * Wave drift damping is considered for surge and sway direction based on Aranha factors
  * The vessel speed must be small compared to the wave speed
  *
  * \param t  simulation time
  * \param x  x coordinate in the global reference frame
  * \param y  y coordinate in the global reference frame
  * \param beta  wave propagation direction in the global reference frame
  * \param U  vessel forward speed
  * \param psi  vessel heading direction
  * \return a vector of wave excitation load
  */
  std::array<double, 6> getWaveDriftForceNewmanApproximation(const double &t, const double &x, const double &y,
                                                             const double &beta, const double &U,
                                                             const double &psi) const;
};

#endif  // QTF_DATABASE_H

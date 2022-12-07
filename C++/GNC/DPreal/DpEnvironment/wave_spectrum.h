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

#ifndef WAVE_SPECTRUM_H
#define WAVE_SPECTRUM_H

#include <vector>
#include "dpenvironment_global.h"

//* class WaveSpectrum
/**
 * WaveSpectrum is a base class, which defines the basic properties of a
 * wave spectrum.
 */
class DPENVIRONMENTSHARED_EXPORT WaveSpectrum
{
public:
  /*!
   * \brief default contructor
   */
  WaveSpectrum() = default;

  /*!
   * \brief contructor
   */
  WaveSpectrum(const double &Hs, const double &T0) : HS_(Hs), T0_(T0)
  {
  }
  /*!
   * \brief a destructor
   */
  ~WaveSpectrum()
  {
  }

  /*!
   * \brief a virtual function to calculate wave spectrum velue [m^2 s]
   */
  virtual double calcWaveSpec(const double &) const = 0;
  virtual std::vector<double> calcWaveSpec(const std::vector<double> &) const = 0;

  /*!
   * \brief function to define the harmonic wave components
   *
   * \param freq_cutoff   Cutoff frequency of spectrum
   * \param freq_num      Number of frequency components
   */
  void initializeWaveComponents(const double &freq_cutoff, const int &freq_num);

  /*!
   * \brief function to define a regular wave component
   *
   * \param freq  wave frequency
   * \param amp   wave amplitude
   * \param phase  wave phase
   */
  void initializeWaveComponents(const double &freq, const double &amp, const double &phase);

  /*!
   * \brief a virtual function to calculate wave elevation at a given time and position
   *
   *
   * \param t     time
   * \param x     x coordinate of vessel in the earth-fixed frame
   * \param y     y coordinate of vessel in the earth-fixed frame
   * \param beta  wave direction
   * \param U     vessel forward speed
   * \param psi   vessel heading
   */
  virtual double getWaveElevation(const double &t, const double &x, const double &y, const double &beta,
                                  const double &U, const double &psi) const;

  bool isInitialized() const
  {
    return initialized_;
  }

  std::vector<double> getWaveFrequency() const
  {
    return freq_;
  }

  int getWaveComponentNum() const
  {
    return freq_.size();
  }

  std::vector<double> getWaveAmplitude() const
  {
    return amp_;
  }

  std::vector<double> getWaveNum() const
  {
    return wave_num_;
  }

  std::vector<double> getWavePhase() const
  {
    return phase_;
  }

  double getGravity() const
  {
    return GRAVITY_;
  }

protected:
  // parameters
  const double GRAVITY_ = 9.81;
  double HS_ = 0.0;          /**< significant wave height */
  double T0_ = 0.0;          /**< modal period (peak period) */
  bool initialized_ = false; /**< flag indicating whether wave components have been initialized */

  // wave components
  std::vector<double> freq_;     /**< vector of wave freqencies */
  std::vector<double> amp_;      /**< vector of wave amplitudes */
  std::vector<double> phase_;    /**< the vector of wave phases */
  std::vector<double> wave_num_; /**< the vector of wave numbers */
};

#endif  // WAVE_SPECTRUM_H

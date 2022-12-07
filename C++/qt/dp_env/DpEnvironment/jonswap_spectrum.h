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

#ifndef JONSWAP_SPECTRUM_H
#define JONSWAP_SPECTRUM_H

#include "wave_spectrum.h"
#include "dpenvironment_global.h"

struct DPENVIRONMENTSHARED_EXPORT JonswapSpectrumProperty
{
  double Hs;
  double T0;
  double gamma;
  double cutoff_frequency;
  int component_number;
  double direction;
};

//* class JonswapSpectrum
/**
 * JonswapSpectrum is a class derived from WaveSpectrum, which defines the
 * Joint North Sea Wave Project (JONSWAP) spectrum.
 */
class DPENVIRONMENTSHARED_EXPORT JonswapSpectrum : public WaveSpectrum
{
  double T1_ = 0.0;    /**< average period */
  double gamma_ = 3.3; /**< peak enhancement factor */

public:
  /*!
   * \brief default contructor
   */
  JonswapSpectrum() = default;

  /*!
   * \brief nondelegating contructor
   */
  JonswapSpectrum(const double &Hs, const double &T0, const double &gamma) : WaveSpectrum(Hs, T0), gamma_(gamma)
  {
    T1_ = 0.834 * T0_;
  }

  /*!
   * \brief a delegating contructor
   */
  JonswapSpectrum(const double &Hs, const double &T0) : JonswapSpectrum(Hs, T0, 3.3)
  {
  }

  /*!
   * \brief a destructor
   */
  ~JonswapSpectrum()
  {
  }

  /*!
   * \brief a virtual function to calculate wave spectrum velue [m^2 s]
   *
   * \param omega    Vector of harmonic wave frequency (rad/s)
   */
  virtual std::vector<double> calcWaveSpec(const std::vector<double> &omega) const override;

  virtual double calcWaveSpec(const double &) const override;
};

#endif  // JONSWAP_SPECTRUM_H

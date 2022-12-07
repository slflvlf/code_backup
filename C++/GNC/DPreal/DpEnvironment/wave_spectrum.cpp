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

#include "wave_spectrum.h"
#include <cmath>
#include <cstdlib>
#include <vector>

double WaveSpectrum::getWaveElevation(const double &t, const double &x, const double &y, const double &beta,
                                      const double &U, const double &psi) const
{
  double wave_elevation = 0;

  // loop over all wave components
  for (std::size_t n = 0; n < freq_.size(); ++n)
  {
    wave_elevation +=
        amp_.at(n) *
        std::cos(std::fabs(freq_.at(n) - std::pow(freq_.at(n), 2) * U / GRAVITY_ * std::cos(beta - psi)) * t -
                 wave_num_.at(n) * (x * std::cos(beta) + y * std::sin(beta)) + phase_.at(n));
  }

  return wave_elevation;
}

void WaveSpectrum::initializeWaveComponents(const double &freq_cutoff, const int &freq_num)
{
  freq_ = std::vector<double>(freq_num, 0);
  amp_ = freq_;
  phase_ = freq_;
  wave_num_ = freq_;

  double freq_interval = freq_cutoff / freq_num;
  for (std::size_t i = 0; i < freq_.size(); ++i)
  {
    freq_.at(i) =
        freq_interval * (i + rand() % 1001 / 1000.0);    // generate random wave frequencies in each freqency interval
    phase_.at(i) = 2 * M_PI * (rand() % 1001 / 1000.0);  // generate random phase of each wave component
    amp_.at(i) = std::sqrt(2 * calcWaveSpec(freq_.at(i)) * freq_interval);  // calculate wave amplitudes
    wave_num_.at(i) = freq_.at(i) * freq_.at(i) / GRAVITY_;  // calculate wave numbers, infinite water depth
  }
  if (freq_.at(0) <= 0.01)  // lower limit
  {
    freq_.at(0) = 0.01;
  }

  initialized_ = true;
}

void WaveSpectrum::initializeWaveComponents(const double &freq, const double &amp, const double &phase)
{
  freq_ = std::vector<double>{ freq };
  amp_ = std::vector<double>{ amp };
  phase_ = std::vector<double>{ phase };
  wave_num_ = std::vector<double>{ freq_.front() * freq_.front() / GRAVITY_ };

  initialized_ = true;
}

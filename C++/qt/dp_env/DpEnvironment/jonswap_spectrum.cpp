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

#include "jonswap_spectrum.h"
#include <cmath>
#include <vector>

std::vector<double> JonswapSpectrum::calcWaveSpec(const std::vector<double> &omega) const
{
  std::vector<double> wave_spec;
  double sigma, r;

  for (std::size_t i = 0; i < omega.size(); ++i)
  {
    if (omega[i] <= 5.24 / T1_)
    {
      sigma = 0.07;
    }
    else
    {
      sigma = 0.09;
    }

    r = std::exp(-std::pow((0.191 * omega[i] * T1_ - 1) / std::sqrt(2) / sigma, 2));

    wave_spec.push_back(155 * std::pow(HS_, 2) / std::pow(T1_, 4) / std::pow(omega[i], 5) *
                        std::exp(-944 / std::pow(T1_, 4) / std::pow(omega[i], 4)) * std::pow(gamma_, r));
  }

  return wave_spec;
}

double JonswapSpectrum::calcWaveSpec(const double &omega) const
{
  double sigma, r;

  if (omega <= 5.24 / T1_)
  {
    sigma = 0.07;
  }
  else
  {
    sigma = 0.09;
  }

  r = std::exp(-std::pow((0.191 * omega * T1_ - 1) / std::sqrt(2) / sigma, 2));

  return 155 * std::pow(HS_, 2) / std::pow(T1_, 4) / std::pow(omega, 5) *
         std::exp(-944 / std::pow(T1_, 4) / std::pow(omega, 4)) * std::pow(gamma_, r);
}

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

#include "wave_excitation.h"
#include "wave_spectrum.h"

#include <QDir>
#include <QFile>
#include <QString>
#include <QVector>
#include <cmath>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_spline.h>
#include <stdexcept>

WaveExcitation::WaveExcitation(std::string path,
                               const std::vector<double> &frequency,
                               const std::vector<int> &direction)
    : frequency_(frequency), direction_(direction) {
  frequency_num_ = frequency.size();
  direction_num_ = direction.size();

  QString fileName = QString::fromStdString(path);
  // read the data
  QFile file(fileName);
  if (!file.open(QIODevice::ReadOnly)) {
    throw std::runtime_error(file.errorString().toStdString());
  }

  QVector<double> dataList;
  while (!file.atEnd()) {
    QByteArray line = file.readLine();
    line.replace("\r", "");
    QList<QByteArray> data_temp = line.split('\n');
    QList<QByteArray> data = data_temp.at(0).split(',');

    foreach (QByteArray s, data) { dataList.append(s.toDouble()); }
  }

  // check the size of the data
  if (dataList.size() != frequency_num_ * 6 * direction_num_ * 5) {
    throw std::runtime_error(
        "The size of dataset is not correct. Check the file!");
  }

  for (int dof = 0; dof < 6; ++dof) {
    std::vector<std::vector<double>> amp_temp2, phase_temp2;
    for (int i = 0; i < direction_num_; ++i) {
      std::vector<double> amp_temp, phase_temp;
      for (int j = 0; j < frequency_num_; ++j) {
        amp_temp.push_back(
            dataList.at(dof * frequency_num_ * direction_num_ * 5 +
                        i * frequency_num_ * 5 + j * 5 + 3));
        phase_temp.push_back(
            dataList.at(dof * frequency_num_ * direction_num_ * 5 +
                        i * frequency_num_ * 5 + j * 5 + 4));
      }
      amp_temp2.push_back(amp_temp);
      phase_temp2.push_back(phase_temp);
      amp_temp.clear();
      phase_temp.clear();
    }
    amplitude_.at(dof) = amp_temp2;
    phase_.at(dof) = phase_temp2;
    amp_temp2.clear();
    phase_temp2.clear();
  }
}

void WaveExcitation::phaseTo360(std::vector<std::vector<double>> &data) {
  for (auto &row : data) {
    for (auto &phase : row) {
      phase += (phase < 0) ? 360 : 0;
    }
  }
}

std::vector<std::vector<double>> WaveExcitation::interpolateWaveFrequency(
    std::vector<std::vector<double>> &data,
    const std::vector<double> &wave_frequency) {
  std::vector<std::vector<double>> data_interp;
  wave_frequency_ = wave_frequency;
  if (wave_frequency_.back() > frequency_.back() ||
      wave_frequency_.front() < frequency_.front()) {
    throw std::runtime_error("Extrapolation is not supported!");
  }
  gsl_interp_accel *acc = gsl_interp_accel_alloc();
  //    gsl_spline *spline = gsl_spline_alloc(gsl_interp_linear,
  //    frequency_num_);
  gsl_spline *spline = gsl_spline_alloc(gsl_interp_steffen, frequency_num_);

  // interpolate for each direction
  for (unsigned int i = 0; i < direction_num_; ++i) {
    double x[frequency_num_], y[frequency_num_];
    for (unsigned int j = 0; j < frequency_num_; ++j) {
      x[j] = frequency_.at(j);
      y[j] = (data.at(i)).at(j);
    }
    gsl_spline_init(spline, x, y, frequency_num_);

    std::vector<double> temp;
    for (double freq : wave_frequency) {
      temp.push_back(gsl_spline_eval(spline, freq, acc));
    }
    data_interp.push_back(temp);
    gsl_interp_accel_reset(acc);
    temp.clear();
  }
  gsl_spline_free(spline);
  gsl_interp_accel_free(acc);

  return data_interp;
}

void WaveExcitation::interpolateWaveFrequency(
    const std::vector<double> &wave_frequency) {
  // interpolate RAO amplitudes
  for (unsigned int i = 0; i < 6; ++i) {
    amplitude_interp_.at(i) =
        interpolateWaveFrequency(amplitude_.at(i), wave_frequency);
  }

  // interploate RAO phases
  for (unsigned int i = 0; i < 6; ++i) {
    phaseTo360(phase_.at(i));
    phase_interp_.at(i) =
        interpolateWaveFrequency(phase_.at(i), wave_frequency);
  }
}

std::array<double, 2> WaveExcitation::getRAO(const double &direction,
                                             const double &frequency,
                                             const unsigned int &dof) const {
  unsigned int freq_index = 0;
  std::array<double, 2> output;

  // check whether frequency is in the list
  for (freq_index = 0; freq_index < wave_frequency_.size(); ++freq_index) {
    if (std::fabs(wave_frequency_.at(freq_index) - frequency) < 1E-5) {
      break;
    }
  }

  if (freq_index == wave_frequency_.size()) {
    throw std::runtime_error(
        "Input frequency is not in wave frequency vector!");
  }

  std::vector<std::vector<double>> amplitude = amplitude_interp_.at(dof);
  std::vector<std::vector<double>> phase = phase_interp_.at(dof);
  std::vector<double> amp_dir, phase_dir;

  for (unsigned int i = 0; i < direction_num_; ++i) {
    amp_dir.push_back(amplitude.at(i).at(freq_index));
    phase_dir.push_back(phase.at(i).at(freq_index));
  }

  gsl_interp_accel *acc = gsl_interp_accel_alloc();
  //    gsl_spline *spline = gsl_spline_alloc(gsl_interp_linear,
  //    frequency_num_);
  gsl_spline *spline = gsl_spline_alloc(gsl_interp_steffen, direction_num_);

  // interpolate amplitude
  double x[direction_num_], y_amp[direction_num_], y_phase[direction_num_];
  for (unsigned int j = 0; j < direction_num_; ++j) {
    x[j] = direction_.at(j);
    y_amp[j] = amp_dir.at(j);
    y_phase[j] = phase_dir.at(j);
  }
  gsl_spline_init(spline, x, y_amp, direction_num_);
  output.at(0) = gsl_spline_eval(spline, direction, acc);
  gsl_spline_init(spline, x, y_phase, direction_num_);
  output.at(1) = gsl_spline_eval(spline, direction, acc);

  gsl_spline_free(spline);
  gsl_interp_accel_free(acc);

  return output;
}

void WaveExcitation::initializeWaveExcitation(const WaveSpectrum &wave) {
  if (!wave.isInitialized()) {
    throw std::runtime_error("Initialize wave components first!");
  }

  interpolateWaveFrequency(wave.getWaveFrequency());
  wave_amplitude_ = wave.getWaveAmplitude();
  wave_phase_ = wave.getWavePhase();
  wave_num_ = wave.getWaveNum();
  wave_component_num_ = wave_amplitude_.size();
  gravity_ = wave.getGravity();
}

std::array<double, 6>
WaveExcitation::getWaveExcitation(const double &t, const double &x,
                                  const double &y, const double &beta,
                                  const double &U, const double &psi) const {
  std::array<double, 6> output;

  // wave direction
  double direction = beta - psi; // in radians
  direction = std::atan2(std::sin(direction),
                         std::cos(direction)); // wrap the angle to [-pi, pi]
  double abs_direction = std::fabs(direction);

  double wave_excitation;

  for (unsigned int dof = 0; dof < 6; ++dof) {
    wave_excitation = 0;

    // loop over all wave components
    // note that negative encounter frequency is not considered yet
    for (std::size_t n = 0; n < wave_component_num_; ++n) {
      auto rao = getRAO(abs_direction * 180 / M_PI, wave_frequency_.at(n), dof);

      // sign of direction
      if (direction < 0) {
        if (dof == 1 || dof == 3 || dof == 5) {
          rao.at(1) += 180;
        }
      }

      wave_excitation +=
          wave_amplitude_.at(n) * rao.at(0) *
          std::cos(std::fabs(wave_frequency_.at(n) -
                             std::pow(wave_frequency_.at(n), 2) * U / gravity_ *
                                 std::cos(direction)) *
                       t -
                   wave_num_.at(n) * (x * std::cos(beta) + y * std::sin(beta)) +
                   wave_phase_.at(n) + rao.at(1) * M_PI / 180);
    }

    output.at(dof) = wave_excitation;
  }

  return output;
}

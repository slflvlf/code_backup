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

#include "qtf_database.h"
#include "wave_spectrum.h"

#include <QDir>
#include <QFile>
#include <QString>
#include <QVector>
#include <cmath>
#include <complex>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_interp2d.h>
#include <gsl/gsl_spline.h>
#include <gsl/gsl_spline2d.h>
#include <stdexcept>

#include <iostream>

std::vector<std::vector<std::vector<double>>>
QtfDataBase::interpolateWaveFrequency(
    std::vector<std::vector<std::vector<double>>> &data,
    const std::vector<double> &wave_frequency) {
  std::vector<std::vector<std::vector<double>>> data_interp;
  wave_frequency_ = wave_frequency;
  if (wave_frequency_.back() > frequency_.back() ||
      wave_frequency_.front() < frequency_.front()) {
    throw std::runtime_error("Extrapolation is not supported!");
  }
  const gsl_interp2d_type *T = gsl_interp2d_bilinear;

  // grid points
  double xa[frequency_num_], ya[frequency_num_];
  auto *za = (double *)malloc(frequency_num_ * frequency_num_ * sizeof(double));
  gsl_spline2d *spline = gsl_spline2d_alloc(T, frequency_num_, frequency_num_);
  gsl_interp_accel *xacc = gsl_interp_accel_alloc();
  gsl_interp_accel *yacc = gsl_interp_accel_alloc();

  for (int i = 0; i < frequency_num_; ++i) {
    // set grid point
    xa[i] = frequency_.at(i);
    ya[i] = frequency_.at(i);
  }

  for (int dir_index = 0; dir_index < direction_num_; ++dir_index) {
    for (int i = 0; i < frequency_num_; ++i) {
      // se grid values
      for (int j = 0; j < frequency_num_; ++j) {
        gsl_spline2d_set(spline, za, i, j, data.at(dir_index).at(i).at(j));
      }
    }
    gsl_spline2d_init(spline, xa, ya, za, frequency_num_, frequency_num_);

    // interpolate
    std::vector<double> temp1;
    std::vector<std::vector<double>> temp2;
    for (unsigned int i = 0; i < wave_frequency_.size(); ++i) {
      for (unsigned int j = 0; j < wave_frequency_.size(); ++j) {
        double zij = gsl_spline2d_eval(spline, wave_frequency_.at(i),
                                       wave_frequency_.at(j), xacc, yacc);
        temp1.push_back(zij);
      }
      temp2.push_back(temp1);
      temp1.clear();
    }
    data_interp.push_back(temp2);
    temp2.clear();
    gsl_interp_accel_reset(xacc);
    gsl_interp_accel_reset(yacc);
  }

  gsl_spline2d_free(spline);
  gsl_interp_accel_free(xacc);
  gsl_interp_accel_free(yacc);
  free(za);

  return data_interp;
}

QtfDataBase::QtfDataBase(const std::string &path,
                         const std::vector<double> &frequency)
    : frequency_(frequency) {
  // list all the data files in the directory
  QDir directory(QString::fromStdString(path));
  if (!directory.exists()) {
    qWarning("Cannot find the data directory!");
  }

  directory.setFilter(QDir::Files | QDir::NoSymLinks | QDir::NoDot |
                      QDir::NoDotAndDotDot | QDir::NoDotDot);
  QFileInfoList fileInfoList = directory.entryInfoList();

  QList<QString> qtfFileList;
  QList<int> qtfDirection;

  for (int i = 0; i < fileInfoList.size(); ++i) {
    QFileInfo fileInfo = fileInfoList.at(i);
    QString fileName = fileInfo.fileName();

    if (fileName.contains("2nd")) {
      qtfFileList.push_back(fileName);
      qtfDirection.push_back(fileName.split("_").front().toDouble());
    }
  }
  // sort the directions
  std::sort(qtfDirection.begin(), qtfDirection.end());

  frequency_num_ = frequency.size();

  foreach (auto dir, qtfDirection) { direction_.push_back(dir); }
  direction_num_ = direction_.size();

  // read QTF data from data files,
  // save them into two separate variables (i.e., P and Q) for each dof
  // prepare the file names
  for (int dof = 0; dof < 6; ++dof) {
    std::vector<std::vector<std::vector<double>>> p_temp3, q_temp3;
    std::vector<std::vector<double>> p_diag_temp2;
    for (int i = 0; i < direction_num_; ++i) {
      QString fileName = directory.absolutePath() + "/" +
                         QString::number(qtfDirection.at(i)) + "_2nd.csv";
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
        QList<QByteArray> data = data_temp.at(0).split(';');

        if (data.size() != 7)
          break;

        foreach (QByteArray s, data) {
          dataList.append(s.trimmed().toDouble());
        }
      }

      // check the size of the data
      if (dataList.size() != frequency_num_ * frequency_num_ * 6 * 7)
        throw std::runtime_error(
            "The size of dataset is not correct. Check the file!");

      std::vector<double> p_temp1, q_temp1, p_diag_temp1;
      std::vector<std::vector<double>> p_temp2, q_temp2;

      for (int j = 0; j < frequency_num_; ++j) {
        for (int k = 0; k < frequency_num_; ++k) {
          p_temp1.push_back(
              dataList.at(frequency_num_ * frequency_num_ * dof * 7 +
                          frequency_num_ * j * 7 + 7 * k + 3));
          q_temp1.push_back(
              dataList.at(frequency_num_ * frequency_num_ * dof * 7 +
                          frequency_num_ * j * 7 + 7 * k + 4));
          if (j == k) {
            p_diag_temp1.push_back(
                dataList.at(frequency_num_ * frequency_num_ * dof * 7 +
                            frequency_num_ * j * 7 + 7 * k + 3));
          }
        }
        p_temp2.push_back(p_temp1);
        q_temp2.push_back(q_temp1);
        p_temp1.clear();
        q_temp1.clear();
      }
      p_temp3.push_back(p_temp2);
      q_temp3.push_back(q_temp2);
      p_temp2.clear();
      q_temp2.clear();
      p_diag_temp2.push_back(p_diag_temp1);
      p_diag_temp1.clear();
    }
    p_.at(dof) = p_temp3;
    q_.at(dof) = q_temp3;
    p_temp3.clear();
    q_temp3.clear();
    p_diag_temp2.clear();
  }
}

QtfDataBase::QtfDataBase(const std::string &path,
                         const std::vector<double> &frequency,
                         const std::vector<int> &direction)
    : frequency_(frequency), direction_(direction) {
  is_diagonal_ = true;
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
  if (dataList.size() != frequency_num_ * 6 * direction_num_ * 2)
    throw std::runtime_error(
        "The size of dataset is not correct. Check the file!");

  for (int dof = 0; dof < 6; ++dof) {
    std::vector<std::vector<double>> p_temp2;
    for (int i = 0; i < direction_num_; ++i) {
      std::vector<double> p_temp;
      for (int j = 0; j < frequency_num_; ++j) {
        p_temp.push_back(dataList.at(frequency_num_ * direction_num_ * dof * 2 +
                                     frequency_num_ * i * 2 + j * 2 + 1));
      }
      p_temp2.push_back(p_temp);
      p_temp.clear();
    }
    p_diag_.at(dof) = p_temp2;
    p_temp2.clear();
  }
}

void QtfDataBase::interpolateWaveFrequency(
    const std::vector<double> &wave_frequency) {
  if (!is_diagonal_) {
    // interpolate QTF P
    for (int i = 0; i < 6; ++i) {
      p_interp_.at(i) = interpolateWaveFrequency(p_.at(i), wave_frequency);
    }

    // interploate QTF Q
    for (int i = 0; i < 6; ++i) {
      q_interp_.at(i) = interpolateWaveFrequency(q_.at(i), wave_frequency);
    }
  } else {
    wave_frequency_ = wave_frequency;
    gsl_interp_accel *acc = gsl_interp_accel_alloc();
    gsl_spline *spline = gsl_spline_alloc(gsl_interp_steffen, frequency_num_);

    // interpolate for each direction
    for (int dof = 0; dof < 6; ++dof) {
      std::vector<std::vector<double>> data = p_diag_.at(dof);
      std::vector<std::vector<double>> temp2;
      for (int i = 0; i < direction_num_; ++i) {
        double x[frequency_num_], y[frequency_num_];

        for (int j = 0; j < frequency_num_; ++j) {
          x[j] = frequency_.at(j);
          y[j] = (data.at(i)).at(j);
        }
        gsl_spline_init(spline, x, y, frequency_num_);

        std::vector<double> temp;
        for (double freq : wave_frequency) {
          temp.push_back(gsl_spline_eval(spline, freq, acc));
        }
        temp2.push_back(temp);
        gsl_interp_accel_reset(acc);
        temp.clear();
      }
      p_diag_interp_.at(dof) = temp2;
      temp2.clear();
    }
    gsl_spline_free(spline);
    gsl_interp_accel_free(acc);
  }
}

void QtfDataBase::initializeWaveDriftForce(const WaveSpectrum &wave) {
  if (!wave.isInitialized()) {
    throw std::runtime_error("Initialize wave components first!");
  }

  interpolateWaveFrequency(
      wave.getWaveFrequency()); // interpolate frequency pair first

  wave_amplitude_ = wave.getWaveAmplitude();
  wave_phase_ = wave.getWavePhase();
  wave_num_ = wave.getWaveNum();
  wave_component_num_ = wave_amplitude_.size();
  gravity_ = wave.getGravity();
}

std::array<double, 2> QtfDataBase::getPandQ(const double &direction,
                                            const double &frequency1,
                                            const double &frequency2,
                                            const int &dof) const {
  unsigned int freq_index_1 = 0, freq_index_2 = 0;
  std::array<double, 2> output;

  // check whether frequency is in the list
  // first frequency
  for (freq_index_1 = 0; freq_index_1 < wave_frequency_.size();
       ++freq_index_1) {
    if (std::fabs(wave_frequency_.at(freq_index_1) - frequency1) < 1E-5) {
      break;
    }
  }

  if (freq_index_1 == wave_frequency_.size()) {
    throw std::runtime_error(
        "Input frequency is not in wave frequency vector!");
  }

  // second frequency
  for (freq_index_2 = 0; freq_index_2 < wave_frequency_.size();
       ++freq_index_2) {
    if (std::fabs(wave_frequency_.at(freq_index_2) - frequency2) < 1E-5) {
      break;
    }
  }

  if (freq_index_2 == wave_frequency_.size()) {
    throw std::runtime_error(
        "Input frequency is not in wave frequency vector!");
  }

  std::vector<double> p_dir, q_dir;

  for (int i = 0; i < direction_num_; ++i) {
    p_dir.push_back(p_interp_.at(dof).at(i).at(freq_index_1).at(freq_index_2));
    q_dir.push_back(q_interp_.at(dof).at(i).at(freq_index_1).at(freq_index_2));
  }

  gsl_interp_accel *acc = gsl_interp_accel_alloc();
  gsl_spline *spline = gsl_spline_alloc(gsl_interp_linear, direction_num_);

  double x[direction_num_], y_p[direction_num_], y_q[direction_num_];
  for (int j = 0; j < direction_num_; ++j) {
    x[j] = direction_.at(j);
    y_p[j] = p_dir.at(j);
    y_q[j] = q_dir.at(j);
  }
  gsl_spline_init(spline, x, y_p, direction_num_);
  output.at(0) = gsl_spline_eval(spline, direction, acc);
  gsl_spline_init(spline, x, y_q, direction_num_);
  output.at(1) = gsl_spline_eval(spline, direction, acc);

  gsl_spline_free(spline);
  gsl_interp_accel_free(acc);

  return output;
}

double QtfDataBase::getDiagonalP(const double &direction,
                                 const double &frequency,
                                 const int &dof) const {
  unsigned int freq_index;
  double output;

  // check whether frequency is in the list
  // first frequency
  for (freq_index = 0; freq_index < wave_frequency_.size(); ++freq_index) {
    if (std::fabs(wave_frequency_.at(freq_index) - frequency) < 1E-5) {
      break;
    }
  }

  if (freq_index == wave_frequency_.size()) {
    throw std::runtime_error(
        "Input frequency is not in wave frequency vector!");
  }

  std::vector<double> p_dir;

  for (int i = 0; i < direction_num_; ++i) {
    p_dir.push_back(p_diag_interp_.at(dof).at(i).at(freq_index));
  }

  gsl_interp_accel *acc = gsl_interp_accel_alloc();
  gsl_spline *spline = gsl_spline_alloc(gsl_interp_linear, direction_num_);

  double x[direction_num_], y_p[direction_num_];
  for (int j = 0; j < direction_num_; ++j) {
    x[j] = direction_.at(j);
    y_p[j] = p_dir.at(j);
  }
  gsl_spline_init(spline, x, y_p, direction_num_);
  output = gsl_spline_eval(spline, direction, acc);

  gsl_spline_free(spline);
  gsl_interp_accel_free(acc);

  return output;
}

std::array<double, 6> QtfDataBase::getWaveDriftForceFullQTF(
    const double &t, const double &x, const double &y, const double &beta,
    const double &U, const double &psi) const {
  std::array<double, 6> output;

  // relative direction
  double direction = beta - psi; // in radians
  direction = std::atan2(std::sin(direction),
                         std::cos(direction)); // wrap the angle to [-pi, pi]
  double abs_direction = std::fabs(direction);

  double wave_drift_load;

  for (int dof = 0; dof < 6; ++dof) {
    wave_drift_load = 0;

    // loop over all wave components
    // note that negative encounter frequency is not considered (assume vessel
    // velocity is small compared to the wave speed)
    for (std::size_t n = 0; n < wave_component_num_; ++n) {
      for (std::size_t m = n; m < wave_component_num_; ++m) {
        auto qtf = getPandQ(abs_direction * 180 / M_PI, wave_frequency_.at(n),
                            wave_frequency_.at(m), dof);

        if (m == n) {
          wave_drift_load +=
              wave_amplitude_.at(n) * wave_amplitude_.at(n) * qtf.at(0);
        } else {
          wave_drift_load +=
              2 * wave_amplitude_.at(n) * wave_amplitude_.at(m) * qtf.at(0) *
                  std::cos((std::fabs(wave_frequency_.at(n) -
                                      std::pow(wave_frequency_.at(n), 2) * U /
                                          gravity_ * std::cos(direction)) -
                            std::fabs(wave_frequency_.at(m) -
                                      std::pow(wave_frequency_.at(m), 2) * U /
                                          gravity_ * std::cos(direction))) *
                               t -
                           (wave_num_.at(n) - wave_num_.at(m)) *
                               (x * std::cos(beta) + y * std::sin(beta)) +
                           (wave_phase_.at(n) - wave_phase_.at(m))) +
              2 * wave_amplitude_.at(n) * wave_amplitude_.at(m) * qtf.at(1) *
                  std::sin((std::fabs(wave_frequency_.at(n) -
                                      std::pow(wave_frequency_.at(n), 2) * U /
                                          gravity_ * std::cos(direction)) -
                            std::fabs(wave_frequency_.at(m) -
                                      std::pow(wave_frequency_.at(m), 2) * U /
                                          gravity_ * std::cos(direction))) *
                               t -
                           (wave_num_.at(n) - wave_num_.at(m)) *
                               (x * std::cos(beta) + y * std::sin(beta)) +
                           (wave_phase_.at(n) - wave_phase_.at(m)));
        }
      }
    }

    output.at(dof) = wave_drift_load;

    // sign of direction
    if (direction < 0) {
      if (dof == 1 || dof == 3 || dof == 5) {
        output.at(dof) *= -1;
      }
    }
  }

  return output;
}

std::array<double, 6> QtfDataBase::getWaveDriftForceNewmanApproximation(
    const double &t, const double &x, const double &y, const double &beta,
    const double &U, const double &psi) const {
  std::array<double, 6> output;

  // relative direction
  double direction = beta - psi; // in radians
  direction = std::atan2(std::sin(direction),
                         std::cos(direction)); // wrap the angle to [-pi, pi]

  std::complex<double> wave_drift_load(0, 0);
  std::complex<double> wave_drift_load_helper(0, 0);
  double direction_e, A_e;

  for (int dof = 0; dof < 6; ++dof) {
    wave_drift_load = std::complex<double>(0, 0);
    wave_drift_load_helper = std::complex<double>(0, 0);

    // loop over all wave components
    // note that negative encounter frequency is not considered (assume vessel
    // velocity is small compared to the wave speed)
    for (std::size_t n = 0; n < wave_component_num_; ++n) {
      direction_e = direction + 2 * wave_frequency_.at(n) / gravity_ * U *
                                    std::sin(direction); // encounter direction

      if ((dof == 0) | (dof == 1)) {
        A_e = 1 - 4 * wave_frequency_.at(n) / gravity_ * U *
                      std::cos(direction); // Aranha scaling factor
        A_e = (A_e < 0) ? 0 : A_e; // Aranha scaling factor cannot be negative
      } else {
        A_e = 1;
      }

      double P;
      if (!is_diagonal_) {
        auto qtf = getPandQ(std::fabs(direction_e) * 180 / M_PI,
                            std::fabs(wave_frequency_.at(n) -
                                      std::pow(wave_frequency_.at(n), 2) * U /
                                          gravity_ * std::cos(direction)),
                            std::fabs(wave_frequency_.at(n) -
                                      std::pow(wave_frequency_.at(n), 2) * U /
                                          gravity_ * std::cos(direction)),
                            dof);

        P = A_e * qtf.at(0);
      } else {
        P = A_e *
            getDiagonalP(std::fabs(direction_e) * 180 / M_PI,
                         std::fabs(wave_frequency_.at(n) -
                                   std::pow(wave_frequency_.at(n), 2) * U /
                                       gravity_ * std::cos(direction)),
                         dof);
      }

      std::complex<double> P_sqrt;

      if (P >= 0) {
        P_sqrt = std::complex<double>(std::sqrt(std::fabs(P)), 0);
      } else {
        P_sqrt = std::complex<double>(0, std::sqrt(std::fabs(P)));
      }

      wave_drift_load +=
          wave_amplitude_.at(n) * P_sqrt *
          std::cos(std::fabs(wave_frequency_.at(n) -
                             std::pow(wave_frequency_.at(n), 2) * U / gravity_ *
                                 std::cos(direction)) *
                       t -
                   wave_num_.at(n) * (x * std::cos(beta) + y * std::sin(beta)) +
                   wave_phase_.at(n));
      wave_drift_load_helper +=
          wave_amplitude_.at(n) * P_sqrt *
          std::sin(std::fabs(wave_frequency_.at(n) -
                             std::pow(wave_frequency_.at(n), 2) * U / gravity_ *
                                 std::cos(direction)) *
                       t -
                   wave_num_.at(n) * (x * std::cos(beta) + y * std::sin(beta)) +
                   wave_phase_.at(n));
    }

    output.at(dof) =
        (std::pow(wave_drift_load, 2) + std::pow(wave_drift_load_helper, 2))
            .real();

    // sign of direction
    if (direction < 0) {
      if (dof == 1 || dof == 3 || dof == 5) {
        output.at(dof) *= -1;
      }
    }
  }

  return output;
}

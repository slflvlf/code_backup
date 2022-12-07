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

#include "radiation_force.h"

#include <QDir>
#include <QFile>
#include <QString>
#include <cmath>
#include <stdexcept>

RadiationForce::RadiationForce(std::string path,
                               const std::vector<double> &frequency)
    : frequency_(frequency) {
  // list all the data files in the directory
  QDir directory(QString::fromStdString(path));
  if (!directory.exists())
    qWarning("Cannot find the directory!");

  directory.setFilter(QDir::Files | QDir::NoSymLinks | QDir::NoDot |
                      QDir::NoDotAndDotDot | QDir::NoDotDot);
  QFileInfoList fileInfoList = directory.entryInfoList();

  QList<QString> fileList;

  for (int i = 0; i < fileInfoList.size(); ++i) {
    QFileInfo fileInfo = fileInfoList.at(i);
    QString fileName = fileInfo.fileName();
    if (fileName.contains("added") || fileName.contains("damping")) {
      fileList.push_back(fileName);
    }
  }

  if (fileList.size() > 2) {
    throw std::runtime_error("More than two files are found in the directory!");
  }

  frequency_num_ = frequency.size();

  // read added mass and potential damping data from data files,
  std::array<std::array<double, 6>, 6> temp2;
  for (int index = 0; index < 2; ++index) {
    QString fileName = directory.absolutePath() + "/" + fileList.at(index);
    // read the data
    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly)) {
      throw std::runtime_error(file.errorString().toStdString());
    }

    QList<double> dataList;
    while (!file.atEnd()) {
      QByteArray line = file.readLine();
      line.replace("\r", "");
      QList<QByteArray> data_temp = line.split('\n');
      QList<QByteArray> data = data_temp.at(0).split(',');

      foreach (QByteArray s, data) { dataList.append(s.toDouble()); }
    }

    // check the size of the data
    if (dataList.size() != frequency_num_ * 6 * 7)
      throw std::runtime_error(
          "The size of dataset is not correct. Check the file!");

    std::array<double, 6> temp;
    for (int i = 0; i < frequency_num_; ++i) {
      for (int j = 0; j < 6; ++j) {
        for (int k = 0; k < 6; ++k) {
          temp.at(k) = dataList.at(6 * 7 * i + 7 * j + k + 1);
        }
        temp2.at(j) = temp;
      }
      if (fileList.at(index).contains("added")) {
        added_mass_.push_back(temp2);
      } else {
        damping_.push_back(temp2);
      }
    }
  }

  // symmetrize the added mass and damping matrices
  for (int i = 0; i < frequency_num_; ++i) {
    for (int j = 0; j < 6; ++j) {
      for (int k = 0; k < j; ++k) {
        added_mass_.at(i).at(j).at(k) = added_mass_.at(i).at(k).at(j);
        damping_.at(i).at(j).at(k) = damping_.at(i).at(k).at(j);
      }
    }
  }
}

void RadiationForce::InitializeRaditionForce(const double &time_step,
                                             const double &cutoff_time) {
  step_size_ = time_step;
  cutoff_time_ = cutoff_time;

  // compute the impulse response function
  computeImpulseResponse(step_size_, cutoff_time_);

  // compute the infinite-frequency added mass
  computeAddedMassInf();
}

void RadiationForce::computeImpulseResponse(const double &time_step,
                                            const double &cutoff_time) {
  impulse_response_size_ = std::round(cutoff_time / time_step) + 1;
  std::array<std::array<double, 6>, 6> temp2;

  for (unsigned int step = 0; step < impulse_response_size_; ++step) {
    for (int i = 0; i < 6; ++i) {
      for (int j = 0; j < 6; ++j) {
        // use trapezoidal method to perform the integration
        double integration = 0;
        for (int k = 0; k < frequency_num_; ++k) {
          if (k == 0 || k == frequency_num_ - 1) {
            integration += damping_.at(k).at(i).at(j) *
                           std::cos(frequency_.at(k) * step * time_step);
          } else {
            integration += 2 * damping_.at(k).at(i).at(j) *
                           std::cos(frequency_.at(k) * step * time_step);
          }
        }
        integration = integration * (frequency_.back() - frequency_.front()) /
                      (2 * frequency_num_ - 2);
        temp2.at(i).at(j) =
            integration * 2 / M_PI *
            std::exp(-std::pow(3 * time_step * step / cutoff_time, 2));
        ;
      }
    }
    impulse_response_.push_back(temp2);
  }
}

void RadiationForce::computeAddedMassInf() {
  if (impulse_response_.empty())
    throw std::runtime_error(
        "Impulse response functions have not been generated!");

  // initialize added_mass_inf to a zero matrix
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      added_mass_inf_.at(i).at(j) = 0;
    }
  }

  for (int index = 0; index < frequency_num_; ++index) {
    for (int i = 0; i < 6; ++i) {
      for (int j = 0; j < 6; ++j) {
        // use trapezoidal method to perform the integration
        double integration = 0;
        for (unsigned int k = 0; k < impulse_response_size_; ++k) {
          if (k == 0 || k == impulse_response_size_ - 1) {
            integration += impulse_response_.at(k).at(i).at(j) *
                           std::sin(frequency_.at(index) * k * step_size_);
          } else {
            integration += 2 * impulse_response_.at(k).at(i).at(j) *
                           std::sin(frequency_.at(index) * k * step_size_);
          }
        }
        integration = integration * step_size_ / 2;

        // estimate A(infinity) at each frequency, and take it to be the mean of
        // the estimates
        added_mass_inf_.at(i).at(j) +=
            (added_mass_.at(index).at(i).at(j) +
             1 / frequency_.at(index) * integration) /
            frequency_num_;
      }
    }
  }
}

std::array<double, 6> RadiationForce::getRadiationDampingForce(
    std::deque<std::array<double, 6>> velocity_history) const {
  if (impulse_response_.empty())
    throw std::runtime_error(
        "Impulse response functions have not been generated!");

  unsigned int history_size = velocity_history.size();

  if (history_size >= impulse_response_size_)
    throw std::runtime_error("The size of impulse response functions is not "
                             "greater than the velocity history!");

  std::array<double, 6> output;
  for (unsigned int dof = 0; dof < 6; ++dof) {
    // use trapezoidal method to perform the convolution integral
    double integration = 0;

    for (unsigned int i = 0; i < 6; ++i) {
      integration += impulse_response_.at(history_size).at(dof).at(i) *
                     velocity_history.at(0).at(i);
    }

    for (unsigned int k = 1; k < history_size; ++k) {
      for (unsigned int i = 0; i < 6; ++i) {
        integration += 2 *
                       impulse_response_.at(history_size - k).at(dof).at(i) *
                       velocity_history.at(k).at(i);
      }
    }
    integration *= step_size_ / 2;
    output.at(dof) = integration;
  }
  return output;
}

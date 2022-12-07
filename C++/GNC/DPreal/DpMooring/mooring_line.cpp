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

#include "mooring_line.h"
#include <Eigen>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <vector>

void MooringLine::initializeLineShape()
{
  node_coordinate_.push_back(fairlead_pos_);

  // compute the coefficients of the parabola
  double eta_x = std::sqrt(std::pow(trench_pos_.at(0) - fairlead_pos_.at(0), 2) +
                           std::pow(trench_pos_.at(1) - fairlead_pos_.at(1), 2));
  double depth = fairlead_pos_.at(2) + depth_;
  double a = 2 * depth / eta_x / eta_x;
  double b = -depth / eta_x;

  std::array<double, 3> coordinate;
  for (size_t i = 1; i <= total_segment_num_; ++i)
  {
    coordinate.at(0) = fairlead_pos_.at(0) + (trench_pos_.at(0) - fairlead_pos_.at(0)) * i / total_segment_num_;
    coordinate.at(1) = fairlead_pos_.at(1) + (trench_pos_.at(1) - fairlead_pos_.at(1)) * i / total_segment_num_;
    double dist = std::sqrt(std::pow(coordinate.at(0) - fairlead_pos_.at(0), 2) +
                            std::pow(coordinate.at(1) - fairlead_pos_.at(1), 2));
    coordinate.at(2) = (a * dist + b) * (dist - eta_x) + trench_pos_.at(2);
    node_coordinate_.push_back(coordinate);
  }

  // update segment length
  updateSegmentLength();
}

void MooringLine::initializeLineShape(const std::array<double, 3> &fairlead)
{
  node_coordinate_.at(0) = fairlead;

  // compute the coefficients of the parabola
  double eta_x =
      std::sqrt(std::pow(trench_pos_.at(0) - fairlead.at(0), 2) + std::pow(trench_pos_.at(1) - fairlead.at(1), 2));
  double depth = fairlead.at(2) + depth_;
  double a = 2 * depth / eta_x / eta_x;
  double b = -depth / eta_x;

  std::array<double, 3> coordinate;
  for (size_t i = 1; i <= total_segment_num_; ++i)
  {
    coordinate.at(0) = fairlead.at(0) + (trench_pos_.at(0) - fairlead.at(0)) * i / total_segment_num_;
    coordinate.at(1) = fairlead.at(1) + (trench_pos_.at(1) - fairlead.at(1)) * i / total_segment_num_;
    double dist =
        std::sqrt(std::pow(coordinate.at(0) - fairlead.at(0), 2) + std::pow(coordinate.at(1) - fairlead.at(1), 2));
    coordinate.at(2) = (a * dist + b) * (dist - eta_x) + trench_pos_.at(2);
    node_coordinate_.at(i) = coordinate;
  }

  // update segment length
  updateSegmentLength();
}

void MooringLine::writeNodeCoordinate(const std::string &node_file) const
{
  std::ofstream node_out;
  // open the file
  node_out.open(node_file.c_str());

  // write data into the file
  for (size_t i = 0; i < total_segment_num_ + 1; i++)
  {
    node_out << std::fixed << i + 1 << '\t';  // first column is node index
    for (size_t index = 0; index < 3; ++index)
    {
      node_out << std::setprecision(6) << std::scientific << node_coordinate_[i][index] << '\t';
    }
    node_out << std::endl;
  }

  // close the file
  node_out.close();
}

Eigen::SparseMatrix<double> MooringLine::getInertiaMatrix() const
{
  Eigen::SparseMatrix<double> mass(3 * total_segment_num_ - 3, 3 * total_segment_num_ - 3);
  std::vector<Eigen::Triplet<double>> coefficients;  // list of non-zeros coefficients

  for (size_t i = 0; i < total_segment_num_ - 1; ++i)
  {
    coefficients.emplace_back(Eigen::Triplet<double>(3 * i, 3 * i, node_weight_.at(i + 1)));
    coefficients.emplace_back(Eigen::Triplet<double>(3 * i + 1, 3 * i + 1, node_weight_.at(i + 1)));
    coefficients.emplace_back(Eigen::Triplet<double>(3 * i + 2, 3 * i + 2, node_weight_.at(i + 1)));
  }
  mass.setFromTriplets(coefficients.begin(), coefficients.end());

  return mass;
}

Eigen::VectorXd MooringLine::getGravityForce() const
{
  Eigen::VectorXd gravity = Eigen::VectorXd::Zero(3 * total_segment_num_ + 3);
  for (size_t i = 0; i < total_segment_num_ + 1; ++i)
  {
    gravity(3 * i + 2) = -node_weight_.at(i) * GRAVITY_;
  }
  return gravity;
}

Eigen::VectorXd MooringLine::getContactForce() const
{
  Eigen::VectorXd contact = Eigen::VectorXd::Zero(3 * total_segment_num_ - 3);
  for (size_t i = 0; i < total_segment_num_ - 1; ++i)
  {
    contact(3 * i + 2) = node_weight_.at(i + 1) * GRAVITY_ * (1 - std::tanh(node_coordinate_[i + 1][2] + depth_));
  }
  return contact;
}

Eigen::SparseMatrix<double> MooringLine::getTensionMatrix() const
{
  Eigen::SparseMatrix<double> tension(3 * total_segment_num_ - 3, 3 * total_segment_num_ - 3);
  Eigen::MatrixXd a_mat = Eigen::MatrixXd::Zero(3 * total_segment_num_, 3);
  std::vector<Eigen::Triplet<double>> coefficients;  // list of non-zeros coefficients

  // prepare a_mat
  for (size_t section_num = 0; section_num < segment_num_.size(); ++section_num)
  {
    for (size_t n = 0; n < segment_num_.at(section_num); ++n)
    {
      auto index = n + std::accumulate(segment_num_.cbegin(), segment_num_.cbegin() + section_num, 0);

      for (size_t i = 0; i < 3; i++)
      {
        a_mat(3 * index + i, i) =
            EA_.at(section_num) * 1E6 * (1 / original_segment_length_.at(section_num) - 1 / segment_length_.at(index));
      }
    }
  }

  // generate the tension matrix
  for (size_t block = 0; block < total_segment_num_ - 2; block++)
  {
    for (size_t i = 0; i < 3; i++)
    {
      for (size_t j = 0; j < 3; j++)
      {
        coefficients.emplace_back(
            Eigen::Triplet<double>(3 * block + i, 3 * block + 3 + j, a_mat(3 * block + 3 + i, j)));
        coefficients.emplace_back(
            Eigen::Triplet<double>(3 * block + 3 + i, 3 * block + j, a_mat(3 * block + 3 + i, j)));
      }
    }
  }
  for (size_t block = 0; block < total_segment_num_ - 1; block++)
  {
    for (size_t i = 0; i < 3; i++)
    {
      for (size_t j = 0; j < 3; j++)
      {
        coefficients.emplace_back(Eigen::Triplet<double>(3 * block + i, 3 * block + j,
                                                         -a_mat(3 * block + i, j) - a_mat(3 * block + 3 + i, j)));
      }
    }
  }
  tension.setFromTriplets(coefficients.begin(), coefficients.end());

  return tension;
}

Eigen::SparseMatrix<double> MooringLine::getTensionStiffnessMatrix() const
{
  Eigen::SparseMatrix<double> K(3 * total_segment_num_ - 3, 3 * total_segment_num_ - 3);
  Eigen::MatrixXd k_mat = Eigen::MatrixXd::Zero(3 * total_segment_num_, 3);
  std::vector<Eigen::Triplet<double>> coefficients;  // list of non-zeros coefficients

  // prepare k_mat
  for (size_t section_num = 0; section_num < segment_num_.size(); ++section_num)
  {
    for (size_t n = 0; n < segment_num_.at(section_num); ++n)
    {
      auto index = n + std::accumulate(segment_num_.cbegin(), segment_num_.cbegin() + section_num, 0);

      for (size_t i = 0; i < 3; i++)
      {
        for (size_t j = 0; j < 3; j++)
        {
          if (i == j)
          {
            k_mat(3 * index + i, j) = EA_.at(section_num) * 1E6 *
                                      (1 / segment_length_.at(index) - 1 / original_segment_length_.at(section_num) -
                                       (node_coordinate_[index + 1][i] - node_coordinate_[index][i]) *
                                           (node_coordinate_[index + 1][j] - node_coordinate_[index][j]) /
                                           std::pow(segment_length_.at(index), 3));
          }
          else
          {
            k_mat(3 * index + i, j) =
                -EA_.at(section_num) * 1E6 * (node_coordinate_[index + 1][i] - node_coordinate_[index][i]) *
                (node_coordinate_[index + 1][j] - node_coordinate_[index][j]) / std::pow(segment_length_.at(index), 3);
          }
        }
      }
    }
  }

  // assemble sparse matrix K
  for (size_t block = 0; block < total_segment_num_ - 2; block++)
  {
    for (size_t i = 0; i < 3; i++)
    {
      for (size_t j = 0; j < 3; j++)
      {
        coefficients.emplace_back(
            Eigen::Triplet<double>(3 * block + i, 3 * block + 3 + j, -k_mat(3 * block + 3 + i, j)));
        coefficients.emplace_back(
            Eigen::Triplet<double>(3 * block + 3 + i, 3 * block + j, -k_mat(3 * block + 3 + i, j)));
      }
    }
  }
  for (size_t block = 0; block < total_segment_num_ - 1; block++)
  {
    for (size_t i = 0; i < 3; i++)
    {
      for (size_t j = 0; j < 3; j++)
      {
        coefficients.emplace_back(Eigen::Triplet<double>(3 * block + i, 3 * block + j,
                                                         k_mat(3 * block + i, j) + k_mat(3 * block + 3 + i, j)));
      }
    }
  }
  K.setFromTriplets(coefficients.begin(), coefficients.end());

  return K;
}

Eigen::SparseMatrix<double> MooringLine::getContactStiffnessMatrix() const
{
  Eigen::SparseMatrix<double> Kb(3 * total_segment_num_ - 3, 3 * total_segment_num_ - 3);
  std::vector<Eigen::Triplet<double>> coefficients;  // list of non-zeros coefficients

  for (size_t i = 0; i < total_segment_num_ - 1; i++)
  {
    coefficients.emplace_back(Eigen::Triplet<double>(3 * i + 2, 3 * i + 2,
                                                     -node_weight_.at(i + 1) * GRAVITY_ /
                                                         std::pow(std::cosh(node_coordinate_[i + 1][2] + depth_), 2)));
  }

  Kb.setFromTriplets(coefficients.begin(), coefficients.end());
  return Kb;
}

void MooringLine::updateSegmentLength()
{
  for (size_t i = 0; i < total_segment_num_; ++i)
  {
    segment_length_.at(i) = std::sqrt(std::pow(node_coordinate_[i][0] - node_coordinate_[i + 1][0], 2) +
                                      std::pow(node_coordinate_[i][1] - node_coordinate_[i + 1][1], 2) +
                                      std::pow(node_coordinate_[i][2] - node_coordinate_[i + 1][2], 2));
  }
}

int MooringLine::solveStaticEquation(const std::array<double, 3> &fairlead_pos, const double &tolerance,
                                     const int &max_iteration, const double &relaxation_factor)
{
  Eigen::VectorXd B = gravity_force_.segment(3, 3 * total_segment_num_ - 3);
  Eigen::VectorXd node_coordinate = Eigen::VectorXd::Zero(3 * total_segment_num_ - 3);
  Eigen::Matrix3d a_mat = Eigen::Matrix3d::Zero();

  // update line shape and segment length
  initializeLineShape(fairlead_pos);
  updateSegmentLength();

  // define the local node_coordinate
  for (size_t i = 0; i < node_coordinate_.size() - 2; i++)
  {
    node_coordinate(3 * i) = node_coordinate_[i + 1][0];
    node_coordinate(3 * i + 1) = node_coordinate_[i + 1][1];
    node_coordinate(3 * i + 2) = node_coordinate_[i + 1][2];
  }

  // use Newton's method to solve the static equation
  int counter = 0;
  Eigen::VectorXd increment;
  Eigen::SparseMatrix<double> K;
  Eigen::VectorXd P;
  Eigen::Vector3d fairlead(fairlead_pos.at(0), fairlead_pos.at(1), fairlead_pos.at(2));
  Eigen::Vector3d trench((node_coordinate_.back())[0], (node_coordinate_.back())[1], (node_coordinate_.back())[2]);

  while (counter < max_iteration)
  {
    // update vector B
    for (size_t i = 0; i < 3; i++)
    {
      a_mat(i, i) = EA_.front() * 1E6 * (1 / original_segment_length_.front() - 1 / segment_length_.front());
    }

    B.segment(0, 3) = gravity_force_.segment(3, 3) + a_mat * fairlead;

    // compute a_1 at trench
    for (size_t i = 0; i < 3; i++)
    {
      a_mat(i, i) = EA_.back() * 1E6 * (1 / original_segment_length_.back() - 1 / segment_length_.back());
    }
    B.segment(3 * total_segment_num_ - 6, 3) = gravity_force_.segment(3 * total_segment_num_ - 3, 3) + a_mat * trench;

    // compute K and P
    K = -(getTensionStiffnessMatrix() + getContactStiffnessMatrix());  // ensure that K is SPD
    P = B + getContactForce();
    P += getTensionMatrix() * node_coordinate;  // for performance purpose

    // solve Kx = P
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
    increment = solver.compute(K).solve(P);
    if (solver.info() != Eigen::ComputationInfo::Success)
    {
      return -1;  // fail to compute the increment
    }

    node_coordinate += relaxation_factor * increment;

    // save back node_coordinate in order to compute new matrices and segment length
    for (size_t i = 0; i < node_coordinate_.size() - 2; i++)
    {
      node_coordinate_[i + 1][0] = node_coordinate(3 * i);
      node_coordinate_[i + 1][1] = node_coordinate(3 * i + 1);
      node_coordinate_[i + 1][2] = node_coordinate(3 * i + 2);
    }

    // update segment length
    updateSegmentLength();

    ++counter;  // increase counter

    if (increment.norm() < tolerance)
    {
      top_tension_ = getStaticTopTension();
      return 0;
    }
  }
  return 1;  // number of iteration exceeds maximum iteration
}

inline Eigen::Vector3d MooringLine::getTangentUnitVector(const int &index) const
{
  Eigen::Vector3d tau(0, 0, 0);
  tau << (node_coordinate_.at(index + 1))[0] - (node_coordinate_.at(index))[0],
      (node_coordinate_.at(index + 1))[1] - (node_coordinate_.at(index))[1],
      (node_coordinate_.at(index + 1))[2] - (node_coordinate_.at(index))[2];
  tau /= tau.norm();

  return tau;
}

Eigen::SparseMatrix<double> MooringLine::getAddedMassMatrix() const
{
  Eigen::SparseMatrix<double> Ma(3 * total_segment_num_ - 3, 3 * total_segment_num_ - 3);
  Eigen::MatrixXd Ma_b = Eigen::MatrixXd::Zero(3 * total_segment_num_, 3);
  Eigen::Vector3d tau(0, 0, 0);  // tangential unit vector

  // prepare blocks
  for (size_t section_num = 0; section_num < segment_num_.size(); ++section_num)
  {
    for (size_t n = 0; n < segment_num_.at(section_num); ++n)
    {
      auto index = n + std::accumulate(segment_num_.cbegin(), segment_num_.cbegin() + section_num, 0);
      tau = getTangentUnitVector(index);
      Ma_b.block<3, 3>(3 * index, 0) = 0.125 * WATER_DENSITY_ * M_PI * std::pow(diameter_.at(section_num), 2) *
                                       Ca_.at(section_num) * original_segment_length_.at(section_num) *
                                       (Eigen::Matrix3d::Identity() - tau * tau.transpose());
    }
  }

  // calculate Ma
  std::vector<Eigen::Triplet<double>> coefficients;  // list of non-zeros coefficients
  for (size_t block = 1; block < total_segment_num_; block++)
  {
    for (size_t i = 0; i < 3; i++)
    {
      for (size_t j = 0; j < 3; j++)
      {
        coefficients.emplace_back(Eigen::Triplet<double>(3 * block - 3 + i, 3 * block - 3 + j,
                                                         Ma_b(3 * block + i, j) + Ma_b(3 * block - 3 + i, j)));
      }
    }
  }

  Ma.setFromTriplets(coefficients.begin(), coefficients.end());

  return Ma;
}

/** @todo define a function to compute current velocity profile */
Eigen::VectorXd MooringLine::getDragForce(const Eigen::VectorXd &node_velocity,
                                          const Eigen::VectorXd &current_velocity) const
{
  Eigen::VectorXd drag = Eigen::VectorXd::Zero(3 * total_segment_num_ - 3);
  Eigen::Vector3d relative_velocity;
  Eigen::Vector3d tau, tau1;  // tangential unit vector

  for (size_t section_num = 0; section_num < segment_num_.size(); ++section_num)
  {
    for (size_t n = 0; n < segment_num_.at(section_num); ++n)
    {
      auto index = 1 + n + std::accumulate(segment_num_.cbegin(), segment_num_.cbegin() + section_num, 0);

      if (section_num == segment_num_.size() - 1 && n == segment_num_.at(section_num) - 1)
      {
        break;
      }

      Eigen::Vector3d tau = getTangentUnitVector(index - 1);
      relative_velocity = current_velocity.segment(3 * index, 3) - node_velocity.segment(3 * index - 3, 3);
      tau1 = getTangentUnitVector(index);
      drag.segment(3 * index - 3, 3) = 0.25 * WATER_DENSITY_ * diameter_.at(section_num) *
                                       original_segment_length_.at(section_num) * Cd_.at(section_num) *
                                       (relative_velocity - (relative_velocity.dot(tau)) * tau) *
                                       (relative_velocity - (relative_velocity.dot(tau)) * tau).norm();
      if (n == segment_num_.at(section_num) - 1 && section_num != segment_num_.size() - 1)  // across the boundary
      {
        drag.segment(3 * index - 3, 3) += 0.25 * WATER_DENSITY_ * diameter_.at(section_num + 1) *
                                          original_segment_length_.at(section_num + 1) * Cd_.at(section_num + 1) *
                                          (relative_velocity - (relative_velocity.dot(tau1)) * tau1) *
                                          (relative_velocity - (relative_velocity.dot(tau1)) * tau1).norm();
      }
      else
      {
        drag.segment(3 * index - 3, 3) += 0.25 * WATER_DENSITY_ * diameter_.at(section_num) *
                                          original_segment_length_.at(section_num) * Cd_.at(section_num) *
                                          (relative_velocity - (relative_velocity.dot(tau1)) * tau1) *
                                          (relative_velocity - (relative_velocity.dot(tau1)) * tau1).norm();
      }
    }
  }

  return drag;
}

int MooringLine::solveDynamicEquation(const std::array<double, 3> &fairlead_pos,
                                      const std::array<double, 3> &current_velocity, const double &time_step,
                                      const double &alpha, const double &beta, const double &tolerance,
                                      const int &max_iteration)
{
  // calculate the parameters used in Newmark-beta method
  double a0 = 1 / beta / time_step / time_step;
  double a1 = 1 / beta / time_step;
  double a2 = 1 / (2 * beta) - 1;
  double a3 = alpha / beta / time_step;
  double a4 = 1 - alpha / beta;
  double a5 = time_step * (alpha / beta / 2 - 1);

  // define local vectors, used in iterations
  Eigen::VectorXd node_coordinate = Eigen::VectorXd::Zero(3 * total_segment_num_ - 3);
  Eigen::VectorXd delta_y = node_coordinate;
  Eigen::VectorXd node_velocity = a3 * delta_y + a4 * node_velocity_ - a5 * node_acceleration_;
  Eigen::VectorXd node_acceleration = a0 * delta_y - a1 * node_velocity_ - a2 * node_acceleration_;

  Eigen::Matrix3d a_mat = Eigen::Matrix3d::Zero();
  Eigen::VectorXd B = gravity_force_.segment(3, 3 * total_segment_num_ - 3);

  // compute the Jacobian using the previous step data, and ensure that Jacobian is SPD
  Eigen::SparseMatrix<double> Jacobian =
      -getTensionStiffnessMatrix() - getContactStiffnessMatrix() + a0 * (getAddedMassMatrix() + getInertiaMatrix());

  Eigen::SparseMatrix<double> damper(3 * total_segment_num_ - 3, 3 * total_segment_num_ - 3);
  std::vector<Eigen::Triplet<double>> coefficients;  // list of non-zeros coefficients
  for (size_t i = 0; i < 3 * total_segment_num_ - 3; i++)
  {
    coefficients.emplace_back(Eigen::Triplet<double>(i, i, 1E4));
  }
  damper.setFromTriplets(coefficients.begin(), coefficients.end());

  Jacobian -= damper;  // damped

  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
  solver.compute(Jacobian);
  if (solver.info() != Eigen::ComputationInfo::Success)
  {
    return -1;  // fail to compute the increment
  }

  // update first node coordinate and segment length
  node_coordinate_.front() = fairlead_pos;
  updateSegmentLength();

  // solve for delta_y
  int counter = 0;
  Eigen::VectorXd epsilon;
  Eigen::VectorXd P;

  while (counter < max_iteration)
  {
    // update vector B
    for (size_t i = 0; i < 3; i++)
    {
      a_mat(i, i) = EA_.front() * 1E6 * (1 / original_segment_length_.front() - 1 / segment_length_.front());
    }

    B.segment(0, 3) = gravity_force_.segment(3, 3) +
                      a_mat *
                          (Eigen::Vector3d() << (node_coordinate_.front())[0], (node_coordinate_.front())[1],
                           (node_coordinate_.front())[2])
                              .finished();

    // compute a_1 at trench
    for (size_t i = 0; i < 3; i++)
    {
      a_mat(i, i) = EA_.back() * 1E6 * (1 / original_segment_length_.back() - 1 / segment_length_.back());
    }
    B.segment(3 * total_segment_num_ - 6, 3) = gravity_force_.segment(3 * total_segment_num_ - 3, 3) +
                                               a_mat *
                                                   (Eigen::Vector3d() << (node_coordinate_.back())[0],
                                                    (node_coordinate_.back())[1], (node_coordinate_.back())[2])
                                                       .finished();

    // compute P
    Eigen::VectorXd current_profile = getCurrentVelocity(current_velocity);
    P = B + getContactForce() + getDragForce(node_velocity, current_profile);

    for (size_t i = 0; i < node_coordinate_.size() - 2; i++)
    {
      node_coordinate(3 * i) = node_coordinate_[i + 1][0];
      node_coordinate(3 * i + 1) = node_coordinate_[i + 1][1];
      node_coordinate(3 * i + 2) = node_coordinate_[i + 1][2];
    }
    P += getTensionMatrix() * node_coordinate;
    P -= (getAddedMassMatrix() + getInertiaMatrix()) * node_acceleration;

    epsilon = solver.solve(P);

    if (solver.info() != Eigen::ComputationInfo::Success)
    {
      return -1;  // fail to compute the increment
    }

    delta_y += epsilon;
    node_coordinate += epsilon;
    node_velocity = a3 * delta_y + a4 * node_velocity_ - a5 * node_acceleration_;
    node_acceleration = a0 * delta_y - a1 * node_velocity_ - a2 * node_acceleration_;

    // save back node_coordinate in order to compute new matrices and segment length
    for (size_t i = 0; i < node_coordinate_.size() - 2; i++)
    {
      node_coordinate_[i + 1][0] = node_coordinate(3 * i);
      node_coordinate_[i + 1][1] = node_coordinate(3 * i + 1);
      node_coordinate_[i + 1][2] = node_coordinate(3 * i + 2);
    }

    // update segment length
    updateSegmentLength();

    // increase counter
    ++counter;

    if (epsilon.norm() < tolerance)
    {
      node_acceleration_ = node_acceleration;
      node_velocity_ = node_velocity;
      top_tension_ = getStaticTopTension();
      return 0;
    }
  }
  return 1;  // number of iteration exceeds max iteration
}

std::vector<std::array<double, 3>> MooringLine::readFairleadPosition(const std::string &file) const
{
  std::array<double, 3> fairlead;
  std::vector<std::array<double, 3>> out;
  double data;

  std::ifstream my_file(file.c_str());
  if (my_file.is_open())
  {
    size_t counter = 0;
    while (my_file >> data)
    {
      fairlead.at(counter) = data;
      counter++;
      if (counter == 3)
      {
        out.push_back(fairlead);
        counter = 0;
      }
    }
    my_file.close();
  }
  else
  {
    std::cout << "Unable to open the file containing fairlead positions." << std::endl;
  }

  return out;
}

void MooringLine::printProgBar(const size_t &percent)
{
  std::string bar;

  for (size_t i = 0; i < 50; ++i)
  {
    if (i < (percent / 2))
    {
      bar.replace(i, 1, "-");
    }
    else if (i == (percent / 2))
    {
      bar.replace(i, 1, ">");
    }
    else
    {
      bar.replace(i, 1, " ");
    }
  }

  std::cout << "\r"
               "["
            << bar << "]";
  std::cout.width(3);
  std::cout << percent << "%    " << std::flush;
}

int MooringLine::conductDynamicAnalysis(const std::vector<std::array<double, 3>> &fairlead_pos,
                                        const std::array<double, 3> &current_velocity, const double &time_step,
                                        const double &alpha, const double &beta, const double &tolerance,
                                        const int &max_iteration)
{
  auto iteration = fairlead_pos.size();
  int exit_flag;

  if (fairlead_pos.empty())
  {
    return -2;  // fairlead vector is empty
  }

  // perform a static analysis to initialize the line shape
  exit_flag = solveStaticEquation(fairlead_pos.at(0));

  // write top tension into a file
  const std::string tension_file = "/home/sjtu/Documents/top_tension.txt";
  writeTopTension(0, tension_file, "trunc");

  if (exit_flag != 0)
  {
    return -3;  // static analysis fails
  }

  std::cout << "The dynamic analysis will run for " << iteration - 1 << " steps. "
            << "The time step is " << time_step << " second." << std::endl;
  std::cout << "Please be patient. Running.............." << std::endl;
  for (size_t counter = 1; counter < iteration; ++counter)
  {
    std::array<double, 3> pos = fairlead_pos.at(counter);
    exit_flag = solveDynamicEquation(pos, current_velocity, time_step, alpha, beta, tolerance, max_iteration);

    // display a progress bar on the console
    size_t percent = counter * 100 / (iteration - 1);
    printProgBar(percent);

    // write top tension into the file
    writeTopTension(counter, tension_file, "app");

    if (exit_flag != 0)
    {
      break;
    }
  }

  std::cout << std::endl;

  return exit_flag;
}

Eigen::Vector3d MooringLine::getStaticTopTension() const
{
  Eigen::Vector3d unit_vector = getTangentUnitVector(0);
  Eigen::Vector3d top_tension = unit_vector;

  top_tension(0) = (segment_length_.at(0) - original_segment_length_.at(0)) / original_segment_length_.at(0) *
                   EA_.at(0) * unit_vector(0) * 1E3;
  top_tension(1) = (segment_length_.at(0) - original_segment_length_.at(0)) / original_segment_length_.at(0) *
                   EA_.at(0) * unit_vector(1) * 1E3;
  top_tension(2) = (segment_length_.at(0) - original_segment_length_.at(0)) / original_segment_length_.at(0) *
                       EA_.at(0) * unit_vector(2) * 1E3 -
                   wet_weight_.at(0) * original_segment_length_.at(0) * GRAVITY_ / 2 / 1E3;

  return top_tension;  // unit: kN
}

void MooringLine::writeTopTension(const int &step, const std::string &tension_file, const std::string &mode) const
{
  std::ofstream tension_out;
  // open the file
  if (mode == "app")
  {
    tension_out.open(tension_file.c_str(), std::ofstream::app);
  }
  else
  {
    tension_out.open(tension_file.c_str(), std::ofstream::trunc);
  }

  tension_out << std::fixed << step << '\t';

  // write data into the file
  for (size_t i = 0; i < 3; i++)
  {
    tension_out << std::setprecision(6) << std::scientific << top_tension_(i) << '\t';
  }

  tension_out << std::endl;

  // close the file
  tension_out.close();
}

Eigen::VectorXd MooringLine::getCurrentVelocity(const std::array<double, 3> &current_velocity) const
{
  Eigen::VectorXd current_profile = Eigen::VectorXd::Zero(3 * total_segment_num_ + 3);

  // assume a linear profile from ocean surface to bottom
  for (size_t i = 0; i < total_segment_num_ + 1; ++i)
  {
    current_profile.segment(3 * i, 3) =
        (1 + node_coordinate_[i][2] / depth_) *
        (Eigen::Vector3d() << current_velocity[0], current_velocity[1], current_velocity[2]).finished();
  }

  return current_profile;
}

std::array<double, 6> MooringLine::ComputeMooringForce(const std::array<double, 3> &top_tension, const double &roll,
                                                       const double &pitch, const double &yaw) const
{
  std::array<double, 6> mooring_force;

  // assume roll and pitch angles are really small
  mooring_force.at(0) = std::cos(yaw) * top_tension.at(0) + std::sin(yaw) * top_tension.at(1);
  mooring_force.at(1) = -std::sin(yaw) * top_tension.at(0) + std::cos(yaw) * top_tension.at(1);
  mooring_force.at(2) = top_tension.at(2);

  // compute moment
  mooring_force.at(3) = fairlead_pos_.at(1) * mooring_force.at(2) - fairlead_pos_.at(2) * mooring_force.at(1);
  mooring_force.at(4) = fairlead_pos_.at(2) * mooring_force.at(0) - fairlead_pos_.at(0) * mooring_force.at(2);
  mooring_force.at(5) = fairlead_pos_.at(0) * mooring_force.at(1) - fairlead_pos_.at(1) * mooring_force.at(0);

  return mooring_force;
}

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

#include "marine_structure.h"
#include <cmath>
#include <iostream>

// constructor definition
MarineStructure::MarineStructure(const std::string &name, const double &mass,
                                 const std::array<double, 3> &gravity_center,
                                 const std::array<double, 3> &moment_of_inertia,
                                 const std::array<double, 6> &init_velocity,
                                 const std::array<double, 6> &init_position)
    : name_(name), MASS_(mass), CENTER_OF_GRAVITY_(gravity_center),
      INERTIA_TENSOR_(Eigen::Matrix3d(
          ((Eigen::Vector3d()
                << moment_of_inertia.at(0) +
                       mass * (gravity_center.at(2) * gravity_center.at(2)),
            moment_of_inertia.at(1) +
                mass * (gravity_center.at(2) * gravity_center.at(2)),
            moment_of_inertia.at(2))
               .finished())
              .asDiagonal())) {
  velocity_ << init_velocity[0], init_velocity[1], init_velocity[2],
      init_velocity[3], init_velocity[4], init_velocity[5];
  position_ << init_position[0], init_position[1], init_position[2],
      init_position[3], init_position[4], init_position[5];
  euler_ << init_position[3], init_position[4], init_position[5];
  quaternion_ = Euler2Q(euler_);
}

// cross product operator
Eigen::Matrix3d
MarineStructure::CrossProductOperator(const Eigen::Vector3d &vec) {
  Eigen::Matrix3d cross_product;
  cross_product << 0, -vec[2], vec[1], vec[2], 0, -vec[0], -vec[1], vec[0], 0;
  return cross_product;
}

// overloaded cross product operator
Eigen::Matrix3d
MarineStructure::CrossProductOperator(const std::array<double, 3> &vec) {
  Eigen::Matrix3d cross_product;
  cross_product << 0, -vec.at(2), vec.at(1), vec.at(2), 0, -vec.at(0),
      -vec.at(1), vec.at(0), 0;
  return cross_product;
}

// rotation matrix definition     different from station keeping
Eigen::Matrix3d MarineStructure::RotationMatrix(const Eigen::Vector3d &angle) {
  Eigen::Matrix3d rz, ry, rx;
  rz << std::cos(angle[2]), -std::sin(angle[2]), 0, std::sin(angle[2]),
      std::cos(angle[2]), 0, 0, 0, 1;
  ry << std::cos(angle[1]), 0, std::sin(angle[1]), 0, 1, 0, -std::sin(angle[1]),
      0, std::cos(angle[1]);
  rx << 1, 0, 0, 0, std::cos(angle[0]), -std::sin(angle[0]), 0,
      std::sin(angle[0]), std::cos(angle[0]);
  return rx * ry * rz;
}

// transformation from rotation matrix to an unit quaternion
Eigen::Vector4d MarineStructure::Euler2Q(const Eigen::Vector3d &angle) {
  Eigen::Vector4d q;
  float p1, p2, p3, p4;
  Eigen::Matrix3d R = RotationMatrix(angle);
  float trace = R.trace();
  q << R(0, 0), R(1, 1), R(2, 2), trace;
  Eigen::Vector4d::Index maxRow, maxCol;
  float max = q.maxCoeff(&maxRow, &maxCol);
  float p_i = std::sqrt(1 + 2 * max - trace);

  switch (maxRow) {
  case 0:
    p1 = p_i;
    p2 = (R(1, 0) + R(0, 1)) / p_i;
    p3 = (R(0, 2) + R(2, 0)) / p_i;
    p4 = (R(2, 1) - R(1, 2)) / p_i;
    break;
  case 1:
    p1 = (R(1, 0) + R(0, 1)) / p_i;
    p2 = p_i;
    p3 = (R(2, 1) + R(1, 2)) / p_i;
    p4 = (R(0, 2) - R(2, 0)) / p_i;
    break;
  case 2:
    p1 = (R(0, 2) + R(2, 0)) / p_i;
    p2 = (R(2, 1) + R(1, 2)) / p_i;
    p3 = p_i;
    p4 = (R(1, 0) - R(0, 1)) / p_i;
    break;
  case 3:
    p1 = (R(2, 1) - R(1, 2)) / p_i;
    p2 = (R(0, 2) - R(2, 0)) / p_i;
    p3 = (R(1, 0) - R(0, 1)) / p_i;
    p4 = p_i;
    break;
  }
  q << 0.5 * p1, 0.5 * p2, 0.5 * p3, 0.5 * p4;
  q = q / q.dot(q);
  return q;
}

// angular transformation matrix definition
Eigen::Matrix3d
MarineStructure::AngularTransMatrix(const Eigen::Vector3d &angle) {
  if (std::fabs(std::fabs(angle[1]) - M_PI / 2) <= M_PI / 6) {
    std::cerr << "Warning: the pitch motion is almost singular!" << std::endl;
  }
  Eigen::Matrix3d ang_trans;
 // ang_trans << 1, std::sin(angle[0]) * std::tan(angle[1]),
 //    std::cos(angle[0]) * std::tan(angle[1]), 0, std::cos(angle[0]),
 //     -std::sin(angle[0]), 0, std::sin(angle[0]) / std::cos(angle[1]),
 //     std::cos(angle[0]) / std::cos((angle[1]));
    ang_trans << std::cos(angle[2]) / std::cos(angle[1]), -std::sin(angle[2]) / std::cos(angle[1]), 0,
                 std::sin(angle[2]), std::cos(angle[2]), 0,
                 -std::tan(angle[1]) * std::cos(angle[2]), std::tan(angle[2]) * std::sin(angle[2]), 1;
  return ang_trans;
}

// transformation matrix
MarineStructure::Matrix6d
MarineStructure::TransformationMatrix(const Eigen::Vector3d &angle) {
  Matrix6d trans_mat;
  Eigen::Matrix3d zeros = Eigen::Matrix3d::Zero();
  trans_mat << RotationMatrix(angle), zeros, zeros, AngularTransMatrix(angle);
  return trans_mat;
}

// inertia matrix of the rigid body
MarineStructure::Matrix6d MarineStructure::RigidBodyInertiaMatrix() const {
  Matrix6d inertia_mat;
  Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
  inertia_mat << MASS_ * identity,
      -MASS_ * CrossProductOperator(CENTER_OF_GRAVITY_),
      MASS_ * CrossProductOperator(CENTER_OF_GRAVITY_), INERTIA_TENSOR_;
  return inertia_mat;
}

// centripetal-coriolis matrix of rigid body
MarineStructure::Matrix6d
MarineStructure::RigidBodyCCMatrix(const Vector6d &vec) const {
  Eigen::Vector3d angular_vel = {vec[3], vec[4], vec[5]};
  Matrix6d cc_mat;
  cc_mat << MASS_ * CrossProductOperator(angular_vel),
      -MASS_ * CrossProductOperator(angular_vel) *
          CrossProductOperator(CENTER_OF_GRAVITY_),
      MASS_ * CrossProductOperator(CENTER_OF_GRAVITY_) *
          CrossProductOperator(angular_vel),
      -CrossProductOperator(INERTIA_TENSOR_ * angular_vel);
  return cc_mat;
}

// centripetal-coriolis matrix of added mass   ????
MarineStructure::Matrix6d
MarineStructure::AddedMassCCMatrix(const Vector6d &vec) const {
  Matrix6d temp;
  Eigen::Vector3d lin_vel = {vec[0], vec[1], vec[2]};
  Eigen::Vector3d ang_vel = {vec[3], vec[4], vec[5]};
  temp << CrossProductOperator(ang_vel), Eigen::Matrix3d::Zero(),
      CrossProductOperator(lin_vel), CrossProductOperator(ang_vel);
  return temp * ADDED_MASS_MATRIX_;
}

// quadratic damping force
MarineStructure::Vector6d
MarineStructure::QuadraticDampingForce(const Vector6d &vec) const {
  return 0 * vec;
}

// restoring force vector    ????  to be override later
MarineStructure::Vector6d
MarineStructure::RestoringForce(const Vector6d &vec) const {
  Vector6d restoring;
  Eigen::Vector3d angle = {vec[3], vec[4], vec[5]};
  Eigen::Vector3d vg = {0, 0, MASS_ * GRAVITY_};
  Eigen::Vector3d vb = {0, 0, MASS_ * GRAVITY_};
  restoring << RotationMatrix(angle).transpose() * (vg - vb),
      CrossProductOperator(CENTER_OF_GRAVITY_) *
              RotationMatrix(angle).transpose() * vg -
          CrossProductOperator(CENTER_OF_BUOYANCY_) *
              RotationMatrix(angle).transpose() * vb;
  return restoring;
}

// time derivative of the system state
MarineStructure::Vector12d
MarineStructure::StateDerivative(const Vector6d &velocity,
                                 const Vector6d &relative_velocity,
                                 const Vector6d &position, const double) const {
  Vector12d state_derivative;
  Vector6d vec1, vec2;
  Eigen::Vector3d angle(position[3], position[4], position[5]);

  Matrix6d total_mass_matrix = RigidBodyInertiaMatrix() + ADDED_MASS_MATRIX_;

  vec1 = total_mass_matrix.inverse() *
         (-RigidBodyCCMatrix(velocity) * velocity -
          AddedMassCCMatrix(relative_velocity) * relative_velocity +
          LINEAR_DAMPING_MATRIX_ * relative_velocity +
          QuadraticDampingForce(relative_velocity) + RestoringForce(position) +
          external_force_);
  vec2 = TransformationMatrix(angle) * velocity;
  state_derivative << vec1, vec2;

  return state_derivative;
}

// class operator to integrate
void MarineStructure::operator()(const std::vector<double> &state,
                                 std::vector<double> &state_derivative,
                                 const double t) {
  Vector6d velocity, position, relative_velocity;
  Vector12d dy;
  velocity << state[0], state[1], state[2], state[3], state[4], state[5];
  position << state[6], state[7], state[8], state[9], state[10], state[11];
  relative_velocity << state[12], state[13], state[14], state[15], state[16],
      state[17];
  dy = StateDerivative(velocity, relative_velocity, position, t);
  for (size_t index = 0; index < 12; ++index) {
    state_derivative[index] = dy[index];
  }
}

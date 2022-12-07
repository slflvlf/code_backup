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

#ifndef MARINE_STRUCTURE_H
#define MARINE_STRUCTURE_H

#include <Eigen>
#include <array>
#include <string>
#include <vector>
#include "dpvessel_global.h"

//* class MarineStructure
/**
* MarineSturcture is a base class, which defines the basic properties, kinematics,
* and dynamics of a marine structure. Waves, winds, currents and controls are not
* considered here.
*/
class DPVESSELSHARED_EXPORT MarineStructure
{
protected:
  std::string name_; /**< name id of the structore */

  // new matrix types
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  typedef Eigen::Matrix<double, 12, 1> Vector12d;
  typedef Eigen::Matrix<double, 6, 6> Matrix6d;

  // physical parameters of the marine structure
  const double MASS_;
  const double GRAVITY_ = 9.81;
  const std::array<double, 3> CENTER_OF_GRAVITY_;
  const std::array<double, 3> CENTER_OF_BUOYANCY_{ { 0, 0, 0 } };
  const Eigen::Matrix3d INERTIA_TENSOR_;
  const Matrix6d ADDED_MASS_MATRIX_; /**< added mass matrix at the lowest frequency */
  const Matrix6d LINEAR_DAMPING_MATRIX_;

  // motion states of the marine structure
  Vector6d velocity_;          /**< current velocity */
  Vector6d relative_velocity_; /**< current velocity with respect to ocean current */
  Vector6d position_;          /**< current position and orientation */
  Eigen::Vector3d euler_;      /**< current Euler angle */
  Eigen::Vector4d quaternion_; /**< current quaternion */
  Vector6d external_force_;    /**< external force including wave, wind and control force, if exist */

  inline static Eigen::Matrix3d CrossProductOperator(const Eigen::Vector3d &vec);  //!< matrix which maps a vector to
                                                                                   //! the Lie algebra of SO(3)
  inline static Eigen::Matrix3d CrossProductOperator(const std::array<double, 3> &vec);  //! overloaaded
                                                                                         //! CrossProductorOperator
  static Eigen::Matrix3d RotationMatrix(const Eigen::Vector3d &angle);                   //!< returns rotation matrix
  static Eigen::Matrix3d AngularTransMatrix(const Eigen::Vector3d &angle);  //!< returns angular transformation matrix
  static Matrix6d TransformationMatrix(const Eigen::Vector3d &angle);  //!< transformation matrix from the body-fixed
                                                                       //! frame to
  //! the earth-fixed frame
  static Eigen::Vector4d Euler2Q(const Eigen::Vector3d &angle);  //!< transformation from Euler angle representation to
                                                                 //! quaternion

  // methods return the matrices and vectors in the equations of motion
  Matrix6d RigidBodyInertiaMatrix() const;                //!< returns inertia matrix
  Matrix6d RigidBodyCCMatrix(const Vector6d &vec) const;  //!< returns centripetal-coriolis matrix
  Matrix6d AddedMassCCMatrix(const Vector6d &vec) const;  //!< returns centripetal-coriolis matrix of added mass
  virtual Vector6d QuadraticDampingForce(const Vector6d &vec) const;  //!< returns quadratic damping force
  virtual Vector6d RestoringForce(const Vector6d &vec) const;  //!< returns restoring force due to gravity and buoyancy
  virtual Vector12d StateDerivative(const Vector6d &velocity, const Vector6d &relative_velocity,
                                    const Vector6d &position,
                                    const double /* t */) const;  //!< time derivative of the system state

public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*!
  * \brief a constructor
  */
  MarineStructure(const std::string &name, const double &mass, const std::array<double, 3> &gravity_center,
                  const std::array<double, 3> &moment_of_inertia, const std::array<double, 6> &init_velocity,
                  const std::array<double, 6> &init_position);

  /*!
  * \brief a destructor
  */
  ~MarineStructure()
  {
  }

  /*!
   * \brief return structure name
   */
  inline std::string getName()
  {
    return name_;
  }

  /*!
  * \brief sets a value to external_force_
  */
  inline void setExternalForce(const Vector6d &vec)
  {
    external_force_ = vec;
  }

  /*!
  * \brief overloaded setExternalForce function
  */
  inline void setExternalForce(const std::array<double, 6> &vec)
  {
    for (int i = 0; i < 6; ++i)
    {
      external_force_(i) = vec.at(i);
    }
  }

  /*!
  * \brief add a value to external_force_
  */
  inline void addExternalForce(const std::array<double, 6> &vec)
  {
    for (int i = 0; i < 6; ++i)
    {
      external_force_(i) += vec.at(i);
    }
  }

  /*!
   * \brief return structure velocity
   */
  std::array<double, 6> getVelocity() const
  {
    std::array<double, 6> state;
    for (size_t i = 0; i < 6; i++)
    {
      state.at(i) = velocity_(i);
    }

    return state;
  }

  /*!
   * \brief return structure position
   */
  std::array<double, 6> getPosition() const
  {
    std::array<double, 6> state;
    for (size_t i = 0; i < 6; i++)
    {
      state.at(i) = position_(i);
    }
    return state;
  }

  void operator()(const std::vector<double> &state, std::vector<double> &state_derivative,
                  const double);  //!<  operator function to integrate
};

#endif  // MARINE_STRUCTURE_H

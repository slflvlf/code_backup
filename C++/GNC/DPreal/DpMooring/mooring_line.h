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

#ifndef MOORING_LINE_H
#define MOORING_LINE_H

#include <Eigen>
#include <array>
#include <numeric>
#include <vector>
#include "dpmooring_global.h"

//* class MooringLine
/**
* Class MooringLine defines the properties of a single mooring line, and
* provides static and dynamic solutions using lumped mass method.
*/
class DPMOORINGSHARED_EXPORT MooringLine
{
private:
  // constants
  const double GRAVITY_ = 9.80665;
  const double WATER_DENSITY_ = 1025;

  // positions of fairlead and trench
  std::array<double, 3> fairlead_pos_;
  std::array<double, 3> trench_pos_;
  double depth_;

  // properties of a mooring line composed of several sections
  // the properties are defined from fairlead to ground
  std::vector<double> Cd_;                /**< normal drag coefficient */
  std::vector<double> Ca_;                /**< normal added mass coefficient */
  std::vector<double> EA_;                /**< tensile stiffness (unit: MN) */
  std::vector<double> section_length_;    /**< length of each section */
  std::vector<unsigned int> segment_num_; /**< segment number of each section */
  unsigned int total_segment_num_;        /**< total number of segments */
  std::vector<double> wet_weight_;        /**< wet weight of each section (unit: kg/m) */
  std::vector<double> diameter_;          /**< diameter of each section (unit: m) */

  std::vector<double> original_segment_length_;        /**< original segment length in each section */
  std::vector<double> segment_length_;                 /**< length of each segment  */
  std::vector<double> segment_weight_;                 /**< weight of each segment */
  std::vector<double> node_weight_;                    /**< wet weight of each node */
  std::vector<std::array<double, 3>> node_coordinate_; /**< coordinate of each node */
  Eigen::VectorXd gravity_force_;                      /**< gravity force vector */
  Eigen::Vector3d top_tension_;                        /**< top tension at fairlead (unit: kN) */

  void printProgBar(const size_t &percent);

  // method to initialize line shape
  void initializeLineShape();  //!< initilize line shape using a parabola function in class constructor
  void initializeLineShape(const std::array<double, 3> &);  //!< initialize line shape using a parabola function with a
                                                            //! new fairlead position

  // methods to generate matrices and vectors
  Eigen::SparseMatrix<double> getInertiaMatrix() const;           //!< return inertia matrix
  Eigen::SparseMatrix<double> getAddedMassMatrix() const;         //!< return added mass inertia matrix
  Eigen::VectorXd getGravityForce() const;                        //!< return gravity force vector
  Eigen::VectorXd getContactForce() const;                        //!< return seabed contact force vector
  Eigen::SparseMatrix<double> getTensionMatrix() const;           //!< return tension matrix
  Eigen::SparseMatrix<double> getTensionStiffnessMatrix() const;  //!< return tension stiffness matrix K^{Te}
  Eigen::SparseMatrix<double> getContactStiffnessMatrix() const;  //!< return seabed contact stiffness matrix K^{b}
  Eigen::VectorXd getDragForce(const Eigen::VectorXd &,
                               const Eigen::VectorXd &) const;  //!< return hydrodynamic drag force vector
  Eigen::Vector3d getTangentUnitVector(const int &) const;      //!< return tangent unit vector
  Eigen::Vector3d getStaticTopTension() const;   //!< return the tension at fairlead after a static analysis
  Eigen::Vector3d getDynamicTopTension() const;  //!< return the tension at fairlead in a dynamic analysis
  Eigen::VectorXd getCurrentVelocity(const std::array<double, 3> &current_velocity) const;  //!< return current velocity
                                                                                            //! profile

  // vectors which store node velocity and acceleration, mainly used in dynamic analysis
  Eigen::VectorXd node_velocity_, node_acceleration_;

  void updateSegmentLength();  //!< update segment length

public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*!
  * \brief a constructor
  */
  MooringLine(const std::array<double, 3> &fairlead_pos, const std::array<double, 3> &trench_pos,
              std::vector<double> &Cd, std::vector<double> &Ca, std::vector<double> &EA,
              std::vector<double> &section_length, std::vector<unsigned int> &segment_num,
              std::vector<double> &wet_weight, std::vector<double> &diameter)
    : fairlead_pos_(fairlead_pos)
    , trench_pos_(trench_pos)
    , Cd_(Cd)
    , Ca_(Ca)
    , EA_(EA)
    , section_length_(section_length)
    , segment_num_(segment_num)
    , wet_weight_(wet_weight)
    , diameter_(diameter)
  {
    total_segment_num_ = std::accumulate(segment_num_.cbegin(), segment_num_.cend(), 0);
    depth_ = -trench_pos_.back();

    // initialize segment length and weight
    for (unsigned int section_num = 0; section_num < segment_num_.size(); ++section_num)
    {
      double length = section_length_.at(section_num) / segment_num_.at(section_num);
      original_segment_length_.push_back(length);

      for (unsigned int n = 0; n < segment_num_.at(section_num); ++n)
      {
        segment_length_.push_back(length);
        segment_weight_.push_back(wet_weight_.at(section_num) * segment_length_.back());
      }
    }

    // compute each node weight
    node_weight_.push_back(segment_weight_.front() / 2);  // first node at fairlead
    for (unsigned int n = 0; n < total_segment_num_ - 1; ++n)
    {
      node_weight_.push_back(segment_weight_.at(n) / 2 + segment_weight_.at(n + 1) / 2);
    }
    node_weight_.push_back(segment_weight_.back() / 2);  // last node

    // compute the gravity force
    gravity_force_ = getGravityForce();

    // initialize line shape
    initializeLineShape();

    // initialize node velocity and acceleration
    node_velocity_ = node_acceleration_ = Eigen::VectorXd::Zero(3 * total_segment_num_ - 3);
  }

  /*!
  * \brief a destructor
  */
  ~MooringLine()
  {
  }

  // save node coordinates to a file
  void writeNodeCoordinate(const std::string &) const;

  // return top tension vector
  std::array<double, 3> getTopTension() const
  {
    return std::array<double, 3>{ top_tension_[0], top_tension_[1], top_tension_[2] };
  };

  // save top tension to a file
  void writeTopTension(const int &, const std::string &, const std::string &) const;

  // read fairlead motion data from a file
  std::vector<std::array<double, 3>> readFairleadPosition(const std::string &) const;

  /*!
  * \brief the method to solve static equation
  *
  * \param fairlead_pos fairlead cooridinate in the global frame
  * \param tolerance convergence tolerance
  * \param max_iteration maximum iteration number
  * \param relaxation_factor relaxation factor of the Newton-Raphson method
  */
  int solveStaticEquation(const std::array<double, 3> &fairlead_pos, const double &tolerance = 1E-7,
                          const int &max_iteration = 500, const double &relaxation_factor = 0.25);
  /*!
  * \brief the method to solve dynamic equation during one time step
  */
  int solveDynamicEquation(const std::array<double, 3> &, const std::array<double, 3> &, const double &time_step = 0.1,
                           const double &alpha = 0.505, const double &beta = 0.256, const double &tolerance = 1E-7,
                           const int &max_iteration = 500);

  /*!
  * \brief the method to conduct a dynamic analysis with predefined fairlead motions
  */
  int conductDynamicAnalysis(const std::vector<std::array<double, 3>> &, const std::array<double, 3> &,
                             const double &time_step = 0.1, const double &alpha = 0.505, const double &beta = 0.256,
                             const double &tolerance = 1E-7, const int &max_iteration = 500);

  /*!
  * \brief compute mooring force in body-fixed frame
  * \param mooring_force mooring line top tension expressed in the global frame
  * \param euler_angle euler angle of the vessel
  */
  std::array<double, 6> ComputeMooringForce(const std::array<double, 3> &top_tension, const double &roll,
                                            const double &pitch, const double &yaw) const;

  /*!
   * \brief return node coordinates
   */
  std::vector<std::array<double, 3>> getNodeCoordinate() const
  {
    return node_coordinate_;
  }
};

#endif  // MOORING_LINE_H

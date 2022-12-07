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

#ifndef MOORING_SIMULATION_H
#define MOORING_SIMULATION_H

#include <array>
#include <memory>
#include <mooring_line.h>
#include <vector>

struct MooringSystemProperty {
  std::string name;
  std::vector<double> Cd;
  std::vector<double> Ca;
  std::vector<double> EA;
  std::vector<double> section_length;
  std::vector<unsigned int> segment_number;
  std::vector<double> wet_weight;
  std::vector<double> diameter;
  std::vector<std::array<double, 3>> fairlead_positions;
  std::vector<std::array<double, 3>> trench_positions;
};

//* class MooringSimulation
/**
 * The class defines a time domain simulation interface for a mooring system.
 */
class MooringSimulation {
private:
  // properties of a mooring line composed of several sections
  // the properties are defined from fairlead to ground
  std::vector<double> Cd_;             /**< normal drag coefficient */
  std::vector<double> Ca_;             /**< normal added mass coefficient */
  std::vector<double> EA_;             /**< tensile stiffness (unit: MN) */
  std::vector<double> section_length_; /**< length of each section */
  std::vector<unsigned int>
      segment_number_; /**< segment number of each section */
  std::vector<double>
      wet_weight_;               /**< wet weight of each section (unit: kg/m) */
  std::vector<double> diameter_; /**< diameter of each section (unit: m) */

  // mooring system
  std::vector<std::array<double, 3>>
      fairlead_coor_; /**< fairlead coordinates of mooring lines in the
                         body-fixed frame */
  std::vector<std::array<double, 3>>
      trench_coor_; /**< trench coordinates of mooring lines in the Earth-fixed
                       frame */
  std::vector<std::array<double, 3>>
      top_tensions_;                       /**< top tensions of mooring lines */
  unsigned int line_number_;               /**< number of mooring lines */
  //  数据类型是类？？？
  std::vector<MooringLine> mooring_lines_; /**< mooring lines */

  // ocean environment
  std::array<double, 3> current_velocity_; /**< ocean surface current velocity
                                              vector (unit: m/s) */

  // simulation parameters
  double time_step_; /**< time step of dynamical analysis */

public:
  /*!
   * \brief a constructor
   */
  MooringSimulation(const MooringSystemProperty &mooring_system,
                    const std::array<double, 3> &current_velocity,
                    const double &time_step);

  /*!
   * \brief conduct a static analysis
   *
   * \param center  horizontal coordinate of the platform center
   * \param tolerance  convergence tolerance for solving the static equation
   * \param max_iteration  maximum iteration for solving the static equation
   * \param relaxation_factor  relaxation factor for solving the static equation
   */
  void ConductStaticAnalysis(const std::array<double, 3> &center,
                             const double &tolerance = 1e-7,
                             const int &max_iteration = 500,
                             const double &relaxation_factor = 0.2);

  /*!
   * \brief compute fairlead coordinates in the Earth-fixed reference frame
   *
   * \param vessel_position  vessel position in the Earth-fixed reference frame
   */
  std::vector<std::array<double, 3>>
  ComputeFairleadCoords(const std::array<double, 6> &vessel_position);

  /*!
   * \brief compute cross product of two vectors
   */
  std::array<double, 3> CrossProduct(const std::array<double, 3> &vec1,
                                     const std::array<double, 3> &vec2) {
    std::array<double, 3> out;
    out.at(0) = vec1.at(1) * vec2.at(2) - vec1.at(2) * vec2.at(1);
    out.at(1) = vec1.at(2) * vec2.at(0) - vec1.at(0) * vec2.at(2);
    out.at(2) = vec1.at(0) * vec2.at(1) - vec1.at(1) * vec2.at(0);
    return out;
  }

  /*!
   * \brief publish the coordinates of mooring line nodes
   */
  std::vector<double>
  OneStepSimulation(const std::array<double, 6> &vessel_position);

  /*!
   * \brief return number of mooring lines
   */
  unsigned int getMooringLineNumber() const { return line_number_; }

  /*!
   * \brief return number of mooring lines
   */
  std::vector<std::array<double, 3>>
  getLineNodeCoordinate(const unsigned int &index) const {
    return mooring_lines_.at(index).getNodeCoordinate();
  }
};

#endif // MOORING_SIMULATION_H

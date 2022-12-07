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

#include "mooring_simulation.h"
#include <cstdio>

MooringSimulation::MooringSimulation(
    const MooringSystemProperty &mooring_system,
    const std::array<double, 3> &current_velocity, const double &time_step)
    : current_velocity_(current_velocity), time_step_(time_step) {
  // mooring line properties
  fairlead_coor_ = mooring_system.fairlead_positions;
  trench_coor_ = mooring_system.trench_positions;
  Cd_ = mooring_system.Cd;
  Ca_ = mooring_system.Ca;
  EA_ = mooring_system.EA;
  section_length_ = mooring_system.section_length;
  segment_number_ = mooring_system.segment_number;
  wet_weight_ = mooring_system.wet_weight;
  diameter_ = mooring_system.diameter;

  // define the mooring lines
  line_number_ = fairlead_coor_.size();

  for (unsigned int i = 0; i < line_number_; ++i) {
    MooringLine mooring_line =
        MooringLine(fairlead_coor_.at(i), trench_coor_.at(i), Cd_, Ca_, EA_,
                    section_length_, segment_number_, wet_weight_, diameter_);

    mooring_lines_.push_back(mooring_line);
  }

  // conduct a static analysis to initialize the mooring system
  ConductStaticAnalysis({{0, 0, 0}});
}

std::vector<double> MooringSimulation::OneStepSimulation(
    const std::array<double, 6> &vessel_position) {
  auto fairlead_coords = ComputeFairleadCoords(vessel_position);

  std::vector<double> mooring_force(6, 0);

  // solve dynamic equation
  for (unsigned int i = 0; i < line_number_; i++) {
    auto exit_flag = mooring_lines_.at(i).solveDynamicEquation(
        fairlead_coords.at(i), current_velocity_, time_step_);

    if (exit_flag != 0) {
      printf(
          "Mooring line No. %2u failed to solve its dynamic equation. The exit "
          "flag is %2d",
          i, exit_flag);
      break;
    }

    auto top_tension = mooring_lines_.at(i).getTopTension();
    auto mooring_vector = mooring_lines_.at(i).ComputeMooringForce(
        top_tension, vessel_position.at(3), vessel_position.at(4),
        vessel_position.at(5));

    for (size_t j = 0; j < 6; j++) {
      mooring_force.at(j) += mooring_vector.at(j);
    }
  }

  return mooring_force;
}

void MooringSimulation::ConductStaticAnalysis(
    const std::array<double, 3> &center, const double &tolerance,
    const int &max_iteration, const double &relaxation_factor) {
  std::array<double, 6> top_tension = {{0, 0, 0, 0, 0, 0}};

  // printf("Conduct a static analysis, the center position is at (%5.2f, %5.2f,
  // %5.2f).", center.at(0), center.at(1),
  //        center.at(2));

  auto fairlead = ComputeFairleadCoords(
      {{center.at(0), center.at(1), 0, 0, 0, center.at(2)}});

  for (unsigned int i = 0; i < line_number_; ++i) {
    auto exit_flag = mooring_lines_.at(i).solveStaticEquation(
        fairlead.at(i), tolerance, max_iteration, relaxation_factor);

    if (exit_flag != 0) {
      printf(
          "Solving the static equation of line No.%1i failed. The flag is %1i.",
          i + 1, exit_flag);
      return;
    }

    for (unsigned int j = 0; j < top_tension.size(); ++j) {
      if (j <= 2) {
        top_tension.at(j) += mooring_lines_.at(i).getTopTension().at(j);
      } else {
        top_tension.at(j) +=
            (CrossProduct(fairlead.at(i), mooring_lines_.at(i).getTopTension()))
                .at(j - 3);
      }
    }
  }

  // printf("The total force is (%5.2f, %5.2f, %5.2f, %5.2f, %5.2f, %5.2f).",
  // top_tension.at(0), top_tension.at(1),
  //        top_tension.at(2), top_tension.at(3), top_tension.at(4),
  //        top_tension.at(5));
}

std::vector<std::array<double, 3>> MooringSimulation::ComputeFairleadCoords(
    const std::array<double, 6> &vessel_position) {
  std::vector<std::array<double, 3>> global_fairlead = fairlead_coor_;

  double roll = vessel_position.at(3);
  double pitch = vessel_position.at(4);
  double yaw = vessel_position.at(5);

  //   definition of the transformation matrix ????
  for (size_t i = 0; i < line_number_; i++) {
    global_fairlead.at(i).at(0) =
        std::cos(yaw) * std::cos(pitch) * fairlead_coor_.at(i).at(0) -
        std::cos(pitch) * std::sin(yaw) * fairlead_coor_.at(i).at(1) +
        std::sin(pitch) * fairlead_coor_.at(i).at(2);

    global_fairlead.at(i).at(1) =
        -std::cos(pitch) * std::sin(roll) * fairlead_coor_.at(i).at(2) +
        (std::cos(roll) * std::sin(yaw) +
         std::sin(roll) * std::sin(pitch) * std::cos(yaw)) *
            fairlead_coor_.at(i).at(0) +
        (std::cos(roll) * std::cos(yaw) -
         std::sin(roll) * std::sin(pitch) * std::sin(yaw)) *
            fairlead_coor_.at(i).at(1);

    global_fairlead.at(i).at(2) =
        std::cos(roll) * std::cos(pitch) * fairlead_coor_.at(i).at(2) +
       (std::sin(roll) * std::sin(yaw) - std::cos(roll) * std::sin(pitch) * std::cos(yaw)) *
            fairlead_coor_.at(i).at(0) +
       (std::sin(roll) * std::cos(yaw) + std::cos(roll) * std::sin(pitch) * std::sin(yaw)) *
            fairlead_coor_.at(i).at(1);

    global_fairlead.at(i).at(0) += vessel_position.at(0);
    global_fairlead.at(i).at(1) += vessel_position.at(1);
    global_fairlead.at(i).at(2) += vessel_position.at(2);
  }

  return global_fairlead;
}

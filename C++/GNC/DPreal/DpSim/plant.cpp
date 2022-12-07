/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Bo Li <lither@sjtu.edu.cn>
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

#include "plant.h"
#include <QDebug>

Plant::Plant(const QString &mode, QObject *parent) : QObject(parent), positioning_mode(mode) {
  // define the simulation interface
  SemiSubmersibleProperty vessel_sim;

  std::vector<double> frequency_r; // radiation frequency list
  for (int i = 0; i < 84; ++i) {
    frequency_r.push_back(0.03 * i + 0.01);
  }

  std::vector<int> direction; // direction list
  for (int i = 0; i < 41; ++i) {
    direction.push_back(-5 + 5 * i);
  }

  std::vector<double> frequency_w; // wave load frequency list
  for (int i = 0; i < 50; ++i) {
    frequency_w.push_back(0.03 * i + 0.01);
  }

  vessel_sim.name = "Oil_rig";
  vessel_sim.mass = 5.2509E7;
  vessel_sim.gravity_center = {{0, 0, 6.84}};
  vessel_sim.moment_of_inertia = {{56491282560, 57877520160, 75026959560}};
  vessel_sim.restoring_vector = {{1.29E7, 2.49E9, 3.55E9}};
  vessel_sim.direction_list = direction;
  vessel_sim.radiation_frequency_list = frequency_r;
  vessel_sim.wave_load_frequency_list = frequency_w;
  vessel_sim.data_directory = "C:/Code/C++/GNC/semi/data/";
  vessel_sim.is_diagonal = false;

  WindLoadData wind_load;
  wind_load.wind_x = {1E6,      1.078E6,  0.794E6,  0.692E6, -0.079E6,
                      -0.916E6, -1.133E6, -1.196E6, -1.030E6};
  wind_load.wind_y = {-0.0057E6, 0.688E6, 0.908E6, 1.047E6,  1.06E6,
                      0.873E6,   0.759E6, 0.474E6, -0.0292E6};
  wind_load.wind_n = {-0.2E6,   4.524E6, 4.323E6, 3.086E6, 0.201E6,
                      -0.917E6, 1.401E6, 2.853E6, 0.119E6};
  wind_load.angle_list = {0,          M_PI / 6,     M_PI_4,
                          M_PI / 3,   M_PI_2,       M_PI / 3 * 2,
                          M_PI_4 * 3, M_PI / 6 * 5, M_PI};
  wind_load.wind_speed = 27;

  CurrentLoadData current_load;
  current_load.current_x = {0.274E6,   0.307E6,  0.2966E6, 0.121E6, -0.035E6,
                            -0.1867E6, -0.274E6, -0.331E6, -0.243E6};
  current_load.current_y = {-0.029E6, 0.411E6, 0.531E6,  0.595E6,  0.4726E6,
                            0.518E6,  0.473E6, 0.3475E6, -0.0113E6};
  current_load.current_n = {-0.54E6,  2.394E6,  4.676E6,  5.461E6, -0.4148E6,
                            -4.368E6, -4.512E6, -3.249E6, -0.113E6};
  current_load.angle_list = {0,          M_PI / 6,     M_PI_4,
                             M_PI / 3,   M_PI_2,       M_PI / 3 * 2,
                             M_PI_4 * 3, M_PI / 6 * 5, M_PI};
  current_load.current_speed = 0.65;

  JonswapSpectrumProperty wave;
  wave.Hs = 5.27;
  wave.T0 = 10.4;
  wave.gamma = 3.3;
  wave.cutoff_frequency = 1.4;
  wave.component_number = 50;

  ConstantWindProperty wind;
  ConstantCurrentProperty current;
  wind.speed = 27;
  current.speed = 0.65;

  double env_direction = 60;
  wind.direction = env_direction;
  current.direction = env_direction;
  wave.direction = env_direction;

  MooringSystemProperty mooring;
  mooring.Cd = {1.0, 1.0, 1.0};
  mooring.Ca = {1.0, 1.0, 1.0};
  mooring.EA = {633, 264, 633};
  mooring.section_length = {150, 2650, 1450};
  mooring.segment_number = {3, 53, 29};
  mooring.wet_weight = {139.2, 4.2, 141.36};
  mooring.diameter = {0.084, 0.16, 0.084};

  std::array<double, 3> fairlead_pos_1 = {{34.3, -41.8, 2}};
  std::array<double, 3> fairlead_pos_2 = {{28.5, -41.8, 2}};
  std::array<double, 3> fairlead_pos_3 = {{-28.5, -41.8, 2}};
  std::array<double, 3> fairlead_pos_4 = {{-34.3, -41.8, 2}};
  std::array<double, 3> fairlead_pos_5 = {{-34.3, 41.8, 2}};
  std::array<double, 3> fairlead_pos_6 = {{-28.5, 41.8, 2}};
  std::array<double, 3> fairlead_pos_7 = {{28.5, 41.8, 2}};
  std::array<double, 3> fairlead_pos_8 = {{34.3, 41.8, 2}};

  std::array<double, 3> trench_pos_1 = {{3414.63, -1993.52, -1500}};
  std::array<double, 3> trench_pos_2 = {{1979.47, -3422.01, -1500}};
  std::array<double, 3> trench_pos_3 = {{-1979.47, -3422.01, -1500}};
  std::array<double, 3> trench_pos_4 = {{-3414.63, -1993.52, -1500}};
  std::array<double, 3> trench_pos_5 = {{-3414.63, 1993.52, -1500}};
  std::array<double, 3> trench_pos_6 = {{-1979.47, 3422.01, -1500}};
  std::array<double, 3> trench_pos_7 = {{1979.47, 3422.01, -1500}};
  std::array<double, 3> trench_pos_8 = {{3414.63, 1993.52, -1500}};

  mooring.fairlead_positions.push_back(fairlead_pos_1);
  mooring.fairlead_positions.push_back(fairlead_pos_2);
  mooring.fairlead_positions.push_back(fairlead_pos_3);
  mooring.fairlead_positions.push_back(fairlead_pos_4);
  mooring.fairlead_positions.push_back(fairlead_pos_5);
  mooring.fairlead_positions.push_back(fairlead_pos_6);
  mooring.fairlead_positions.push_back(fairlead_pos_7);
  mooring.fairlead_positions.push_back(fairlead_pos_8);

  mooring.trench_positions.push_back(trench_pos_1);
  mooring.trench_positions.push_back(trench_pos_2);
  mooring.trench_positions.push_back(trench_pos_3);
  mooring.trench_positions.push_back(trench_pos_4);
  mooring.trench_positions.push_back(trench_pos_5);
  mooring.trench_positions.push_back(trench_pos_6);
  mooring.trench_positions.push_back(trench_pos_7);
  mooring.trench_positions.push_back(trench_pos_8);

  simulation_interface = new SimulationInterface(
      vessel_sim, mooring, wave, wind, current, wind_load, current_load, 200,
      3600 * 3, 0.1, positioning_mode.toStdString(), {{0, 0, 0, 0, 0, 0}});
  simulation_interface->moveToThread(&plant_thread);

  connect(&plant_thread, &QThread::finished, simulation_interface,
          &QObject::deleteLater);
  connect(&plant_thread, &QThread::started, simulation_interface,
          &SimulationInterface::StartSimulation);
  connect(this, &Plant::messageReceived, this, &Plant::sendActuation);
  connect(simulation_interface, &SimulationInterface::messageReady, this,
          &Plant::messageReady);
  connect(this, &Plant::abortSimulation, this, &Plant::sendAbort);

  plant_thread.start();
}

Plant::~Plant() {
  plant_thread.quit();
  plant_thread.wait();
  qDebug() << "Plant has exited";
}

void Plant::sendActuation(const QVector<double> &msg) {
  simulation_interface->setControlForce(msg);
}

void Plant::sendAbort() {
  simulation_interface->setAbort();
  qDebug() << "Send abort signal to the running simulation";
}

#include "env_load_interface.h"
#include <QDebug>


env_load_interface::env_load_interface()
{

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

    env_dir_ = env_direction;


    // define wind load
    std::array<std::vector<double>, 3> data_list = {
        {wind_load.wind_x, wind_load.wind_y, wind_load.wind_n}};
    ptr_wind_load_ = std::make_unique<WindLoad>(data_list, wind_load.angle_list,
                                                wind_load.wind_speed);

    // define current load
    data_list = {
        {current_load.current_x, current_load.current_y, current_load.current_n}};
    ptr_current_load_ = std::make_unique<CurrentLoad>(
        data_list, current_load.angle_list, current_load.current_speed);

    // define a constant wind
    ptr_wind_ = std::make_unique<ConstantWind>(wind.speed, wind.direction);

    // define a constant current
    ptr_current_ =
        std::make_unique<ConstantCurrent>(current.speed, current.direction);

    // define and initialize Jonswap wave
    ptr_wave_ = std::make_unique<JonswapSpectrum>(wave.Hs, wave.T0, wave.gamma);
    ptr_wave_->initializeWaveComponents(wave.cutoff_frequency,
                                        wave.component_number);


    std::string rao_path = "C:/Code/C++/GNC/semi/data/forcerao.csv";
    std::string path = "C:/Code/C++/GNC/semi/data";
    ptr_tau_wave1_ = std::make_unique<WaveExcitation>(rao_path, frequency_w);
    ptr_tau_wave2_ = std::make_unique<QtfDataBase>(path, frequency_w);

    ptr_tau_wave1_->initializeWaveExcitation(*ptr_wave_);

    ptr_tau_wave2_->initializeWaveDriftForce(*ptr_wave_);





}

void env_load_interface::compute_envload(const std::array<double, 3> &pose, const int &simulate_time)
{
    for (int i = 0; i < simulate_time; i++)
    {
        double heading = pose[2];
        tau_wind_ = ptr_wind_load_-> getWindLoad(
            ptr_wind_->getSpeed(), ptr_wind_->getDirection() * M_PI / 180 - heading);

        tau_wave2_ = ptr_tau_wave2_->getWaveDriftForceFullQTF(i, pose.at(0), pose.at(1), env_dir_* M_PI / 180, 0, heading);

        qDebug()<<tau_wave2_.at(0)<<"\n";




    }



}

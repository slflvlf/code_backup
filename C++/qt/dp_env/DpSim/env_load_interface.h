#ifndef ENV_LOAD_INTERFACE_H
#define ENV_LOAD_INTERFACE_H

#include <Eigen>
#include <memory>

#include <constant_current.h>
#include <constant_wind.h>
#include <jonswap_spectrum.h>
#include "qtf_database.h"
#include "radiation_force.h"
#include "semi_submersible.h"
#include "wave_excitation.h"



#include "rao_database.h"
#include "wave_excitation.h"

class env_load_interface
{
    std::array<double, 3>  tau_wind_, tau_current_;
    std::array<double, 6> tau_wave1_, tau_wave2_;

    std::unique_ptr<JonswapSpectrum> ptr_wave_; /**< Jonswap wave */
    std::unique_ptr<QtfDataBase> ptr_qtf_;      /**< QTF of the semi */
    std::unique_ptr<WindLoad> ptr_wind_load_;   /**< wind load of the semi */
    std::unique_ptr<CurrentLoad>
        ptr_current_load_;                   /**< current load of the semi */
    std::unique_ptr<ConstantWind> ptr_wind_; /**< constant wind in simulation */
    std::unique_ptr<ConstantCurrent>
        ptr_current_;


    std::unique_ptr<RaoDataBase> ptr_rao_;

    std::unique_ptr<WaveExcitation> ptr_tau_wave1_;
    std::unique_ptr<QtfDataBase> ptr_tau_wave2_;


    double env_dir_;

public:
    env_load_interface();
    void compute_envload(const std::array<double, 3>& pose,
                         const int& simulate_time);
};

#endif // ENV_LOAD_INTERFACE_H

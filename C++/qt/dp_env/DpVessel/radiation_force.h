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

#ifndef RADIATION_FORCE_H
#define RADIATION_FORCE_H

#include <array>
#include <deque>
#include <vector>
#include "dpvessel_global.h"

//* class RadiationForce
/**
* RadiationForce is a class, which computes impulse response functons and infinite-frequency added mass
*/
class DPVESSELSHARED_EXPORT RadiationForce
{
private:
  std::vector<double> frequency_; /**< frequency list of the added mass and damping */
  int frequency_num_ = 0;
  double step_size_ = 0.0;   /**< time step size of the impulse response function */
  double cutoff_time_ = 0.0; /**< cutoff time of the impulse response function */
  unsigned int impulse_response_size_ = 0;
  std::vector<std::array<std::array<double, 6>, 6>> added_mass_; /**< frequency-dependent added mass matrices */
  std::vector<std::array<std::array<double, 6>, 6>> damping_;    /**< frequency-dependent potential damping matrices */
  std::vector<std::array<std::array<double, 6>, 6>> impulse_response_;
  std::array<std::array<double, 6>, 6> added_mass_inf_; /**< infinite-frequency added mass */

public:
  /*!
  * \brief default constructor
  */
  RadiationForce() = default;

  /*!
  * \brief a constructor
  */
  RadiationForce(std::string, const std::vector<double> &);

  /*!
  * \brief destructor
  */
  ~RadiationForce()
  {
  }

  /*!
  * \brief calls computeImpulseResponse and computeAddedMassInf
  *
  * \param step_size time step size of the impulse response function
  * \param cutoff_time   cutoff time of the cutoff scaling function when computing impulse response function
  */
  void InitializeRaditionForce(const double &step_size, const double &cutoff_time = 60.0);

  /*!
  * \brief computes impulse response functions
  *
  * \param step_size  time step size of the impulse response function
  * \param cutoff_time   cutoff time of the cutoff scaling function when computing impulse response function
  */
  void computeImpulseResponse(const double &step_size, const double &cutoff_time);

  /*!
  * \brief computes infinite-frequency added mass
  */
  void computeAddedMassInf();

  /*!
  * \brief returns infinite-frequency added mass
  */
  std::array<std::array<double, 6>, 6> getAddedMassInf() const
  {
    return added_mass_inf_;
  }

  /*!
  * \brief returns impulse response functions
  */
  std::vector<std::array<std::array<double, 6>, 6>> getImpulseResponse() const
  {
    return impulse_response_;
  }

  /*!
  * \brief returns impulse response function at t
  */
  std::array<std::array<double, 6>, 6> getImpulseResponse(const unsigned int &index) const
  {
    return impulse_response_.at(index);
  }

  /*!
  * \brief returns the number of impulse response functions
  */
  unsigned int getImpulseResponseSize() const
  {
    return impulse_response_size_;
  }

  /*!
  * \brief computes and returns part of the radiation damping force given the wave perturbation velocity history
  *       which doesn't contain the contribution of the instantaneous vessel velocity
  *
  * \param velocity_history  wave perturbation velocity history
  * \return radiation damping force vector
  */
  std::array<double, 6> getRadiationDampingForce(std::deque<std::array<double, 6>> velocity_history) const;

  /*!
  * \brief returns step size of impulse response functions
  */
  double getTimeStepSize() const
  {
    return step_size_;
  }
};

#endif  // RADIATION_FORCE_H

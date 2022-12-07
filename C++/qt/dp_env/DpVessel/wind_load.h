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

#ifndef WIND_LOAD_H
#define WIND_LOAD_H

#include <gsl/gsl_errno.h>
#include <gsl/gsl_spline.h>
#include <array>
#include <memory>
#include <vector>
#include "dpvessel_global.h"

struct DPVESSELSHARED_EXPORT WindLoadData
{
  std::vector<double> wind_x;
  std::vector<double> wind_y;
  std::vector<double> wind_n;
  std::vector<double> angle_list;
  double wind_speed;
};

//* class WindLoad
/**
 * WindLoad is a class which uses vessel wind coefficients to compute wind force and moment.
 */

class DPVESSELSHARED_EXPORT WindLoad
{
private:
  // wind model test data
  std::array<std::vector<double>, 3> data_list_; /**< list of wind coefficient */
  std::vector<double> angle_list_;               /**< list of angle of attack */
  double data_speed_;                            /**< wind speed in model test */
  unsigned int data_size_;                       /**< list size */

  // gsl 1D interpolation interface
  double *x_;
  double *y_;
  gsl_interp_accel *acc_;
  gsl_spline *spline_;

public:
  /*!
   * \brief a constructor
   */
  WindLoad(const std::array<std::vector<double>, 3> &data, const std::vector<double> &angle, const double &speed)
    : data_list_(data), angle_list_(angle), data_speed_(speed)
  {
    data_size_ = angle_list_.size();
    acc_ = gsl_interp_accel_alloc();
    spline_ = gsl_spline_alloc(gsl_interp_steffen, data_size_);

    x_ = new double[data_size_];
    y_ = new double[data_size_];

    for (size_t i = 0; i < data_size_; i++)
    {
      x_[i] = angle_list_.at(i);
    }
  }

  /*!
   * \brief a destructor
   */
  ~WindLoad()
  {
    // deallocate array
    delete[] x_;
    delete[] y_;

    // free gsl 1D interpolation interface
    gsl_spline_free(spline_);
    gsl_interp_accel_free(acc_);
  }

  /*!
   * \brief returns a vector of wave excitation load
   *
   * \param speed  wind speed in m/s
   * \param angle_of_attack  angle of attack in radian
   * \return a vector of wind load
   */
  std::array<double, 3> getWindLoad(const double &speed, const double &angle_of_attack);
};

#endif  // WIND_LOAD_H

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

#ifndef CONSTANT_CURRENT_H
#define CONSTANT_CURRENT_H

#include "dpenvironment_global.h"

struct DPENVIRONMENTSHARED_EXPORT ConstantCurrentProperty
{
  double direction;
  double speed;
};

//* class ConstantCurrent
/**
 * Class ConstantCurrent defines an ocean current environment with constant speed
 */
class DPENVIRONMENTSHARED_EXPORT ConstantCurrent
{
  double speed_ = 0.0;     /**< current speed in m/s */
  double direction_ = 0.0; /**< current direction in degree */

public:
  /*!
   * \brief default contructor
   */
  ConstantCurrent() = default;

  /*!
   * \brief a contructor
   */
  ConstantCurrent(const double &speed, const double &direction) : speed_(speed), direction_(direction)
  {
  }

  /*!
   * \brief a destructor
   */
  ~ConstantCurrent()
  {
  }

  /*!
   * \brief return current speed
   */
  double getSpeed() const
  {
    return speed_;
  }

  /*!
   * \brief set current speed
   */
  void setSpeed(const double &speed)
  {
    speed_ = speed;
  }

  /*!
   * \brief return current speed
   */
  double getDirection() const
  {
    return direction_;
  }

  /*!
   * \brief set current direction
   */
  void setDirection(const double &direction)
  {
    direction_ = direction;
  }
};

#endif  // CONSTANT_CURRENT_H
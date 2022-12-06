/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * user_ode.h
 *
 * Code generation for function 'user_ode'
 *
 */

#pragma once

/* Include files */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "mex.h"
#include "emlrt.h"
#include "rtwtypes.h"
#include "BBS_types.h"

/* Function Declarations */
void user_ode(const emlrtStack *sp, const real_T x[6], const real_T u_data[],
              const int32_T u_size[1], const real_T p_ode_M[9], const real_T
              p_ode_D[9], const real_T p_ode_thrust_config[8], real_T xdot[6]);

/* End of code generation (user_ode.h) */

/*
 * user_ode.h
 *
 * Code generation for function 'user_ode'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void user_ode(const emlrtStack *sp, const real_T x[4], const real_T u_data[],
              int32_T u_size, const real_T p_ode_w[3], real_T xdot[4]);

/* End of code generation (user_ode.h) */

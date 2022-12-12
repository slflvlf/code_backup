/*
 * user_control_profile.h
 *
 * Code generation for function 'user_control_profile'
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
void user_control_profile(const emlrtStack *sp, const real_T p[4],
                          real_T p_uparam_nu, real_T p_uparam_Np,
                          const real_T p_uparam_R[80], real_T u_profile_data[],
                          int32_T u_profile_size[2]);

/* End of code generation (user_control_profile.h) */

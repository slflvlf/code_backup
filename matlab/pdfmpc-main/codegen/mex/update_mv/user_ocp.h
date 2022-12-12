/*
 * user_ocp.h
 *
 * Code generation for function 'user_ocp'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "update_mv_types.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void user_ocp(const emlrtStack *sp, const emxArray_real_T *xx,
              const real_T uu_data[], const int32_T uu_size[2], real_T p_ode_u0,
              real_T p_uparam_Np, const real_T p_ocp_Q[16], real_T p_ocp_R,
              real_T p_ocp_M, real_T p_ocp_rd, real_T p_ocp_theta_max,
              real_T p_ocp_thetap_max, real_T *J, real_T *g);

/* End of code generation (user_ocp.h) */

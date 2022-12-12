/*
 * BBS.h
 *
 * Code generation for function 'BBS'
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
void BBS(const emlrtStack *sp, real_T eta, const real_T param_p[4],
         real_T param_ell, real_T param_uparam_nu, real_T param_uparam_Np,
         const real_T param_uparam_R[80], real_T param_ode_tau,
         real_T param_ode_rk_order, real_T param_ode_u0,
         const real_T param_ode_w[3], const struct4_T *param_ocp,
         const real_T param_x0[4], real_T *J, real_T *g);

/* End of code generation (BBS.h) */

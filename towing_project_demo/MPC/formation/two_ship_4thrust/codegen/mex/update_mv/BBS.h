/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * BBS.h
 *
 * Code generation for function 'BBS'
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
#include "update_mv_types.h"

/* Function Declarations */
void BBS(const emlrtStack *sp, real_T eta, const real_T param_p[64], real_T
         param_ell, real_T param_uparam_nu, real_T param_uparam_Np, const real_T
         param_uparam_R[20480], real_T param_ode_Nship, real_T param_ode_tau,
         real_T param_ode_rk_order, const real_T param_ode_M[9], const real_T
         param_ode_D[9], const real_T param_ode_thrust_config[8], const real_T
         param_ode_u0[16], const struct4_T *param_ocp, const real_T param_x0[12],
         real_T *J, real_T *g);

/* End of code generation (BBS.h) */

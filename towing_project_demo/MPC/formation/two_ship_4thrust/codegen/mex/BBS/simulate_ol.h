/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * simulate_ol.h
 *
 * Code generation for function 'simulate_ol'
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
void simulate_ol(const emlrtStack *sp, const real_T p[64], real_T p_ode_Nship,
                 real_T p_ode_tau, real_T p_ode_rk_order, const real_T p_ode_M[9],
                 const real_T p_ode_D[9], const real_T p_ode_thrust_config[8],
                 const real_T p_ode_x0[12], real_T p_uparam_nu, real_T
                 p_uparam_Np, const real_T p_uparam_R[20480], emxArray_real_T
                 *tt, emxArray_real_T *xx, emxArray_real_T *uu);

/* End of code generation (simulate_ol.h) */

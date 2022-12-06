/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * user_control_profile.h
 *
 * Code generation for function 'user_control_profile'
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
void user_control_profile(const emlrtStack *sp, const real_T p[32], real_T
  p_uparam_nu, real_T p_uparam_Np, const real_T p_uparam_R[5120],
  emxArray_real_T *u_profile);

/* End of code generation (user_control_profile.h) */

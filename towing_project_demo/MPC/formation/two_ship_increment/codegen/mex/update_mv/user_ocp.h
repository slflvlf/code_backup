/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * user_ocp.h
 *
 * Code generation for function 'user_ocp'
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
void user_ocp(const emlrtStack *sp, const emxArray_real_T *xx, const
              emxArray_real_T *uu, real_T p_ode_Nship, real_T p_uparam_Np, const
              real_T p_ocp_Q[144], const real_T p_ocp_R[144], const real_T
              p_ocp_Rdu[144], const real_T p_ocp_xd_his[240], real_T p_ocp_F_max,
              real_T p_ocp_a_max, const real_T p_ocp_u_last[12], real_T *J,
              real_T *g);

/* End of code generation (user_ocp.h) */

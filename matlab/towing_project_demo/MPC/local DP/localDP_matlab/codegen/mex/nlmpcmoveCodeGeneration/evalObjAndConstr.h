/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * evalObjAndConstr.h
 *
 * Code generation for function 'evalObjAndConstr'
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
void evalObjAndConstr(const emlrtStack *sp,
                      const real_T c_obj_objfun_workspace_runtimed[6],
                      const real_T d_obj_objfun_workspace_runtimed[6],
                      const real_T e_obj_objfun_workspace_runtimed[60],
                      const real_T f_obj_objfun_workspace_runtimed[60],
                      const real_T g_obj_objfun_workspace_runtimed[60],
                      const real_T h_obj_objfun_workspace_runtimed[60],
                      const real_T i_obj_objfun_workspace_runtimed[60],
                      const real_T c_obj_nonlcon_workspace_runtime[6],
                      const real_T d_obj_nonlcon_workspace_runtime[60],
                      const real_T e_obj_nonlcon_workspace_runtime[60],
                      int32_T obj_mCineq, const real_T x[79],
                      real_T Cineq_workspace_data[],
                      const int32_T *Cineq_workspace_size, int32_T ineq0,
                      real_T Ceq_workspace[60], real_T *fval, int32_T *status);

/* End of code generation (evalObjAndConstr.h) */

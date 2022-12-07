/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * updatePenaltyParam.h
 *
 * Code generation for function 'updatePenaltyParam'
 *
 */

#pragma once

/* Include files */
#include "nlmpcmoveCodeGeneration_internal_types.h"
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void updatePenaltyParam(const emlrtStack *sp, struct_T *obj, real_T fval,
                        const real_T ineq_workspace_data[],
                        int32_T ineq_workspace_size, int32_T mIneq,
                        const real_T eq_workspace[60], int32_T sqpiter,
                        real_T qpval, const real_T x_data[], int32_T iReg0,
                        int32_T nRegularized);

/* End of code generation (updatePenaltyParam.h) */

/*
 * updateWorkingSetForNewQP.h
 *
 * Code generation for function 'updateWorkingSetForNewQP'
 *
 */

#pragma once

/* Include files */
#include "nlmpcmoveCodeGeneration_types.h"
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void b_updateWorkingSetForNewQP(const emlrtStack *sp, const real_T xk[85],
                                h_struct_T *WorkingSet, int32_T mIneq,
                                int32_T mNonlinIneq, const real_T cIneq_data[],
                                int32_T cIneq_size, const real_T cEq[60],
                                int32_T mLB, const real_T lb[85], int32_T mUB,
                                int32_T mFixed);

void updateWorkingSetForNewQP(const emlrtStack *sp, const real_T xk[85],
                              h_struct_T *WorkingSet, int32_T mIneq,
                              const real_T cIneq_data[], int32_T cIneq_size,
                              const real_T cEq[60], int32_T mLB,
                              const real_T lb[85]);

/* End of code generation (updateWorkingSetForNewQP.h) */

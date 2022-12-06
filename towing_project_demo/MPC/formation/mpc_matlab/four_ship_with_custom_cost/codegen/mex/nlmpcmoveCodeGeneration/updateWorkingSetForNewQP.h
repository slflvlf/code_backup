/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
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
void b_updateWorkingSetForNewQP(const emlrtStack *sp, const real_T xk[265],
                                h_struct_T *WorkingSet, int32_T mIneq,
                                int32_T mNonlinIneq, const real_T cIneq_data[],
                                int32_T cIneq_size, const real_T cEq[192],
                                int32_T mLB, const real_T lb[265], int32_T mUB,
                                int32_T mFixed);

void updateWorkingSetForNewQP(const emlrtStack *sp, const real_T xk[265],
                              h_struct_T *WorkingSet, int32_T mIneq,
                              const real_T cIneq_data[], int32_T cIneq_size,
                              const real_T cEq[192], int32_T mLB,
                              const real_T lb[265]);

/* End of code generation (updateWorkingSetForNewQP.h) */

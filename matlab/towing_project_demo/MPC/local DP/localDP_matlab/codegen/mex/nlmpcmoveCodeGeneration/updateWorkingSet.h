/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * updateWorkingSet.h
 *
 * Code generation for function 'updateWorkingSet'
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
void updateWorkingSet(const emlrtStack *sp, h_struct_T *WorkingSet,
                      const real_T TrialState_cIneq_data[],
                      int32_T TrialState_cIneq_size,
                      const real_T TrialState_cEq[60],
                      const real_T TrialState_searchDir_data[],
                      int32_T workspace_int_data[],
                      const int32_T *workspace_int_size);

/* End of code generation (updateWorkingSet.h) */

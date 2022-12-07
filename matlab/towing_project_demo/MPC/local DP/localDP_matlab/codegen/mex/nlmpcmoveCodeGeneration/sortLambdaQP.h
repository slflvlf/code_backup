/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * sortLambdaQP.h
 *
 * Code generation for function 'sortLambdaQP'
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
void sortLambdaQP(const emlrtStack *sp, real_T lambda_data[],
                  const int32_T *lambda_size, int32_T WorkingSet_nActiveConstr,
                  const int32_T WorkingSet_sizes[5],
                  const int32_T WorkingSet_isActiveIdx[6],
                  const int32_T WorkingSet_Wid_data[],
                  int32_T WorkingSet_Wid_size,
                  const int32_T WorkingSet_Wlocalidx_data[],
                  int32_T WorkingSet_Wlocalidx_size, real_T workspace_data[],
                  const int32_T workspace_size[2]);

/* End of code generation (sortLambdaQP.h) */

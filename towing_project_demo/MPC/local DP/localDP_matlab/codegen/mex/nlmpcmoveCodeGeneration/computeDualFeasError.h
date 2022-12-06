/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * computeDualFeasError.h
 *
 * Code generation for function 'computeDualFeasError'
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
void b_computeDualFeasError(const emlrtStack *sp, int32_T nVar,
                            const real_T gradLag_data[], boolean_T *gradOK,
                            real_T *val);

void computeDualFeasError(const emlrtStack *sp, int32_T nVar,
                          const real_T gradLag_data[], int32_T gradLag_size,
                          boolean_T *gradOK, real_T *val);

/* End of code generation (computeDualFeasError.h) */

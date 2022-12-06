/*
 * computeComplError.h
 *
 * Code generation for function 'computeComplError'
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
real_T computeComplError(
    const emlrtStack *sp, int32_T fscales_lineq_constraint_size,
    int32_T fscales_cineq_constraint_size, const real_T xCurrent[85],
    int32_T mIneq, const real_T cIneq_data[], int32_T cIneq_size,
    const int32_T finiteLB_data[], int32_T finiteLB_size, int32_T mLB,
    const real_T lb[85], const int32_T finiteUB_data[], int32_T finiteUB_size,
    int32_T mUB, const real_T lambda_data[], int32_T lambda_size, int32_T iL0);

/* End of code generation (computeComplError.h) */

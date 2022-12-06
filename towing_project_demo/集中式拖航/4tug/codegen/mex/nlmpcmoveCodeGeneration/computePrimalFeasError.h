/*
 * computePrimalFeasError.h
 *
 * Code generation for function 'computePrimalFeasError'
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
real_T computePrimalFeasError(
    const emlrtStack *sp, const real_T x[85], int32_T mLinIneq,
    int32_T mNonlinIneq, const real_T cIneq_data[], int32_T cIneq_size,
    const real_T cEq[60], const int32_T finiteLB_data[], int32_T finiteLB_size,
    int32_T mLB, const real_T lb[85], const int32_T finiteUB_data[],
    int32_T finiteUB_size, int32_T mUB);

/* End of code generation (computePrimalFeasError.h) */

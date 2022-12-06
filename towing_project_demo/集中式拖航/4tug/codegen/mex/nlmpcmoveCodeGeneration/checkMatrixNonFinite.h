/*
 * checkMatrixNonFinite.h
 *
 * Code generation for function 'checkMatrixNonFinite'
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
int32_T checkMatrixNonFinite(const emlrtStack *sp, int32_T ncols,
                             const real_T mat_data[], int32_T mat_size,
                             int32_T col0, int32_T ldm);

/* End of code generation (checkMatrixNonFinite.h) */

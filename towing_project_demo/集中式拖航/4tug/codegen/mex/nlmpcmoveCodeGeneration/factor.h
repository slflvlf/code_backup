/*
 * factor.h
 *
 * Code generation for function 'factor'
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
void factor(const emlrtStack *sp, j_struct_T *obj, const real_T A[7225],
            int32_T ndims, int32_T ldA);

/* End of code generation (factor.h) */

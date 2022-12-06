/*
 * loadProblem.h
 *
 * Code generation for function 'loadProblem'
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
void loadProblem(const emlrtStack *sp, h_struct_T *obj, int32_T mIneq,
                 int32_T mLinIneq, const real_T Aineq_data[],
                 const int32_T Aineq_size[2], int32_T mLB, int32_T mConstrMax);

/* End of code generation (loadProblem.h) */

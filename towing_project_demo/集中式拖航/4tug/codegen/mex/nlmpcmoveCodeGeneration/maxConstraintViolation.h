/*
 * maxConstraintViolation.h
 *
 * Code generation for function 'maxConstraintViolation'
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
real_T b_maxConstraintViolation(const emlrtStack *sp, h_struct_T *obj,
                                const real_T x_data[], int32_T ix0);

real_T c_maxConstraintViolation(const emlrtStack *sp, h_struct_T *obj,
                                const real_T x_data[], int32_T x_size);

real_T maxConstraintViolation(const emlrtStack *sp, h_struct_T *obj,
                              const real_T x_data[]);

/* End of code generation (maxConstraintViolation.h) */

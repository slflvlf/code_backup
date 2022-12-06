/*
 * checkLinearInputs.h
 *
 * Code generation for function 'checkLinearInputs'
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
void checkLinearInputs(const emlrtStack *sp, const real_T Aineq_data[],
                       const int32_T Aineq_size[2], const real_T bineq_data[],
                       int32_T bineq_size, const real_T lb[85],
                       const real_T ub[85]);

/* End of code generation (checkLinearInputs.h) */

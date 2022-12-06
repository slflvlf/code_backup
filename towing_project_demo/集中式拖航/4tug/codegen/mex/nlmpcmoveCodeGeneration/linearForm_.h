/*
 * linearForm_.h
 *
 * Code generation for function 'linearForm_'
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
void linearForm_(const emlrtStack *sp, boolean_T obj_hasLinear,
                 int32_T obj_nvar, real_T workspace_data[],
                 const real_T H[7225], const real_T f_data[], int32_T f_size,
                 const real_T x_data[]);

/* End of code generation (linearForm_.h) */

/*
 * computeGrad_StoreHx.h
 *
 * Code generation for function 'computeGrad_StoreHx'
 *
 */

#pragma once

/* Include files */
#include "nlmpcmoveCodeGeneration_internal_types.h"
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void computeGrad_StoreHx(const emlrtStack *sp, f_struct_T *obj,
                         const real_T H[7225], const real_T f_data[],
                         const real_T x_data[], int32_T x_size);

/* End of code generation (computeGrad_StoreHx.h) */

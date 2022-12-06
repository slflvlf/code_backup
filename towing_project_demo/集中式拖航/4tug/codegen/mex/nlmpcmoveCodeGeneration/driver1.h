/*
 * driver1.h
 *
 * Code generation for function 'driver1'
 *
 */

#pragma once

/* Include files */
#include "nlmpcmoveCodeGeneration_internal_types.h"
#include "nlmpcmoveCodeGeneration_types.h"
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void b_driver(c_nlmpcmoveCodeGenerationStackD *SD, const emlrtStack *sp,
              const real_T H[7225], const real_T f_data[], int32_T f_size,
              e_struct_T *solution, g_struct_T *memspace,
              h_struct_T *workingset, i_struct_T *qrmanager,
              j_struct_T *cholmanager, f_struct_T *objective,
              d_struct_T *options, d_struct_T *runTimeOptions);

/* End of code generation (driver1.h) */

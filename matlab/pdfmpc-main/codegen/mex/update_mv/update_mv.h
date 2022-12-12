/*
 * update_mv.h
 *
 * Code generation for function 'update_mv'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "update_mv_types.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void update_mv(const emlrtStack *sp, struct0_T mv[4],
               const real_T subset_data[], const int32_T subset_size[1],
               struct1_T *param, real_T u_sol_data[], int32_T u_sol_size[2],
               real_T p_sol[4], emxArray_real_T *lesp);

/* End of code generation (update_mv.h) */

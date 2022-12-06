/*
 * znlmpc_generateRuntimeData.h
 *
 * Code generation for function 'znlmpc_generateRuntimeData'
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
void znlmpc_generateRuntimeData(const emlrtStack *sp, const real_T x[6],
                                const real_T lastMV[8], const real_T ref0[6],
                                const real_T MVTarget0[8], const real_T X0[60],
                                const real_T MV0[80], real_T Slack0,
                                l_struct_T *runtimedata, k_struct_T *userdata,
                                real_T z0[85]);

/* End of code generation (znlmpc_generateRuntimeData.h) */

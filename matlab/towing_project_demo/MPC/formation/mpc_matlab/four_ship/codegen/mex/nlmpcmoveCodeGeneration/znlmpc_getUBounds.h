/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * znlmpc_getUBounds.h
 *
 * Code generation for function 'znlmpc_getUBounds'
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
void znlmpc_getUBounds(c_nlmpcmoveCodeGenerationStackD *SD,
                       const emlrtStack *sp, const l_struct_T *runtimedata,
                       real_T A_data[], int32_T A_size[2], real_T Bu_data[],
                       int32_T *Bu_size);

/* End of code generation (znlmpc_getUBounds.h) */

/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * znlmpc_reformJacobian.h
 *
 * Code generation for function 'znlmpc_reformJacobian'
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
void znlmpc_reformJacobian(c_nlmpcmoveCodeGenerationStackD *SD,
                           const emlrtStack *sp, const real_T Jx_data[],
                           const int32_T Jx_size[3], const real_T Jmv_data[],
                           const int32_T Jmv_size[3], const real_T Je_data[],
                           int32_T Je_size, real_T Jc_data[],
                           int32_T Jc_size[2]);

/* End of code generation (znlmpc_reformJacobian.h) */

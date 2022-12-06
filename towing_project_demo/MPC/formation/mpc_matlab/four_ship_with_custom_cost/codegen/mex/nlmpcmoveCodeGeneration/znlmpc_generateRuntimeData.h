/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * znlmpc_generateRuntimeData.h
 *
 * Code generation for function 'znlmpc_generateRuntimeData'
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
void znlmpc_generateRuntimeData(const emlrtStack *sp, const real_T x[24],
                                const real_T lastMV[24], const real_T ref0[192],
                                const real_T MVTarget0[24],
                                const real_T X0[192], const real_T MV0[192],
                                real_T Slack0, l_struct_T *runtimedata,
                                k_struct_T *userdata, real_T z0[265]);

/* End of code generation (znlmpc_generateRuntimeData.h) */

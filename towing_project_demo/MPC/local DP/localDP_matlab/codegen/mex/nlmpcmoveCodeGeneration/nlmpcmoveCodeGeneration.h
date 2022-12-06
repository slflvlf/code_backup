/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * nlmpcmoveCodeGeneration.h
 *
 * Code generation for function 'nlmpcmoveCodeGeneration'
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
void c_nlmpcmoveCodeGeneration_anonF(
    c_nlmpcmoveCodeGenerationStackD *SD, const emlrtStack *sp,
    const real_T runtimedata_x[6], const real_T runtimedata_OutputMin[60],
    const real_T runtimedata_OutputMax[60], const real_T z[79],
    real_T varargout_1_data[], int32_T varargout_1_size[2],
    real_T varargout_2[60], real_T varargout_3_data[],
    int32_T varargout_3_size[2], real_T varargout_4[4740]);

void nlmpcmoveCodeGeneration(c_nlmpcmoveCodeGenerationStackD *SD,
                             const emlrtStack *sp, const real_T x[6],
                             const real_T lastMV[6], struct1_T *onlinedata,
                             real_T mv[6], struct2_T *info);

/* End of code generation (nlmpcmoveCodeGeneration.h) */

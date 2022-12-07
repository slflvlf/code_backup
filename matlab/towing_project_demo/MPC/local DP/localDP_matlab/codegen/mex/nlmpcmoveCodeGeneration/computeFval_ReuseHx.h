/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * computeFval_ReuseHx.h
 *
 * Code generation for function 'computeFval_ReuseHx'
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
real_T computeFval_ReuseHx(const emlrtStack *sp, const f_struct_T *obj,
                           real_T workspace_data[], const real_T f_data[],
                           int32_T f_size, const real_T x_data[],
                           int32_T x_size);

/* End of code generation (computeFval_ReuseHx.h) */

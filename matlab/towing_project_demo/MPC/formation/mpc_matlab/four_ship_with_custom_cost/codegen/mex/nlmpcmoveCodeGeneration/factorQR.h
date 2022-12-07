/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * factorQR.h
 *
 * Code generation for function 'factorQR'
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
void factorQR(const emlrtStack *sp, i_struct_T *obj, const real_T A_data[],
              int32_T mrows, int32_T ncols, int32_T ldA);

/* End of code generation (factorQR.h) */

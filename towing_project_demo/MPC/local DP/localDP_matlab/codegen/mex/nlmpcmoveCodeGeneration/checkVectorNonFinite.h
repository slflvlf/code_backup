/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * checkVectorNonFinite.h
 *
 * Code generation for function 'checkVectorNonFinite'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
int32_T b_checkVectorNonFinite(const emlrtStack *sp, const real_T vec[60]);

int32_T checkVectorNonFinite(const emlrtStack *sp, int32_T N,
                             const real_T vec_data[], int32_T vec_size,
                             int32_T iv0);

/* End of code generation (checkVectorNonFinite.h) */

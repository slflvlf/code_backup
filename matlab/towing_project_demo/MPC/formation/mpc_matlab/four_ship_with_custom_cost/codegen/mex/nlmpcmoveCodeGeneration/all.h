/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * all.h
 *
 * Code generation for function 'all'
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
boolean_T all(const boolean_T x_data[], int32_T x_size);

void b_all(const emlrtStack *sp, const boolean_T x[192], boolean_T y[24]);

/* End of code generation (all.h) */
